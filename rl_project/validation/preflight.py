from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

from ..core import atomic_json
from ..geometry.optimizer import evaluate
from shapely import union_all

HEADER = ("robot_id", "time_s", "x_mm", "y_mm", "z_mm", "mode")


def check_partition_shape(tasks, layers, cell):
    by_layer, cache, metrics = {}, {}, []
    for task in tasks:
        by_layer.setdefault(task.layer, []).append(task)
    for layer in layers:
        group = by_layer.get(layer.index, [])
        key = (layer.cache_key, tuple(t.footprint.wkb for t in group))
        if key not in cache:
            cache[key] = evaluate(layer.polygon, union_all([t.footprint for t in group]))
        metrics.append(dict(cache[key], layer=layer.index))
    target = sum(m["target_area"] for m in metrics)
    deposited = sum(m["deposited_area"] for m in metrics)
    intersection = sum(m["intersection_area"] for m in metrics)
    if target <= 0:
        raise ValueError("Empty target")
    result = {"coverage": intersection / target, "overfill": (deposited - intersection) / target,
              "iou": intersection / (target + deposited - intersection),
              "failed_layer_ratio": sum(m["iou"] < cell.shape["minimum_layer_iou"] for m in metrics) / len(metrics),
              "layers": metrics}
    thresholds = cell.shape
    if (result["coverage"] < thresholds["minimum_overall_coverage"]
            or result["overfill"] > thresholds["maximum_overall_overfill_ratio"]
            or result["iou"] < thresholds["minimum_overall_iou"]
            or result["failed_layer_ratio"] > thresholds["maximum_failed_layer_ratio"]):
        raise ValueError("Partition geometry does not meet internal shape thresholds")
    return result


def check_solution(solution, scheduler):
    tasks, checker, parks = scheduler.tasks, scheduler.checker, scheduler.planner.parks
    if len(solution.reservations) != len(tasks):
        raise ValueError("Incomplete schedule")
    by_id = {r.task_id: r for r in solution.reservations}
    if set(by_id) != set(range(len(tasks))):
        raise ValueError("Duplicate or missing task")
    if not math.isfinite(solution.makespan) or abs(solution.makespan - max(r.end for r in solution.reservations)) > 1e-8:
        raise ValueError("Makespan does not match completed schedule")
    agendas = [sorted((r for r in solution.reservations if r.motion.robot == robot), key=lambda r: r.start)
               for robot in range(3)]
    for task in tasks:
        r = by_id[task.id]
        if not math.isfinite(r.start) or r.start < 0:
            raise ValueError("Invalid task start")
        if any(by_id[p].end > r.start + 1e-9 for p in task.predecessors):
            raise ValueError("Local support precedence violation")
        if not np.array_equal(r.motion.xyz[0], parks[r.motion.robot]) or not np.array_equal(r.motion.xyz[-1], parks[r.motion.robot]):
            raise ValueError("Trip does not return to certified park")
        expected = scheduler.planner.motion(task, r.motion.robot, solution.orientations.get(task.id, 0))
        if expected.key != r.motion.key:
            raise ValueError("Motion does not match assigned task geometry")
        deposited_length = sum(float(np.linalg.norm(b[:2] - a[:2])) for a, b, mode in
                               zip(r.motion.xyz[:-1], r.motion.xyz[1:], r.motion.modes) if mode == "D")
        if abs(deposited_length - task.length) > max(0.001, task.length * 1e-7):
            raise ValueError("Motion lost or introduced deposition length")
        if not checker.safe_against_parks(r.motion.robot, r.motion.xyz, parks):
            raise ValueError("Motion interferes with idle or completed robot")
    for robot in range(3):
        for a, b in zip(agendas[robot][:-1], agendas[robot][1:]):
            if a.end > b.start + 1e-9:
                raise ValueError("Overlapping tasks on one robot")
        for other in range(robot):
            a, b = agendas[robot], agendas[other]
            i = j = 0
            while i < len(a) and j < len(b):
                left, right = a[i], b[j]
                if left.end >= right.start and right.end >= left.start:
                    conflict = checker.motions(left.motion, left.start, right.motion, right.start)
                    if conflict:
                        raise ValueError(f"Continuous collision remains: {conflict}")
                if left.end <= right.end:
                    i += 1
                else:
                    j += 1
    return {"task_completeness": True, "local_support": True, "continuous_collision": True,
            "stationary_after_finish": True}


def trajectory_rows(solution, parks):
    horizon = max(solution.makespan, 1e-6)
    for robot in range(3):
        time, position = 0.0, parks[robot]
        agenda = sorted((r for r in solution.reservations if r.motion.robot == robot), key=lambda r: r.start)
        for reservation in agenda:
            if reservation.start > time:
                yield (robot + 1, time, *position, "W")
            for i, mode in enumerate(reservation.motion.modes):
                yield (robot + 1, reservation.start + reservation.motion.times[i],
                       *reservation.motion.xyz[i], mode)
            time, position = reservation.end, reservation.motion.xyz[-1]
        if time < horizon:
            yield (robot + 1, time, *position, "W")
        yield (robot + 1, horizon, *position, "W")


def check_interval(previous, row, cell):
    robot, time, *rest = previous
    a, b = np.asarray(rest[:3]), np.asarray(row[2:5])
    mode, duration = previous[5], row[1] - time
    if duration <= 0:
        raise ValueError("Non-increasing CSV time")
    delta, length = b - a, float(np.linalg.norm(b - a))
    if mode == "W":
        if length > cell.raw["validation"]["wait_position_tolerance_mm"]:
            raise ValueError("W interval moves")
        return
    if length <= cell.epsilon:
        raise ValueError("Zero-length D/T interval")
    speed = length / duration
    tolerance = cell.raw["validation"]["speed_relative_tolerance"]
    if mode == "D":
        if np.linalg.norm(delta[:2]) <= cell.epsilon:
            raise ValueError("Zero-length XY deposition after float32 conversion")
        z_tolerance = cell.raw["validation"]["layer_z_tolerance_mm"]
        if abs(delta[2]) > z_tolerance:
            raise ValueError("D moves in Z")
        reference = cell.process["deposition_speed_mm_s"]
        if abs(speed / reference - 1) > tolerance + 1e-10:
            raise ValueError("Deposition speed violation")
        offset = 1 if cell.process["tcp_z_reference"] == "top" else 0.5
        raw_layer = ((a[2] + b[2]) / 2 - cell.process["build_plane_z_mm"]) / cell.process["layer_height_mm"] - offset
        layer = round(raw_layer)
        if layer < 0 or abs((a[2] + b[2]) / 2 - cell.z(layer)) > z_tolerance:
            raise ValueError("Invalid deposition layer")
        workspace = cell.raw["workspace"]
        radii = np.linalg.norm(np.array([a[:2], b[:2]]) - workspace["center_xy_mm"], axis=1)
        if max(radii) + cell.width / 2 > workspace["radius_mm"] + cell.epsilon:
            raise ValueError("Bead exceeds workspace")
    elif mode == "T":
        if speed > cell.process["travel_speed_mm_s"] * (1 + tolerance) + 1e-8:
            raise ValueError("Travel speed violation")
    else:
        raise ValueError("Unknown CSV mode")


def check_rows(rows, cell, expected=None):
    previous, counts, intervals = None, [0, 0, 0], {"D": 0, "T": 0, "W": 0}
    for row in rows:
        robot = row[0]
        if robot not in (1, 2, 3) or row[5] not in ("D", "T", "W"):
            raise ValueError("Invalid CSV robot or mode")
        if not np.isfinite(row[1:5]).all():
            raise ValueError("Nonfinite CSV value")
        # Exactly reproduce the official coordinate storage type.
        point = np.asarray(row[2:5], dtype=np.float32).astype(float)
        row = (robot, row[1], *point, row[5])
        if expected is not None:
            reference = next(expected, None)
            if reference is None or tuple(reference) != row:
                raise ValueError("Serialized trajectory differs from collision-certified motion")
        if not cell.reachable(robot - 1, [point]):
            raise ValueError("CSV point outside reach")
        if previous is None or previous[0] != robot:
            if row[1] != 0 or (previous is not None and (robot != previous[0] + 1 or previous[5] != "W")):
                raise ValueError("CSV robot ordering, start time or terminal mode violation")
        else:
            check_interval(previous, row, cell)
            intervals[previous[5]] += 1
        counts[robot - 1] += 1
        previous = row
    if previous is None or previous[5] != "W" or min(counts) < 2:
        raise ValueError("All three robots require at least two rows and terminal W")
    if expected is not None and next(expected, None) is not None:
        raise ValueError("CSV truncated")
    return {"rows": sum(counts), "robot_rows": counts, "intervals": intervals}


def write_trajectory(path: Path, solution, scheduler):
    checks = check_solution(solution, scheduler)
    parks, cell = scheduler.planner.parks, scheduler.cell
    checks.update(check_rows(trajectory_rows(solution, parks), cell))
    temporary = path.with_suffix(path.suffix + ".tmp")
    path.parent.mkdir(parents=True, exist_ok=True)
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(HEADER)
        for row in trajectory_rows(solution, parks):
            # 17 significant digits round-trip doubles, including float32 coordinates.
            writer.writerow([row[0], *(format(float(x), ".17g") for x in row[1:5]), row[5]])
    with temporary.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        if tuple(next(reader)) != HEADER:
            raise ValueError("Invalid serialized header")
        rows = ((int(r[0]), *(float(x) for x in r[1:5]), r[5]) for r in reader)
        checks["csv_roundtrip"] = check_rows(rows, cell, expected=iter(trajectory_rows(solution, parks)))
    temporary.replace(path)
    return checks


def checkpoint(path, solution, scheduler, label):
    checks = write_trajectory(path / f"trajectory.{label}.csv", solution, scheduler)
    atomic_json(path / f"schedule.{label}.json", {**solution.summary(), "checks": checks,
        "reservations": [{"task": r.task_id, "robot_id": r.motion.robot + 1,
                          "start_s": r.start, "end_s": r.end,
                          "orientation": solution.orientations.get(r.task_id, 0)}
                         for r in solution.reservations]})
    return checks
