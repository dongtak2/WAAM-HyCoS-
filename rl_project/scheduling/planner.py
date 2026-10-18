from __future__ import annotations

from collections import OrderedDict

import numpy as np
from shapely import MultiPoint

from ..core import Motion, digest, quantize


class MotionPlanner:
    def __init__(self, cell, checker, cache_size=12000):
        self.cell, self.checker, self.cache_size = cell, checker, cache_size
        self.cache = OrderedDict()
        self.parks = quantize([r.get("home_xyz_mm") or r["base_xyz_mm"] for r in cell.robots])
        if not self._valid_parks():
            self.parks = quantize([r["base_xyz_mm"] for r in cell.robots])
        if not self._valid_parks():
            raise ValueError("No safe parking arrangement at home or base positions")

    def _valid_parks(self):
        for r in range(3):
            if not self.cell.reachable(r, [self.parks[r]]):
                return False
            for s in range(r):
                if self.checker.linear_conflict(r, s, self.parks[r], self.parks[r],
                                                self.parks[s], self.parks[s], 1):
                    return False
        return True

    def _order_paths(self, task, robot, orientation):
        remaining = [np.array(p, copy=True) for p in task.paths]
        ordered, position = [], self.parks[robot][:2]
        while remaining:
            choices = []
            for i, path in enumerate(remaining):
                if np.linalg.norm(path[0] - path[-1]) < 1e-4:
                    index = int(np.argmin(np.linalg.norm(path[:-1] - position, axis=1)))
                    ring = np.roll(path[:-1], -index, axis=0)
                    remaining[i] = path = np.vstack([ring, ring[0]])
                for reverse in (False, True):
                    distance = np.linalg.norm(path[-1 if reverse else 0] - position)
                    choices.append((distance, i, reverse))
            choices.sort()
            _, i, reverse = choices[0]
            path = remaining.pop(i)
            if reverse:
                path = path[::-1]
            ordered.append(path)
            position = path[-1]
        if orientation % 2:
            ordered = [p[::-1] for p in ordered[::-1]]
        if orientation >= 2 and len(ordered) > 1:
            shift = (orientation // 2) % len(ordered)
            ordered = ordered[shift:] + ordered[:shift]
        return ordered

    def motion(self, task, robot, orientation=0):
        if robot not in task.eligible:
            raise ValueError("Unreachable robot assignment")
        signature = digest(b"".join(len(p).to_bytes(8, "little") + p.tobytes() for p in task.paths))
        key = (signature, task.layer, robot, orientation)
        if key in self.cache:
            self.cache.move_to_end(key)
            return self.cache[key]
        z = float(quantize(self.cell.z(task.layer)))
        points, times, modes = [self.parks[robot]], [0.0], []
        process = self.cell.process

        def append(point, mode, wait=0.0):
            point = quantize(point)
            distance = float(np.linalg.norm(point - points[-1]))
            if mode == "W":
                if wait <= 0:
                    return
                point = points[-1]
                duration = wait
            else:
                if distance <= max(1e-4, self.cell.epsilon * 4):
                    return
                speed = process["deposition_speed_mm_s"] if mode == "D" else process["travel_speed_mm_s"]
                duration = distance / speed
            modes.append(mode)
            times.append(times[-1] + duration)
            points.append(point)

        def travel(point):
            point = quantize(point)
            segment = np.array([points[-1], point])
            if not self.checker.safe_against_parks(robot, segment, self.parks):
                # Spatial repair is in XY: retract through the certified park.
                route = np.array([points[-1], self.parks[robot], point])
                if not self.checker.safe_against_parks(robot, route, self.parks):
                    raise ValueError(f"Robot {robot + 1} travel obstructed by a parked robot")
                append(self.parks[robot], "T")
            append(point, "T")

        for path in self._order_paths(task, robot, orientation):
            travel([*path[0], z])
            append(points[-1], "W", process.get("arc_on_time_s") or 0.0)
            for xy in path[1:]:
                append([*xy, z], "D")
            append(points[-1], "W", process.get("arc_off_time_s") or 0.0)
        travel(self.parks[robot])
        xyz, times = np.asarray(points), np.asarray(times)
        if not self.cell.reachable(robot, xyz):
            raise ValueError("Motion exceeds XY reach")
        if not self.checker.safe_against_parks(robot, xyz, self.parks):
            raise ValueError("Deposition interferes with stationary parked robot")
        motion_key = digest(str(robot).encode() + xyz.tobytes() + times.tobytes() + "".join(modes).encode())
        motion = Motion(robot, times, xyz, tuple(modes), motion_key,
                        MultiPoint(np.vstack([self.checker.bases[robot], xyz[:, :2]])).convex_hull,
                        MultiPoint(xyz[:, :2]).convex_hull,
                        digest(str(robot).encode() + xyz[:, :2].tobytes()))
        self.cache[key] = motion
        if len(self.cache) > self.cache_size:
            self.cache.popitem(last=False)
        return motion
