from __future__ import annotations

import logging
import random
import time

import numpy as np

from ..partition.tasks import build_tasks
from ..scheduling.scheduler import Scheduler, SearchTimeout
from .local_cp import local_cp

LOG = logging.getLogger("hycos")
OPERATORS = ("critical_relocate", "workload_balance", "conflict_reorder", "path_reorientation",
             "geometry_repartition", "timing_repair", "local_cp_sat")


def critical_chain(scheduler, solution):
    """Backtrack binding robot, local-support and collision-release predecessors."""
    by_id = {r.task_id: r for r in solution.reservations}
    robot_previous = {}
    for robot in range(3):
        agenda = sorted((r for r in solution.reservations if r.motion.robot == robot), key=lambda r: r.start)
        for a, b in zip(agenda[:-1], agenda[1:]):
            robot_previous[b.task_id] = a.task_id
    by_end = sorted(solution.reservations, key=lambda r: r.end)
    import bisect
    end_times = [r.end for r in by_end]
    current = max(solution.reservations, key=lambda r: r.end)
    chain = []
    while current.task_id not in chain:
        chain.append(current.task_id)
        predecessors = set(scheduler.tasks[current.task_id].predecessors)
        if current.task_id in robot_previous:
            predecessors.add(robot_previous[current.task_id])
        position = bisect.bisect_right(end_times, current.start + 1e-7) - 1
        if position >= 0:
            candidate = by_end[position]
            if (abs(candidate.end - current.start) < 0.001
                    and candidate.motion.robot != current.motion.robot
                    and scheduler.checker.hull_conflict(candidate.motion, current.motion)):
                predecessors.add(candidate.task_id)
        if not predecessors:
            break
        current = max((by_id[p] for p in predecessors), key=lambda r: r.end)
    return chain


def optimize(scheduler, initial, layers, seconds, iterations, seed, checkpoint, history_callback=None):
    rng = random.Random(seed)
    deadline = time.monotonic() + seconds
    weights = {name: 1.0 for name in OPERATORS}
    stats = {name: {"attempts": 0, "improvements": 0, "gain_s": 0.0} for name in OPERATORS}
    best, history = initial, []
    for iteration in range(iterations):
        if time.monotonic() >= deadline:
            break
        operator = OPERATORS[iteration] if iteration < len(OPERATORS) else rng.choices(
            OPERATORS, weights=[weights[o] for o in OPERATORS], k=1)[0]
        stats[operator]["attempts"] += 1
        started = time.monotonic()
        trial_scheduler = scheduler
        priorities = {task_id: rank for rank, task_id in enumerate(best.order)}
        assignments, orientations = dict(best.assignments), dict(best.orientations)
        releases, changed, detail = {}, set(), {}
        # Follow the ending robot and its blocking/support predecessors to the critical region.
        ending = max(best.reservations, key=lambda r: r.end)
        critical_robot = ending.motion.robot
        critical = critical_chain(scheduler, best)
        # Start at the tail, then move bounded neighborhoods along the full critical chain.
        # Early changes may propagate through a longer suffix; the deadline protects the incumbent.
        end_rank = len(best.order)
        if iteration >= len(OPERATORS) and rng.random() < 0.6:
            anchor = rng.choice(critical)
            end_rank = best.order.index(anchor) + 1
        suffix = best.order[max(0, end_rank - 72):end_rank]
        pool = [t for t in critical if t in suffix] or suffix
        window = suffix[max(0, len(suffix) - 24):]
        candidate = None
        outcome = "no_candidate"
        try:
            if operator in ("critical_relocate", "workload_balance"):
                workload = [sum(r.motion.duration for r in best.reservations if r.motion.robot == robot)
                            for robot in range(3)]
                if operator == "workload_balance":
                    busiest = int(np.argmax(workload))
                    balance_pool = [t for t in suffix if assignments[t] == busiest]
                    task_id = rng.choice(balance_pool or pool)
                else:
                    task_id = rng.choice(pool)
                alternatives = [r for r in scheduler.tasks[task_id].eligible if r != assignments[task_id]]
                if alternatives:
                    assignments[task_id] = (min(alternatives, key=lambda r: workload[r])
                                             if operator == "workload_balance" else rng.choice(alternatives))
                    changed.add(task_id)
                    if operator == "workload_balance":
                        for t in rng.sample(window, min(5, len(window))):
                            assignments.pop(t, None)
                            changed.add(t)
            elif operator == "conflict_reorder":
                if len(window) > 1:
                    pairs = []
                    by_id = {r.task_id: r for r in best.reservations}
                    for a in window:
                        for b in window:
                            if a < b and by_id[a].motion.robot != by_id[b].motion.robot:
                                if scheduler.checker.hull_conflict(by_id[a].motion, by_id[b].motion):
                                    pairs.append((a, b))
                    if pairs:
                        a, b = rng.choice(pairs)
                        priorities[a], priorities[b] = priorities[b], priorities[a]
                        changed.update((a, b))
            elif operator == "path_reorientation":
                task_id = rng.choice(pool)
                orientations[task_id] = (orientations.get(task_id, 0) + 1) % 4
                changed.add(task_id)
            elif operator == "timing_repair":
                for t in window:
                    priorities[t] = min(priorities[x] for x in window) + (best.makespan - scheduler.tail[t]) / max(1, best.makespan)
                    assignments.pop(t, None)
                    changed.add(t)
            elif operator == "geometry_repartition":
                bias = np.zeros(3)
                bias[critical_robot] = rng.uniform(25000, 90000)
                # Repartition only the final occupied layers: earlier geometry and IDs stay stable.
                first_layer = max(l.index for l in layers) - 2
                tail_layers = [l for l in layers if l.index >= first_layer]
                replacement = build_tasks(tail_layers, scheduler.cell, weights=bias,
                                          chunk_length=rng.choice((400.0, 800.0)))
                first_task = next(t.id for t in scheduler.tasks if t.layer >= first_layer)
                kept = scheduler.tasks[:first_task]
                lower = [t for t in kept if t.layer == tail_layers[0].index - 1]
                for task in replacement:
                    task.id += first_task
                    task.predecessors = {p + first_task for p in task.predecessors}
                    if task.layer == tail_layers[0].index and lower:
                        task.predecessors.update(t.id for t in lower if
                            t.footprint.intersects(task.footprint.buffer(scheduler.cell.width / 2)))
                        if not task.predecessors:
                            task.predecessors.update(t.id for t in lower)
                trial_scheduler = Scheduler(scheduler.cell, kept + replacement, scheduler.planner, scheduler.checker)
                prefix = []
                for reservation in best.reservations:
                    if reservation.task_id >= first_task:
                        break
                    prefix.append(reservation)
                candidate = trial_scheduler.solve(prefix=prefix, deadline=deadline, progress=False)
                detail["repartitioned_layers"] = [l.index for l in tail_layers]
            elif operator == "local_cp_sat":
                selected = window[-min(14, len(window)):]
                result, detail = local_cp(scheduler, best, selected,
                    min(3.0, max(0.01, deadline - time.monotonic())), seed + iteration)
                if result:
                    cp_assignment, order, releases = result
                    assignments.update(cp_assignment)
                    first_rank = min(priorities[t] for t in selected)
                    priorities.update({task_id: first_rank + rank for rank, task_id in enumerate(order)})
                    changed.update(selected)
            if candidate is None and changed:
                candidate = scheduler.incremental(best, priorities, assignments, orientations,
                                                    changed, deadline, releases)
            gain = best.makespan - candidate.makespan if candidate else 0.0
            if candidate and gain > 1e-6:
                # Transactional acceptance: checkpoint itself runs structural/collision preflight.
                checkpoint(candidate, trial_scheduler, "improved")
                best, scheduler = candidate, trial_scheduler
                stats[operator]["improvements"] += 1
                stats[operator]["gain_s"] += gain
                reward, outcome = 5.0 + min(10, gain / max(1, best.makespan) * 100), "accepted"
            else:
                reward, outcome = 0.2, "rejected" if candidate else "no_candidate"
        except SearchTimeout:
            reward, outcome = 0.1, "time_limit"
        except (ValueError, RuntimeError) as exc:
            reward, outcome, detail = 0.1, "infeasible", {"reason": str(exc)}
        weights[operator] = max(0.1, 0.8 * weights[operator] + 0.2 * reward)
        row = {"iteration": iteration + 1, "operator": operator, "outcome": outcome,
               "elapsed_s": time.monotonic() - started, "best_makespan_s": best.makespan,
               "weight": weights[operator], "detail": detail}
        history.append(row)
        if history_callback is not None:
            history_callback({"history": history, "operators": stats, "weights": weights})
        LOG.info("ALNS %d %s: %s, best=%.2fs, elapsed=%.2fs", iteration + 1, operator,
                 outcome, best.makespan, row["elapsed_s"])
    return best, scheduler, {"history": history, "operators": stats, "weights": weights}
