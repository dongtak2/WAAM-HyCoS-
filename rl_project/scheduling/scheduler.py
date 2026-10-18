from __future__ import annotations

import bisect
import heapq
import logging
import time

from ..core import Reservation, Solution

LOG = logging.getLogger("hycos")


class SearchTimeout(Exception):
    """Candidate abandoned; the last complete solution remains available."""


class Scheduler:
    def __init__(self, cell, tasks, planner, checker):
        self.cell, self.tasks, self.planner, self.checker = cell, tasks, planner, checker
        self.learned = set()
        self.events = []
        self.children = [[] for _ in tasks]
        for t in tasks:
            for p in t.predecessors:
                self.children[p].append(t.id)
        self.tail = [t.length / cell.process["deposition_speed_mm_s"] for t in tasks]
        for task in reversed(tasks):
            if self.children[task.id]:
                self.tail[task.id] += max(self.tail[c] for c in self.children[task.id])

    def earliest(self, motion, release, agendas, ends, deadline, conservative=False):
        start = max(release, ends[motion.robot][-1] if ends[motion.robot] else 0.0)
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                raise SearchTimeout()
            blocking = None
            for other in range(3):
                if other == motion.robot:
                    continue
                index = bisect.bisect_left(ends[other], start - 1e-8)
                for reservation in agendas[other][index:]:
                    if reservation.start > start + motion.duration + 1e-8:
                        break
                    if conservative:
                        conflict = self.checker.hull_conflict(motion, reservation.motion)
                    else:
                        conflict = self.checker.motions(motion, start, reservation.motion, reservation.start)
                    if conflict:
                        self.learned.add(tuple(sorted((motion.key, reservation.motion.key))))
                        if len(self.events) < 2000:
                            self.events.append({"robot_a": motion.robot + 1,
                                "robot_b": other + 1, "attempt_start_s": start,
                                "blocking_task": reservation.task_id,
                                "conflict_start_s": getattr(conflict, "start", start),
                                "conflict_end_s": getattr(conflict, "end", reservation.end),
                                "kind": getattr(conflict, "kind", "swept_envelope")})
                        blocking = max(blocking or 0, reservation.end + 0.0001)
                        break
            if blocking is None:
                return start
            start = blocking

    def solve(self, priorities=None, assignments=None, orientations=None, releases=None,
              prefix=None, deadline=None, conservative=False, serial=False, progress=True):
        priorities = priorities or {}
        assignments, orientations, releases = assignments or {}, orientations or {}, releases or {}
        agendas, ends = [[] for _ in range(3)], [[] for _ in range(3)]
        completed, reservations, order, chosen = {}, [], [], {}
        prefix = prefix or []
        for reservation in prefix:
            task_id, robot = reservation.task_id, reservation.motion.robot
            if not self.tasks[task_id].predecessors <= completed.keys():
                raise ValueError("Incremental prefix is not topological")
            agendas[robot].append(reservation)
            ends[robot].append(reservation.end)
            completed[task_id] = reservation.end
            reservations.append(reservation)
            order.append(task_id)
            chosen[task_id] = robot
        degree = [len(t.predecessors - completed.keys()) for t in self.tasks]
        ready = []
        for task in self.tasks:
            if degree[task.id] == 0 and task.id not in completed:
                heapq.heappush(ready, (priorities.get(task.id, task.layer * 1e6 - self.tail[task.id]), task.id))
        last_log = time.monotonic()
        while ready:
            if deadline is not None and time.monotonic() >= deadline:
                raise SearchTimeout()
            _, task_id = heapq.heappop(ready)
            task = self.tasks[task_id]
            release = max([releases.get(task_id, 0.0), *(completed[p] for p in task.predecessors)])
            if serial and completed:
                release = max(release, max(completed.values()) + 0.0001)
            best = None
            robots = (assignments[task_id],) if task_id in assignments else task.eligible
            for robot in robots:
                try:
                    motion = self.planner.motion(task, robot, orientations.get(task_id, 0))
                except ValueError:
                    continue
                start = self.earliest(motion, release, agendas, ends, deadline, conservative)
                busy = sum(r.motion.duration for r in agendas[robot])
                score = start + motion.duration + 0.001 * busy + (0 if robot == task.preferred else 0.05)
                if best is None or score < best[0]:
                    best = score, Reservation(task_id, start, motion)
            if best is None:
                raise ValueError(f"No collision-free reachable motion for task {task_id}")
            reservation = best[1]
            robot = reservation.motion.robot
            agendas[robot].append(reservation)
            ends[robot].append(reservation.end)
            completed[task_id] = reservation.end
            reservations.append(reservation)
            order.append(task_id)
            chosen[task_id] = robot
            for child in self.children[task_id]:
                degree[child] -= 1
                if degree[child] == 0:
                    task_child = self.tasks[child]
                    heapq.heappush(ready, (priorities.get(child, task_child.layer * 1e6 - self.tail[child]), child))
            if progress and (time.monotonic() - last_log > 10 or len(completed) == len(self.tasks)):
                LOG.info("Schedule %d/%d: makespan=%.2fs, conflict_cache=%d/%d", len(completed),
                         len(self.tasks), max(completed.values()), self.checker.cache_hits, self.checker.calls)
                last_log = time.monotonic()
        if len(completed) != len(self.tasks):
            raise ValueError("Precedence cycle or incomplete schedule")
        return Solution(reservations, order, chosen, dict(orientations), max(completed.values()),
                        {"continuous_checked": True, "park_checked": True,
                         "support_edges": sum(len(t.predecessors) for t in self.tasks)})

    def incremental(self, incumbent, priorities, assignments, orientations, changed, deadline, releases=None):
        """Reuse the unaffected topological prefix and rebuild only its causal suffix."""
        first = min((incumbent.order.index(i) for i in changed), default=0)
        # A moved task may jump earlier in the priority sequence.
        if changed:
            minimum_rank = min(priorities.get(i, len(self.tasks)) for i in changed)
            first = min(first, max(0, int(minimum_rank)))
        return self.solve(priorities, assignments, orientations, releases,
                          prefix=incumbent.reservations[:first], deadline=deadline, progress=False)
