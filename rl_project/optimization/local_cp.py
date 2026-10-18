from __future__ import annotations

import math

from ortools.sat.python import cp_model


def local_cp(scheduler, solution, neighborhood, seconds, seed):
    """Optimize assignment, order and starts with frozen outside reservations.

    Swept-envelope conflicts are conservative disjunctions. The continuous
    scheduler subsequently validates the proposal and can reject it.
    """
    model = cp_model.CpModel()
    scale = 1000
    selected = set(neighborhood)
    horizon = math.ceil((solution.makespan + 100) * scale)
    starts, ends, options, robot_intervals = {}, {}, {}, [[], [], []]
    by_task = {r.task_id: r for r in solution.reservations}
    origin = min(by_task[i].start for i in selected)
    left = max(0, math.floor(origin * scale))
    for task_id in neighborhood:
        task = scheduler.tasks[task_id]
        starts[task_id] = model.new_int_var(left, horizon, f"s{task_id}")
        ends[task_id] = model.new_int_var(left, horizon, f"e{task_id}")
        choices = []
        for robot in task.eligible:
            try:
                motion = scheduler.planner.motion(task, robot, solution.orientations.get(task_id, 0))
            except ValueError:
                continue
            present = model.new_bool_var(f"x{task_id}_{robot}")
            duration = math.ceil(motion.duration * scale)
            interval = model.new_optional_interval_var(starts[task_id], duration, ends[task_id],
                                                       present, f"i{task_id}_{robot}")
            robot_intervals[robot].append(interval)
            options[task_id, robot] = (present, motion, interval)
            choices.append(present)
            model.add_hint(present, int(solution.assignments[task_id] == robot))
        if not choices:
            return None, {"status": "NO_FEASIBLE_MOTION"}
        model.add_exactly_one(choices)
        for predecessor in task.predecessors:
            if predecessor not in selected:
                model.add(starts[task_id] >= math.ceil(by_task[predecessor].end * scale))
        model.add_hint(starts[task_id], max(left, math.ceil(by_task[task_id].start * scale)))
    for task_id in neighborhood:
        for predecessor in scheduler.tasks[task_id].predecessors & selected:
            model.add(starts[task_id] >= ends[predecessor])
    for outside in solution.reservations:
        if outside.task_id in selected:
            continue
        if outside.end < origin - 0.001:
            continue
        a = math.floor(outside.start * scale)
        b = math.ceil(outside.end * scale)
        fixed = model.new_fixed_size_interval_var(a, max(1, b - a), f"fixed{outside.task_id}")
        robot_intervals[outside.motion.robot].append(fixed)
        for (task_id, robot), (present, motion, _) in options.items():
            if robot == outside.motion.robot:
                continue
            if scheduler.checker.hull_conflict(motion, outside.motion):
                before = model.new_bool_var(f"f{task_id}_{robot}_{outside.task_id}")
                model.add(ends[task_id] <= a).only_enforce_if([present, before])
                model.add(starts[task_id] >= b).only_enforce_if([present, before.Not()])
        # Fixed successor deadlines retain the external support DAG.
        for predecessor in scheduler.tasks[outside.task_id].predecessors & selected:
            model.add(ends[predecessor] <= a)
    for intervals in robot_intervals:
        model.add_no_overlap(intervals)
    option_items = list(options.items())
    graph_edges = 0
    for index, ((ta, ra), (xa, ma, _)) in enumerate(option_items):
        for (tb, rb), (xb, mb, _) in option_items[index + 1:]:
            if ta == tb or ra == rb:
                continue
            learned = tuple(sorted((ma.key, mb.key))) in scheduler.learned
            if learned or scheduler.checker.hull_conflict(ma, mb):
                graph_edges += 1
                before = model.new_bool_var(f"order{ta}_{ra}_{tb}_{rb}")
                model.add(ends[ta] <= starts[tb]).only_enforce_if([xa, xb, before])
                model.add(ends[tb] <= starts[ta]).only_enforce_if([xa, xb, before.Not()])
    maximum = model.new_int_var(0, horizon, "local_makespan")
    model.add_max_equality(maximum, list(ends.values()))
    model.minimize(maximum * (len(selected) + 1) + sum(ends.values()))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = max(0.01, seconds)
    solver.parameters.num_search_workers = 1
    solver.parameters.random_seed = seed
    status = solver.solve(model)
    info = {"status": solver.status_name(status), "tasks": len(selected),
            "conflict_edges": graph_edges, "wall_s": solver.wall_time}
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return None, info
    assignment = {task_id: robot for (task_id, robot), (present, _, _) in options.items()
                  if solver.value(present)}
    ordered = sorted(neighborhood, key=lambda t: (solver.value(starts[t]), t))
    release = {t: solver.value(starts[t]) / scale for t in selected}
    return (assignment, ordered, release), info
