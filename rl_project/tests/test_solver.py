from __future__ import annotations

import copy
import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np
import trimesh
from shapely import LineString, MultiPoint, Polygon, box

from rl_project.core import Cell, Layer, Motion, quantize
from rl_project.geometry.optimizer import GeometryOptimizer, footprint, slice_polygon
from rl_project.optimization.alns import OPERATORS, optimize
from rl_project.optimization.local_cp import local_cp
from rl_project.partition.tasks import build_tasks
from rl_project.scheduling.planner import MotionPlanner
from rl_project.scheduling.scheduler import Scheduler
from rl_project.validation.collision import CollisionChecker, segment_distance
from rl_project.validation.preflight import (check_interval, check_partition_shape, check_solution,
                                              write_trajectory)

ROOT = Path(__file__).resolve().parents[1]


def cell():
    return Cell.load(ROOT.parent / "tests" / "01" / "config.yaml")


def fixture():
    config = cell()
    layers = []
    for index in range(2):
        paths = [quantize([[x - length / 2, y], [x + length / 2, y]])
                 for x, y, length in [(-280, 0, 24), (170, 290, 80), (170, -290, 48)]]
        poly = footprint(paths, config)
        layers.append(Layer(index, poly, paths, {}, str(index)))
    tasks = build_tasks(layers, config, cluster_radius=100)
    checker = CollisionChecker(config)
    planner = MotionPlanner(config, checker)
    return Scheduler(config, tasks, planner, checker), layers


class CollisionTests(unittest.TestCase):
    def test_tcp_crosses_between_safe_endpoints(self):
        config = cell()
        config.raw["collision"]["check_arm_envelope"] = False
        checker = CollisionChecker(config)
        a0, a1 = np.array([-400, 0, 0]), np.array([400, 0, 0])
        b0, b1 = np.array([0, -400, 0]), np.array([0, 400, 0])
        self.assertGreater(np.linalg.norm(a0 - b0), 200)
        self.assertGreater(np.linalg.norm(a1 - b1), 200)
        self.assertEqual(checker.linear_conflict(0, 1, a0, a1, b0, b1, 1), "tcp")

    def test_certificate_against_dense_independent_samples(self):
        checker = CollisionChecker(cell())
        random = np.random.default_rng(17)
        certified = 0
        for _ in range(35):
            a0, a1, b0, b1 = random.uniform(-600, 600, (4, 2))
            if checker.linear_conflict(0, 1, a0, a1, b0, b1, 1) is None:
                certified += 1
                arm, tcp = checker.thresholds(0, 1)
                for fraction in np.linspace(0, 1, 81):
                    a, b = a0 + fraction * (a1 - a0), b0 + fraction * (b1 - b0)
                    self.assertGreater(segment_distance(checker.bases[0], a, checker.bases[1], b), arm)
                    self.assertGreater(np.linalg.norm(a - b), tcp)
        self.assertGreater(certified, 0)

    def test_completed_robot_still_collides(self):
        checker = CollisionChecker(cell())
        p = np.array([0.0, 0, 1])
        self.assertIsNotNone(checker.linear_conflict(0, 1, p, p, p + [300, 0, 0], p, 2))

    def test_collision_cache_distinguishes_relative_timing(self):
        config = cell()
        config.raw["collision"]["check_arm_envelope"] = False
        checker = CollisionChecker(config)
        motions = []
        for robot, xyz in enumerate((np.array([[-400., 0, 1], [400., 0, 1]]),
                                     np.array([[0., -400, 1], [0., 400, 1]]))):
            hull = MultiPoint(xyz[:, :2]).convex_hull
            motions.append(Motion(robot, np.array([0., 2.]), xyz, ("T",), str(robot), hull, hull))
        self.assertIsNotNone(checker.motions(motions[0], 0, motions[1], 0))
        self.assertIsNone(checker.motions(motions[0], 0, motions[1], 1))
        self.assertIsNotNone(checker.motions(motions[0], 10, motions[1], 10))
        self.assertGreater(checker.cache_hits, 0)


class GeometryTests(unittest.TestCase):
    def test_hole_slice_and_disconnected_components(self):
        # Extrude an explicitly holed polygon using an independent triangulated ring mesh.
        outer = np.array([[-20, -20], [20, -20], [20, 20], [-20, 20]])
        inner = np.array([[-10, -10], [10, -10], [10, 10], [-10, 10]])
        vertices = np.vstack([np.c_[outer, np.zeros(4)], np.c_[inner, np.zeros(4)],
                              np.c_[outer, np.full(4, 2)], np.c_[inner, np.full(4, 2)]])
        faces = []
        for i in range(4):
            j = (i + 1) % 4
            for a, b, c, d in [(i, j, j + 8, i + 8), (i + 4, i + 12, j + 12, j + 4),
                                (i, i + 4, j + 4, j), (i + 8, j + 8, j + 12, i + 12)]:
                faces.extend([[a, b, c], [a, c, d]])
        mesh = trimesh.Trimesh(vertices, faces, process=True)
        small_box = trimesh.creation.box([8, 8, 2])
        small_box.apply_translation([40, 0, 1])
        mesh = trimesh.util.concatenate([mesh, small_box])
        section = slice_polygon(mesh, 1, cell())
        self.assertAlmostEqual(section.area, 1264, places=3)
        self.assertEqual(len(section.geoms), 2)
        self.assertEqual(sum(len(p.interiors) for p in section.geoms), 1)

    def test_thin_centerline_and_geometry_cache(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            optimizer = GeometryOptimizer(cell(), Path(directory))
            polygon = box(-60, -4, 60, 4)
            paths, metrics, key = optimizer.optimize(polygon)
            again, second, repeated = optimizer.optimize(polygon)
            self.assertGreaterEqual(metrics["iou"], 0.9)
            self.assertEqual(key, repeated)
            self.assertEqual(optimizer.hits, 1)
            self.assertTrue(all(np.isfinite(p).all() for p in paths))


class SchedulingTests(unittest.TestCase):
    def test_local_support_parallelism_and_csv_roundtrip(self):
        scheduler, layers = fixture()
        self.assertEqual(len(scheduler.tasks), 6)
        upper = [t for t in scheduler.tasks if t.layer == 1]
        self.assertTrue(all(len(t.predecessors) == 1 for t in upper))
        solution = scheduler.solve()
        checked = check_solution(solution, scheduler)
        self.assertTrue(checked["continuous_collision"])
        by_id = {r.task_id: r for r in solution.reservations}
        lower_finish = max(by_id[t.id].end for t in scheduler.tasks if t.layer == 0)
        self.assertTrue(any(by_id[t.id].start < lower_finish for t in upper))
        self.assertEqual(set(solution.assignments.values()), {0, 1, 2})
        self.assertGreaterEqual(check_partition_shape(scheduler.tasks, layers, scheduler.cell)["iou"], 0.999)
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            target = Path(directory) / "trajectory.csv"
            info = write_trajectory(target, solution, scheduler)
            self.assertGreater(info["rows"], 10)
            with target.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual({int(r["robot_id"]) for r in rows}, {1, 2, 3})

    def test_cp_sat_and_all_alns_operators(self):
        scheduler, layers = fixture()
        initial = scheduler.solve(conservative=True)
        result, info = local_cp(scheduler, initial, initial.order, 0.5, 3)
        self.assertIn(info["status"], ("OPTIMAL", "FEASIBLE"))
        self.assertIsNotNone(result)
        saved = []
        def save(solution, active, label):
            check_solution(solution, active)
            saved.append(solution.makespan)
        best, active, history = optimize(scheduler, initial, layers, 8, 7, 42, save)
        self.assertLessEqual(best.makespan, initial.makespan)
        self.assertEqual(set(row["operator"] for row in history["history"]), set(OPERATORS))
        check_solution(best, active)

    def test_budget_exhaustion_preserves_incumbent(self):
        scheduler, layers = fixture()
        initial = scheduler.solve()
        best, _, history = optimize(scheduler, initial, layers, 0, 100, 1, lambda *args: None)
        self.assertIs(best, initial)
        self.assertEqual(history["history"], [])

    def test_zero_length_and_stationary_wait_rejected(self):
        config = cell()
        with self.assertRaises(ValueError):
            check_interval((1, 0., 0., 0., 1., "D"), (1, 1., 0., 0., 1., "W"), config)
        with self.assertRaises(ValueError):
            check_interval((1, 0., 0., 0., 1., "W"), (1, 1., 1., 0., 1., "W"), config)


if __name__ == "__main__":
    unittest.main()
