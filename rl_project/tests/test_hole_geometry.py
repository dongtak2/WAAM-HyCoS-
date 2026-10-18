"""Regression tests for material parity and deposition across voids."""
import tempfile
import unittest
from pathlib import Path

import numpy as np
import trimesh
from shapely import LineString, Polygon, box, union_all

from rl_project.core import Cell, Task
from rl_project.geometry.debug_shape import reference_section
from rl_project.geometry.optimizer import (GeometryOptimizer, centerlines, clip_paths, contours,
                                           even_odd_material, footprint, parts, raster, slice_polygon)
from rl_project.scheduling.planner import MotionPlanner
from rl_project.validation.collision import CollisionChecker

ROOT = Path(__file__).resolve().parents[1]


class HoleGeometryTests(unittest.TestCase):
    def test_self_touching_hole_is_not_dropped(self):
        outer = LineString(box(-10, -10, 10, 10).exterior.coords)
        touching = LineString([(0,0),(4,0),(4,4),(0,4),(0,0),
                               (-4,0),(-4,-4),(0,-4),(0,0)])
        self.assertFalse(Polygon(touching.coords).is_valid)
        result = even_odd_material([outer, touching], 1e-4)
        expected = box(-10,-10,10,10).difference(union_all([box(0,0,4,4),box(-4,-4,0,0)]))
        self.assertLess(result.symmetric_difference(expected).area, 1e-8)

    def test_nested_island_and_disconnected_component(self):
        rings = [LineString(p.exterior.coords) for p in
                 [box(-10,-10,10,10),box(-6,-6,6,6),box(-2,-2,2,2),box(20,0,24,4)]]
        result = even_odd_material(rings[::-1], 1e-4)
        self.assertAlmostEqual(result.area, 400 - 144 + 16 + 16)
        self.assertEqual(len(parts(result)), 3)

    def test_all_path_families_preserve_holes_and_gaps(self):
        material = box(-60,-50,60,50).difference(box(-32,-25,32,25)).union(box(90,-8,120,8))
        families = [contours(material,8), raster(material,8,0), raster(material,8,60),
                    centerlines(material,8)]
        for family in families:
            self.assertTrue(family)
            for path in family:
                self.assertTrue(material.buffer(2e-4).covers(LineString(path)))
        clipped = clip_paths([np.array([[-70.,0],[130.,0]])], material)
        self.assertEqual(len(clipped), 3)

    def test_separate_fragments_use_travel(self):
        c = Cell.load(ROOT.parent / "tests/01/config.yaml")
        paths = (np.array([[-35.,0],[-15.,0]]),np.array([[15.,0],[35.,0]]))
        task = Task(0,0,paths,0,(0,1,2),40,footprint(paths,c))
        motion = MotionPlanner(c,CollisionChecker(c)).motion(task,0)
        void = box(-14,-3,14,3)
        travel_across_void = False
        for a,b,mode in zip(motion.xyz[:-1],motion.xyz[1:],motion.modes):
            crossing = LineString([a[:2],b[:2]]).intersection(void).length
            if mode == "D":
                self.assertEqual(crossing,0)
            elif mode == "T" and crossing > 0:
                travel_across_void = True
        self.assertTrue(travel_across_void)

    def test_model03_all_layers_match_raw_loop_reference(self):
        c = Cell.load(ROOT.parent / "tests/03/config.yaml")
        mesh = trimesh.load_mesh(ROOT.parent / "tests/03/target.stl")
        for index in range(6):
            actual = slice_polygon(mesh,2*index+1,c)
            expected,_ = reference_section(mesh,2*index+1,c)
            self.assertLess(actual.symmetric_difference(expected).area,0.001)
            self.assertAlmostEqual(actual.area,16439.58,delta=0.1)
            self.assertEqual(sum(len(p.interiors) for p in parts(actual)),9)

    def test_model01_fresh_geometry_all_layers(self):
        c = Cell.load(ROOT.parent / "tests/01/config.yaml")
        mesh = trimesh.load_mesh(ROOT.parent / "tests/01/target.stl")
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            optimizer = GeometryOptimizer(c,Path(directory))
            for index in range(6):
                poly = slice_polygon(mesh,2*index+1,c)
                reference,_ = reference_section(mesh,2*index+1,c)
                self.assertLess(poly.symmetric_difference(reference).area,0.001)
                paths,metrics,_ = optimizer.optimize(poly)
                self.assertGreaterEqual(metrics["coverage"],0.95)
                self.assertGreaterEqual(metrics["iou"],0.90)
                self.assertLessEqual(metrics["overfill"],c.shape["maximum_overall_overfill_ratio"])


if __name__ == "__main__":
    unittest.main()
