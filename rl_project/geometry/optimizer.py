from __future__ import annotations

import json
import logging
import math
from collections import OrderedDict
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import Voronoi, QhullError
from shapely import (LineString, MultiLineString, Point, Polygon, buffer, linestrings, line_merge,
                     make_valid, normalize, set_precision, union_all)
from shapely.affinity import rotate

from ..core import Cell, Layer, atomic_json, digest, quantize

LOG = logging.getLogger("hycos")


def parts(geometry, kind="Polygon"):
    if geometry.is_empty:
        return []
    if geometry.geom_type == kind:
        return [geometry]
    return [p for g in getattr(geometry, "geoms", ()) for p in parts(g, kind)]


def clean_paths(paths, epsilon=1e-4):
    result, seen = [], set()
    for points in paths:
        points = quantize(points)
        if len(points) < 2:
            continue
        kept = [points[0]]
        for point in points[1:]:
            if np.linalg.norm(point - kept[-1]) > epsilon:
                kept.append(point)
        if len(kept) < 2:
            continue
        points = np.asarray(kept)
        key = min(points.tobytes(), points[::-1].tobytes())
        if key not in seen:
            result.append(points)
            seen.add(key)
    return result


def footprint(paths, cell):
    # Buffer individual intervals, matching round segment caps in the contract.
    segments = [np.stack([path[:-1], path[1:]], axis=1) for path in paths if len(path) > 1]
    if not segments:
        return Polygon()
    curves = linestrings(np.concatenate(segments))
    beads = buffer(curves, cell.width / 2, quad_segs=cell.shape["polygon_buffer_resolution"])
    return set_precision(union_all(beads), cell.shape["polygon_snap_tolerance_mm"])


def evaluate(target, deposited):
    intersection = target.intersection(deposited).area
    area = target.area
    union = area + deposited.area - intersection
    return {"target_area": area, "deposited_area": deposited.area,
            "intersection_area": intersection, "coverage": intersection / max(area, 1e-20),
            "overfill": (deposited.area - intersection) / max(area, 1e-20),
            "iou": intersection / max(union, 1e-20)}


def acceptable(metrics, cell, margin=0.002):
    s = cell.shape
    return (metrics["iou"] >= min(1.0, max(s["minimum_overall_iou"], s["minimum_layer_iou"]) + margin)
            and metrics["coverage"] >= max(0.95, s["minimum_overall_coverage"])
            and metrics["overfill"] <= s["maximum_overall_overfill_ratio"])


def clip_paths(paths, poly, tolerance=1e-4):
    """Keep D centerlines in material; disconnected pieces remain separate paths.

    This clips centerlines, not a universal bead-radius inset. Narrow struts
    still use their medial lines; round-cap bead overfill is evaluated separately.
    """
    domain = poly.buffer(tolerance)
    result = []
    for path in paths:
        line = LineString(path)
        if domain.covers(line):
            result.append(path)
        else:
            result.extend(np.asarray(piece.coords)
                          for piece in parts(line.intersection(domain), "LineString"))
    return clean_paths(result)


def even_odd_material(loops, snap):
    """XOR filled loops after repairing self-touching rings, including islands.

    Polygonizing unnoded input lines can silently drop an invalid interior ring.
    Repair each loop independently so its lobes retain their parity contribution.
    """
    rings = [Polygon(loop.coords) for loop in loops if len(loop.coords) >= 4]
    material = Polygon()
    for ring in sorted(rings, key=lambda p: (-p.area, p.bounds)):
        repaired = ring if ring.is_valid else make_valid(ring)
        filled = union_all(parts(repaired))
        material = material.symmetric_difference(filled)
    if not material.is_valid:
        material = make_valid(material)
    return set_precision(union_all(parts(material)), snap)


def contours(poly, width, offset=0.5):
    result = []
    inner = poly.buffer(-width * offset)
    while not inner.is_empty:
        for p in parts(inner):
            result.append(np.asarray(p.exterior.coords))
            result.extend(np.asarray(r.coords) for r in p.interiors)
        inner = inner.buffer(-width * 0.93)
    return clip_paths(result, poly)


def raster(poly, width, angle):
    rotated = rotate(poly, -angle, origin=(0, 0))
    inner = rotated.buffer(-width * 0.48)
    if inner.is_empty:
        return []
    xmin, ymin, xmax, ymax = inner.bounds
    count = max(1, math.ceil((ymax - ymin) / (width * 0.93)))
    ys = np.linspace(ymin, ymax, count + 1) if count > 1 else [(ymin + ymax) / 2]
    paths = []
    for i, y in enumerate(ys):
        segments = parts(inner.intersection(LineString([(xmin - width, y), (xmax + width, y)])), "LineString")
        for segment in sorted(segments, key=lambda p: p.bounds[0], reverse=bool(i % 2)):
            p = np.asarray(rotate(segment, angle, origin=(0, 0)).coords)
            paths.append(p[::-1] if i % 2 else p)
    # Connect alternating scans only if the connector's entire bead is admissible.
    linked = []
    for p in paths:
        if linked and np.linalg.norm(linked[-1][-1] - p[0]) < width * 1.6:
            connector = LineString([linked[-1][-1], p[0]]).buffer(width / 2)
            if (poly.covers(LineString([linked[-1][-1], p[0]]))
                    and poly.buffer(width * 0.015).covers(connector)):
                linked[-1] = np.vstack([linked[-1], p])
                continue
        linked.append(p)
    return clip_paths(linked, poly)


def centerlines(poly, width):
    """Boundary Voronoi medial graph, with short corner twigs removed."""
    samples = []
    for p in parts(poly):
        for ring in [p.exterior, *p.interiors]:
            line = LineString(ring.coords)
            n = max(8, math.ceil(line.length / (width * 0.30)))
            samples.extend(line.interpolate(float(t)).coords[0]
                           for t in np.linspace(0, line.length, n, endpoint=False))
    sites = np.unique(np.asarray(samples), axis=0)
    if len(sites) < 4:
        return []
    try:
        diagram = Voronoi(sites)
    except QhullError:
        return []
    edges = []
    for (a, b), vertices in zip(diagram.ridge_points, diagram.ridge_vertices):
        if len(vertices) != 2 or min(vertices) < 0:
            continue
        # Reject neighbouring samples on the same boundary flank.
        if np.linalg.norm(sites[a] - sites[b]) < width * 0.45:
            continue
        segment = LineString(diagram.vertices[vertices])
        if segment.length > 1e-5 and poly.covers(segment):
            midpoint = segment.interpolate(0.5, normalized=True)
            if midpoint.distance(poly.boundary) >= width * 0.16:
                edges.append(segment)
    if not edges:
        return []
    merged = parts(line_merge(union_all(edges)), "LineString")
    degree = {}
    for line in merged:
        for endpoint in (line.coords[0], line.coords[-1]):
            key = tuple(np.round(endpoint, 6))
            degree[key] = degree.get(key, 0) + 1
    result = []
    for line in merged:
        leaf = any(degree[tuple(np.round(p, 6))] == 1 for p in (line.coords[0], line.coords[-1]))
        if leaf and line.length < width * 0.65:
            continue
        result.append(np.asarray(line.simplify(width * 0.015, preserve_topology=True).coords))
    return clip_paths(result, poly)


class GeometryOptimizer:
    def __init__(self, cell: Cell, cache_dir: Path):
        self.cell, self.cache_dir = cell, cache_dir
        self.memory = OrderedDict()
        self.hits = 0
        self.cache_dir.mkdir(parents=True, exist_ok=True)

    def optimize(self, poly):
        cell = self.cell
        canonical = normalize(set_precision(poly.simplify(1e-6), cell.shape["polygon_snap_tolerance_mm"]))
        key = digest(b"geometry-v4-even-odd-coverage95" + canonical.wkb + json.dumps(cell.raw, sort_keys=True).encode())
        cache_file = self.cache_dir / (key + ".json")
        if key in self.memory:
            self.hits += 1
            self.memory.move_to_end(key)
            return (*self.memory[key], key)
        if cache_file.exists():
            data = json.loads(cache_file.read_text())
            paths = [np.asarray(p) for p in data["paths"]]
            metrics = data["metrics"]
            self.hits += 1
        else:
            paths, metrics = self._search(canonical)
            atomic_json(cache_file, {"paths": [p.tolist() for p in paths], "metrics": metrics})
        self.memory[key] = (paths, metrics)
        if len(self.memory) > 128:
            self.memory.popitem(last=False)
        return paths, metrics, key

    def _search(self, poly):
        cell = self.cell
        skeleton = centerlines(poly, cell.width)
        def candidates():
            yield "centerline", skeleton
            for offset in (0.50, 0.44):
                yield f"contour-{offset}", contours(poly, cell.width, offset)
            for angle in (0, 60, 90):
                yield f"raster-{angle}", raster(poly, cell.width, angle)
        best = None
        for name, raw in candidates():
            raw = [np.asarray(LineString(p).simplify(cell.width * 0.008).coords) for p in raw]
            paths = clip_paths(clean_paths(raw, max(cell.epsilon * 4, 1e-4)), poly)
            if not paths:
                continue
            deposited = footprint(paths, cell)
            # Hybrid repair: add only skeleton portions that fill missing material.
            initial_metrics = evaluate(poly, deposited)
            repair_paths = [] if name == "centerline" or acceptable(initial_metrics, cell) else skeleton
            additions_batch = []
            covered_core = deposited.buffer(-cell.width * 0.42)
            for path in repair_paths:
                residual = LineString(path).difference(covered_core)
                additions = [np.asarray(p.coords) for p in parts(residual, "LineString")
                             if p.length > cell.width * 0.08]
                if not additions:
                    continue
                added = footprint(additions, cell)
                gain = added.intersection(poly).difference(deposited).area
                waste = added.difference(poly).difference(deposited).area
                if gain > max(cell.width ** 2 * 0.015, waste * 2):
                    additions_batch.extend(additions)
            paths.extend(additions_batch)
            paths = clip_paths(paths, poly)
            metrics = evaluate(poly, footprint(paths, cell))
            length = sum(LineString(p).length for p in paths)
            setup_cost = len(paths) * cell.process["deposition_speed_mm_s"] * (
                (cell.process.get("arc_on_time_s") or 0) + (cell.process.get("arc_off_time_s") or 0))
            score = (0 if acceptable(metrics, cell) else 1, -metrics["iou"] if not acceptable(metrics, cell) else length + setup_cost)
            if best is None or score < best[0]:
                best = (score, paths, metrics, name)
            LOG.info("Geometry candidate %s: paths=%d IoU=%.4f length=%.1fmm", name, len(paths), metrics["iou"], length)
            # Stop near the area/width length bound, avoiding repeated expensive Boolean operations.
            if acceptable(metrics, cell) and length < poly.area / cell.width * 1.18:
                break
        if best is None:
            raise ValueError("No printable path for non-empty section")
        _, paths, metrics, name = best
        # Remove redundant whole paths only when the shape gate remains satisfied.
        if acceptable(metrics, cell) and len(paths) <= 48:
            buffers = [footprint([p], cell) for p in paths]
            for index in sorted(range(len(paths)), key=lambda i: LineString(paths[i]).length):
                if paths[index] is None:
                    continue
                trial = [p for i, p in enumerate(paths) if i != index and p is not None]
                if not trial:
                    continue
                trial_metrics = evaluate(poly, union_all([g for i, g in enumerate(buffers)
                                                         if i != index and paths[i] is not None]))
                if acceptable(trial_metrics, cell):
                    paths[index], metrics = None, trial_metrics
            paths = [p for p in paths if p is not None]
        metrics = dict(metrics, strategy=name + "+adaptive-repair",
                       length_mm=sum(LineString(p).length for p in paths))
        metrics["deposition_s"] = metrics["length_mm"] / cell.process["deposition_speed_mm_s"]
        if not acceptable(metrics, cell, margin=0):
            raise ValueError(f"Geometry gate not reached: IoU={metrics['iou']:.5f}, "
                             f"coverage={metrics['coverage']:.5f}, overfill={metrics['overfill']:.5f}")
        return paths, metrics


def slice_polygon(mesh, z, cell):
    segments = trimesh.intersections.mesh_plane(mesh, [0, 0, 1], [0, 0, z])
    if len(segments) == 0:
        return Polygon()
    linework = set_precision(MultiLineString(list(segments[:, :, :2])),
                             cell.shape["polygon_snap_tolerance_mm"])
    loops = parts(line_merge(linework), "LineString")
    if any(not p.is_closed for p in loops):
        raise ValueError(f"Open STL section at z={z:g}; repair the input mesh")
    # A self-touching interior loop must contribute to parity even if
    # polygonize would omit it. Do not infer material from un-noded faces.
    return even_odd_material(loops, cell.shape["polygon_snap_tolerance_mm"])



def generate_layers(stl_path, cell, cache_dir, max_layers=None):
    mesh = trimesh.load_mesh(stl_path, process=True)
    if not isinstance(mesh, trimesh.Trimesh) or not np.isfinite(mesh.vertices).all():
        raise ValueError("STL must contain a finite triangle mesh")
    if not mesh.is_watertight and cell.raw["validation"]["attempt_target_repair"]:
        trimesh.repair.fill_holes(mesh)
    if cell.raw["validation"]["require_watertight_target"] and not mesh.is_watertight:
        raise ValueError("Target STL is not watertight")
    base = cell.process["build_plane_z_mm"]
    if mesh.bounds[0, 2] < base - cell.epsilon:
        raise ValueError("STL extends below build plane")
    height = cell.process["layer_height_mm"]
    count = math.ceil((mesh.bounds[1, 2] - base) / height)
    if max_layers is not None:
        count = min(count, max_layers)
    optimizer = GeometryOptimizer(cell, cache_dir)
    layers = []
    LOG.info("STL: %d faces, bounds=%s, layers=%d", len(mesh.faces), mesh.bounds.tolist(), count)
    for index in range(count):
        z = base + (index + 0.5) * height
        exact_error = None
        try:
            poly = slice_polygon(mesh, z, cell)
        except ValueError as exc:
            exact_error, poly = exc, Polygon()
        if poly.is_empty and mesh.bounds[0, 2] < z < mesh.bounds[1, 2]:
            for offset in (min(1e-4, height * 1e-6), -min(1e-4, height * 1e-6)):
                try:
                    poly = slice_polygon(mesh, z + offset, cell)
                except ValueError:
                    continue
                if not poly.is_empty:
                    break
        if poly.is_empty:
            if exact_error:
                raise exact_error
            continue
        paths, metrics, key = optimizer.optimize(poly)
        layers.append(Layer(index, poly, paths, metrics, key))
        LOG.info("Geometry %d/%d: paths=%d IoU=%.4f D=%.1fs cache_hits=%d",
                 index + 1, count, len(paths), metrics["iou"], metrics["deposition_s"], optimizer.hits)
    if not layers:
        raise ValueError("No non-empty target layers")
    return layers
