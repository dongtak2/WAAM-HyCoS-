from __future__ import annotations

import logging

import numpy as np
from shapely import LineString, STRtree
from shapely.ops import substring

from ..core import Task
from ..geometry.optimizer import clean_paths, footprint

LOG = logging.getLogger("hycos")


def build_tasks(layers, cell, weights=None, chunk_length=650.0, cluster_radius=160.0):
    """Weighted Voronoi labels along each path, followed by bounded spatial clustering.

    Boundary changes split existing lines; they never discard or add deposited material.
    """
    weights = np.zeros(3) if weights is None else np.asarray(weights)
    bases = np.array([r["base_xyz_mm"][:2] for r in cell.robots])
    reaches = np.array([r["xy_reach_radius_mm"] for r in cell.robots])
    tasks, by_layer = [], {}
    for layer in layers:
        fragments = []
        for original in layer.paths:
            line = LineString(original)
            cuts = np.linspace(0, line.length, max(2, int(np.ceil(line.length / 20)) + 1))
            last_label, start = None, 0.0
            for a, b in zip(cuts[:-1], cuts[1:]):
                section = np.asarray(substring(line, float(a), float(b)).coords)
                radii = np.linalg.norm(section[:, None, :2] - bases[None, :, :], axis=2)
                cost = radii.mean(axis=0) ** 2 + weights
                cost[(radii > reaches[None, :] - 0.002).any(axis=0)] = np.inf
                if not np.isfinite(cost).any():
                    raise ValueError(f"No robot reaches layer {layer.index}, segment {a:g}..{b:g}")
                label = int(np.argmin(cost))
                if last_label is not None and (label != last_label or b - start > chunk_length):
                    fragments.append((last_label, np.asarray(substring(line, start, float(a)).coords)))
                    start = float(a)
                last_label = label
            if line.length - start > 1e-4:
                fragments.append((last_label, np.asarray(substring(line, start, line.length).coords)))
        buckets = []
        for robot, path in fragments:
            cleaned = clean_paths([path])
            if not cleaned:
                continue
            path = cleaned[0]
            center, length = path.mean(axis=0), LineString(path).length
            best = None
            for i, bucket in enumerate(buckets):
                if bucket[0] == robot and bucket[3] + length <= chunk_length:
                    distance = np.linalg.norm(bucket[2] - center)
                    if distance <= cluster_radius and (best is None or distance < best[0]):
                        best = distance, i
            if best is None:
                buckets.append([robot, [path], center, length])
            else:
                bucket = buckets[best[1]]
                bucket[2] = (bucket[2] * bucket[3] + center * length) / (bucket[3] + length)
                bucket[1].append(path)
                bucket[3] += length
        current = []
        for robot, paths, _, length in buckets:
            all_points = np.vstack(paths)
            eligible = tuple(r for r in range(3) if cell.reachable(r, all_points))
            if not eligible:
                raise ValueError("Clustering produced an unreachable task")
            task = Task(len(tasks), layer.index, tuple(paths), robot, eligible, length,
                        footprint(paths, cell))
            tasks.append(task)
            current.append(task)
        previous = by_layer.get(layer.index - 1, [])
        if previous:
            tree = STRtree([t.footprint for t in previous])
            for task in current:
                support = task.footprint.buffer(cell.width * 0.5)
                ids = tree.query(support, predicate="intersects")
                task.predecessors = {previous[int(i)].id for i in ids}
                if not task.predecessors:
                    # A disconnected new region has no local evidence of support: use a barrier.
                    task.predecessors = {p.id for p in previous}
                    LOG.warning("Task %d has no intersecting lower footprint; layer barrier applied", task.id)
        elif by_layer:
            # A gap in occupied layers cannot justify independent starts.
            for task in current:
                task.predecessors = {p.id for p in by_layer[max(by_layer)]}
        by_layer[layer.index] = current
    LOG.info("Partition: %d tasks, %d local support edges", len(tasks),
             sum(len(t.predecessors) for t in tasks))
    return tasks
