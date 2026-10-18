"""Inspect actual CSV bead geometry without running the official Validator."""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely import LineString, MultiLineString, Polygon, line_merge, set_precision
from shapely.geometry.polygon import orient

from ..core import Cell, atomic_json
from .optimizer import evaluate, footprint, parts

ROOT = Path(__file__).resolve().parents[1]


def reference_section(mesh, z, cell):
    """Independent audit: apply even-odd XOR directly to the raw closed loops."""
    segments = trimesh.intersections.mesh_plane(mesh, [0, 0, 1], [0, 0, z])
    if len(segments) == 0:
        return Polygon(), []
    linework = set_precision(MultiLineString(list(segments[:, :, :2])),
                             cell.shape["polygon_snap_tolerance_mm"])
    loops = parts(line_merge(linework), "LineString")
    if any(not loop.is_closed for loop in loops):
        raise ValueError("Audit section has an open boundary")
    rings = sorted([Polygon(loop.coords) for loop in loops if len(loop.coords) >= 4],
                   key=lambda p: (-p.area, p.bounds))
    material = Polygon()
    for ring in rings:
        material = material.symmetric_difference(ring)
    return set_precision(material, cell.shape["polygon_snap_tolerance_mm"]), rings


def read_deposition(csv_path, cell):
    layers, previous = {}, None
    with Path(csv_path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            robot = int(row["robot_id"])
            xyz = np.asarray([float(row[k]) for k in ("x_mm", "y_mm", "z_mm")],
                             dtype=np.float32).astype(np.float64)
            if previous and previous[0] == robot and previous[2] == "D":
                start = previous[1]
                z = (start[2] + xyz[2]) / 2
                offset = 1 if cell.process["tcp_z_reference"] == "top" else 0.5
                layer = round((z - cell.process["build_plane_z_mm"]) /
                              cell.process["layer_height_mm"] - offset)
                layers.setdefault(layer, []).append(np.array([start[:2], xyz[:2]]))
            previous = robot, xyz, row["mode"]
    return layers


def audit_csv(csv_path, mesh, cell):
    paths = read_deposition(csv_path, cell)
    count = math.ceil((mesh.bounds[1, 2] - cell.process["build_plane_z_mm"]) /
                      cell.process["layer_height_mm"])
    records, geometries = [], {}
    for index in sorted(set(range(count)) | set(paths)):
        z = cell.process["build_plane_z_mm"] + (index + 0.5) * cell.process["layer_height_mm"]
        target, rings = reference_section(mesh, z, cell)
        layer_paths = paths.get(index, [])
        deposited = footprint(layer_paths, cell)
        over, under = deposited.difference(target), target.difference(deposited)
        metrics = evaluate(target, deposited)
        crossing = sum(LineString(path).difference(target.buffer(2e-4)).length for path in layer_paths)
        record = dict(metrics, layer=index, z_slice_mm=z, overfill_area_mm2=over.area,
                      underfill_area_mm2=under.area, outside_D_centerline_mm=crossing,
                      target_holes=sum(len(p.interiors) for p in parts(target)),
                      raw_loop_count=len(rings),
                      invalid_loops=[{"area_mm2": r.area, "bounds": list(r.bounds),
                                      "deposited_area_mm2": deposited.intersection(r).area}
                                     for r in rings if not r.is_valid],
                      largest_overfill=[{"area_mm2": p.area, "bounds": list(p.bounds)}
                                        for p in sorted(parts(over), key=lambda p: -p.area)[:5]])
        records.append(record)
        geometries[index] = target, deposited, over, under, layer_paths
    total = sum(r["target_area"] for r in records)
    intersection = sum(r["intersection_area"] for r in records)
    deposited = sum(r["deposited_area"] for r in records)
    summary = {"coverage": intersection / total, "overfill": (deposited - intersection) / total,
               "iou": intersection / (total + deposited - intersection),
               "minimum_layer_iou": min(r["iou"] for r in records),
               "minimum_layer_coverage": min(r["coverage"] for r in records),
               "failed_layers": sum(r["iou"] < cell.shape["minimum_layer_iou"] for r in records),
               "layer_count": len(records),
               "outside_D_centerline_mm": sum(r["outside_D_centerline_mm"] for r in records)}
    return {"summary": summary, "layers": records}, geometries


def plot_comparison(destination, audits, layer=0):
    import os
    os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "results" / ".mplconfig"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.path import Path as PlotPath
    from matplotlib.patches import PathPatch

    def paint(axis, geometry, color, alpha=1):
        for polygon in parts(geometry):
            polygon = orient(polygon, sign=1)
            vertices, codes = [], []
            for ring in [polygon.exterior, *polygon.interiors]:
                xy = np.asarray(ring.coords)
                vertices.extend(xy)
                codes.extend([PlotPath.MOVETO] + [PlotPath.LINETO] * (len(xy) - 2) + [PlotPath.CLOSEPOLY])
            axis.add_patch(PathPatch(PlotPath(vertices, codes), facecolor=color,
                                     edgecolor="#526170", linewidth=0.25, alpha=alpha))

    figure = plt.figure(figsize=(18, 5.0 * len(audits)))
    grid = figure.add_gridspec(2 * len(audits), 4, height_ratios=[0.13, 1] * len(audits),
                               left=0.045, right=0.985, top=0.92, bottom=0.065,
                               wspace=0.27, hspace=0.20)
    axes = [[figure.add_subplot(grid[2 * r + 1, c]) for c in range(4)] for r in range(len(audits))]
    colors = ("#bdc8d2", "#428bca", "#d84343", "#f3a024")
    for row, (label, report, geometry) in enumerate(audits):
        target, deposited, over, under, paths = geometry[layer]
        metrics = next(r for r in report["layers"] if r["layer"] == layer)
        titles = (f"Target: {target.area:,.1f} mm^2", f"Deposited: {deposited.area:,.1f} mm^2",
                  f"Overfill: {over.area:,.1f} mm^2", f"Underfill: {under.area:,.1f} mm^2")
        for column, (shape, color, title) in enumerate(zip((target, deposited, over, under), colors, titles)):
            axis = axes[row][column]
            if column > 1:
                paint(axis, target, "#edf0f3")
            paint(axis, shape, color)
            if column == 1:
                for path in paths:
                    axis.plot(path[:, 0], path[:, 1], color="#124f79", linewidth=0.25)
            xmin, ymin, xmax, ymax = target.bounds
            axis.set(xlim=(xmin - 12, xmax + 12), ylim=(ymin - 12, ymax + 12),
                     xlabel="X [mm]", ylabel="Y [mm]", title=title, aspect="equal")
            axis.grid(alpha=0.15)
        header = figure.add_subplot(grid[2 * row, :])
        header.set_axis_off()
        header.text(0, 0.25,
            f"{label} | layer {layer} | Coverage {metrics['coverage']:.2%} | "
            f"Overfill {metrics['overfill']:.2%} | IoU {metrics['iou']:.2%}",
            transform=header.transAxes, fontsize=12, fontweight="bold")
    figure.suptitle("WAAM-HyCoS: actual CSV deposition, 8 mm bead / 4 mm radius", fontsize=15)
    figure.savefig(destination, dpi=170)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--job", type=Path, default=ROOT / "results" / "03")
    parser.add_argument("--before", type=Path)
    parser.add_argument("--layer", type=int, default=0)
    args = parser.parse_args()
    if not args.job.resolve().is_relative_to(ROOT):
        parser.error("--job must be inside rl_project")
    cell = Cell.load(args.job / "config.yaml")
    mesh = trimesh.load_mesh(args.job / "target.stl", process=True)
    audits, report = [], {"official_validator_run": False}
    if args.before:
        before, geometry = audit_csv(args.before, mesh, cell)
        report["before"] = before
        audits.append(("Before fix", before, geometry))
    after, geometry = audit_csv(args.job / "trajectory.csv", mesh, cell)
    report["after"] = after
    audits.append(("After fix", after, geometry))
    atomic_json(args.job / "shape_debug.json", report)
    plot_comparison(args.job / "shape_debug.png", audits, args.layer)
    print(report["after"]["summary"])


if __name__ == "__main__":
    main()
