"""CLI entry point; never imports or executes the official Validator."""
from __future__ import annotations

import argparse
import hashlib
import logging
import shutil
import sys
import time
from pathlib import Path

from .core import Cell, atomic_json
from .geometry.optimizer import generate_layers
from .optimization.alns import optimize
from .partition.tasks import build_tasks
from .scheduling.planner import MotionPlanner
from .scheduling.scheduler import Scheduler
from .validation.collision import CollisionChecker
from .validation.preflight import check_partition_shape, checkpoint, write_trajectory

ROOT = Path(__file__).resolve().parent
LOG = logging.getLogger("hycos")


def arguments(argv=None):
    parser = argparse.ArgumentParser(description="WAAM-HyCoS independent three-robot solver")
    parser.add_argument("--ids", nargs="+", type=int, choices=range(1, 7), default=list(range(1, 7)))
    parser.add_argument("--mode", choices=("generate", "optimize"), default="generate")
    parser.add_argument("--input-root", type=Path, default=ROOT.parent / "tests")
    parser.add_argument("--output-root", type=Path, default=ROOT / "results")
    parser.add_argument("--time-limit", type=float, default=60.0, help="ALNS search seconds per model, after initial solution")
    parser.add_argument("--iterations", type=int, default=80)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-layers", type=int, help="Partial smoke generation; always saved under smoke/, never the full-job CSV")
    parser.add_argument("--chunk-length", type=float, default=650.0)
    args = parser.parse_args(argv)
    if args.time_limit < 0 or args.iterations < 0 or args.chunk_length <= 20:
        parser.error("Time/iterations must be nonnegative; chunk length must exceed 20 mm")
    if args.max_layers is not None and args.max_layers <= 0:
        parser.error("--max-layers must be positive")
    # All generated files, including custom output roots, stay inside rl_project.
    if not args.output_root.resolve().is_relative_to(ROOT.resolve()):
        parser.error("--output-root must be inside rl_project")
    return args


def copy_exact(source, destination):
    with source.open("rb") as handle:
        expected = hashlib.file_digest(handle, "sha256").hexdigest()
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    shutil.copyfile(source, temporary)
    with temporary.open("rb") as handle:
        if hashlib.file_digest(handle, "sha256").hexdigest() != expected:
            raise OSError(f"Input copy hash mismatch: {source}")
    temporary.replace(destination)
    return expected


def solve_one(model_id, args):
    started = time.monotonic()
    source = args.input_root / f"{model_id:02d}"
    output = args.output_root / f"{model_id:02d}"
    if args.max_layers is not None:
        output = output / "smoke"
    output.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(output / "generation.log", mode="a", encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    LOG.addHandler(handler)
    stage_times = {}
    try:
        LOG.info("Model %02d mode=%s partial=%s", model_id, args.mode, args.max_layers is not None)
        if (output / "trajectory.csv").exists():
            for name in ("config.yaml", "target.stl"):
                if (output / name).exists():
                    with (source / name).open("rb") as a, (output / name).open("rb") as b:
                        if hashlib.file_digest(a, "sha256").digest() != hashlib.file_digest(b, "sha256").digest():
                            raise ValueError("Existing trajectory belongs to different inputs; use a new --output-root")
        hashes = {name: copy_exact(source / name, output / name) for name in ("config.yaml", "target.stl")}
        atomic_json(output / "inputs.json", {"sha256": hashes, "partial": args.max_layers is not None})
        cell = Cell.load(source / "config.yaml")
        stage = time.monotonic()
        layers = generate_layers(source / "target.stl", cell, args.output_root / ".geometry_cache", args.max_layers)
        stage_times["geometry_s"] = time.monotonic() - stage
        atomic_json(output / "geometry.json", [{"layer": l.index, "cache_key": l.cache_key, **l.metrics} for l in layers])
        stage = time.monotonic()
        tasks = build_tasks(layers, cell, chunk_length=args.chunk_length)
        shape = check_partition_shape(tasks, layers, cell)
        atomic_json(output / "shape_estimate.json", shape)
        stage_times["partition_s"] = time.monotonic() - stage
        checker = CollisionChecker(cell)
        planner = MotionPlanner(cell, checker)
        scheduler = Scheduler(cell, tasks, planner, checker)
        stage = time.monotonic()
        # First construct a conservative valid incumbent; refine timing only afterwards.
        initial = scheduler.solve(conservative=True)
        checks = checkpoint(output, initial, scheduler, "initial")
        # Publish the initial best immediately, before any optimization can fail or time out.
        copy_exact(output / "trajectory.initial.csv", output / "trajectory.csv")
        stage_times["initial_schedule_s"] = time.monotonic() - stage
        LOG.info("Initial saved: makespan=%.2fs rows=%d", initial.makespan, checks["rows"])
        best, history = initial, {"history": [], "operators": {}}
        verified_tasks = scheduler.tasks

        def save_best(solution, active_scheduler, label):
            nonlocal verified_tasks
            if active_scheduler.tasks is not verified_tasks:
                check_partition_shape(active_scheduler.tasks, layers, cell)
            checkpoint(output, solution, active_scheduler, label)
            copy_exact(output / f"trajectory.{label}.csv", output / "trajectory.csv")
            atomic_json(output / "best.json", {**solution.summary(), "elapsed_s": time.monotonic() - started})
            verified_tasks = active_scheduler.tasks

        atomic_json(output / "best.json", initial.summary())
        stage = time.monotonic()
        if args.mode == "optimize" and args.time_limit > 0 and args.iterations > 0:
            best, scheduler, history = optimize(scheduler, initial, layers, args.time_limit,
                                                 args.iterations, args.seed, save_best,
                                                 lambda data: atomic_json(output / "optimization.json", data))
        stage_times["optimization_s"] = time.monotonic() - stage
        # Keep an explicitly named final/improved snapshot even when no candidate improves.
        checkpoint(output, best, scheduler, "improved")
        final_checks = write_trajectory(output / "trajectory.csv", best, scheduler)
        shape = check_partition_shape(scheduler.tasks, layers, cell)
        atomic_json(output / "shape_estimate.json", shape)
        atomic_json(output / "optimization.json", history)
        atomic_json(output / "conflict_repairs.json", {"events": scheduler.events,
                    "learned_motion_pairs": len(scheduler.learned), "events_truncated_at": 2000})
        result = {"model": model_id, "mode": args.mode, "partial": args.max_layers is not None,
                  "official_validator_run": False, "initial_makespan_s": initial.makespan,
                  **best.summary(), "runtime_s": time.monotonic() - started,
                  "stages": stage_times, "preflight": final_checks, "input_sha256": hashes}
        atomic_json(output / "summary.json", result)
        LOG.info("Complete %02d: makespan=%.2fs elapsed=%.2fs -> %s", model_id,
                 best.makespan, result["runtime_s"], output / "trajectory.csv")
        return result
    except BaseException as exc:
        atomic_json(output / "last_error.json", {"type": type(exc).__name__, "message": str(exc),
                    "elapsed_s": time.monotonic() - started,
                    "previous_checkpoint_preserved": (output / "trajectory.csv").exists()})
        raise
    finally:
        LOG.removeHandler(handler)
        handler.close()


def main(argv=None):
    args = arguments(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", handlers=[logging.StreamHandler(sys.stdout)])
    results, failed = [], []
    for model_id in dict.fromkeys(args.ids):
        try:
            results.append(solve_one(model_id, args))
        except Exception:
            LOG.exception("Model %02d failed; completed checkpoints retained", model_id)
            failed.append(model_id)
    index = args.output_root / ("batch.smoke.json" if args.max_layers is not None else "batch.json")
    atomic_json(index, {"results": results, "failed_ids": failed, "official_validator_run": False})
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
