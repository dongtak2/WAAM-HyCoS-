from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


def quantize(points):
    """Use the validator's stored coordinate precision before computing times."""
    return np.asarray(points, dtype=np.float32).astype(np.float64)


def digest(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def atomic_json(path: Path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


@dataclass
class Cell:
    raw: dict

    @classmethod
    def load(cls, path):
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if sorted(r["id"] for r in data["robots"]) != [1, 2, 3]:
            raise ValueError("Exactly robot IDs 1, 2, 3 are required")
        data["robots"].sort(key=lambda r: r["id"])
        cell = cls(data)
        for key in ("bead_width_mm", "layer_height_mm", "deposition_speed_mm_s", "travel_speed_mm_s"):
            if not np.isfinite(cell.process[key]) or cell.process[key] <= 0:
                raise ValueError(f"Invalid process.{key}")
        return cell

    @property
    def process(self):
        return self.raw["process"]

    @property
    def shape(self):
        return self.raw["shape_validation"]

    @property
    def robots(self):
        return self.raw["robots"]

    @property
    def width(self):
        return self.process["bead_width_mm"]

    @property
    def epsilon(self):
        return self.raw["collision"]["geometry_epsilon_mm"]

    def z(self, layer):
        offset = 1.0 if self.process["tcp_z_reference"] == "top" else 0.5
        return self.process["build_plane_z_mm"] + (layer + offset) * self.process["layer_height_mm"]

    def reachable(self, robot, points):
        spec = self.robots[robot]
        d = np.asarray(points)[..., :2] - np.asarray(spec["base_xyz_mm"][:2])
        return bool(np.all(np.linalg.norm(d, axis=-1) <= spec["xy_reach_radius_mm"] - 0.002))


@dataclass
class Layer:
    index: int
    polygon: object
    paths: list[np.ndarray]
    metrics: dict
    cache_key: str


@dataclass
class Task:
    id: int
    layer: int
    paths: tuple[np.ndarray, ...]
    preferred: int
    eligible: tuple[int, ...]
    length: float
    footprint: object
    predecessors: set[int] = field(default_factory=set)

    @property
    def center(self):
        p = self.footprint.centroid
        return np.array([p.x, p.y])


@dataclass
class Motion:
    """Relative piecewise-linear trip, beginning and ending at its robot's park."""
    robot: int
    times: np.ndarray
    xyz: np.ndarray
    modes: tuple[str, ...]
    key: str
    arm_hull: object
    tcp_hull: object
    spatial_key: str = ""

    @property
    def duration(self):
        return float(self.times[-1])


@dataclass
class Reservation:
    task_id: int
    start: float
    motion: Motion

    @property
    def end(self):
        return self.start + self.motion.duration


@dataclass
class Solution:
    reservations: list[Reservation]
    order: list[int]
    assignments: dict[int, int]
    orientations: dict[int, int]
    makespan: float
    checks: dict = field(default_factory=dict)

    def summary(self):
        return {"makespan_s": self.makespan, "tasks": len(self.order), "checks": self.checks,
                "robot_busy_s": [sum(r.motion.duration for r in self.reservations
                                      if r.motion.robot == robot) for robot in range(3)]}
