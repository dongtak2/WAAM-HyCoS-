from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass

import numpy as np
from shapely import LineString, MultiPoint, Point

from ..core import digest


@dataclass
class Conflict:
    start: float
    end: float
    robot_a: int
    robot_b: int
    kind: str


def point_segment_distance(p, a, b):
    v = b - a
    denominator = float(v @ v)
    t = float(np.clip((p - a) @ v / denominator, 0, 1)) if denominator > 1e-24 else 0.0
    return float(np.linalg.norm(p - (a + t * v)))


def segment_distance(a, b, c, d):
    u, v, w = b - a, d - c, c - a
    cross = lambda x, y: x[0] * y[1] - x[1] * y[0]
    den = cross(u, v)
    if abs(den) > 1e-12:
        s, t = cross(w, v) / den, cross(w, u) / den
        if 0 <= s <= 1 and 0 <= t <= 1:
            return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


class CollisionChecker:
    def __init__(self, cell, cache_size=30000):
        self.cell, self.cache_size = cell, cache_size
        self.bases = [np.asarray(r["base_xyz_mm"][:2]) for r in cell.robots]
        self.cache = OrderedDict()
        self.hull_cache = OrderedDict()
        self.park_cache = OrderedDict()
        self.calls = self.cache_hits = 0
        # Covers float32 interpolation error as well as touching/epsilon semantics.
        self.margin = max(0.005, cell.epsilon * 10)

    def thresholds(self, a, b):
        robots, config = self.cell.robots, self.cell.raw["collision"]
        arm = (robots[a]["arm_envelope_radius_mm"] + robots[b]["arm_envelope_radius_mm"]
               + config["arm_clearance_mm"] + self.margin) if config["check_arm_envelope"] else -1
        tcp = (robots[a]["tcp_radius_mm"] + robots[b]["tcp_radius_mm"]
               + self.margin) if config["check_tcp_radius"] else -1
        return arm, tcp

    def hull_conflict(self, a, b):
        """Lazy conflict graph on robot/motion alternatives, with bounded edge cache."""
        key = (a.spatial_key or a.key, b.spatial_key or b.key)
        if key in self.hull_cache:
            return self.hull_cache[key]
        arm, tcp = self.thresholds(a.robot, b.robot)
        conflict = ((arm >= 0 and a.arm_hull.distance(b.arm_hull) <= arm)
                    or (tcp >= 0 and a.tcp_hull.distance(b.tcp_hull) <= tcp))
        self.hull_cache[key] = conflict
        if len(self.hull_cache) > self.cache_size:
            self.hull_cache.popitem(last=False)
        return conflict

    def linear_conflict(self, a, b, a0, a1, b0, b1, duration):
        """Certify an entire pair of linear intervals, not discrete time samples.

        TCP distance is minimized analytically. Arm distance uses a Lipschitz
        lower bound under moving endpoints; unresolved near contacts are rejected.
        """
        arm, tcp = self.thresholds(a, b)
        a0, a1, b0, b1 = (np.asarray(p[:2]) for p in (a0, a1, b0, b1))
        if tcp >= 0:
            delta, velocity = a0 - b0, (a1 - a0) - (b1 - b0)
            denominator = float(velocity @ velocity)
            fraction = np.clip(-float(delta @ velocity) / denominator, 0, 1) if denominator > 1e-24 else 0
            if np.linalg.norm(delta + fraction * velocity) <= tcp:
                return "tcp"
        if arm < 0:
            return None
        base_a, base_b = self.bases[a], self.bases[b]
        # Swept base-tip triangles enclose every capsule centerline in the interval.
        hull_a = MultiPoint([base_a, a0, a1]).convex_hull
        hull_b = MultiPoint([base_b, b0, b1]).convex_hull
        if hull_a.distance(hull_b) > arm:
            return None
        displacement = float(np.linalg.norm(a1 - a0) + np.linalg.norm(b1 - b0))
        stack = [(0.0, 1.0, 0)]
        while stack:
            lo, hi, depth = stack.pop()
            mid = (lo + hi) * 0.5
            pa, pb = a0 + mid * (a1 - a0), b0 + mid * (b1 - b0)
            distance = segment_distance(base_a, pa, base_b, pb)
            if distance <= arm:
                return "arm"
            if distance - displacement * (hi - lo) * 0.5 > arm:
                continue
            if depth >= 22 or (hi - lo) * duration < 1e-7:
                return "arm_uncertain"
            stack.extend([(lo, mid, depth + 1), (mid, hi, depth + 1)])
        return None

    def motions(self, a, start_a, b, start_b):
        self.calls += 1
        lo = max(start_a, start_b)
        hi = min(start_a + a.duration, start_b + b.duration)
        if hi < lo or not self.hull_conflict(a, b):
            return None
        # Relative offset must be exact: identical geometry does not imply identical timing.
        key = (a.key, b.key, float(start_b - start_a).hex())
        if key in self.cache:
            self.cache_hits += 1
            cached = self.cache[key]
            if cached is None:
                return None
            return Conflict(cached[0] + start_a, cached[1] + start_a, a.robot, b.robot, cached[2])
        ia = max(0, int(np.searchsorted(a.times, lo - start_a, side="right")) - 1)
        ib = max(0, int(np.searchsorted(b.times, lo - start_b, side="right")) - 1)
        result = None
        t = lo
        while t < hi - 1e-10:
            end = min(a.times[ia + 1] + start_a, b.times[ib + 1] + start_b, hi)
            def at(motion, start, index, when):
                alpha = np.clip((when - start - motion.times[index]) /
                                (motion.times[index + 1] - motion.times[index]), 0, 1)
                return motion.xyz[index] + alpha * (motion.xyz[index + 1] - motion.xyz[index])
            kind = self.linear_conflict(a.robot, b.robot, at(a, start_a, ia, t), at(a, start_a, ia, end),
                                        at(b, start_b, ib, t), at(b, start_b, ib, end), end - t)
            if kind:
                result = Conflict(t, end, a.robot, b.robot, kind)
                break
            t = end
            if ia + 1 < len(a.times) - 1 and t >= a.times[ia + 1] + start_a - 1e-10:
                ia += 1
            if ib + 1 < len(b.times) - 1 and t >= b.times[ib + 1] + start_b - 1e-10:
                ib += 1
        self.cache[key] = None if result is None else (result.start - start_a, result.end - start_a, result.kind)
        if len(self.cache) > self.cache_size:
            self.cache.popitem(last=False)
        return result

    def safe_against_parks(self, robot, xyz, parks):
        """A swept triangle is an exact spatial union for one moving base-tip arm."""
        key = (robot, digest(np.asarray(xyz)[:, :2].tobytes() + np.asarray(parks)[:, :2].tobytes()))
        if key in self.park_cache:
            return self.park_cache[key]
        safe = self._park_check(robot, xyz, parks)
        self.park_cache[key] = safe
        if len(self.park_cache) > self.cache_size:
            self.park_cache.popitem(last=False)
        return safe

    def _park_check(self, robot, xyz, parks):
        for other in range(3):
            if other == robot:
                continue
            arm, tcp = self.thresholds(robot, other)
            stationary_arm = LineString([self.bases[other], parks[other][:2]])
            stationary_tcp = Point(parks[other][:2])
            for a, b in zip(xyz[:-1], xyz[1:]):
                if arm >= 0 and MultiPoint([self.bases[robot], a[:2], b[:2]]).convex_hull.distance(stationary_arm) <= arm:
                    return False
                if tcp >= 0 and LineString([a[:2], b[:2]]).distance(stationary_tcp) <= tcp:
                    return False
        return True
