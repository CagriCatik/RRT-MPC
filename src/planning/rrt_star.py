"""Deterministic RRT* planner operating on binary occupancy grids."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from ..logging_setup import get_logger
from .plan_result import PlanResult, RRTStarNode

LOG = get_logger(__name__)


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangular obstacle."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float


@dataclass
class PlannerParameters:
    step: float
    goal_radius: float
    max_iterations: int
    rewire_radius: float
    goal_sample_rate: float
    random_seed: int
    prune_path: bool = True
    spline_samples: int = 20
    spline_alpha: float = 0.5
    dedupe_tolerance: float = 1e-9
    collision_step: float = 1.0


def inflate_rect(rect: Rect, radius: float, x_range: Tuple[float, float], y_range: Tuple[float, float]) -> Rect:
    """Inflate a rectangle by ``radius`` clamping the result to ``x_range``/``y_range``."""

    r = max(0.0, float(radius))
    xmin = max(x_range[0], rect.xmin - r)
    ymin = max(y_range[0], rect.ymin - r)
    xmax = min(x_range[1], rect.xmax + r)
    ymax = min(y_range[1], rect.ymax + r)
    return Rect(xmin, ymin, xmax, ymax)


def inflate_obstacles(
    obstacles: Iterable[Rect],
    radius: float,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
) -> List[Rect]:
    """Return inflated rectangles respecting the workspace bounds."""

    return [inflate_rect(obs, radius, x_range, y_range) for obs in obstacles]


def point_in_rect(x: float, y: float, rect: Rect) -> bool:
    return rect.xmin <= x <= rect.xmax and rect.ymin <= y <= rect.ymax


def segment_collides_aabb(
    start: Tuple[float, float],
    end: Tuple[float, float],
    rect: Rect,
    *,
    step: float = 0.5,
) -> bool:
    """Discrete collision test between a segment and an axis-aligned rectangle."""

    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    length = math.hypot(dx, dy)
    if length == 0.0:
        return point_in_rect(ax, ay, rect)
    steps = max(1, int(length / max(step, 1e-9)))
    for i in range(steps + 1):
        t = i / steps
        x = ax + t * dx
        y = ay + t * dy
        if point_in_rect(x, y, rect):
            return True
    return False


def _dedupe_consecutive(points: Sequence[Tuple[float, float]], tol: float = 1e-9) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return pts
    out = [pts[0]]
    for idx in range(1, len(pts)):
        if np.linalg.norm(pts[idx] - out[-1]) > tol:
            out.append(pts[idx])
    return np.asarray(out)


def catmull_rom_spline(
    points: Sequence[Tuple[float, float]],
    *,
    samples_per_segment: int = 20,
    alpha: float = 0.5,
    eps: float = 1e-9,
    dedupe_tol: float = 1e-9,
) -> np.ndarray:
    """Robust centripetal Catmull-Rom spline interpolation for 2D points."""

    pts = _dedupe_consecutive(points, tol=dedupe_tol)
    n = len(pts)
    if n == 0:
        return pts.copy()
    if n == 1:
        return pts.copy()
    if n == 2:
        t = np.linspace(0.0, 1.0, max(2, samples_per_segment + 1))
        return (1 - t)[:, None] * pts[0] + t[:, None] * pts[1]

    extended = np.vstack([pts[0], pts, pts[-1]])

    def tj(ti: float, pi: np.ndarray, pj: np.ndarray) -> float:
        d = np.linalg.norm(pj - pi)
        inc = (d ** alpha) if d > eps else eps
        return ti + inc

    out: List[np.ndarray] = []
    for i in range(0, n - 1):
        p0, p1, p2, p3 = extended[i], extended[i + 1], extended[i + 2], extended[i + 3]

        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)

        dt01 = max(t1 - t0, eps)
        dt12 = max(t2 - t1, eps)
        dt23 = max(t3 - t2, eps)
        dt02 = max(t2 - t0, eps)
        dt13 = max(t3 - t1, eps)

        t_vals = np.linspace(t1, t2, max(2, samples_per_segment + 1), endpoint=False)
        for t in t_vals:
            a1 = (t1 - t) / dt01 * p0 + (t - t0) / dt01 * p1
            a2 = (t2 - t) / dt12 * p1 + (t - t1) / dt12 * p2
            a3 = (t3 - t) / dt23 * p2 + (t - t2) / dt23 * p3

            b1 = (t2 - t) / dt02 * a1 + (t - t0) / dt02 * a2
            b2 = (t3 - t) / dt13 * a2 + (t - t1) / dt13 * a3

            c = (t2 - t) / dt12 * b1 + (t - t1) / dt12 * b2
            out.append(c)

    out.append(pts[-1])
    return np.asarray(out)


def prune_path(
    path: Sequence[Tuple[float, float]],
    obstacles: Sequence[Rect],
    *,
    collision_step: float = 0.5,
) -> np.ndarray:
    """Shortcut pruning for collision-free paths given rectangular obstacles."""

    if path is None:
        return np.asarray([])
    pts = [tuple(p) for p in path]
    if len(pts) <= 2:
        return np.asarray(pts, dtype=float)

    def collides(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        for rect in obstacles:
            if segment_collides_aabb(a, b, rect, step=collision_step):
                return True
        return False

    pruned = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and collides(pts[i], pts[j]):
            j -= 1
        pruned.append(pts[j])
        i = j
    return np.asarray(pruned, dtype=float)


class RRTStarPlanner:
    """Compute kinodynamically-feasible paths on an inflated occupancy grid."""

    def __init__(self, occupancy: np.ndarray, params: PlannerParameters) -> None:
        self.occupancy = occupancy
        self.params = params
        self.rng = np.random.default_rng(params.random_seed)

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> PlanResult:
        LOG.info(
            "Running RRT* planner from %s to %s (max_iterations=%d)",
            start,
            goal,
            self.params.max_iterations,
        )
        nodes: List[RRTStarNode] = [RRTStarNode(start[0], start[1], 0.0, parent=None)]
        goal_index: Optional[int] = None
        iterations = 0
        progress_interval = max(1, self.params.max_iterations // 5)
        for iterations in range(1, self.params.max_iterations + 1):
            sample = self._sample(goal)
            nearest_idx = self._nearest(nodes, sample)
            new_node = self._steer(nodes[nearest_idx], sample)

            if not self._is_within_bounds(new_node):
                continue
            if not self._collision_free(nodes[nearest_idx], new_node):
                continue

            parent_idx, cost = self._choose_parent(nodes, new_node, nearest_idx)
            nodes.append(RRTStarNode(new_node.x, new_node.y, cost, parent=parent_idx))
            new_idx = len(nodes) - 1
            self._rewire(nodes, new_idx)

            if iterations % progress_interval == 0:
                LOG.info(
                    "Iteration %d/%d: nodes=%d, best_cost=%.2f",
                    iterations,
                    self.params.max_iterations,
                    len(nodes),
                    nodes[new_idx].cost,
                )

            if self._reached_goal(nodes[new_idx], goal):
                goal_probe = RRTStarNode(goal[0], goal[1], 0.0, parent=None)
                if not self._collision_free(nodes[new_idx], goal_probe):
                    continue
                goal_index = len(nodes)
                goal_cost = nodes[new_idx].cost + self._distance(nodes[new_idx], goal_probe)
                nodes.append(RRTStarNode(goal[0], goal[1], goal_cost, parent=new_idx))
                LOG.info("Goal reached at iteration %d with %d nodes", iterations, len(nodes))
                break

        success = goal_index is not None
        raw_path: List[Tuple[float, float]] = []
        pruned_path: Optional[List[Tuple[float, float]]] = None
        smoothed_path: Optional[List[Tuple[float, float]]] = None
        final_path: List[Tuple[float, float]] = []

        if success and goal_index is not None:
            raw_path = self._extract_path(nodes, goal_index)
            LOG.info("Extracted path with %d waypoints (cost=%.2f)", len(raw_path), nodes[goal_index].cost)
            working_path: List[Tuple[float, float]] = list(raw_path)

            if self.params.prune_path and len(working_path) >= 2:
                pruned = self._shortcut_prune(working_path)
                if len(pruned) >= 2:
                    pruned_path = pruned
                    working_path = pruned
                    LOG.info("Pruned path to %d waypoints", len(pruned_path))

            if len(working_path) >= 2 and self.params.spline_samples > 1:
                spline = catmull_rom_spline(
                    working_path,
                    samples_per_segment=self.params.spline_samples,
                    alpha=self.params.spline_alpha,
                    dedupe_tol=self.params.dedupe_tolerance,
                )
                if len(spline) >= 2:
                    smoothed_path = [tuple(map(float, pt)) for pt in spline]
                    working_path = smoothed_path
                    LOG.info(
                        "Smoothed path using Catmull-Rom spline (points=%d -> %d)",
                        len(pruned_path or raw_path),
                        len(smoothed_path),
                    )

            final_path = [tuple(map(float, pt)) for pt in working_path]
        else:
            LOG.warning("Failed to find a path within %d iterations", self.params.max_iterations)

        LOG.info("Planning loop completed after %d iterations (success=%s)", iterations, success)
        return PlanResult(
            success=success,
            path=final_path,
            nodes=nodes,
            iterations=iterations,
            goal_index=goal_index,
            raw_path=[tuple(map(float, pt)) for pt in raw_path],
            pruned_path=None if pruned_path is None else [tuple(map(float, pt)) for pt in pruned_path],
            smoothed_path=smoothed_path,
        )

    def _sample(self, goal: Tuple[float, float]) -> RRTStarNode:
        if self.rng.random() < self.params.goal_sample_rate:
            return RRTStarNode(goal[0], goal[1], 0.0, None)
        y = self.rng.integers(0, self.occupancy.shape[0])
        x = self.rng.integers(0, self.occupancy.shape[1])
        return RRTStarNode(float(x), float(y), 0.0, None)

    def _nearest(self, nodes: Sequence[RRTStarNode], rnd: RRTStarNode) -> int:
        distances = [self._distance(node, rnd) for node in nodes]
        return int(np.argmin(distances))

    def _steer(self, from_node: RRTStarNode, to_node: RRTStarNode) -> RRTStarNode:
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.params.step * math.cos(theta)
        new_y = from_node.y + self.params.step * math.sin(theta)
        return RRTStarNode(new_x, new_y, 0.0, parent=None)

    def _is_within_bounds(self, node: RRTStarNode) -> bool:
        return 0 <= node.x < self.occupancy.shape[1] and 0 <= node.y < self.occupancy.shape[0]

    def _segment_is_free(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        step = max(self.params.collision_step, 1e-3)
        samples = max(1, int(math.ceil(distance / step)))
        xs = np.linspace(start[0], end[0], samples + 1)
        ys = np.linspace(start[1], end[1], samples + 1)
        for x, y in zip(xs, ys):
            xi = int(np.clip(round(x), 0, self.occupancy.shape[1] - 1))
            yi = int(np.clip(round(y), 0, self.occupancy.shape[0] - 1))
            if self.occupancy[yi, xi] == 0:
                return False
        return True

    def _collision_free(self, a: RRTStarNode, b: RRTStarNode) -> bool:
        return self._segment_is_free((a.x, a.y), (b.x, b.y))

    def _choose_parent(self, nodes: List[RRTStarNode], new_node: RRTStarNode, nearest_idx: int) -> Tuple[int, float]:
        best_idx = nearest_idx
        best_cost = nodes[nearest_idx].cost + self._distance(nodes[nearest_idx], new_node)
        for idx, node in enumerate(nodes):
            if self._distance(node, new_node) > self.params.rewire_radius:
                continue
            if not self._collision_free(node, new_node):
                continue
            cost = node.cost + self._distance(node, new_node)
            if cost < best_cost:
                best_idx = idx
                best_cost = cost
        return best_idx, best_cost

    def _rewire(self, nodes: List[RRTStarNode], new_idx: int) -> None:
        new_node = nodes[new_idx]
        for idx, node in enumerate(nodes[:-1]):
            if self._distance(node, new_node) > self.params.rewire_radius:
                continue
            if node.parent is None:
                continue
            cost = new_node.cost + self._distance(node, new_node)
            if cost < node.cost and self._collision_free(new_node, node):
                nodes[idx] = RRTStarNode(node.x, node.y, cost, parent=new_idx)

    def _reached_goal(self, node: RRTStarNode, goal: Tuple[float, float]) -> bool:
        return math.hypot(node.x - goal[0], node.y - goal[1]) < self.params.goal_radius

    def _distance(self, a: RRTStarNode, b: RRTStarNode) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _extract_path(self, nodes: Sequence[RRTStarNode], goal_index: int) -> List[Tuple[float, float]]:
        path: List[Tuple[float, float]] = []
        idx: Optional[int] = goal_index
        while idx is not None:
            node = nodes[idx]
            path.append((node.x, node.y))
            idx = node.parent
        path.reverse()
        return path

    def _shortcut_prune(self, path: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if len(path) <= 2:
            return list(path)

        pts = [tuple(map(float, pt)) for pt in path]
        pruned: List[Tuple[float, float]] = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._segment_is_free(pts[i], pts[j]):
                j -= 1
            pruned.append(pts[j])
            i = j
        return pruned


__all__ = [
    "RRTStarPlanner",
    "PlannerParameters",
    "Rect",
    "inflate_rect",
    "inflate_obstacles",
    "catmull_rom_spline",
    "prune_path",
    "segment_collides_aabb",
]
