"""Probabilistic RRT planner with Catmull-Rom smoothing utilities."""
from __future__ import annotations

import math
import random
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
class RRTParameters:
    """Tunables for the RRT planner and post-processing."""

    step: float = 2.5
    goal_sample_rate: float = 0.05
    max_iterations: int = 10_000
    goal_tolerance: float = 3.0
    collision_step: float = 0.5
    rng_seed: int = 42
    prune_path: bool = True
    spline_samples: int = 20
    spline_alpha: float = 0.5
    dedupe_tolerance: float = 1e-9


@dataclass
class _TreeNode:
    """Internal RRT tree node with parent index and accumulated cost."""

    x: float
    y: float
    parent: Optional[int]
    cost: float


def inflate_rect(rect: Rect, radius: float, x_range: Tuple[float, float], y_range: Tuple[float, float]) -> Rect:
    """Inflate a rectangle by ``radius`` while clamping to the workspace bounds."""

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
    """Return inflated obstacles clamped to the workspace."""

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
    """Shortcut pruning for collision-free paths."""

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


class RRTPlanner:
    """Plan collision-free paths using a probabilistic RRT with smoothing."""

    def __init__(
        self,
        *,
        obstacles: Sequence[Rect],
        workspace: Tuple[Tuple[float, float], Tuple[float, float]],
        params: RRTParameters,
    ) -> None:
        self.obstacles = list(obstacles)
        self.x_range = tuple(map(float, workspace[0]))
        self.y_range = tuple(map(float, workspace[1]))
        self.params = params
        self.rng = random.Random(params.rng_seed)
        self._nodes: List[_TreeNode] = []

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> PlanResult:
        self._nodes = [
            _TreeNode(float(start[0]), float(start[1]), parent=None, cost=0.0),
        ]
        goal_node = _TreeNode(float(goal[0]), float(goal[1]), parent=None, cost=0.0)

        if self._collides_point(start[0], start[1]):
            raise ValueError("Start is inside an obstacle.")
        if self._collides_point(goal[0], goal[1]):
            raise ValueError("Goal is inside an obstacle.")

        reached_idx: Optional[int] = None
        iterations = 0

        for iterations in range(1, self.params.max_iterations + 1):
            sample = self._sample(goal_node)
            nearest_idx = self._nearest(sample)
            new_node = self._steer(nearest_idx, sample)
            if new_node is None:
                continue

            if self._collides_segment(self._nodes[nearest_idx], new_node):
                continue

            self._nodes.append(new_node)
            new_idx = len(self._nodes) - 1

            if self._distance(new_node, goal_node) <= self.params.goal_tolerance:
                if self._collides_segment(new_node, goal_node):
                    continue
                goal_with_parent = _TreeNode(goal_node.x, goal_node.y, parent=new_idx, cost=new_node.cost + self._distance(new_node, goal_node))
                self._nodes.append(goal_with_parent)
                reached_idx = len(self._nodes) - 1
                LOG.info("Goal reached after %d iterations (nodes=%d)", iterations, len(self._nodes))
                break

        success = reached_idx is not None
        path: List[Tuple[float, float]] = []
        raw_path: List[Tuple[float, float]] = []
        pruned: Optional[np.ndarray] = None
        spline: Optional[np.ndarray] = None

        if not success:
            # Attempt final connection to the closest node.
            nearest_idx = self._nearest(goal_node)
            if not self._collides_segment(self._nodes[nearest_idx], goal_node):
                goal_with_parent = _TreeNode(
                    goal_node.x,
                    goal_node.y,
                    parent=nearest_idx,
                    cost=self._nodes[nearest_idx].cost + self._distance(self._nodes[nearest_idx], goal_node),
                )
                self._nodes.append(goal_with_parent)
                reached_idx = len(self._nodes) - 1
                success = True

        if success and reached_idx is not None:
            raw_path = self._extract_path(reached_idx)
            path = raw_path
            if self.params.prune_path:
                pruned = prune_path(path, self.obstacles, collision_step=self.params.collision_step)
                if len(pruned) >= 2:
                    path = pruned.tolist()
            if len(path) >= 2:
                spline = catmull_rom_spline(
                    path,
                    samples_per_segment=self.params.spline_samples,
                    alpha=self.params.spline_alpha,
                    dedupe_tol=self.params.dedupe_tolerance,
                )
                if len(spline) >= 2:
                    path = spline.tolist()
        else:
            raw_path = []

        plan_nodes = [
            RRTStarNode(node.x, node.y, node.cost, node.parent) for node in self._nodes
        ]
        final_path = [tuple(map(float, p)) for p in path]
        raw_path_tuples = [tuple(map(float, p)) for p in raw_path]
        pruned_path_tuples = (
            None
            if pruned is None
            else [tuple(map(float, p)) for p in pruned]
        )
        smoothed_path_tuples = (
            None
            if spline is None
            else [tuple(map(float, p)) for p in spline]
        )
        result = PlanResult(
            success=success,
            path=final_path,
            nodes=plan_nodes,
            iterations=iterations,
            goal_index=reached_idx,
            raw_path=raw_path_tuples,
            pruned_path=pruned_path_tuples,
            smoothed_path=smoothed_path_tuples,
        )
        if not success:
            LOG.warning("Failed to find a path after %d iterations", iterations)
        return result

    def _sample(self, goal: _TreeNode) -> _TreeNode:
        if self.rng.random() < self.params.goal_sample_rate:
            return goal
        x = self.rng.uniform(self.x_range[0], self.x_range[1])
        y = self.rng.uniform(self.y_range[0], self.y_range[1])
        return _TreeNode(x, y, parent=None, cost=0.0)

    def _nearest(self, sample: _TreeNode) -> int:
        distances = [self._distance(node, sample) for node in self._nodes]
        return int(np.argmin(distances))

    def _steer(self, from_idx: int, to_node: _TreeNode) -> Optional[_TreeNode]:
        from_node = self._nodes[from_idx]
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        if d == 0.0:
            return None
        scale = min(self.params.step, d) / d
        nx = from_node.x + dx * scale
        ny = from_node.y + dy * scale
        cost = from_node.cost + math.hypot(nx - from_node.x, ny - from_node.y)
        return _TreeNode(nx, ny, parent=from_idx, cost=cost)

    def _collides_point(self, x: float, y: float) -> bool:
        return any(point_in_rect(x, y, rect) for rect in self.obstacles)

    def _collides_segment(self, a: _TreeNode, b: _TreeNode) -> bool:
        start = (a.x, a.y)
        end = (b.x, b.y)
        return any(
            segment_collides_aabb(start, end, rect, step=self.params.collision_step)
            for rect in self.obstacles
        )

    @staticmethod
    def _distance(a: _TreeNode, b: _TreeNode) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _extract_path(self, goal_index: int) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        idx: Optional[int] = goal_index
        while idx is not None:
            node = self._nodes[idx]
            pts.append((node.x, node.y))
            idx = node.parent
        pts.reverse()
        return pts


__all__ = [
    "Rect",
    "RRTParameters",
    "RRTPlanner",
    "catmull_rom_spline",
    "inflate_obstacles",
    "inflate_rect",
    "point_in_rect",
    "prune_path",
    "segment_collides_aabb",
]

