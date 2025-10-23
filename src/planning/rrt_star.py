"""Deterministic RRT* planner operating on binary occupancy grids."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..logging_setup import get_logger
from .plan_result import PlanResult, RRTStarNode

LOG = get_logger(__name__)


@dataclass
class PlannerParameters:
    step: float
    goal_radius: float
    max_iterations: int
    rewire_radius: float
    goal_sample_rate: float
    random_seed: int


class RRTStarPlanner:
    """Compute kinodynamically-feasible paths on an inflated occupancy grid."""

    def __init__(self, occupancy: np.ndarray, params: PlannerParameters) -> None:
        self.occupancy = occupancy
        self.params = params
        self.rng = np.random.default_rng(params.random_seed)

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> PlanResult:
        nodes: List[RRTStarNode] = [RRTStarNode(start[0], start[1], 0.0, parent=None)]
        goal_index: Optional[int] = None
        iterations = 0
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

            if self._reached_goal(nodes[new_idx], goal):
                goal_probe = RRTStarNode(goal[0], goal[1], 0.0, parent=None)
                if not self._collision_free(nodes[new_idx], goal_probe):
                    continue
                goal_index = len(nodes)
                goal_cost = nodes[new_idx].cost + self._distance(nodes[new_idx], goal_probe)
                nodes.append(RRTStarNode(goal[0], goal[1], goal_cost, parent=new_idx))
                LOG.info("Goal reached at iteration %d with %d nodes", iterations, len(nodes))
                break

        path: List[Tuple[float, float]] = []
        success = goal_index is not None
        if success and goal_index is not None:
            path = self._extract_path(nodes, goal_index)
        else:
            LOG.warning("Failed to find a path within %d iterations", self.params.max_iterations)

        return PlanResult(success=success, path=path, nodes=nodes, iterations=iterations, goal_index=goal_index)

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

    def _collision_free(self, a: RRTStarNode, b: RRTStarNode) -> bool:
        dist = max(1, int(self._distance(a, b)))
        xs = np.linspace(a.x, b.x, dist + 1)
        ys = np.linspace(a.y, b.y, dist + 1)
        for x, y in zip(xs, ys):
            xi = int(np.clip(round(x), 0, self.occupancy.shape[1] - 1))
            yi = int(np.clip(round(y), 0, self.occupancy.shape[0] - 1))
            if self.occupancy[yi, xi] == 0:
                return False
        return True

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


__all__ = ["RRTStarPlanner", "PlannerParameters"]
