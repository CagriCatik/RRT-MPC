"""Structured results for deterministic RRT* planning."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class RRTStarNode:
    """RRT* tree node with parent index."""

    x: float
    y: float
    cost: float
    parent: Optional[int]


@dataclass
class PlanResult:
    """Container summarising an RRT* planning run."""

    success: bool
    path: List[Tuple[float, float]]
    nodes: List[RRTStarNode]
    iterations: int
    goal_index: Optional[int]


__all__ = ["PlanResult", "RRTStarNode"]
