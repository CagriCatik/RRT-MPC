"""Structured results for deterministic RRT* planning."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple


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
    raw_path: Sequence[Tuple[float, float]] = field(default_factory=list)
    pruned_path: Optional[Sequence[Tuple[float, float]]] = None
    smoothed_path: Optional[Sequence[Tuple[float, float]]] = None


__all__ = ["PlanResult", "RRTStarNode"]
