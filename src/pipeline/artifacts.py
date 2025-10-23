"""Shared dataclasses representing intermediate pipeline artifacts."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence, Tuple

import numpy as np

from ..common.types import FloatArray
from ..planning.plan_result import PlanResult


@dataclass(frozen=True)
class MapArtifacts:
    """Occupancy-grid assets derived from the configured map."""

    occupancy: np.ndarray
    start: Tuple[float, float]
    goal: Tuple[float, float]


@dataclass(frozen=True)
class PlanningArtifacts:
    """Output from the planning stage."""

    plan: PlanResult


@dataclass
class TrackingResult:
    """Trajectory roll-out information returned by the control stage."""

    states: Sequence[FloatArray] = field(default_factory=list)


@dataclass(frozen=True)
class PipelineResult:
    """Aggregated output from the end-to-end pipeline."""

    map: MapArtifacts
    planning: PlanningArtifacts
    control: TrackingResult

    @property
    def plan(self) -> PlanResult:
        return self.planning.plan

    @property
    def states(self) -> Sequence[FloatArray]:
        return self.control.states


__all__ = [
    "MapArtifacts",
    "PlanningArtifacts",
    "TrackingResult",
    "PipelineResult",
]
