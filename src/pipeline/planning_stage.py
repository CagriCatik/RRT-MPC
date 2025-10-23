"""Planning stage bridging the occupancy grid and motion planner."""
from __future__ import annotations

from dataclasses import dataclass

from ..config import PlannerConfig
from ..logging_setup import get_logger
from ..planning.rrt_star import RRTStarPlanner
from .artifacts import MapArtifacts, PlanningArtifacts

LOG = get_logger(__name__)


@dataclass
class PlanningStage:
    """Execute the configured planner and wrap its output."""

    config: PlannerConfig

    def plan(self, maps: MapArtifacts) -> PlanningArtifacts:
        params = self.config.to_parameters()
        LOG.info(
            "Starting planning stage (max_iterations=%d, step=%.1f, goal_radius=%.1f)",
            params.max_iterations,
            params.step,
            params.goal_radius,
        )
        planner = RRTStarPlanner(maps.occupancy, params)
        result = planner.plan(maps.start, maps.goal)
        LOG.info(
            "Planning finished: success=%s iterations=%d nodes=%d",
            result.success,
            result.iterations,
            len(result.nodes),
        )
        return PlanningArtifacts(plan=result)


__all__ = ["PlanningStage"]
