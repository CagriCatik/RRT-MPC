"""Planning stage bridging the occupancy grid and motion planner."""
from __future__ import annotations

from dataclasses import dataclass

from ..config import PlannerConfig
from ..logging_setup import get_logger
from ..planning.plan_result import PlanResult
from ..planning.rrt import RRTPlanner, catmull_rom_spline
from ..planning.rrt_star import RRTStarPlanner
from .artifacts import MapArtifacts, PlanningArtifacts

LOG = get_logger(__name__)


@dataclass
class PlanningStage:
    """Execute the configured planner and wrap its output."""

    config: PlannerConfig

    def plan(self, maps: MapArtifacts) -> PlanningArtifacts:
        algorithm = self.config.algorithm.lower()
        if algorithm == "rrt":
            params = self.config.to_rrt_parameters()
            LOG.info(
                "Starting RRT planner (max_iterations=%d, step=%.1f, goal_tol=%.1f)",
                params.max_iterations,
                params.step,
                params.goal_tolerance,
            )
            planner = RRTPlanner(obstacles=maps.rect_obstacles, workspace=maps.workspace, params=params)
            result = planner.plan(maps.start, maps.goal)
        else:
            params = self.config.to_parameters()
            LOG.info(
                "Starting RRT* planner (max_iterations=%d, step=%.1f, goal_radius=%.1f)",
                params.max_iterations,
                params.step,
                params.goal_radius,
            )
            planner = RRTStarPlanner(maps.occupancy, params)
            result = planner.plan(maps.start, maps.goal)
        self._smooth_path_if_configured(result)
        LOG.info(
            "Planning finished: success=%s iterations=%d nodes=%d",
            result.success,
            result.iterations,
            len(result.nodes),
        )
        return PlanningArtifacts(plan=result)

    def _smooth_path_if_configured(self, result: PlanResult) -> None:
        """Apply Catmull-Rom smoothing unless the planner already did so."""

        if not result.success:
            return
        if result.smoothed_path:
            return
        if not result.path or len(result.path) < 2:
            return

        samples = max(1, int(self.config.rrt_spline_samples))
        alpha = float(self.config.rrt_spline_alpha)
        tol = float(self.config.rrt_dedupe_tolerance)

        raw_path = [tuple(map(float, pt)) for pt in result.path]
        if not result.raw_path:
            result.raw_path = list(raw_path)

        smoothed = catmull_rom_spline(
            raw_path,
            samples_per_segment=samples,
            alpha=alpha,
            dedupe_tol=tol,
        )
        if len(smoothed) == 0:
            return

        smoothed_path = [tuple(map(float, pt)) for pt in smoothed]
        LOG.info(
            "Smoothed planned path using Catmull-Rom spline (points=%d -> %d)",
            len(raw_path),
            len(smoothed_path),
        )
        result.smoothed_path = smoothed_path
        result.path = list(smoothed_path)


__all__ = ["PlanningStage"]
