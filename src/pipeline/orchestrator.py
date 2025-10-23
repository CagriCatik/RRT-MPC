"""High-level orchestration utilities for the RRT-MPC pipeline."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from ..common.paths import resolve_plot_path
from ..config import PipelineConfig
from ..logging_setup import get_logger
from ..viz.visualization import configure_backend, plot_rrt_star
from .artifacts import PipelineResult
from .control_stage import TrajectoryTracker
from .map_stage import MapStage
from .planning_stage import PlanningStage

if TYPE_CHECKING:  # pragma: no cover - import only for typing
    from matplotlib.axes import Axes

LOG = get_logger(__name__)


@dataclass
class PipelineOrchestrator:
    """Execute the configured map, planning, and control stages."""

    config: PipelineConfig

    def run(self, *, visualize: bool = True) -> PipelineResult:
        LOG.info("Starting pipeline run with visualization=%s", visualize)
        run_start = time.perf_counter()
        configure_backend(self.config.viz.backend)

        map_stage = MapStage(self.config.map)
        stage_start = time.perf_counter()
        maps = map_stage.build()
        LOG.info(
            "Map stage completed in %.2fs (grid=%sx%s)",
            time.perf_counter() - stage_start,
            maps.occupancy.shape[1],
            maps.occupancy.shape[0],
        )

        planning_stage = PlanningStage(self.config.planner)
        stage_start = time.perf_counter()
        planning = planning_stage.plan(maps)
        LOG.info(
            "Planning stage completed in %.2fs (success=%s, path_points=%d)",
            time.perf_counter() - stage_start,
            planning.plan.success,
            len(planning.plan.path),
        )

        axis: Optional["Axes"] = None
        if visualize:
            from matplotlib import pyplot as plt

            plt.ion()
            save_path = "planner/rrtstar_path.png" if self.config.viz.animate_tree else None
            axis = plot_rrt_star(
                maps.occupancy,
                maps.start,
                maps.goal,
                planning.plan,
                show_tree=self.config.viz.animate_tree,
                inflation_mask=maps.inflation_mask,
                save_path=save_path,
                keep_open=True,
            )
            if save_path is not None:
                LOG.info("Saved planning visualisation to %s", resolve_plot_path(save_path))

        tracker = TrajectoryTracker(self.config.mpc, self.config.viz)
        stage_start = time.perf_counter()
        control = tracker.track(
            planning,
            maps,
            map_resolution=self.config.map.map_resolution,
            visualize=visualize,
            occupancy=maps.occupancy,
            axis=axis,
        )
        LOG.info(
            "Control stage completed in %.2fs (tracked_states=%d)",
            time.perf_counter() - stage_start,
            len(control.states),
        )

        if visualize:
            from matplotlib import pyplot as plt

            plt.ioff()

        LOG.info(
            "Pipeline run finished in %.2fs with %d tracked states",
            time.perf_counter() - run_start,
            len(control.states),
        )
        return PipelineResult(map=maps, planning=planning, control=control)


__all__ = ["PipelineOrchestrator"]
