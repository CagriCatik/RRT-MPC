"""High-level orchestration utilities for the RRT-MPC pipeline."""
from __future__ import annotations

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
        configure_backend(self.config.viz.backend)

        map_stage = MapStage(self.config.map)
        maps = map_stage.build()

        planning_stage = PlanningStage(self.config.planner)
        planning = planning_stage.plan(maps)

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
                save_path=save_path,
                keep_open=True,
            )
            if save_path is not None:
                LOG.info("Saved planning visualisation to %s", resolve_plot_path(save_path))

        tracker = TrajectoryTracker(self.config.mpc, self.config.viz)
        control = tracker.track(
            planning,
            maps,
            map_resolution=self.config.map.map_resolution,
            visualize=visualize,
            occupancy=maps.occupancy,
            axis=axis,
        )

        if visualize:
            from matplotlib import pyplot as plt

            plt.ioff()

        LOG.info("Pipeline run finished with %d tracked states", len(control.states))
        return PipelineResult(map=maps, planning=planning, control=control)


__all__ = ["PipelineOrchestrator"]
