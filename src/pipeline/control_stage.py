"""MPC-based control stage for executing the planned trajectory."""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Optional, Tuple

import numpy as np

from ..common.types import FloatArray
from ..config import MPCConfig, VizConfig
from ..control.mpc_controller import MPCController, MPCParameters
from ..control.ref_builder import build_reference
from ..control.vehicle_model import f_discrete
from ..logging_setup import get_logger
from ..viz.record import FrameRecorder
from ..viz.vehicle_draw import VehicleParams
from ..viz.visualization import plot_prediction
from .artifacts import MapArtifacts, PlanningArtifacts, TrackingResult

if TYPE_CHECKING:  # pragma: no cover - import only for type checking
    from matplotlib.axes import Axes

LOG = get_logger(__name__)


@dataclass
class TrajectoryTracker:
    """Run MPC closed-loop tracking over the planned path."""

    mpc: MPCConfig
    viz: VizConfig

    def _solve_with_relaxation(
        self,
        state: FloatArray,
        reference: np.ndarray,
        u_prev: FloatArray,
        base_params: MPCParameters,
    ) -> Tuple[Optional[FloatArray], Optional[FloatArray], Optional[FloatArray]]:
        controller = MPCController(base_params)
        u0, Xp, Up = controller.solve(state, reference, u_prev=u_prev)
        if u0 is not None:
            return u0, Xp, Up

        LOG.warning("MPC infeasible; applying rate relaxation and speed reduction")
        relaxed_reference = reference.copy()
        relaxed_reference[:, 3] *= 0.6
        relaxed_params = replace(
            base_params,
            du_bounds=(
                (base_params.du_bounds[0][0] - 5.0, base_params.du_bounds[0][1] + 5.0),
                (base_params.du_bounds[1][0] - 0.05, base_params.du_bounds[1][1] + 0.05),
            ),
        )
        relaxed_controller = MPCController(relaxed_params)
        return relaxed_controller.solve(state, relaxed_reference, u_prev=u_prev)

    def track(
        self,
        planning: PlanningArtifacts,
        maps: MapArtifacts,
        *,
        map_resolution: float,
        visualize: bool = True,
        occupancy: Optional[np.ndarray] = None,
        axis: Optional["Axes"] = None,
    ) -> TrackingResult:
        plan = planning.plan
        if not plan.success:
            raise RuntimeError("Planning stage did not succeed; cannot start control stage")
        if not plan.path:
            raise RuntimeError("Planner returned an empty path")

        base_params = self.mpc.to_parameters(map_resolution)
        horizon = base_params.horizon
        wheelbase_px = base_params.wheelbase_px
        vehicle_params = VehicleParams.from_wheelbase(wheelbase_px)

        path = plan.path
        if len(path) > 1:
            yaw0 = float(np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0]))
        else:
            yaw0 = 0.0
        state = np.array([maps.start[0], maps.start[1], yaw0, 5.0], dtype=float)
        u_prev = np.zeros(2)

        ref_global = build_reference(path, self.mpc.v_px_s, horizon, self.mpc.dt)
        states: list[FloatArray] = []
        recorder = FrameRecorder(self.viz.record_dir) if (visualize and self.viz.record_frames) else None

        path_idx = 0
        for step in range(self.mpc.sim_steps):
            end = min(path_idx + horizon + 1, len(ref_global))
            ref_window = ref_global[path_idx:end]
            if len(ref_window) < horizon + 1:
                tail = np.repeat(ref_window[-1:], horizon + 1 - len(ref_window), axis=0)
                ref_window = np.vstack((ref_window, tail))

            u0, Xp, _ = self._solve_with_relaxation(state, ref_window, u_prev, base_params)
            if u0 is None or Xp is None:
                LOG.error("MPC remained infeasible at step %d; aborting tracking", step)
                break

            if visualize and occupancy is not None:
                plot_prediction(
                    occupancy,
                    path,
                    Xp,
                    state,
                    step,
                    self.viz.prediction_pause,
                    ax=axis,
                    vehicle_params=vehicle_params,
                    control=u0,
                )
                if recorder and axis is not None:
                    recorder.capture(axis.figure)

            state = f_discrete(state, u0, self.mpc.dt, wheelbase_px)
            states.append(state.copy())
            u_prev = u0.copy()

            if path_idx < len(ref_global) - 2:
                dx = state[0] - ref_global[path_idx][0]
                dy = state[1] - ref_global[path_idx][1]
                if dx * dx + dy * dy > 25.0:
                    path_idx += 1

            if np.hypot(state[0] - maps.goal[0], state[1] - maps.goal[1]) < 8.0:
                LOG.info("Reached goal region at step %d", step)
                break

        return TrackingResult(states=states)


__all__ = ["TrajectoryTracker"]
