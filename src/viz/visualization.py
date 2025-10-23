"""Visualization helpers for planning and control results."""
from __future__ import annotations

from pathlib import Path as FilePath
from typing import Dict, List, Optional, Sequence

import matplotlib
from matplotlib import colors as mcolors, rcsetup
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Patch
import matplotlib.pyplot as plt
import numpy as np

from ..common.types import FloatArray, Path
from ..common.paths import resolve_plot_path
from ..logging_setup import get_logger
from ..planning.plan_result import PlanResult
from .vehicle_draw import VehicleParams, draw_vehicle

LOG = get_logger(__name__)

_INTERACTIVE_BACKENDS = {name.lower() for name in rcsetup.interactive_bk}


def _backend_supports_interaction() -> bool:
    backend = matplotlib.get_backend().lower()
    if backend.startswith("module://"):
        # Only ipympl provides interactive support among module backends.
        return backend.startswith("module://ipympl")
    return backend in _INTERACTIVE_BACKENDS


def _warn_once(message: str) -> None:
    if getattr(_warn_once, "_emitted", False):
        return
    LOG.info(message)
    _warn_once._emitted = True


def configure_backend(backend: str) -> None:
    requested = (backend or "").strip().lower()
    if requested == "auto":
        if _backend_supports_interaction():
            return
        original = matplotlib.get_backend()
        for candidate in rcsetup.interactive_bk:
            matplotlib.use(candidate, force=True)
            if _backend_supports_interaction():
                LOG.info(
                    "Using Matplotlib backend '%s' for interactive display.",
                    matplotlib.get_backend(),
                )
                return
        matplotlib.use(original, force=True)
        if not _backend_supports_interaction():
            LOG.info(
                "No interactive Matplotlib backend detected; continuing with '%s'.",
                matplotlib.get_backend(),
            )
        return
    matplotlib.use(backend, force=True)
    LOG.info("Configured Matplotlib backend '%s'.", matplotlib.get_backend())


def _prepare_axis(ax: Optional[Axes]) -> tuple[Figure, Axes, bool]:
    if ax is None:
        fig, axis = plt.subplots(figsize=(7, 7))
        created = True
    else:
        fig = ax.figure
        axis = ax
        axis.clear()
        created = False
    axis.set_aspect("equal", adjustable="box")
    return fig, axis, created


def _update_legend(ax: Axes, *, extra_handles: Sequence[object] = ()) -> None:
    stored: Dict[str, object] = dict(getattr(ax, "_rrt_extra_legend", {}))  # type: ignore[attr-defined]
    for handle in extra_handles:
        label = getattr(handle, "get_label", lambda: "")()
        if not label or label.startswith("_"):
            continue
        stored[label] = handle

    handles, labels = ax.get_legend_handles_labels()
    unique: Dict[str, object] = {}
    for handle, label in zip(handles, labels):
        if not label or label.startswith("_"):
            continue
        unique[label] = handle

    for label, handle in stored.items():
        unique[label] = handle

    if unique:
        ax.legend(list(unique.values()), list(unique.keys()), loc="lower right")

    if stored:
        ax._rrt_extra_legend = stored  # type: ignore[attr-defined]
    elif hasattr(ax, "_rrt_extra_legend"):
        delattr(ax, "_rrt_extra_legend")


def plot_rrt_star(
    occupancy: np.ndarray,
    start: tuple[float, float],
    goal: tuple[float, float],
    result: PlanResult,
    *,
    show_tree: bool = True,
    inflation_mask: Optional[np.ndarray] = None,
    save_path: Optional[str | FilePath] = None,
    ax: Optional[Axes] = None,
    keep_open: bool = False,
) -> Axes:
    fig, axis, created = _prepare_axis(ax)
    base_image = axis.imshow(occupancy, cmap="gray", origin="lower")
    base_image.set_label("Occupancy")

    legend_handles: List[object] = []
    if inflation_mask is not None and inflation_mask.size:
        highlight = np.zeros_like(occupancy, dtype=float)
        highlight[np.asarray(inflation_mask, dtype=bool)] = 1.0
        cmap = mcolors.ListedColormap([(0.0, 0.0, 0.0, 0.0), "#ff7f0e"])
        inflated_image = axis.imshow(
            highlight,
            cmap=cmap,
            origin="lower",
            alpha=0.75,
            interpolation="nearest",
            zorder=3,
            vmin=0.0,
            vmax=1.0,
        )
        inflated_image.set_label("Inflated Obstacle")
        legend_handles.append(
            Patch(
                facecolor="#ff7f0e",
                edgecolor="none",
                alpha=0.75,
                label="Inflated Obstacle",
            )
        )
    axis.plot(start[0], start[1], "blue", marker="o", linestyle="None", label="Start")
    axis.plot(goal[0], goal[1], "red", marker="o", linestyle="None", label="Goal")

    if show_tree:
        label = "Tree"
        for node in result.nodes:
            if node.parent is None:
                continue
            parent = result.nodes[node.parent]
            axis.plot(
                [node.x, parent.x],
                [node.y, parent.y],
                "g-",
                linewidth=0.3,
                label=label,
            )
            label = "_nolegend_"

    artists: Dict[str, object] = getattr(axis, "_prediction_artists", {})  # type: ignore[attr-defined]

    if result.success and result.path:
        px, py = zip(*result.path)
        (path_line,) = axis.plot(px, py, "r-", linewidth=2, label="Planned Path")
        artists.setdefault("planned_path", path_line)

    axis.set_xlim(0, occupancy.shape[1])
    axis.set_ylim(0, occupancy.shape[0])
    axis.set_title("RRT* Path Planning")
    _update_legend(axis, extra_handles=legend_handles)

    axis._prediction_artists = artists  # type: ignore[attr-defined]

    if save_path is not None:
        output = resolve_plot_path(save_path)
        fig.savefig(output, bbox_inches="tight", dpi=200)
    if created:
        if _backend_supports_interaction():
            if not keep_open:
                plt.show()
        else:
            _warn_once(
                "Matplotlib backend '%s' is non-interactive; skipping on-screen display."
                % matplotlib.get_backend()
            )
            if not keep_open:
                plt.close(fig)
    else:
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    return axis


def plot_prediction(
    occupancy: np.ndarray,
    path: Path,
    predicted: Optional[FloatArray],
    state: FloatArray,
    step_idx: int,
    pause_s: float,
    *,
    ax: Optional[Axes] = None,
    vehicle_params: Optional[VehicleParams] = None,
    control: Optional[FloatArray] = None,
) -> None:
    if predicted is None:
        return

    axis = ax if ax is not None else plt.gca()
    artists: Dict[str, object] = getattr(axis, "_prediction_artists", {})  # type: ignore[attr-defined]

    pts = np.asarray(path)
    path_line = artists.get("planned_path")
    if pts.size:
        if path_line is None:
            (path_line,) = axis.plot(
                pts[:, 0],
                pts[:, 1],
                "r-",
                linewidth=2,
                label="Planned Path",
            )
            artists["planned_path"] = path_line
        else:
            path_line.set_data(pts[:, 0], pts[:, 1])
            path_line.set_visible(True)

    if vehicle_params is not None:
        artists["vehicle_params"] = vehicle_params

    params = artists.get("vehicle_params")
    if params is None:
        params = VehicleParams()
        artists["vehicle_params"] = params

    predicted_line = artists.get("predicted_line")
    if predicted_line is None:
        (predicted_line,) = axis.plot(
            predicted[0, :],
            predicted[1, :],
            "go--",
            linewidth=1.5,
            markersize=4,
            label="Predicted",
        )
        artists["predicted_line"] = predicted_line
    else:
        predicted_line.set_data(predicted[0, :], predicted[1, :])
        predicted_line.set_visible(True)

    vehicle_marker = artists.get("vehicle_marker")
    if vehicle_marker is None:
        (vehicle_marker,) = axis.plot(
            [state[0]],
            [state[1]],
            "bo",
            markersize=5,
            label="Vehicle",
        )
        artists["vehicle_marker"] = vehicle_marker
    else:
        vehicle_marker.set_data([state[0]], [state[1]])
        vehicle_marker.set_visible(True)

    vehicle_shape: List[object] = artists.get("vehicle_shape", [])
    for artist in vehicle_shape:
        artist.remove()
    steer = float(control[1]) if control is not None and control.size >= 2 else 0.0
    artists["vehicle_shape"] = draw_vehicle(
        state[0], state[1], state[2], steer, params, ax=axis, color="blue"
    )

    axis.set_xlim(0, occupancy.shape[1])
    axis.set_ylim(0, occupancy.shape[0])
    axis.set_title(f"RRT* Planning & MPC Tracking - step {step_idx}")
    axis._prediction_artists = artists  # type: ignore[attr-defined]
    _update_legend(axis)

    if _backend_supports_interaction():
        axis.figure.canvas.draw_idle()
        axis.figure.canvas.flush_events()
        plt.pause(pause_s)


def animate_vehicle_states(
    occupancy: np.ndarray,
    states: Sequence[FloatArray],
    *,
    color: str = "blue",
    delay: float = 0.03,
    ax: Optional[Axes] = None,
    vehicle_params: Optional[VehicleParams] = None,
) -> None:
    fig, axis, created = _prepare_axis(ax)
    if created:
        axis.imshow(occupancy, cmap="gray", origin="lower")
        axis.set_xlim(0, occupancy.shape[1])
        axis.set_ylim(0, occupancy.shape[0])

    params = vehicle_params or VehicleParams()
    previous: List[object] = []
    for idx, state in enumerate(states):
        for artist in previous:
            artist.remove()
        previous = draw_vehicle(state[0], state[1], state[2], 0.0, params, ax=axis, color=color)
        axis.set_title(f"Vehicle replay - frame {idx}")
        if _backend_supports_interaction():
            axis.figure.canvas.draw_idle()
            axis.figure.canvas.flush_events()
            plt.pause(delay)

    if created:
        if _backend_supports_interaction():
            plt.show()
        else:
            plt.close(fig)
