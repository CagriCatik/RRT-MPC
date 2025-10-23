"""Visualization helpers for planning and control results."""
from __future__ import annotations

from pathlib import Path as FilePath
from typing import Optional, Sequence

import matplotlib
from matplotlib import rcsetup
import numpy as np

from ..common.types import FloatArray, Path
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


def _get_pyplot():
    import matplotlib.pyplot as plt

    return plt


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
            try:
                matplotlib.use(candidate, force=True)
            except Exception:  # pragma: no cover - backend availability is environment specific
                continue
            if _backend_supports_interaction():
                LOG.info(
                    "Using Matplotlib backend '%s' for interactive display.",
                    matplotlib.get_backend(),
                )
                return
        try:
            matplotlib.use(original, force=True)
        except Exception:  # pragma: no cover - fallback should rarely fail
            pass
        if not _backend_supports_interaction():
            LOG.info(
                "No interactive Matplotlib backend detected; continuing with '%s'.",
                matplotlib.get_backend(),
            )
        return
    try:
        matplotlib.use(backend, force=True)
        LOG.info("Configured Matplotlib backend '%s'.", matplotlib.get_backend())
    except Exception as exc:
        LOG.warning(
            "Failed to set Matplotlib backend to '%s': %s. Using '%s'.",
            backend,
            exc,
            matplotlib.get_backend(),
        )


def plot_rrt_star(
    occupancy: np.ndarray,
    start: tuple[float, float],
    goal: tuple[float, float],
    result: PlanResult,
    *,
    show_tree: bool = True,
    save_path: Optional[str | FilePath] = None,
) -> None:
    plt = _get_pyplot()

    plt.figure(figsize=(7, 7))
    plt.imshow(occupancy, cmap="gray", origin="lower")
    plt.plot(start[0], start[1], "bo", label="Start")
    plt.plot(goal[0], goal[1], "ro", label="Goal")

    if show_tree:
        for node in result.nodes:
            if node.parent is None:
                continue
            parent = result.nodes[node.parent]
            plt.plot([node.x, parent.x], [node.y, parent.y], "g-", linewidth=0.3)

    if result.success:
        px, py = zip(*result.path)
        plt.plot(px, py, "r-", linewidth=2, label="Path")

    plt.legend(loc="upper right")
    plt.title("RRT* Path Planning")
    if save_path is not None:
        plt.savefig(save_path, bbox_inches="tight", dpi=200)
    if _backend_supports_interaction():
        plt.show()
    else:
        _warn_once(
            "Matplotlib backend '%s' is non-interactive; skipping on-screen display."
            % matplotlib.get_backend()
        )
        plt.close()


def plot_prediction(
    occupancy: np.ndarray,
    path: Path,
    predicted: Optional[FloatArray],
    state: FloatArray,
    step_idx: int,
    pause_s: float,
) -> None:
    if predicted is None:
        return
    plt = _get_pyplot()

    plt.clf()
    plt.imshow(occupancy, cmap="gray", origin="lower")
    pts = np.asarray(path)
    plt.plot(pts[:, 0], pts[:, 1], "r-", linewidth=2, label="Planned Path")
    plt.plot(state[0], state[1], "bo", markersize=5, label="Vehicle")
    plt.plot(
        predicted[0, :],
        predicted[1, :],
        "go--",
        linewidth=1.5,
        markersize=4,
        label="Predicted",
    )
    plt.xlim(0, occupancy.shape[1])
    plt.ylim(0, occupancy.shape[0])
    plt.title(f"MPC Step {step_idx}")
    plt.legend(loc="upper right")
    if _backend_supports_interaction():
        plt.pause(pause_s)


def animate_vehicle_states(
    occupancy: np.ndarray,
    states: Sequence[FloatArray],
    *,
    color: str = "blue",
    delay: float = 0.03,
) -> None:
    plt = _get_pyplot()

    params = VehicleParams()
    plt.figure(figsize=(7, 7))
    for state in states:
        plt.clf()
        plt.imshow(occupancy, cmap="gray", origin="lower")
        draw_vehicle(state[0], state[1], state[2], 0.0, params, color=color)
        plt.xlim(0, occupancy.shape[1])
        plt.ylim(0, occupancy.shape[0])
        if _backend_supports_interaction():
            plt.pause(delay)
    if _backend_supports_interaction():
        plt.show()
    else:
        plt.close()
