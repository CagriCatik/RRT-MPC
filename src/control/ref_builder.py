"""Reference trajectory construction from a planned geometric path."""
from __future__ import annotations

import numpy as np

from ..common.geometry import curvature_slowdown, heading_from_path, resample_polyline
from ..common.types import Path


def build_reference(path: Path, desired_speed: float, horizon: int, dt: float) -> np.ndarray:
    """Return an ``(N+1, 4)`` reference trajectory for MPC."""

    step = max(2.0, 0.8 * desired_speed * dt)
    pts = resample_polyline(path, step)
    yaw = heading_from_path(pts)
    slowdown = curvature_slowdown(yaw)
    vref = desired_speed * slowdown
    xref = np.column_stack((pts[:, 0], pts[:, 1], yaw, vref))
    if len(xref) < horizon + 1:
        tail = np.repeat(xref[-1:], horizon + 1 - len(xref), axis=0)
        xref = np.vstack((xref, tail))
    return xref
