"""Geometry helpers shared by planning and control modules."""
from __future__ import annotations

import numpy as np

from .types import Path, Point


def resample_polyline(path: Path, step: float) -> np.ndarray:
    """Resample ``path`` at approximately ``step`` pixel spacing."""

    pts = np.asarray(path, dtype=float)
    if len(pts) < 2:
        return pts

    seg = np.diff(pts, axis=0)
    dist = np.hypot(seg[:, 0], seg[:, 1])
    s = np.insert(np.cumsum(dist), 0, 0.0)
    total = s[-1]
    if total < 1e-9:
        return pts

    samples = np.arange(0.0, total, step)
    if not np.isclose(samples[-1], total):
        samples = np.append(samples, total)

    x_interp = np.interp(samples, s, pts[:, 0])
    y_interp = np.interp(samples, s, pts[:, 1])
    return np.column_stack((x_interp, y_interp))


def heading_from_path(points: np.ndarray) -> np.ndarray:
    """Return yaw angles computed from successive differences."""

    dirs = np.diff(points, axis=0, prepend=points[0:1])
    return np.unwrap(np.arctan2(dirs[:, 1], dirs[:, 0]))


def curvature_slowdown(yaw: np.ndarray) -> np.ndarray:
    """Return a curvature based slowdown factor within ``[0.6, 1.0]``."""

    hd = np.abs(np.diff(yaw, prepend=yaw[0]))
    hd = np.minimum(hd, np.pi - hd)
    slow = 1.0 / (1.0 + 4.0 * hd)
    return 0.6 + 0.4 * slow
