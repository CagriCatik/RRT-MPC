"""Deterministic occupancy grid generation utilities."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from ..logging_setup import get_logger


LOG = get_logger(__name__)


@dataclass
class BoxObstacle:
    """Axis-aligned rectangular obstacle specified in meters."""

    center: Tuple[float, float]
    size: Tuple[float, float]


class MapGenerator:
    """Generate simple occupancy grids for testing motion planning pipelines."""

    def __init__(self, size_m: Tuple[float, float], resolution: float, seed: Optional[int] = None) -> None:
        self.size_m = size_m
        self.resolution = resolution
        self.rng = np.random.default_rng(seed)

    def _allocate(self) -> np.ndarray:
        rows = int(self.size_m[1] / self.resolution)
        cols = int(self.size_m[0] / self.resolution)
        grid = np.ones((rows, cols), dtype=np.float32)
        grid[[0, -1], :] = 0.0
        grid[:, [0, -1]] = 0.0
        return grid

    def generate(self, *, add_box: bool = True) -> np.ndarray:
        grid = self._allocate()
        LOG.info(
            "Generating occupancy grid (size=%.1fm×%.1fm, resolution=%.2fm)",
            self.size_m[0],
            self.size_m[1],
            self.resolution,
        )
        if add_box:
            box_size = min(self.size_m) * 0.125
            box = BoxObstacle(center=(self.size_m[0] / 2.0, self.size_m[1] / 2.0), size=(box_size, box_size))
            self._apply_box(grid, box)
            LOG.debug(
                "Inserted central obstacle centred at (%.1f, %.1f) with size %.1fm×%.1fm",
                box.center[0],
                box.center[1],
                box.size[0],
                box.size[1],
            )
        else:
            LOG.debug("Skipping obstacle insertion for generated grid")
        return grid

    def _apply_box(self, grid: np.ndarray, box: BoxObstacle) -> None:
        cx, cy = box.center
        sx, sy = box.size
        rows = grid.shape[0]
        cols = grid.shape[1]
        x0 = int((cx - sx / 2.0) / self.resolution)
        x1 = int((cx + sx / 2.0) / self.resolution)
        y0 = int((cy - sy / 2.0) / self.resolution)
        y1 = int((cy + sy / 2.0) / self.resolution)
        x0 = np.clip(x0, 0, cols - 1)
        x1 = np.clip(x1, 0, cols - 1)
        y0 = np.clip(y0, 0, rows - 1)
        y1 = np.clip(y1, 0, rows - 1)
        grid[y0:y1, x0:x1] = 0.0


__all__ = ["MapGenerator", "BoxObstacle"]
