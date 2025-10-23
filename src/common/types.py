"""Shared type definitions used across the package."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int64]

Point = Tuple[float, float]
Path = List[Point]


@dataclass
class Trajectory:
    """Simple container for an MPC trajectory."""

    states: FloatArray
    controls: FloatArray

    def last_state(self) -> FloatArray:
        return self.states[:, -1]


__all__ = ["FloatArray", "IntArray", "Point", "Path", "Trajectory"]
