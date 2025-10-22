"""Lightweight recording utilities for simulation frames."""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt


class FrameRecorder:
    """Save matplotlib figures as numbered PNG frames."""

    def __init__(self, directory: str | Path, *, prefix: str = "frame") -> None:
        self.directory = Path(directory)
        self.prefix = prefix
        self.counter = 0
        self.directory.mkdir(parents=True, exist_ok=True)

    def capture(self, figure: Optional[plt.Figure] = None) -> Path:
        fig = figure if figure is not None else plt.gcf()
        path = self.directory / f"{self.prefix}_{self.counter:05d}.png"
        fig.savefig(path, bbox_inches="tight", dpi=150)
        self.counter += 1
        return path
