"""Lightweight recording utilities for simulation frames."""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from matplotlib.figure import Figure


def _get_pyplot():
    import matplotlib.pyplot as plt

    return plt


class FrameRecorder:
    """Save matplotlib figures as numbered PNG frames."""

    def __init__(self, directory: str | Path, *, prefix: str = "frame") -> None:
        self.directory = Path(directory)
        self.prefix = prefix
        self.counter = 0
        self.directory.mkdir(parents=True, exist_ok=True)

    def capture(self, figure: Optional[Figure] = None) -> Path:
        plt = _get_pyplot()
        fig = figure if figure is not None else plt.gcf()
        path = self.directory / f"{self.prefix}_{self.counter:05d}.png"
        fig.savefig(path, bbox_inches="tight", dpi=150)
        self.counter += 1
        return path
