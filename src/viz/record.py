"""Lightweight recording utilities for simulation frames and GIFs."""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

import imageio.v2 as imageio
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

from ..common.paths import resolve_plot_dir, resolve_plot_path


class FrameRecorder:
    """Save matplotlib figures as numbered PNG frames."""

    def __init__(self, directory: str | Path, *, prefix: str = "frame") -> None:
        self.directory = resolve_plot_dir(directory)
        self.prefix = prefix
        self.counter = 0

    def capture(self, figure: Optional[Figure] = None) -> Path:
        fig = figure if figure is not None else plt.gcf()
        path = self.directory / f"{self.prefix}_{self.counter:05d}.png"
        fig.savefig(path, bbox_inches="tight", dpi=150)
        self.counter += 1
        return path


def assemble_gif(frame_paths: Sequence[Path], output: str | Path, *, duration: float = 0.05) -> Path:
    """Combine `frame_paths` into a GIF saved under `plots`."""
    ordered = list(sorted(frame_paths))
    if not ordered:
        raise ValueError("No frames provided for GIF assembly")
    images = [imageio.imread(path) for path in ordered]
    destination = resolve_plot_path(output)
    imageio.mimsave(destination, images, duration=duration)
    return destination
