"""Map loading and saving helpers."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from ..common.paths import resolve_plot_path


def load_grayscale(path: str | Path) -> np.ndarray:
    """Load ``path`` as a grayscale ``uint8`` image."""

    resolved = resolve_plot_path(path)
    img = cv2.imread(str(resolved), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Map not found: {resolved}")
    return img


def save_grayscale(path: str | Path, img: np.ndarray) -> None:
    """Write ``img`` to ``path`` as an ``uint8`` PNG."""

    resolved = resolve_plot_path(path)
    cv2.imwrite(str(resolved), img)


def save_binary(path: str | Path, occupancy: np.ndarray) -> None:
    """Persist a binary occupancy grid as a PNG (white free, black occupied)."""

    img = np.where(occupancy > 0, 255, 0).astype(np.uint8)
    save_grayscale(path, img)
