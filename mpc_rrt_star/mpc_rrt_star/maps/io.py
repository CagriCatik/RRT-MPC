"""Map loading and saving helpers."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def load_grayscale(path: str | Path) -> np.ndarray:
    """Load ``path`` as a grayscale ``uint8`` image."""

    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Map not found: {path}")
    return img


def save_grayscale(path: str | Path, img: np.ndarray) -> None:
    """Write ``img`` to ``path`` as an ``uint8`` PNG."""

    cv2.imwrite(str(path), img)


def save_binary(path: str | Path, occupancy: np.ndarray) -> None:
    """Persist a binary occupancy grid as a PNG (white free, black occupied)."""

    img = np.where(occupancy > 0, 255, 0).astype(np.uint8)
    save_grayscale(path, img)
