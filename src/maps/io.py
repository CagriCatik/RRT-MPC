"""Map loading and saving helpers."""
from __future__ import annotations

from pathlib import Path

import numpy as np

try:  # pragma: no cover - optional dependency
    import cv2
except Exception:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore[assignment]
    from matplotlib import image as mpl_image
else:
    mpl_image = None  # type: ignore[assignment]

from ..common.paths import resolve_plot_path


def load_grayscale(path: str | Path) -> np.ndarray:
    """Load ``path`` as a grayscale ``uint8`` image."""

    resolved = resolve_plot_path(path)
    if cv2 is not None:
        img = cv2.imread(str(resolved), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Map not found: {resolved}")
        return img
    data = mpl_image.imread(str(resolved))
    if data is None:
        raise FileNotFoundError(f"Map not found: {resolved}")
    if data.ndim == 3:
        data = data[..., :3]  # drop alpha if present
        data = np.dot(data, np.array([0.299, 0.587, 0.114]))
    if data.dtype != np.uint8:
        data = np.clip(data * 255.0 if data.max() <= 1.0 else data, 0, 255).astype(np.uint8)
    return data


def save_grayscale(path: str | Path, img: np.ndarray) -> None:
    """Write ``img`` to ``path`` as an ``uint8`` PNG."""

    resolved = resolve_plot_path(path)
    if cv2 is not None:
        cv2.imwrite(str(resolved), img)
        return
    mpl_image.imsave(str(resolved), img, cmap="gray", vmin=0, vmax=255)


def save_binary(path: str | Path, occupancy: np.ndarray) -> None:
    """Persist a binary occupancy grid as a PNG (white free, black occupied)."""

    img = np.where(occupancy > 0, 255, 0).astype(np.uint8)
    save_grayscale(path, img)
