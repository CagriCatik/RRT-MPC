"""Occupancy grid inflation routines."""
from __future__ import annotations

import numpy as np

try:  # pragma: no cover - optional dependency
    import cv2
except Exception:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore[assignment]


def inflation_radius_pixels(radius_m: float, resolution_m: float) -> int:
    """Return the pixel radius for the given metric inflation radius."""

    return int(np.ceil(radius_m / resolution_m))


def _fallback_dilation(occupancy: np.ndarray, radius_px: int) -> np.ndarray:
    """Simple binary dilation used when OpenCV is unavailable."""

    if radius_px <= 0:
        return occupancy.copy()
    offsets = [
        (dy, dx)
        for dy in range(-radius_px, radius_px + 1)
        for dx in range(-radius_px, radius_px + 1)
        if dx * dx + dy * dy <= radius_px * radius_px
    ]
    inflated = occupancy.copy()
    rows, cols = occupancy.shape
    for y, x in np.argwhere(occupancy == 0):
        for dy, dx in offsets:
            ny = y + dy
            nx = x + dx
            if 0 <= ny < rows and 0 <= nx < cols:
                inflated[ny, nx] = 0
    return inflated


def inflate_binary_occupancy(occupancy: np.ndarray, radius_px: int) -> np.ndarray:
    """Dilate obstacle cells by ``radius_px`` using an elliptical kernel."""

    if radius_px <= 0:
        return occupancy.astype(np.uint8)
    if cv2 is None:
        return _fallback_dilation(occupancy.astype(np.uint8), radius_px)

    kernel_size = 2 * radius_px + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    inflated = cv2.dilate((occupancy == 0).astype(np.uint8), kernel)
    return np.where(inflated > 0, 0, 1).astype(np.uint8)


def inflate_grayscale_map(img: np.ndarray, radius_m: float, resolution_m: float) -> np.ndarray:
    """Inflate a grayscale occupancy image (white free, black occupied)."""

    radius_px = inflation_radius_pixels(radius_m, resolution_m)
    occupancy = (img > 200).astype(np.uint8)
    inflated = inflate_binary_occupancy(occupancy, radius_px)
    return (inflated * 255).astype(np.uint8)


def to_occupancy_grid(img: np.ndarray) -> np.ndarray:
    """Convert a grayscale map to an occupancy grid with ``1`` for free space."""

    return (img > 200).astype(np.uint8)
