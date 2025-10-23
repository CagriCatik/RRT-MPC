"""Occupancy grid inflation routines."""
from __future__ import annotations

import cv2
import numpy as np


def inflation_radius_pixels(radius_m: float, resolution_m: float) -> int:
    """Return the pixel radius for the given metric inflation radius."""

    return int(np.ceil(radius_m / resolution_m))


def inflate_binary_occupancy(occupancy: np.ndarray, radius_px: int) -> np.ndarray:
    """Dilate obstacle cells by ``radius_px`` using an elliptical kernel."""

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
