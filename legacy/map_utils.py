#!/usr/bin/env python3
"""
map_utils.py
Utilities for occupancy map loading, inflation, and preprocessing.
"""

import cv2
import numpy as np


def load_map(path: str):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Map not found: {path}")
    return img


def inflate_map(img: np.ndarray, inflation_radius_m: float, resolution_m: float) -> np.ndarray:
    occ = (img < 128).astype(np.uint8)
    inflation_px = int(np.ceil(inflation_radius_m / resolution_m))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflation_px + 1, 2 * inflation_px + 1))
    inflated = cv2.dilate(occ, kernel)
    inflated_img = (1 - inflated) * 255
    return inflated_img


def to_occupancy_grid(img: np.ndarray) -> np.ndarray:
    """Convert grayscale image to occupancy grid (1=free, 0=occupied)."""
    return (img > 200).astype(np.uint8)
