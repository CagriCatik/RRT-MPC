#!/usr/bin/env python3
"""
Simple outdoor occupancy grid map with one box (obstacle) in the center.
White = drivable/free
Black = occupied
No axes or labels.
"""

import numpy as np
import matplotlib.pyplot as plt

def generate_grid(size_m=(80, 80), resolution=1.0):
    rows = int(size_m[0] / resolution)
    cols = int(size_m[1] / resolution)
    grid = np.ones((rows, cols), dtype=np.float32)

    # Boundaries
    grid[0, :] = 0
    grid[-1, :] = 0
    grid[:, 0] = 0
    grid[:, -1] = 0

    # Central obstacle
    box_size_m = 10.0
    box_size = int(box_size_m / resolution)
    r0 = rows // 2 - box_size // 2
    r1 = rows // 2 + box_size // 2
    c0 = cols // 2 - box_size // 2
    c1 = cols // 2 + box_size // 2
    grid[r0:r1, c0:c1] = 0

    return grid

def save_png(grid, path="occupancy_grid.png"):
    plt.imshow(grid, cmap="gray", origin="upper", vmin=0, vmax=1)
    plt.axis("off")
    plt.savefig(path, bbox_inches="tight", pad_inches=0, dpi=200, format="png")
    plt.close()

if __name__ == "__main__":
    grid = generate_grid()
    save_png(grid)
