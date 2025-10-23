#!/usr/bin/env python3
"""
Inflate occupancy grid map.
Expands each occupied (black) cell outward by a specified radius.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt

# ===============================================================
# Configuration
# ===============================================================
CONFIG = {
    "input_map": "occupancy_grid.png",
    "inflation_radius_m": 0.9,   # in meters (robot radius + buffer)
    "map_resolution_m": 0.2,     # meters per pixel (must match generator)
    "output_map": "occupancy_grid_inflated.png"
}
# ===============================================================


def inflate_occupancy_map(img_path, inflation_radius_m, map_resolution_m, output_path):
    # Read map
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Map not found: {img_path}")

    # Threshold (white = free, black = occupied)
    occ = (img < 128).astype(np.uint8)

    # Convert inflation distance to pixels
    inflation_px = int(np.ceil(inflation_radius_m / map_resolution_m))
    print(f"Inflating obstacles by {inflation_px} pixels (~{inflation_radius_m} m)")

    # Inflate using morphological dilation
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflation_px + 1, 2 * inflation_px + 1))
    inflated = cv2.dilate(occ, kernel)

    # Convert back to image (white free, black occupied)
    inflated_img = (1 - inflated) * 255

    # Save and preview
    cv2.imwrite(output_path, inflated_img)
    print("Saved inflated map:", output_path)

    # Visualization with color overlays
    inflated_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    inflated_mask = inflated.astype(bool)
    occ_mask = occ.astype(bool)

    # Apply colors
    inflated_color[inflated_mask] = [150, 150, 150]  # inflation area: gray
    inflated_color[occ_mask] = [0, 0, 0]             # original obstacles: black

    plt.imshow(cv2.cvtColor(inflated_color, cv2.COLOR_BGR2RGB), origin="upper")
    plt.title(f"Inflated Occupancy Map ({inflation_radius_m} m)")
    plt.axis("off")

    # Legend
    import matplotlib.patches as mpatches
    legend_patches = [
        mpatches.Patch(color='black', label='Original Obstacles'),
        mpatches.Patch(color='gray', label='Inflated Area'),
        mpatches.Patch(color='white', label='Free Space')
    ]
    plt.legend(handles=legend_patches, loc='lower right', frameon=True)
    plt.show()




if __name__ == "__main__":
    inflate_occupancy_map(
        CONFIG["input_map"],
        CONFIG["inflation_radius_m"],
        CONFIG["map_resolution_m"],
        CONFIG["output_map"]
    )
