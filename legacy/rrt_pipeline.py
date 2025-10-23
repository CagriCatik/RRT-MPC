#!/usr/bin/env python3
"""
Full RRT Planning Pipeline:
1. Load occupancy map (white=free, black=occupied)
2. Inflate obstacles (safety margin)
3. Run animated RRT path planning
"""

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import random
from math import sqrt, atan2, cos, sin
import time

# ===============================================================
# CONFIGURATION
# ===============================================================
CONFIG = {
    "map_file": "occupancy_grid.png",          # input map (white=free)
    "inflated_map_file": "occupancy_inflated.png",
    "path_output_file": "rrt_path_pipeline.png",
    "map_resolution": 0.2,                     # meters per pixel
    "inflation_radius_m": 0.6,                 # safety margin in meters
    "start": (50, 50),                         # start coordinates (pixels)
    "goal_offset": (50, 50),                   # offset from bottom-right corner
    "step_size": 10,
    "goal_radius": 15,
    "max_iterations": 3000,
    "goal_sample_rate": 0.1,
    "animate_delay": 0.001
}
# ===============================================================


class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


# ---------------------------- Utility Functions ---------------------------- #

def distance(a, b):
    return sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


def steer(from_node, to_node, step):
    theta = atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    return Node(from_node.x + step * cos(theta),
                from_node.y + step * sin(theta),
                from_node)


def collision_free(p1, p2, occ):
    x1, y1, x2, y2 = int(p1.x), int(p1.y), int(p2.x), int(p2.y)
    num = int(distance(p1, p2))
    for u in np.linspace(0, 1, num + 1):
        x = int(x1 + u * (x2 - x1))
        y = int(y1 + u * (y2 - y1))
        if not (0 <= x < occ.shape[1] and 0 <= y < occ.shape[0]):
            return False
        if occ[y, x] == 0:
            return False
    return True


def extract_path(goal_node):
    path = []
    n = goal_node
    while n:
        path.append((n.x, n.y))
        n = n.parent
    return path[::-1]


# ---------------------------- Map Inflation ---------------------------- #

def inflate_map(img, inflation_radius_m, resolution_m):
    occ = (img < 128).astype(np.uint8)
    inflation_px = int(np.ceil(inflation_radius_m / resolution_m))
    print(f"Inflation: {inflation_px} pixels ({inflation_radius_m} m)")

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflation_px + 1, 2 * inflation_px + 1))
    inflated = cv2.dilate(occ, kernel)
    inflated_img = (1 - inflated) * 255
    return inflated_img


# ---------------------------- RRT Algorithm ---------------------------- #

def rrt_with_animation(occ, start, goal, cfg):
    start_time = time.time()
    start_node = Node(*start)
    goal_node = Node(*goal)
    nodes = [start_node]

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(occ, cmap="gray", origin="lower")
    ax.plot(start[0], start[1], "bo", label="Start")
    ax.plot(goal[0], goal[1], "ro", label="Goal")
    ax.legend(loc="upper right")
    ax.set_title("RRT Planning (Animated)")

    for i in range(cfg["max_iterations"]):
        if random.random() < cfg["goal_sample_rate"]:
            rnd = Node(goal[0], goal[1])
        else:
            rnd = Node(random.randint(0, occ.shape[1] - 1),
                       random.randint(0, occ.shape[0] - 1))

        nearest = min(nodes, key=lambda n: distance(n, rnd))
        new = steer(nearest, rnd, cfg["step_size"])

        if 0 <= new.x < occ.shape[1] and 0 <= new.y < occ.shape[0] and collision_free(nearest, new, occ):
            nodes.append(new)
            ax.plot([nearest.x, new.x], [nearest.y, new.y], "g-", linewidth=0.4)
            plt.draw()
            plt.pause(cfg["animate_delay"])

            if i % 100 == 0:
                print(f"[{i}] nodes: {len(nodes)}")

            if distance(new, goal_node) < cfg["goal_radius"] and collision_free(new, goal_node, occ):
                goal_node.parent = new
                print(f"Goal reached at iteration {i}.")
                path = extract_path(goal_node)
                px, py = zip(*path)
                ax.plot(px, py, "r-", linewidth=2)
                plt.ioff()
                plt.savefig(cfg["path_output_file"], bbox_inches="tight", dpi=200)
                plt.show()
                print("Saved:", cfg["path_output_file"])
                return

    print("Failed to find a path after", cfg["max_iterations"], "iterations.")
    plt.ioff()
    plt.show()


# ---------------------------- Main Pipeline ---------------------------- #

def main():
    cfg = CONFIG
    print("=== RRT PIPELINE START ===")

    # Step 1: Load original map
    img = cv2.imread(cfg["map_file"], cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Map not found: {cfg['map_file']}")
    print("Loaded map:", cfg["map_file"])

    # Step 2: Inflate map
    inflated_img = inflate_map(img, cfg["inflation_radius_m"], cfg["map_resolution"])
    cv2.imwrite(cfg["inflated_map_file"], inflated_img)
    print("Saved inflated map:", cfg["inflated_map_file"])

    # Step 3: Convert to occupancy grid (white=free)
    occ = (inflated_img > 200).astype(np.uint8)
    occ = np.flipud(occ)

    # Step 4: Run RRT on inflated map
    start = cfg["start"]
    goal = (occ.shape[1] - cfg["goal_offset"][0],
            occ.shape[0] - cfg["goal_offset"][1])
    print("Start:", start, "Goal:", goal)

    rrt_with_animation(occ, start, goal, cfg)
    print("=== PIPELINE COMPLETE ===")


if __name__ == "__main__":
    main()
