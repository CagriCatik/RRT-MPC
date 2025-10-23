#!/usr/bin/env python3
"""
Fully interactive RRT with animation and debug logging.
Works on any simple occupancy grid (white=free, black=occupied).
"""

import matplotlib
matplotlib.use("TkAgg")  # Force interactive GUI backend

import cv2
import numpy as np
import matplotlib.pyplot as plt
import random
from math import sqrt, atan2, cos, sin
import time
from pathlib import Path

# ===============================================================
# Config
# ===============================================================
CONFIG = {
    "map_file": "maps/occupancy_grid.png",
    "start": (50, 50),
    "goal_offset": (50, 50),
    "step_size": 10,
    "goal_radius": 15,
    "max_iterations": 2000,
    "goal_sample_rate": 0.1,   # chance to sample goal
    "animate_delay": 0.001,
    "save_path": "planner/legacy/rrt_path_live.png"
}
# ===============================================================


class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


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


def rrt_live(occ, start, goal, step, goal_radius, max_iter, goal_prob, delay, save_path):
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
    ax.set_title("RRT Animation (Interactive)")

    for i in range(max_iter):
        # Random sampling with goal bias
        if random.random() < goal_prob:
            rnd = Node(goal[0], goal[1])
        else:
            rnd = Node(random.randint(0, occ.shape[1] - 1),
                       random.randint(0, occ.shape[0] - 1))

        nearest = min(nodes, key=lambda n: distance(n, rnd))
        new = steer(nearest, rnd, step)

        if 0 <= new.x < occ.shape[1] and 0 <= new.y < occ.shape[0] and collision_free(nearest, new, occ):
            nodes.append(new)
            ax.plot([nearest.x, new.x], [nearest.y, new.y], "g-", linewidth=0.5)

            # **Force refresh**
            plt.draw()
            plt.pause(delay)

            if i % 100 == 0:
                print(f"[{i}] nodes: {len(nodes)} elapsed: {time.time() - start_time:.1f}s")

            if distance(new, goal_node) < goal_radius and collision_free(new, goal_node, occ):
                goal_node.parent = new
                print(f"Goal reached in {i} iterations ({time.time() - start_time:.2f}s).")
                path = extract_path(goal_node)
                px, py = zip(*path)
                ax.plot(px, py, "r-", linewidth=2)
                plt.ioff()
                plt.savefig(save_path, bbox_inches="tight", dpi=200)
                plt.show()
                print("Saved final image:", save_path)
                return

    print("Failed to find path after", max_iter, "iterations.")
    plt.ioff()
    plt.show()


def _resolve_plot_path(path):
    target = Path(path)
    if not target.is_absolute():
        target = Path("plots") / target
    target.parent.mkdir(parents=True, exist_ok=True)
    return str(target)


def main():
    cfg = CONFIG.copy()
    for key in ("map_file", "save_path"):
        cfg[key] = _resolve_plot_path(cfg[key])

    img = cv2.imread(cfg["map_file"], cv2.IMREAD_GRAYSCALE)
    occ = (img > 200).astype(np.uint8)
    occ = np.flipud(occ)

    start = CONFIG["start"]
    goal = (occ.shape[1] - CONFIG["goal_offset"][0],
            occ.shape[0] - CONFIG["goal_offset"][1])

    print("Map size:", occ.shape)
    print("Start:", start, "Goal:", goal)

    rrt_live(
        occ,
        start,
        goal,
        cfg["step_size"],
        cfg["goal_radius"],
        cfg["max_iterations"],
        cfg["goal_sample_rate"],
        cfg["animate_delay"],
        cfg["save_path"],
    )


if __name__ == "__main__":
    main()
