#!/usr/bin/env python3
"""
visualization.py
Live plotting for RRT and RRT* algorithms + vehicle animation.
"""

import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from rrt_star import extract_path
from vehicle_draw import draw_car, VehicleParams

def visualize_rrt_star(occ, start, goal, nodes, goal_node, delay=0.001, save_path=None):
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(occ, cmap="gray", origin="lower")
    ax.plot(start[0], start[1], "bo", label="Start")
    ax.plot(goal[0], goal[1], "ro", label="Goal")
    ax.legend(loc="upper right")
    ax.set_title("RRT* Path Planning")

    for n in nodes:
        if n.parent:
            ax.plot([n.x, n.parent.x], [n.y, n.parent.y], "g-", linewidth=0.3)
            plt.pause(delay)

    path = None
    if goal_node:
        path = extract_path(goal_node)
        px, py = zip(*path)
        ax.plot(px, py, "r-", linewidth=2, label="Path")

    plt.ioff()
    if save_path:
        path = Path(save_path)
        if not path.is_absolute():
            path = Path("plots") / path
        path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(path, bbox_inches="tight", dpi=200)
    plt.show()
    return path

def animate_vehicle_states(occ, states, color='blue', delay=0.03):
    params = VehicleParams()
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title("MPC Vehicle Tracking")
    for x, y, yaw, v in states:
        ax.clear()
        ax.imshow(occ, cmap="gray", origin="lower")
        draw_car(x, y, yaw, steer=0.0, params=params, color=color)
        ax.set_xlim(0, occ.shape[1]); ax.set_ylim(0, occ.shape[0])
        plt.pause(delay)
        time.sleep(delay)
    plt.ioff(); plt.show()
