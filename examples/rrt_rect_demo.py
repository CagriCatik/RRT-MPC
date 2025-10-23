"""Demonstration of the RRT* planner with Catmull-Rom smoothing."""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from src.planning.rrt_star import PlannerParameters, RRTStarPlanner


def main() -> None:
    x_range = (0.0, 100.0)
    y_range = (0.0, 100.0)

    base_obstacles = (
        (15, 40, 45, 60),
        (55, 20, 85, 35),
        (60, 65, 90, 85),
        (20, 10, 35, 25),
        (10, 70, 25, 90),
    )

    occupancy = np.ones((int(y_range[1]), int(x_range[1])), dtype=np.uint8)
    for xmin, ymin, xmax, ymax in base_obstacles:
        x0 = max(int(xmin), 0)
        x1 = min(int(xmax), occupancy.shape[1] - 1)
        y0 = max(int(ymin), 0)
        y1 = min(int(ymax), occupancy.shape[0] - 1)
        occupancy[y0:y1, x0:x1] = 0
    occupancy = np.flipud(occupancy)

    params = PlannerParameters(
        step=3.0,
        goal_radius=4.0,
        max_iterations=6_000,
        rewire_radius=12.0,
        goal_sample_rate=0.15,
        random_seed=7,
        prune_path=True,
        spline_samples=20,
        spline_alpha=0.5,
        collision_step=0.75,
    )

    planner = RRTStarPlanner(occupancy, params)
    start = (5.0, 5.0)
    goal = (95.0, 95.0)
    result = planner.plan(start, goal)

    if not result.success:
        raise RuntimeError("Planner failed to find a path")

    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.imshow(occupancy, cmap="gray", origin="lower")

    for node in result.nodes:
        if node.parent is None:
            continue
        parent = result.nodes[node.parent]
        ax.plot([node.x, parent.x], [node.y, parent.y], linewidth=0.5, alpha=0.3, color="gray")

    if result.raw_path:
        rx, ry = zip(*result.raw_path)
        ax.plot(rx, ry, linestyle="--", linewidth=1.5, color="tab:orange", label="Tree path")
    if result.pruned_path:
        px, py = zip(*result.pruned_path)
        ax.plot(px, py, linestyle="-.", linewidth=2.0, color="tab:green", label="Pruned")
    if result.smoothed_path:
        sx, sy = zip(*result.smoothed_path)
        ax.plot(sx, sy, linewidth=3.0, color="tab:blue", label="Spline")
    else:
        sx, sy = zip(*result.path)
        ax.plot(sx, sy, linewidth=3.0, color="tab:blue", label="Final path")

    ax.plot(start[0], start[1], marker="o", markersize=8, label="Start")
    ax.plot(goal[0], goal[1], marker="o", markersize=8, label="Goal")
    ax.legend(loc="best")
    ax.set_title("RRT* with Catmull-Rom Spline")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
