"""Demonstration of the rectangular-obstacle RRT planner with smoothing."""
from __future__ import annotations

import matplotlib.pyplot as plt

from src.planning.rrt import Rect, RRTParameters, RRTPlanner


def main() -> None:
    x_range = (0.0, 100.0)
    y_range = (0.0, 100.0)

    base_obstacles = (
        Rect(15, 40, 45, 60),
        Rect(55, 20, 85, 35),
        Rect(60, 65, 90, 85),
        Rect(20, 10, 35, 25),
        Rect(10, 70, 25, 90),
    )

    params = RRTParameters(
        step=3.0,
        goal_sample_rate=0.07,
        max_iterations=15_000,
        goal_tolerance=4.0,
        collision_step=0.75,
        rng_seed=7,
        prune_path=True,
        spline_samples=20,
        spline_alpha=0.5,
    )

    planner = RRTPlanner(obstacles=base_obstacles, workspace=(x_range, y_range), params=params)
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
    ax.grid(True, linewidth=0.5, alpha=0.3)

    for rect in base_obstacles:
        ax.add_patch(
            plt.Rectangle(
                (rect.xmin, rect.ymin),
                rect.xmax - rect.xmin,
                rect.ymax - rect.ymin,
                fill=True,
                alpha=0.35,
                linewidth=0.0,
            )
        )

    for node in result.nodes:
        if node.parent is None:
            continue
        parent = result.nodes[node.parent]
        ax.plot([node.x, parent.x], [node.y, parent.y], linewidth=0.5, alpha=0.3, color="gray")

    if result.raw_path:
        rx, ry = zip(*result.raw_path)
        ax.plot(rx, ry, linestyle="--", linewidth=1.5, color="tab:orange", label="RRT path")
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
    ax.set_title("RRT with Catmull-Rom Spline")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

