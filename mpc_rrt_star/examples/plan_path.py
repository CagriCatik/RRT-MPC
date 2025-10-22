"""Example script: run deterministic RRT* planning on an inflated map."""
from __future__ import annotations

import numpy as np

from mpc_rrt_star.config import default_config
from mpc_rrt_star.maps.inflate import inflate_grayscale_map, to_occupancy_grid
from mpc_rrt_star.maps.io import load_grayscale
from mpc_rrt_star.planning.rrt_star import RRTStarPlanner
from mpc_rrt_star.viz.visualization import configure_backend, plot_rrt_star


def main() -> None:
    cfg = default_config()
    configure_backend("Agg")
    raw_map = load_grayscale(cfg.map.map_file)
    inflated = inflate_grayscale_map(raw_map, cfg.map.inflation_radius_m, cfg.map.map_resolution)
    occupancy = np.flipud(to_occupancy_grid(inflated))
    planner = RRTStarPlanner(occupancy, cfg.planner.to_parameters())
    start = cfg.map.start
    goal = (occupancy.shape[1] - cfg.map.goal_offset[0], occupancy.shape[0] - cfg.map.goal_offset[1])
    result = planner.plan(start, goal)
    plot_rrt_star(occupancy, start, goal, result, show_tree=True, save_path="rrt_path.png")


if __name__ == "__main__":
    main()
