"""Example script: run deterministic RRT* planning on an inflated map."""
from __future__ import annotations

import numpy as np

from src.common.paths import resolve_plot_path
from src.config import default_config
from src.maps.generator import MapGenerator
from src.maps.inflate import inflate_grayscale_map, to_occupancy_grid
from src.maps.io import load_grayscale, save_grayscale
from src.planning.rrt_star import RRTStarPlanner
from src.viz.visualization import configure_backend, plot_rrt_star


def main() -> None:
    cfg = default_config()
    configure_backend("Agg")

    map_path = resolve_plot_path(cfg.map.map_file)
    if not map_path.exists():
        generator = MapGenerator(cfg.map.size_m, cfg.map.generator_resolution, seed=cfg.map.generator_seed)
        grid = generator.generate()
        save_grayscale(cfg.map.map_file, (grid * 255).astype("uint8"))

    raw_map = load_grayscale(cfg.map.map_file)
    inflated = inflate_grayscale_map(raw_map, cfg.map.inflation_radius_m, cfg.map.map_resolution)
    raw_occupancy = np.flipud(to_occupancy_grid(raw_map))
    occupancy = np.flipud(to_occupancy_grid(inflated))
    inflation_mask = (raw_occupancy == 1) & (occupancy == 0)
    planner = RRTStarPlanner(occupancy, cfg.planner.to_parameters())
    start = cfg.map.start
    goal = (occupancy.shape[1] - cfg.map.goal_offset[0], occupancy.shape[0] - cfg.map.goal_offset[1])
    result = planner.plan(start, goal)
    plot_rrt_star(
        occupancy,
        start,
        goal,
        result,
        show_tree=True,
        inflation_mask=inflation_mask,
        save_path="planner/examples/rrt_path.png",
    )


if __name__ == "__main__":
    main()
