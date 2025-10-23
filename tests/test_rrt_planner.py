"""Unit tests for the rectangular RRT planner and utilities."""
from __future__ import annotations

import math

import math

import numpy as np

from src.planning.rrt_star import (
    Rect,
    PlannerParameters,
    RRTStarPlanner,
    catmull_rom_spline,
    inflate_rect,
    prune_path,
)


def test_inflate_rect_clamps_to_workspace() -> None:
    rect = Rect(2.0, 2.0, 4.0, 4.0)
    inflated = inflate_rect(rect, 3.0, (0.0, 5.0), (0.0, 5.0))
    assert math.isclose(inflated.xmin, 0.0)
    assert math.isclose(inflated.ymin, 0.0)
    assert math.isclose(inflated.xmax, 5.0)
    assert math.isclose(inflated.ymax, 5.0)


def test_catmull_rom_spline_removes_duplicates() -> None:
    pts = [(0.0, 0.0), (1.0, 1.0), (1.0, 1.0), (2.0, 0.0)]
    spline = catmull_rom_spline(pts, samples_per_segment=5)
    assert np.allclose(spline[0], pts[0])
    assert np.allclose(spline[-1], pts[-1])
    assert len(spline) >= 6


def test_prune_path_removes_redundant_waypoints() -> None:
    path = [(0.0, 0.0), (2.0, 2.0), (4.0, 4.0)]
    pruned = prune_path(path, obstacles=())
    assert pruned.shape[0] == 2
    assert tuple(pruned[0]) == (0.0, 0.0)
    assert tuple(pruned[-1]) == (4.0, 4.0)


def test_rrt_star_planner_smooths_path_in_free_space() -> None:
    occupancy = np.ones((40, 40), dtype=np.uint8)
    params = PlannerParameters(
        step=3.0,
        goal_radius=2.5,
        max_iterations=2000,
        rewire_radius=6.0,
        goal_sample_rate=0.2,
        random_seed=5,
        prune_path=True,
        spline_samples=6,
        spline_alpha=0.5,
        dedupe_tolerance=1e-9,
        collision_step=0.5,
    )
    planner = RRTStarPlanner(occupancy, params)
    start = (2.0, 2.0)
    goal = (35.0, 35.0)
    result = planner.plan(start, goal)
    assert result.success
    assert len(result.raw_path) >= 2
    assert result.smoothed_path is not None
    assert len(result.smoothed_path) >= 2
    assert result.smoothed_path[0] == result.raw_path[0]
    assert result.smoothed_path[-1] == result.raw_path[-1]
