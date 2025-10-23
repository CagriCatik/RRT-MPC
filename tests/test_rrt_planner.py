"""Unit tests for the rectangular RRT planner and utilities."""
from __future__ import annotations

import math

import numpy as np

from src.planning.rrt import (
    Rect,
    RRTParameters,
    RRTPlanner,
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


def test_rrt_planner_finds_path_without_obstacles() -> None:
    params = RRTParameters(
        step=2.5,
        goal_sample_rate=0.15,
        max_iterations=4000,
        goal_tolerance=2.0,
        collision_step=0.5,
        rng_seed=5,
        prune_path=True,
        spline_samples=8,
    )
    planner = RRTPlanner(obstacles=(), workspace=((0.0, 30.0), (0.0, 30.0)), params=params)
    start = (2.0, 2.0)
    goal = (28.0, 28.0)
    result = planner.plan(start, goal)
    assert result.success
    assert len(result.raw_path) >= 2
    assert result.smoothed_path is None or len(result.smoothed_path) >= len(result.raw_path)
