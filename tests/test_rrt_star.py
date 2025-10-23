import numpy as np
import pytest

from src.maps.generator import MapGenerator
from src.planning.rrt_star import PlannerParameters, RRTStarPlanner


def test_rrt_star_deterministic_path() -> None:
    generator = MapGenerator((60.0, 60.0), 1.0, seed=2)
    base = generator.generate()
    occupancy = np.flipud((base > 0).astype(np.uint8))
    params = PlannerParameters(step=6.0, goal_radius=8.0, max_iterations=800, rewire_radius=12.0, goal_sample_rate=0.2, random_seed=3)
    planner = RRTStarPlanner(occupancy, params)
    start = (5.0, 5.0)
    goal = (occupancy.shape[1] - 6.0, occupancy.shape[0] - 6.0)
    result = planner.plan(start, goal)
    assert result.success
    assert len(result.path) > 5
    assert result.path[0] == pytest.approx((start[0], start[1]))
    assert result.raw_path
    if result.pruned_path:
        assert len(result.pruned_path) <= len(result.raw_path)
    if result.smoothed_path:
        assert len(result.smoothed_path) >= len(result.raw_path)
