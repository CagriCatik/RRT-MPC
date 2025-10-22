import numpy as np

from mpc_rrt_star.maps.generator import MapGenerator
from mpc_rrt_star.planning.plan_result import PlanResult
from mpc_rrt_star.planning.rrt_star import PlannerParameters, RRTStarPlanner


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
    assert result.path[0] == (start[0], start[1])
