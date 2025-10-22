import numpy as np

from mpc_rrt_star.config import MPCConfig
from mpc_rrt_star.control.mpc_controller import MPCController


def test_mpc_solves_tracking_problem() -> None:
    cfg = MPCConfig(horizon=5)
    params = cfg.to_parameters(map_resolution=0.2)
    controller = MPCController(params)
    x0 = np.array([0.0, 0.0, 0.0, 5.0])
    ref = np.tile(np.array([1.0, 0.0, 0.0, 5.0]), (cfg.horizon + 1, 1))
    u0, Xp, Up = controller.solve(x0, ref)
    assert u0 is not None
    assert Xp is not None
    assert Up is not None
    assert Xp[0, 1] > x0[0]
