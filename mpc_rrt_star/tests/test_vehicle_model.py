import numpy as np
import pytest

from mpc_rrt_star.control.vehicle_model import f_discrete, linearize


def test_forward_motion_increases_position() -> None:
    x = np.array([0.0, 0.0, 0.0, 10.0])
    u = np.array([0.0, 0.0])
    next_state = f_discrete(x, u, 0.1, 5.0)
    assert next_state[0] > x[0]
    assert next_state[1] == pytest.approx(0.0, abs=1e-6)


def test_linearization_shapes() -> None:
    x = np.array([1.0, 2.0, 0.2, 8.0])
    u = np.array([1.0, 0.1])
    A, B, fx = linearize(x, u, 0.1, 5.0)
    assert A.shape == (4, 4)
    assert B.shape == (4, 2)
    assert fx.shape == (4,)
