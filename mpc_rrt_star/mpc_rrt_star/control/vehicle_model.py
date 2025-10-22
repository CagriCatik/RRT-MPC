"""Single-track (kinematic bicycle) vehicle model in pixel coordinates."""
from __future__ import annotations

import numpy as np
import numpy.typing as npt

State = npt.NDArray[np.float64]
Control = npt.NDArray[np.float64]


def f_discrete(x: State, u: Control, dt: float, wheelbase_px: float) -> State:
    """Forward Euler integration of the bicycle model."""

    xk, yk, yaw, v = x
    a, delta = u
    beta = 0.0
    x_next = xk + dt * v * np.cos(yaw + beta)
    y_next = yk + dt * v * np.sin(yaw + beta)
    yaw_next = yaw + dt * (v / wheelbase_px) * np.tan(delta)
    v_next = v + dt * a
    return np.array([x_next, y_next, yaw_next, v_next], dtype=float)


def linearize(x: State, u: Control, dt: float, wheelbase_px: float) -> tuple[np.ndarray, np.ndarray, State]:
    """Return discrete-time Jacobians ``A`` and ``B`` around ``(x, u)``."""

    _, _, yaw, v = x
    _, delta = u
    c, s = np.cos(yaw), np.sin(yaw)
    tan_d = np.tan(delta)
    sec2_d = 1.0 / (np.cos(delta) ** 2 + 1e-9)

    A = np.eye(4)
    A[0, 2] = -dt * v * s
    A[0, 3] = dt * c
    A[1, 2] = dt * v * c
    A[1, 3] = dt * s
    A[2, 3] = dt * (1.0 / wheelbase_px) * tan_d

    B = np.zeros((4, 2))
    B[3, 0] = dt
    B[2, 1] = dt * (v / wheelbase_px) * sec2_d

    fx = f_discrete(x, u, dt, wheelbase_px)
    return A, B, fx
