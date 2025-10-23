"""Linear time-varying MPC solved via OSQP."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cvxpy as cp
import numpy as np

from ..common.types import FloatArray
from ..logging_setup import get_logger
from .vehicle_model import linearize

LOG = get_logger(__name__)


@dataclass
class MPCParameters:
    wheelbase_px: float
    dt: float
    horizon: int
    q: FloatArray
    r: FloatArray
    q_terminal: FloatArray
    u_bounds: Tuple[Tuple[float, float], Tuple[float, float]]
    v_bounds: Tuple[float, float]
    du_bounds: Tuple[Tuple[float, float], Tuple[float, float]]
    slack_velocity: float = 1e3
    slack_input: float = 5e2
    slack_rate: float = 5e2


class MPCController:
    """Quadratic-cost MPC controller with soft bounds and rate limits."""

    def __init__(self, params: MPCParameters) -> None:
        self._params = params

    def solve(
        self,
        x0: FloatArray,
        ref_traj: FloatArray,
        *,
        u_init: Optional[FloatArray] = None,
        u_prev: Optional[FloatArray] = None,
    ) -> Tuple[Optional[FloatArray], Optional[FloatArray], Optional[FloatArray]]:
        nx, nu, N = 4, 2, self._params.horizon
        if u_prev is None:
            u_prev = np.zeros(nu)
        if u_init is None:
            u_init = np.tile(u_prev, (N, 1))

        X = cp.Variable((nx, N + 1))
        U = cp.Variable((nu, N))
        s_v = cp.Variable(N + 1)
        s_du = cp.Variable((nu, N))
        s_u = cp.Variable((nu, N))

        ref = np.copy(ref_traj)
        ref[:, 2] = np.unwrap(ref[:, 2])

        cost = 0
        constraints = [X[:, 0] == x0, s_v >= 0, s_du >= 0, s_u >= 0]

        xlin = ref[0]
        ulin = np.zeros(nu)
        for k in range(N):
            xref = ref[k]
            A, B, fx = linearize(xlin, ulin, self._params.dt, self._params.wheelbase_px)
            c_aff = fx - A @ xlin - B @ ulin
            constraints.append(X[:, k + 1] == A @ X[:, k] + B @ U[:, k] + c_aff)

            err = X[:, k] - xref
            cost += cp.quad_form(err, self._params.q)
            cost += cp.quad_form(U[:, k], self._params.r)
            cost += self._params.slack_velocity * cp.square(s_v[k])
            cost += self._params.slack_rate * cp.sum_squares(s_du[:, k])
            cost += self._params.slack_input * cp.sum_squares(s_u[:, k])

            constraints += [
                X[3, k] <= self._params.v_bounds[1] + s_v[k],
                X[3, k] >= self._params.v_bounds[0] - s_v[k],
                U[0, k] <= self._params.u_bounds[0][1] + s_u[0, k],
                U[0, k] >= self._params.u_bounds[0][0] - s_u[0, k],
                U[1, k] <= self._params.u_bounds[1][1] + s_u[1, k],
                U[1, k] >= self._params.u_bounds[1][0] - s_u[1, k],
            ]

            if k == 0:
                constraints += [
                    U[:, k] - u_prev
                    <= np.array([self._params.du_bounds[0][1], self._params.du_bounds[1][1]])
                    + s_du[:, k],
                    U[:, k] - u_prev
                    >= np.array([self._params.du_bounds[0][0], self._params.du_bounds[1][0]])
                    - s_du[:, k],
                ]
            else:
                constraints += [
                    U[:, k] - U[:, k - 1]
                    <= np.array([self._params.du_bounds[0][1], self._params.du_bounds[1][1]])
                    + s_du[:, k],
                    U[:, k] - U[:, k - 1]
                    >= np.array([self._params.du_bounds[0][0], self._params.du_bounds[1][0]])
                    - s_du[:, k],
                ]

            xlin = ref[k]
            ulin = np.zeros(nu)

        err_terminal = X[:, N] - ref[N]
        cost += cp.quad_form(err_terminal, self._params.q_terminal)
        cost += self._params.slack_velocity * cp.square(s_v[N])
        constraints += [
            X[3, N] <= self._params.v_bounds[1] + s_v[N],
            X[3, N] >= self._params.v_bounds[0] - s_v[N],
        ]

        problem = cp.Problem(cp.Minimize(cost), constraints)
        try:
            problem.solve(
                solver=cp.OSQP,
                warm_start=True,
                eps_abs=1e-3,
                eps_rel=1e-3,
                max_iter=60000,
                polish=True,
                adaptive_rho=True,
                rho=0.1,
                alpha=1.6,
                verbose=False,
            )
        except cp.SolverError:
            LOG.exception("OSQP failed during MPC solve")
            return None, None, None

        if problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE, "optimal", "optimal_inaccurate"):
            LOG.warning("MPC solve returned status %s", problem.status)
            return None, None, None

        return U.value[:, 0], X.value, U.value

    @property
    def params(self) -> MPCParameters:
        return self._params
