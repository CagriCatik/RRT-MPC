#!/usr/bin/env python3
# QP-only MPC with robust slacks (OSQP-compatible)

import numpy as np
import cvxpy as cp
from vehicle_model import linearize

class MPC:
    def __init__(self, WB_px, dt=0.1, N=12,
                 Q=np.diag([4.0, 4.0, 0.6, 0.1]),
                 R=np.diag([0.03, 0.25]),
                 QN=np.diag([8.0, 8.0, 1.0, 0.2]),
                 u_bounds=((-35.0, 35.0), (-0.60, 0.60)),   # a [px/s^2], delta [rad]
                 v_bounds=(0.0, 90.0),                      # v [px/s]
                 du_bounds=((-12.0, 12.0), (-0.15, 0.15))   # rate limits
                 ):
        self.WB_px = WB_px
        self.dt = dt
        self.N = N
        self.Q, self.R, self.QN = Q, R, QN
        self.u_bounds = u_bounds
        self.v_bounds = v_bounds
        self.du_bounds = du_bounds

        # slack weights (quadratic; keep QP)
        self.w_sv   = 1e3    # velocity bound slack
        self.w_sdel = 5e2    # input bound slack
        self.w_sdu  = 5e2    # rate bound slack

    def solve(self, x0, ref_traj, u_init=None, u_prev=np.zeros(2)):
        """
        QP-only MPC. ref_traj shape: (N+1, 4) -> [x,y,yaw,v].
        Returns (u0, X_pred, U_pred) or (None, None, None) if infeasible.
        """
        nx, nu, N = 4, 2, self.N
        if u_init is None:
            u_init = np.tile(u_prev, (N, 1))

        # decision variables
        X    = cp.Variable((nx, N+1))
        U    = cp.Variable((nu, N))
        s_v  = cp.Variable(N+1)        # vel slack >= 0
        s_du = cp.Variable((nu, N))    # rate slacks >= 0
        s_u  = cp.Variable((nu, N))    # input slacks >= 0

        ref = ref_traj.copy()
        ref[:, 2] = np.unwrap(ref[:, 2])

        cost, cons = 0, []
        cons += [X[:, 0] == x0, s_v[0] >= 0, s_du >= 0, s_u >= 0]

        # linearize once around the reference window (LTV)
        xlin = ref[0]; ulin = np.zeros(2)

        for k in range(N):
            xref = ref[k]
            A, B, fx = linearize(xlin, ulin, self.dt, self.WB_px)
            c_aff = fx - A @ xlin - B @ ulin

            cons += [X[:, k+1] == A @ X[:, k] + B @ U[:, k] + c_aff,
                     s_v[k+1] >= 0]

            e = X[:, k] - xref
            cost += cp.quad_form(e, self.Q) \
                    + cp.quad_form(U[:, k], self.R) \
                    + self.w_sv*cp.square(s_v[k]) \
                    + self.w_sdu*cp.sum_squares(s_du[:, k]) \
                    + self.w_sdel*cp.sum_squares(s_u[:, k])

            # soft velocity bounds
            cons += [X[3, k] <= self.v_bounds[1] + s_v[k],
                     X[3, k] >= self.v_bounds[0] - s_v[k]]

            # soft input bounds
            cons += [U[0, k] <= self.u_bounds[0][1] + s_u[0, k],
                     U[0, k] >= self.u_bounds[0][0] - s_u[0, k]]
            cons += [U[1, k] <= self.u_bounds[1][1] + s_u[1, k],
                     U[1, k] >= self.u_bounds[1][0] - s_u[1, k]]

            # soft rate limits
            if k == 0:
                cons += [U[:, k] - u_prev <= [self.du_bounds[0][1], self.du_bounds[1][1]] + s_du[:, k],
                         U[:, k] - u_prev >= [self.du_bounds[0][0], self.du_bounds[1][0]] - s_du[:, k]]
            else:
                cons += [U[:, k] - U[:, k-1] <= [self.du_bounds[0][1], self.du_bounds[1][1]] + s_du[:, k],
                         U[:, k] - U[:, k-1] >= [self.du_bounds[0][0], self.du_bounds[1][0]] - s_du[:, k]]

            xlin = ref[k]; ulin = np.zeros(2)

        eN = X[:, N] - ref[N]
        cost += cp.quad_form(eN, self.QN) + self.w_sv*cp.square(s_v[N])
        cons += [X[3, N] <= self.v_bounds[1] + s_v[N],
                 X[3, N] >= self.v_bounds[0] - s_v[N]]

        prob = cp.Problem(cp.Minimize(cost), cons)
        prob.solve(
            solver=cp.OSQP,
            warm_start=True,
            eps_abs=1e-3, eps_rel=1e-3,
            max_iter=60000,
            polish=True,
            adaptive_rho=True,
            rho=0.1,
            alpha=1.6,
            verbose=False,
        )

        if prob.status not in ("optimal", "optimal_inaccurate"):
            return None, None, None

        return U.value[:, 0], X.value, U.value
