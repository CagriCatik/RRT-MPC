#!/usr/bin/env python3
"""
vehicle_model.py
Kinematic single-track (bicycle) model + linearization in pixel units.
State: x = [x, y, yaw, v]   (pixels, pixels, rad, pixels/sec)
Input: u = [a, delta]       (pixels/sec^2, rad)
"""

import numpy as np

def f_discrete(x, u, dt, WB_px):
    xk, yk, yaw, v = x
    a, delta = u
    beta = 0.0  # kinematic bicycle, front-steer small-angle
    xk1   = xk + dt * v * np.cos(yaw + beta)
    yk1   = yk + dt * v * np.sin(yaw + beta)
    yaw1  = yaw + dt * (v / WB_px) * np.tan(delta)
    v1    = v + dt * a
    return np.array([xk1, yk1, yaw1, v1])

def linearize(x, u, dt, WB_px):
    # x = [x, y, yaw, v], u = [a, delta]
    xk, yk, yaw, v = x
    a, delta = u
    c, s = np.cos(yaw), np.sin(yaw)
    tan_d = np.tan(delta); sec2_d = 1.0 / (np.cos(delta)**2 + 1e-9)

    A = np.eye(4)
    A[0,2] = -dt * v * s         # d x_next / d yaw
    A[0,3] =  dt * c             # d x_next / d v
    A[1,2] =  dt * v * c         # d y_next / d yaw
    A[1,3] =  dt * s             # d y_next / d v
    A[2,3] =  dt * (1.0 / WB_px) * tan_d  # d yaw_next / d v

    B = np.zeros((4,2))
    B[3,0] = dt                                  # d v_next / d a
    B[2,1] = dt * (v / WB_px) * sec2_d           # d yaw_next / d delta

    # affine term for discrete-time linearization about (x,u)
    fx = f_discrete(x, u, dt, WB_px)
    return A, B, fx
