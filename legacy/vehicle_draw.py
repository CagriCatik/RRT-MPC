#!/usr/bin/env python3
"""
vehicle_draw.py
Vehicle and arrow visualization utilities.
"""

import math
import numpy as np
import matplotlib.pyplot as plt


class VehicleParams:
    """Vehicle geometry."""
    def __init__(self):
        self.WB = 2.9   # Wheelbase [m]
        self.W = 2.0    # Car width [m]
        self.L = 4.5    # Car length [m]
        self.TR = 0.4   # Tire radius [m]
        self.TW = 0.3   # Tire width [m]
        self.WD = 1.8   # Wheel distance [m]
        self.RF = 1.0   # Front offset [m]
        self.RB = 1.0   # Rear offset [m]


class Arrow:
    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.4 * L
        w = 2

        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + math.pi - angle
        theta_hat_R = theta + math.pi + angle

        x_hat_end_L = x_end + d * np.cos(theta_hat_L)
        x_hat_end_R = x_end + d * np.cos(theta_hat_R)
        y_hat_end_L = y_end + d * np.sin(theta_hat_L)
        y_hat_end_R = y_end + d * np.sin(theta_hat_R)

        plt.plot([x, x_end], [y, y_end], color=c, linewidth=w)
        plt.plot([x_end, x_hat_end_L], [y_end, y_hat_end_L], color=c, linewidth=w)
        plt.plot([x_end, x_hat_end_R], [y_end, y_hat_end_R], color=c, linewidth=w)


def draw_car(x, y, yaw, steer, params: VehicleParams, color='black'):
    car = np.array([[-params.RB, -params.RB, params.RF, params.RF, -params.RB],
                    [params.W / 2, -params.W / 2, -params.W / 2, params.W / 2, params.W / 2]])

    wheel = np.array([[-params.TR, -params.TR, params.TR, params.TR, -params.TR],
                      [params.TW / 4, -params.TW / 4, -params.TW / 4, params.TW / 4, params.TW / 4]])

    rlWheel, rrWheel, frWheel, flWheel = wheel.copy(), wheel.copy(), wheel.copy(), wheel.copy()

    Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])

    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[params.WB], [-params.WD / 2]])
    flWheel += np.array([[params.WB], [params.WD / 2]])
    rrWheel[1, :] -= params.WD / 2
    rlWheel[1, :] += params.WD / 2

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)
    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)

    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    Arrow(x, y, yaw, params.WB * 0.6, color)
