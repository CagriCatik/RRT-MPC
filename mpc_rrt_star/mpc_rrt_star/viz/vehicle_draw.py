"""Vehicle drawing primitives for matplotlib visualisations."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _get_pyplot():
    import matplotlib.pyplot as plt

    return plt


@dataclass
class VehicleParams:
    wheelbase: float = 2.9
    width: float = 2.0
    length: float = 4.5
    tire_radius: float = 0.4
    tire_width: float = 0.3
    wheel_track: float = 1.8
    rear_overhang: float = 1.0
    front_overhang: float = 1.0


def _rot(theta: float) -> np.ndarray:
    return np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])


def draw_vehicle(x: float, y: float, yaw: float, steer: float, params: VehicleParams, *, color: str = "black") -> None:
    """Draw a planar vehicle footprint using the provided ``params``."""

    plt = _get_pyplot()

    car = np.array(
        [
            [-params.rear_overhang, -params.rear_overhang, params.front_overhang, params.front_overhang, -params.rear_overhang],
            [params.width / 2, -params.width / 2, -params.width / 2, params.width / 2, params.width / 2],
        ]
    )

    wheel = np.array(
        [
            [-params.tire_radius, -params.tire_radius, params.tire_radius, params.tire_radius, -params.tire_radius],
            [params.tire_width / 4, -params.tire_width / 4, -params.tire_width / 4, params.tire_width / 4, params.tire_width / 4],
        ]
    )

    rl, rr, fr, fl = wheel.copy(), wheel.copy(), wheel.copy(), wheel.copy()
    rot_vehicle = _rot(yaw)
    rot_steer = _rot(steer)

    fr = rot_steer @ fr + np.array([[params.wheelbase], [-params.wheel_track / 2]])
    fl = rot_steer @ fl + np.array([[params.wheelbase], [params.wheel_track / 2]])
    rr[1, :] -= params.wheel_track / 2
    rl[1, :] += params.wheel_track / 2

    fr = rot_vehicle @ fr
    fl = rot_vehicle @ fl
    rr = rot_vehicle @ rr
    rl = rot_vehicle @ rl
    car = rot_vehicle @ car

    offset = np.array([[x], [y]])
    for poly in (car, fr, fl, rr, rl):
        poly += offset
        plt.plot(poly[0, :], poly[1, :], color)

    draw_arrow(x, y, yaw, params.wheelbase * 0.6, color)


def draw_arrow(x: float, y: float, theta: float, length: float, color: str) -> None:
    plt = _get_pyplot()

    angle = math.radians(30)
    dx = length * math.cos(theta)
    dy = length * math.sin(theta)
    x_end = x + dx
    y_end = y + dy

    plt.plot([x, x_end], [y, y_end], color=color, linewidth=2)
    left_theta = theta + math.pi - angle
    right_theta = theta + math.pi + angle
    left = (x_end + 0.4 * length * math.cos(left_theta), y_end + 0.4 * length * math.sin(left_theta))
    right = (x_end + 0.4 * length * math.cos(right_theta), y_end + 0.4 * length * math.sin(right_theta))
    plt.plot([x_end, left[0]], [y_end, left[1]], color=color, linewidth=2)
    plt.plot([x_end, right[0]], [y_end, right[1]], color=color, linewidth=2)


__all__ = ["VehicleParams", "draw_vehicle", "draw_arrow"]
