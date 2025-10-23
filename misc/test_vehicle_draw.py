"""Vehicle drawing primitives for matplotlib visualizations (fixed)."""
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
    length: float = 4.9  # default kept consistent with overhangs + wheelbase
    tire_radius: float = 0.4
    tire_width: float = 0.3
    wheel_track: float = 1.8
    rear_overhang: float = 1.0
    front_overhang: float = 1.0


def _rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _validate_params(p: VehicleParams) -> None:
    expected_len = p.rear_overhang + p.wheelbase + p.front_overhang
    if not math.isclose(p.length, expected_len, rel_tol=1e-6, abs_tol=1e-9):
        p.length = expected_len  # non-fatal correction


def draw_vehicle(x: float, y: float, yaw: float, steer: float, params: VehicleParams, *, color: str = "black") -> None:
    """Draw a planar vehicle footprint using the provided params."""
    _validate_params(params)
    plt = _get_pyplot()

    # Body spans from -rear_overhang to wheelbase + front_overhang in the vehicle frame
    x_front = params.wheelbase + params.front_overhang
    x_rear = -params.rear_overhang
    car = np.array(
        [
            [x_rear, x_rear, x_front, x_front, x_rear],
            [params.width / 2, -params.width / 2, -params.width / 2, params.width / 2, params.width / 2],
        ]
    )

    # Tire patches: short along x (tire width), long along y (2 * radius)
    wheel = np.array(
        [
            [-params.tire_width / 2, -params.tire_width / 2, params.tire_width / 2, params.tire_width / 2, -params.tire_width / 2],
            [ params.tire_radius,    -params.tire_radius,   -params.tire_radius,    params.tire_radius,    params.tire_radius],
        ]
    )

    # Create wheels in vehicle frame
    rl, rr, fr, fl = wheel.copy(), wheel.copy(), wheel.copy(), wheel.copy()
    rot_vehicle = _rot(yaw)
    rot_steer = _rot(steer)

    # Front wheels at x = wheelbase, rotated by steering
    fr = rot_steer @ fr + np.array([[params.wheelbase], [-params.wheel_track / 2]])
    fl = rot_steer @ fl + np.array([[params.wheelbase], [ params.wheel_track / 2]])

    # Rear wheels at x = 0, no steering
    rr[1, :] -= params.wheel_track / 2
    rl[1, :] += params.wheel_track / 2

    # Rotate the whole vehicle and translate to world
    parts = [car, fr, fl, rr, rl]
    parts = [rot_vehicle @ p for p in parts]
    offset = np.array([[x], [y]])

    for poly in parts:
        poly += offset
        plt.plot(poly[0, :], poly[1, :], color=color, linewidth=2)

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
