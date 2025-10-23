"""Vehicle drawing primitives for matplotlib visualisations."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np


try:
    from matplotlib.axes import Axes
    from matplotlib.lines import Line2D
except Exception:  # pragma: no cover - matplotlib is optional at import time
    Axes = "Axes"  # type: ignore
    Line2D = "Line2D"  # type: ignore


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


def draw_vehicle(
    x: float,
    y: float,
    yaw: float,
    steer: float,
    params: VehicleParams,
    *,
    color: str = "black",
    ax: Optional[Axes] = None,
) -> List[Line2D]:
    """Draw a planar vehicle footprint using the provided ``params``.

    Parameters
    ----------
    x, y, yaw, steer:
        State of the vehicle in map coordinates.
    params:
        Geometric vehicle parameters used to generate the footprint.
    color:
        Matplotlib-compatible colour for the outline.
    ax:
        Optional axes object to draw onto. When omitted, the current axes
        obtained from :mod:`matplotlib.pyplot` are used.

    Returns
    -------
    list[Line2D]
        Handles to the plotted line artists so callers can update or remove
        them later when animating.
    """

    plt = _get_pyplot()
    axes = ax if ax is not None else plt.gca()

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
    artists: List[Line2D] = []
    for poly in (car, fr, fl, rr, rl):
        poly += offset
        (line,) = axes.plot(poly[0, :], poly[1, :], color)
        artists.append(line)

    artists.extend(draw_arrow(x, y, yaw, params.wheelbase * 0.6, color, ax=axes))
    return artists


def draw_arrow(
    x: float,
    y: float,
    theta: float,
    length: float,
    color: str,
    *,
    ax: Optional[Axes] = None,
) -> List[Line2D]:
    plt = _get_pyplot()
    axes = ax if ax is not None else plt.gca()

    angle = math.radians(30)
    dx = length * math.cos(theta)
    dy = length * math.sin(theta)
    x_end = x + dx
    y_end = y + dy

    artists: List[Line2D] = []
    (shaft,) = axes.plot([x, x_end], [y, y_end], color=color, linewidth=2)
    artists.append(shaft)
    left_theta = theta + math.pi - angle
    right_theta = theta + math.pi + angle
    left = (x_end + 0.4 * length * math.cos(left_theta), y_end + 0.4 * length * math.sin(left_theta))
    right = (x_end + 0.4 * length * math.cos(right_theta), y_end + 0.4 * length * math.sin(right_theta))
    (left_line,) = axes.plot([x_end, left[0]], [y_end, left[1]], color=color, linewidth=2)
    (right_line,) = axes.plot([x_end, right[0]], [y_end, right[1]], color=color, linewidth=2)
    artists.extend((left_line, right_line))
    return artists


__all__ = ["VehicleParams", "draw_vehicle", "draw_arrow"]
