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
    """Geometric parameters for the planar vehicle drawing."""

    length: float = 0.4
    width: float = 0.2
    back_to_wheel: float = 0.1
    wheel_length: float = 0.03
    wheel_width: float = 0.02
    tread: float = 0.07

    @property
    def wheelbase(self) -> float:
        """Distance between the front and rear axle centres."""

        return self.length - self.back_to_wheel


def _rot(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]])


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
    """Draw a planar vehicle footprint using :mod:`matplotlib` primitives."""

    plt = _get_pyplot()
    axes = ax if ax is not None else plt.gca()

    outline = np.array(
        [
            [
                -params.back_to_wheel,
                params.length - params.back_to_wheel,
                params.length - params.back_to_wheel,
                -params.back_to_wheel,
                -params.back_to_wheel,
            ],
            [
                params.width / 2,
                params.width / 2,
                -params.width / 2,
                -params.width / 2,
                params.width / 2,
            ],
        ]
    )

    fr_wheel = np.array(
        [
            [
                params.wheel_length,
                -params.wheel_length,
                -params.wheel_length,
                params.wheel_length,
                params.wheel_length,
            ],
            [
                -params.wheel_width - params.tread,
                -params.wheel_width - params.tread,
                params.wheel_width - params.tread,
                params.wheel_width - params.tread,
                -params.wheel_width - params.tread,
            ],
        ]
    )

    rr_wheel = fr_wheel.copy()
    fl_wheel = fr_wheel.copy()
    rl_wheel = rr_wheel.copy()

    fl_wheel[1, :] *= -1
    rl_wheel[1, :] *= -1

    rot_body = _rot(yaw)
    rot_steer = _rot(steer)

    fr_wheel = rot_steer @ fr_wheel
    fl_wheel = rot_steer @ fl_wheel

    fr_wheel[0, :] += params.wheelbase
    fl_wheel[0, :] += params.wheelbase

    fr_wheel = rot_body @ fr_wheel
    fl_wheel = rot_body @ fl_wheel
    rr_wheel = rot_body @ rr_wheel
    rl_wheel = rot_body @ rl_wheel
    outline = rot_body @ outline

    offset = np.array([[x], [y]])
    artists: List[Line2D] = []
    for poly in (outline, fr_wheel, rr_wheel, fl_wheel, rl_wheel):
        poly = poly + offset
        (line,) = axes.plot(poly[0, :], poly[1, :], color)
        artists.append(line)

    (center,) = axes.plot([x], [y], "*", color=color)
    artists.append(center)

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
