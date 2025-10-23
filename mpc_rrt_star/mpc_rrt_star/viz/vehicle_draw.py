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

    @classmethod
    def from_wheelbase(cls, wheelbase: float) -> "VehicleParams":
        """Scale the reference geometry so the wheelbase matches ``wheelbase``.

        The default dimensions describe the compact planar vehicle shared by
        the user, with a reference wheelbase of 0.3 units. This helper scales
        the footprint proportionally so it can be expressed directly in the
        pixel space used by the simulator (typically metres divided by map
        resolution).
        """

        base = cls()
        if math.isclose(base.wheelbase, 0.0):  # pragma: no cover - defensive
            raise ValueError("Reference vehicle geometry has zero wheelbase")
        scale = wheelbase / base.wheelbase
        return cls(
            length=base.length * scale,
            width=base.width * scale,
            back_to_wheel=base.back_to_wheel * scale,
            wheel_length=base.wheel_length * scale,
            wheel_width=base.wheel_width * scale,
            tread=base.tread * scale,
        )


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
