"""
Planar vehicle with 4WS (rear-axle steer via LUT) and UI.

Adds an on-figure info panel (read-only text field) showing:
- Front steer: degrees
- Velocity: km/h
- Rear steer: degrees

Sliders:
- Front steer (deg): [-30, +30]
- Velocity (m/s): [0, 30]
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


def _get_pyplot():
    import matplotlib.pyplot as plt
    return plt


# ----------------------------
# Parameters (original geometry)
# ----------------------------

@dataclass
class VehicleParams:
    length: float = 0.4
    width: float = 0.2
    rear_overhang: float = 0.1
    wheelbase: float = 0.25
    wheel_len: float = 0.03
    wheel_width: float = 0.02
    tread: float = 0.07  # half-track

    @property
    def front_overhang(self) -> float:
        return self.length - self.rear_overhang - self.wheelbase


def _validate_params(p: VehicleParams) -> None:
    if p.front_overhang < 0:
        p.length = p.rear_overhang + p.wheelbase


# ----------------------------
# Rear-axle steering LUT
# ----------------------------

_VEL_KNOTS = np.array([0.0, 2.0, 5.0, 10.0, 12.0, 18.0, 25.0, 30.0], dtype=float)
_GAIN_KNOTS = np.array([-0.35, -0.30, -0.20, -0.05, 0.0, 0.05, 0.10, 0.12], dtype=float)  # unitless
_REAR_DEG_LIMIT = 8.0  # [deg]


def rear_steer_from_table(vel_mps: float, front_steer_deg: float) -> float:
    g = float(np.interp(max(vel_mps, 0.0), _VEL_KNOTS, _GAIN_KNOTS))
    rear_deg = np.clip(g * front_steer_deg, -_REAR_DEG_LIMIT, _REAR_DEG_LIMIT)
    return math.radians(rear_deg)


# ----------------------------
# Geometry (keeps original rotation order)
# ----------------------------

def _vehicle_polys_original(p: VehicleParams, yaw: float, steer_front: float, steer_rear: float) -> List[np.ndarray]:
    _validate_params(p)

    x_front = p.length - p.rear_overhang
    x_rear = -p.rear_overhang
    outline = np.array(
        [
            [x_rear, x_front, x_front, x_rear, x_rear],
            [p.width / 2, p.width / 2, -p.width / 2, -p.width / 2, p.width / 2],
        ],
        dtype=float,
    )

    fr = np.array(
        [
            [ p.wheel_len, -p.wheel_len, -p.wheel_len,  p.wheel_len,  p.wheel_len],
            [-p.wheel_width - p.tread, -p.wheel_width - p.tread, p.wheel_width - p.tread,
              p.wheel_width - p.tread, -p.wheel_width - p.tread],
        ],
        dtype=float,
    )
    rr = fr.copy()
    fl = fr.copy(); fl[1, :] *= -1.0
    rl = rr.copy(); rl[1, :] *= -1.0

    Rot_yaw = np.array([[math.cos(yaw),  math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]], dtype=float)
    Rot_front = np.array([[math.cos(steer_front),  math.sin(steer_front)],
                          [-math.sin(steer_front), math.cos(steer_front)]], dtype=float)
    Rot_rear = np.array([[math.cos(steer_rear),  math.sin(steer_rear)],
                         [-math.sin(steer_rear), math.cos(steer_rear)]], dtype=float)

    fr = (fr.T @ Rot_front).T
    fl = (fl.T @ Rot_front).T
    rr = (rr.T @ Rot_rear).T
    rl = (rl.T @ Rot_rear).T

    fr[0, :] += p.wheelbase
    fl[0, :] += p.wheelbase

    outline = (outline.T @ Rot_yaw).T
    fr = (fr.T @ Rot_yaw).T
    rr = (rr.T @ Rot_yaw).T
    fl = (fl.T @ Rot_yaw).T
    rl = (rl.T @ Rot_yaw).T

    return [outline, fr, rr, fl, rl]


def _translate_all(parts: List[np.ndarray], x: float, y: float) -> List[np.ndarray]:
    for poly in parts:
        poly[0, :] += x
        poly[1, :] += y
    return parts


# ----------------------------
# Public API
# ----------------------------

def draw_vehicle(x: float, y: float, yaw: float, steer_front: float, vel_mps: float, params: VehicleParams, *, color_front="tab:blue", color_rear="tab:orange") -> None:
    plt = _get_pyplot()
    rear = rear_steer_from_table(vel_mps, math.degrees(steer_front))
    outline, fr, rr, fl, rl = _translate_all(_vehicle_polys_original(params, yaw, steer_front, rear), x, y)

    plt.plot(outline[0, :], outline[1, :], "-k", linewidth=2)
    plt.plot(fr[0, :], fr[1, :], color_front, linewidth=2)
    plt.plot(fl[0, :], fl[1, :], color_front, linewidth=2)
    plt.plot(rr[0, :], rr[1, :], color_rear, linewidth=2)
    plt.plot(rl[0, :], rl[1, :], color_rear, linewidth=2)
    plt.plot([x, x + params.wheelbase * math.cos(yaw)], [y, y + params.wheelbase * math.sin(yaw)], "--", color="0.4", linewidth=1.2)
    plt.plot(x, y, "*k")


# ----------------------------
# Original-API facade
# ----------------------------

_DEFAULT = VehicleParams()


def plot_car(x, y, yaw, steer: float = 0.0, truck_color: str = "-k"):
    plt = _get_pyplot()
    polys = _translate_all(_vehicle_polys_original(_DEFAULT, yaw, steer, 0.0), x, y)
    outline, fr, rr, fl, rl = polys
    plt.plot(outline[0, :], outline[1, :], truck_color, linewidth=2)
    plt.plot(fr[0, :], fr[1, :], truck_color, linewidth=2)
    plt.plot(rr[0, :], rr[1, :], truck_color, linewidth=2)
    plt.plot(fl[0, :], fl[1, :], truck_color, linewidth=2)
    plt.plot(rl[0, :], rl[1, :], truck_color, linewidth=2)
    plt.plot(x, y, "*k")


# ----------------------------
# Slider UI with read-only info panel
# ----------------------------

def _to_xy(poly: np.ndarray) -> np.ndarray:
    return np.vstack([poly[0, :], poly[1, :]]).T


def _fixed_axes(ax, p: VehicleParams):
    ax.set_aspect("equal", adjustable="box")
    x_min = -p.rear_overhang - 0.15
    x_max = p.length - p.rear_overhang + p.wheelbase + 0.15
    half_w = p.width * 0.75
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(-half_w - 0.35, half_w + 0.55)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_title("Planar Vehicle with 4WS (LUT rear steer)")


def demo_ui() -> None:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider
    from matplotlib.patches import Polygon

    p = _DEFAULT
    x0, y0, yaw0 = 0.0, 0.0, 0.0
    steer_deg0 = 0.0
    v0 = 0.0

    fig, ax = plt.subplots(figsize=(7, 5))
    plt.subplots_adjust(bottom=0.32)
    _fixed_axes(ax, p)

    # Initial geometry
    rear0 = rear_steer_from_table(v0, steer_deg0)
    parts = _translate_all(_vehicle_polys_original(p, yaw0, math.radians(steer_deg0), rear0), x0, y0)

    # Patches: body + 4 wheels
    colors = ["k", "tab:blue", "tab:orange", "tab:blue", "tab:orange"]
    patches: List[Polygon] = []
    for poly, col in zip(parts, colors):
        patch = Polygon(_to_xy(poly), closed=True, fill=False, lw=2, ec=col)
        ax.add_patch(patch)
        patches.append(patch)

    # Read-only info panel (text field)
    info_ax = fig.add_axes([0.15, 0.22, 0.70, 0.05])  # x, y, w, h in figure coords
    info_ax.axis("off")
    info_text = info_ax.text(
        0.5, 0.5,
        "",
        ha="center", va="center",
        fontfamily="monospace", fontsize=10,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="0.6")
    )

    def update_info(front_deg: float, vel_mps: float, rear_rad: float) -> None:
        vel_kmh = vel_mps * 3.6
        rear_deg = math.degrees(rear_rad)
        info_text.set_text(
            f"front: {front_deg:5.1f} deg   velocity: {vel_kmh:6.1f} km/h   rear: {rear_deg:5.1f} deg"
        )

    update_info(steer_deg0, v0, rear0)

    # Sliders
    ax_front = plt.axes([0.15, 0.16, 0.70, 0.03])
    ax_vel = plt.axes([0.15, 0.10, 0.70, 0.03])

    s_front = Slider(ax=ax_front, label="Front steer (deg)", valmin=-30.0, valmax=30.0, valinit=steer_deg0, valstep=0.1)
    s_vel = Slider(ax=ax_vel, label="Velocity (m/s)", valmin=0.0, valmax=30.0, valinit=v0, valstep=0.1)

    def on_change(_):
        front_deg = s_front.val
        vel_mps = s_vel.val
        rear = rear_steer_from_table(vel_mps, front_deg)
        new_parts = _translate_all(_vehicle_polys_original(p, yaw0, math.radians(front_deg), rear), x0, y0)
        for patch, poly in zip(patches, new_parts):
            patch.set_xy(_to_xy(poly))
        update_info(front_deg, vel_mps, rear)
        fig.canvas.draw_idle()

    s_front.on_changed(on_change)
    s_vel.on_changed(on_change)

    plt.show()


__all__: Tuple[str, ...] = ("VehicleParams", "draw_vehicle", "plot_car", "rear_steer_from_table", "demo_ui")

if __name__ == "__main__":
    demo_ui()
