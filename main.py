#!/usr/bin/env python3
"""
main_mpc.py
RRT* + MPC tracking (single-track) end-to-end pipeline with prediction visualization.
"""

import logging
import time
import numpy as np
import matplotlib.pyplot as plt
from cv2 import imwrite
from map_utils import load_map, inflate_map, to_occupancy_grid
from rrt_star import rrt_star
from visualization import visualize_rrt_star, animate_vehicle_states
from mpc_controller import MPC
from vehicle_model import f_discrete

# ===============================================================
# LOGGING
# ===============================================================
logging.basicConfig(
    level=logging.INFO,  # Change to DEBUG for more verbose output
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%H:%M:%S",
)
LOG = logging.getLogger("main_mpc")

# ===============================================================
# CONFIG
# ===============================================================
CONFIG = {
    "map_file": "occupancy_grid_inflated.png",
    "inflated_map_file": "occupancy_grid_inflated.png",
    "map_resolution": 0.2,       # m/pixel
    "inflation_radius_m": 0.6,
    "start": (50, 50),
    "goal_offset": (50, 50),
    # RRT*
    "step_size": 10,
    "goal_radius": 15,
    "max_iterations": 3000,
    "rewire_radius": 25,
    "goal_sample_rate": 0.1,
    # MPC (robust defaults)
    "WB_m": 2.8,
    "dt": 0.1,
    "horizon": 12,
    "v_px_s": 28.0,
    "sim_steps": 600,
    # Viz
    "pred_pause": 0.05
}
# ===============================================================

def resample_path(path, step_px=3.0):
    pts = np.asarray(path, float)
    if len(pts) < 2:
        LOG.debug("resample_path: fewer than 2 points, returning original")
        return pts
    seg = np.diff(pts, axis=0)
    d = np.hypot(seg[:,0], seg[:,1])
    s = np.insert(np.cumsum(d), 0, 0.0)
    L = s[-1]
    if L < 1e-6:
        LOG.debug("resample_path: total length ~0, returning original")
        return pts
    s_q = np.arange(0.0, L, step_px)
    if s_q[-1] != L:
        s_q = np.append(s_q, L)
    xq = np.interp(s_q, s, pts[:,0])
    yq = np.interp(s_q, s, pts[:,1])
    LOG.debug("resample_path: input=%d pts, output=%d pts, step_px=%.3f", len(pts), len(xq), step_px)
    return np.column_stack([xq, yq])

def build_reference_from_path(path, v_des_px_s, N, dt):
    # spacing ~= 0.8 * one-step travel
    step_px = max(2.0, 0.8 * v_des_px_s * dt)
    pts = resample_path(path, step_px)

    dirs = np.diff(pts, axis=0, prepend=pts[0:1])
    yaws = np.unwrap(np.arctan2(dirs[:,1], dirs[:,0]))

    # curvature-aware speed taper
    hd = np.abs(np.diff(yaws, prepend=yaws[0]))
    hd = np.minimum(hd, np.pi - hd)
    slow = 1.0 / (1.0 + 4.0 * hd)  # 0..1
    vref = v_des_px_s * (0.6 + 0.4 * slow)

    xref_all = np.column_stack([pts[:,0], pts[:,1], yaws, vref])
    if len(xref_all) < N + 1:
        xref_all = np.vstack([xref_all, np.repeat(xref_all[-1:], N+1-len(xref_all), axis=0)])
    LOG.info("Reference built: %d points (needed at least %d). step_px=%.3f, v_des=%.2f px/s",
             len(xref_all), N + 1, step_px, v_des_px_s)
    return xref_all

def plot_prediction(occ, path, Xp, xk, step_idx, pause_s):
    if Xp is None:
        LOG.debug("plot_prediction: Xp is None, skipping visualization at step %d", step_idx)
        return
    plt.ion()
    plt.cla()
    plt.imshow(occ, cmap="gray", origin="lower")
    p = np.array(path)
    plt.plot(p[:,0], p[:,1], "r-", linewidth=2, label="Planned Path")
    plt.plot(xk[0], xk[1], "bo", markersize=5, label="Vehicle")
    plt.plot(Xp[0, :], Xp[1, :], "go--", linewidth=1.5, markersize=4, label="Predicted")
    plt.xlim(0, occ.shape[1]); plt.ylim(0, occ.shape[0])
    plt.title(f"MPC Step {step_idx}")
    plt.legend(loc="upper right")
    plt.pause(pause_s)

def main():
    cfg = CONFIG
    LOG.info("=== RRT* + MPC PIPELINE START ===")
    LOG.info("Config: %s", {k: cfg[k] for k in sorted(cfg.keys())})

    # Load and inflate
    t0 = time.time()
    try:
        img = load_map(cfg["map_file"])
        LOG.info("Map loaded: %s", cfg["map_file"])
    except Exception:
        LOG.exception("Failed to load map: %s", cfg["map_file"])
        return

    try:
        inflated_img = inflate_map(img, cfg["inflation_radius_m"], cfg["map_resolution"])
        LOG.info("Map inflated: radius=%.3f m, resolution=%.3f m/px", cfg["inflation_radius_m"], cfg["map_resolution"])
        imwrite(cfg["inflated_map_file"], inflated_img)
        LOG.info("Inflated map written: %s", cfg["inflated_map_file"])
    except Exception:
        LOG.exception("Failed to inflate or save map")
        return

    occ = to_occupancy_grid(inflated_img)
    occ = np.flipud(occ)
    LOG.info("Occupancy grid ready: shape=%s (flipped Y). Prep time=%.3f s", tuple(occ.shape), time.time() - t0)

    # RRT*
    start = cfg["start"]
    goal = (occ.shape[1] - cfg["goal_offset"][0],
            occ.shape[0] - cfg["goal_offset"][1])
    LOG.info("Planning start=%s goal=%s", start, goal)
    LOG.info("RRT* params: step=%d, goal_radius=%d, max_iter=%d, rewire_radius=%d, goal_sample_rate=%.2f",
             cfg["step_size"], cfg["goal_radius"], cfg["max_iterations"], cfg["rewire_radius"], cfg["goal_sample_rate"])
    t_rrt = time.time()
    nodes, goal_node = rrt_star(
        occ, start, goal,
        step=cfg["step_size"],
        goal_radius=cfg["goal_radius"],
        max_iter=cfg["max_iterations"],
        rewire_radius=cfg["rewire_radius"],
        goal_prob=cfg["goal_sample_rate"]
    )
    LOG.info("RRT* completed in %.3f s. nodes=%d goal_node=%s", time.time() - t_rrt, len(nodes), str(goal_node))

    # Visualize and get path
    path = visualize_rrt_star(
        occ, start, goal, nodes, goal_node, delay=0.0005, save_path="rrtstar_path.png"
    )
    if not path:
        LOG.error("No path from RRT*. Abort.")
        return
    LOG.info("Path extracted: %d waypoints. Saved snapshot rrtstar_path.png", len(path))

    # MPC setup
    WB_px = cfg["WB_m"] / cfg["map_resolution"]
    mpc = MPC(WB_px, dt=cfg["dt"], N=cfg["horizon"])
    LOG.info("MPC initialized: WB=%.3f px (%.3f m), dt=%.3f, horizon=%d", WB_px, cfg["WB_m"], cfg["dt"], cfg["horizon"])

    # Initial state
    if len(path) > 1:
        yaw0 = np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    else:
        yaw0 = 0.0
    xk = np.array([start[0], start[1], yaw0, 5.0], dtype=float)
    u_prev = np.zeros(2)
    LOG.info("Initial state x0=%s, u_prev=%s", np.array2string(xk, precision=3), np.array2string(u_prev, precision=3))

    # Reference
    xref_global = build_reference_from_path(path, cfg["v_px_s"], cfg["horizon"], cfg["dt"])

    # Online loop
    states = []
    path_idx = 0
    t_loop = time.time()
    for t in range(cfg["sim_steps"]):
        end = min(path_idx + mpc.N + 1, len(xref_global))
        xref_win = xref_global[path_idx:end]
        if len(xref_win) < mpc.N + 1:
            xref_win = np.vstack([xref_win, np.repeat(xref_win[-1:], mpc.N + 1 - len(xref_win), axis=0)])

        LOG.debug("Step %d: path_idx=%d window=[%d:%d] xk=%s",
                  t, path_idx, path_idx, end, np.array2string(xk, precision=3))

        # Nominal solve
        u0, Xp, Up = mpc.solve(xk, xref_win, u_prev=u_prev)

        # Backoff if infeasible
        if u0 is None:
            LOG.warning("MPC infeasible at step %d. Applying relaxed bounds and reduced speed.", t)
            xref_relaxed = xref_win.copy()
            xref_relaxed[:, 3] *= 0.6
            old_du = mpc.du_bounds
            mpc.du_bounds = ((old_du[0][0]-5.0, old_du[0][1]+5.0),
                             (old_du[1][0]-0.05, old_du[1][1]+0.05))
            u0, Xp, Up = mpc.solve(xk, xref_relaxed, u_prev=u_prev)
            mpc.du_bounds = old_du
            if u0 is None:
                LOG.error("MPC infeasible after relaxation at step %d. Aborting control loop.", t)
                break
            else:
                LOG.info("MPC recovered with relaxed settings at step %d.", t)

        # Show prediction
        plot_prediction(occ, path, Xp, xk, t, cfg["pred_pause"])

        # Apply first control
        xk = f_discrete(xk, u0, mpc.dt, WB_px)
        states.append(xk.copy())
        LOG.debug("Applied control u0=%s -> xk+1=%s",
                  np.array2string(u0, precision=3), np.array2string(xk, precision=3))
        u_prev = u0.copy()

        # Advance along path
        if path_idx < len(xref_global) - 2:
            dx = xk[0] - xref_global[path_idx][0]
            dy = xk[1] - xref_global[path_idx][1]
            if (dx*dx + dy*dy) > 25.0:  # > 5 px
                path_idx += 1
                LOG.debug("Advance path_idx -> %d", path_idx)

        # Stop near goal
        if np.hypot(xk[0] - goal[0], xk[1] - goal[1]) < 8.0:
            LOG.info("Reached goal region at step %d. pos=(%.2f, %.2f)", t, xk[0], xk[1])
            break

    LOG.info("Control loop finished. Steps executed: %d. Loop time: %.3f s", len(states), time.time() - t_loop)

    # Animate tracked states
    if states:
        LOG.info("Animating %d states.", len(states))
        animate_vehicle_states(occ, states, color="blue", delay=0.02)
    else:
        LOG.warning("No states recorded. Skipping animation.")

    LOG.info("=== PIPELINE COMPLETE ===")

if __name__ == "__main__":
    try:
        main()
    except Exception:
        LOG.exception("Unhandled exception in main")
        raise
