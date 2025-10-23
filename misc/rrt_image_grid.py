# rrt_image_grid_cv.py
# RRT with Catmull-Rom spline on a binary occupancy grid loaded and processed using OpenCV.
# Includes obstacle inflation (cv2.dilate), start/goal snapping, and inversion control.
#
# Requirements:
#   pip install numpy matplotlib opencv-python
#
# Example:
#   python rrt_image_grid_cv.py --map occupancy_grid.png --start 20 20 --goal 700 700 --robot_radius_px 8
#   python rrt_image_grid_cv.py --map occupancy_grid_inflated.png --skip_inflation --start 20 20 --goal 700 700

import math
import random
import argparse
from typing import Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt
import cv2


# ============================================================
# Utility: OpenCV-based binary dilation
# ============================================================

def _disk_kernel(radius_px: int) -> np.ndarray:
    r = max(0, int(radius_px))
    if r == 0:
        return np.array([[1]], dtype=np.uint8)
    Y, X = np.ogrid[-r:r+1, -r:r+1]
    mask = (X*X + Y*Y) <= (r*r)
    return mask.astype(np.uint8)  # values 0/1; cv2 will treat >0 as kernel elements


def binary_dilate_cv(image_bool: np.ndarray, radius_px: int) -> np.ndarray:
    """
    OpenCV dilation on a boolean obstacle mask.
    image_bool: bool array where True=obstacle, False=free.
    radius_px: disk radius in pixels.
    """
    if radius_px <= 0:
        return image_bool.copy()
    kernel = (_disk_kernel(radius_px) * 255).astype(np.uint8)
    src = image_bool.astype(np.uint8) * 255
    dst = cv2.dilate(src, kernel, iterations=1)
    return (dst > 0)


# ============================================================
# RRT
# ============================================================

class Node:
    __slots__ = ("x", "y", "parent")
    def __init__(self, x: float, y: float, parent: Optional["Node"]=None):
        self.x = float(x)
        self.y = float(y)
        self.parent = parent

class RRT:
    def __init__(self, start_xy, goal_xy, occ_grid,
                 step=5.0, goal_sample_rate=0.05, max_iter=20000,
                 goal_tolerance=4.0, collision_step=1.0, rng_seed=7):
        self.start = Node(*start_xy)
        self.goal = Node(*goal_xy)
        self.occ = occ_grid.astype(bool)
        self.H, self.W = occ_grid.shape
        self.step = float(step)
        self.goal_sample_rate = float(goal_sample_rate)
        self.max_iter = int(max_iter)
        self.goal_tolerance = float(goal_tolerance)
        self.collision_step = float(collision_step)
        self.nodes = [self.start]
        self.rng = random.Random(rng_seed)

    def _inside(self, x, y) -> bool:
        return (0 <= x < self.W) and (0 <= y < self.H)

    def _is_occupied(self, x, y) -> bool:
        if not self._inside(x, y):
            return True
        ix = int(round(x))
        iy = int(round(y))
        ix = min(max(ix, 0), self.W - 1)
        iy = min(max(iy, 0), self.H - 1)
        return bool(self.occ[iy, ix])

    def _collides_segment(self, a, b) -> bool:
        ax, ay = a
        bx, by = b
        dx = bx - ax
        dy = by - ay
        dist = math.hypot(dx, dy)
        if dist == 0:
            return self._is_occupied(ax, ay)
        steps = max(1, int(dist / max(self.collision_step, 1e-6)))
        for i in range(steps + 1):
            t = i / steps
            x = ax + t * dx
            y = ay + t * dy
            if self._is_occupied(x, y):
                return True
        return False

    @staticmethod
    def _dist(a: "Node", b: "Node") -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _nearest(self, x, y) -> "Node":
        best = None
        best_d = float("inf")
        for n in self.nodes:
            d = (n.x - x)*(n.x - x) + (n.y - y)*(n.y - y)
            if d < best_d:
                best_d = d
                best = n
        return best  # type: ignore

    def _sample(self):
        if self.rng.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        return (self.rng.uniform(0, self.W - 1), self.rng.uniform(0, self.H - 1))

    def _steer(self, n: "Node", px: float, py: float) -> "Node":
        dx = px - n.x
        dy = py - n.y
        d = math.hypot(dx, dy)
        if d == 0:
            return Node(n.x, n.y, n)
        s = min(self.step, d) / d
        return Node(n.x + dx * s, n.y + dy * s, n)

    def plan(self) -> Optional[np.ndarray]:
        if self._is_occupied(self.start.x, self.start.y):
            raise ValueError("Start lies in obstacle.")
        if self._is_occupied(self.goal.x, self.goal.y):
            raise ValueError("Goal lies in obstacle.")

        for _ in range(self.max_iter):
            rx, ry = self._sample()
            near = self._nearest(rx, ry)
            new = self._steer(near, rx, ry)
            if not self._collides_segment((near.x, near.y), (new.x, new.y)):
                self.nodes.append(new)
                if self._dist(new, self.goal) <= self.goal_tolerance:
                    if not self._collides_segment((new.x, new.y), (self.goal.x, self.goal.y)):
                        goal_node = Node(self.goal.x, self.goal.y, new)
                        self.nodes.append(goal_node)
                        return self._extract_path(goal_node)
        return None

    @staticmethod
    def _extract_path(node: "Node") -> np.ndarray:
        pts = []
        cur = node
        while cur is not None:
            pts.append((cur.x, cur.y))
            cur = cur.parent
        pts.reverse()
        return np.array(pts, dtype=float)


# ============================================================
# Catmull-Rom spline (robust)
# ============================================================

def _dedupe_consecutive(points, tol=1e-9):
    P = np.asarray(points, dtype=float)
    if len(P) == 0:
        return P
    out = [P[0]]
    for i in range(1, len(P)):
        if np.linalg.norm(P[i] - out[-1]) > tol:
            out.append(P[i])
    return np.asarray(out)

def catmull_rom_spline(points, samples_per_segment=20, alpha=0.5, eps=1e-9):
    P = _dedupe_consecutive(points)
    n = len(P)
    if n < 2:
        return P
    if n == 2:
        t = np.linspace(0.0, 1.0, samples_per_segment + 1)
        return (1 - t)[:, None] * P[0] + t[:, None] * P[1]
    P_ext = np.vstack([P[0], P, P[-1]])

    def tj(ti, pi, pj):
        d = np.linalg.norm(pj - pi)
        inc = (d ** alpha) if d > eps else eps
        return ti + inc

    out = []
    for i in range(0, n - 1):
        p0, p1, p2, p3 = P_ext[i], P_ext[i + 1], P_ext[i + 2], P_ext[i + 3]
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)

        dt01 = max(t1 - t0, eps)
        dt12 = max(t2 - t1, eps)
        dt23 = max(t3 - t2, eps)
        dt02 = max(t2 - t0, eps)
        dt13 = max(t3 - t1, eps)

        t_vals = np.linspace(t1, t2, samples_per_segment, endpoint=False)
        for t in t_vals:
            A1 = (t1 - t)/dt01*p0 + (t - t0)/dt01*p1
            A2 = (t2 - t)/dt12*p1 + (t - t1)/dt12*p2
            A3 = (t3 - t)/dt23*p2 + (t - t2)/dt23*p3
            B1 = (t2 - t)/dt02*A1 + (t - t0)/dt02*A2
            B2 = (t3 - t)/dt13*A2 + (t - t1)/dt13*A3
            C  = (t2 - t)/dt12*B1 + (t - t1)/dt12*B2
            out.append(C)
    out.append(P[-1])
    return np.asarray(out)


# ============================================================
# Prune + utilities
# ============================================================

def prune_path(path: Optional[np.ndarray], occ: np.ndarray, collision_step=1.0) -> Optional[np.ndarray]:
    if path is None or len(path) <= 2:
        return path

    H, W = occ.shape

    def occupied(x, y) -> bool:
        if not (0 <= x < W and 0 <= y < H):
            return True
        ix, iy = int(round(x)), int(round(y))
        ix = min(max(ix, 0), W-1)
        iy = min(max(iy, 0), H-1)
        return bool(occ[iy, ix])

    def blocked(a, b) -> bool:
        ax, ay = a; bx, by = b
        dx, dy = bx - ax, by - ay
        dist = math.hypot(dx, dy)
        steps = max(1, int(dist / max(collision_step, 1e-6)))
        for i in range(steps + 1):
            t = i / steps
            x = ax + t * dx
            y = ay + t * dy
            if occupied(x, y):
                return True
        return False

    pts = [tuple(p) for p in path]
    i = 0
    pruned = [pts[0]]
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and blocked(pts[i], pts[j]):
            j -= 1
        pruned.append(pts[j])
        i = j
    return np.array(pruned, dtype=float)


def nearest_free(occ: np.ndarray, x: float, y: float, r_max: int = 2000) -> Tuple[float, float]:
    H, W = occ.shape
    ix, iy = int(round(x)), int(round(y))
    if 0 <= ix < W and 0 <= iy < H and not occ[iy, ix]:
        return float(ix), float(iy)

    for r in range(1, r_max + 1):
        x0, x1 = max(0, ix - r), min(W - 1, ix + r)
        y0, y1 = max(0, iy - r), min(H - 1, iy + r)
        for cx in range(x0, x1 + 1):
            if not occ[y0, cx]:
                return float(cx), float(y0)
            if not occ[y1, cx]:
                return float(cx), float(y1)
        for cy in range(y0 + 1, y1):
            if not occ[cy, x0]:
                return float(x0), float(cy)
            if not occ[cy, x1]:
                return float(x1), float(cy)
    return float(ix), float(iy)


# ============================================================
# Image load (OpenCV) + visualization
# ============================================================

def load_occupancy_from_image_cv(path: str, invert: bool = True, threshold: int = 200) -> np.ndarray:
    """
    Load an image using OpenCV and convert to boolean occupancy.
    True = obstacle, False = free.
    invert=True assumes black obstacles on white background.
    """
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {path}")
    # Normalize to 0..255 uint8 already ensured by OpenCV
    if invert:
        # black (low) -> obstacle
        _, mask = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY_INV)
    else:
        # white (high) -> obstacle
        _, mask = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
    return (mask > 0)


def plot_all(occ_base, occ_infl, rrt, path, pruned, spline, start, goal):
    H, W = occ_base.shape
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, W - 1)
    ax.set_ylim(H - 1, 0)  # image coordinates
    # Draw base map (free=white, obstacle=black)
    base_vis = np.where(occ_base, 0, 255).astype(np.uint8)
    ax.imshow(base_vis, cmap="gray", vmin=0, vmax=255, interpolation="nearest")

    # Inflated boundary visualization
    infl_outline = occ_infl ^ occ_base
    ys, xs = np.nonzero(infl_outline)
    if len(xs) > 0:
        ax.plot(xs, ys, "s", markersize=2, alpha=0.4)

    # RRT tree
    for n in rrt.nodes:
        if n.parent is not None:
            ax.plot([n.parent.x, n.x], [n.parent.y, n.y], linewidth=0.5, alpha=0.35)

    if path is not None:
        ax.plot(path[:, 0], path[:, 1], "--", linewidth=1.5, label="RRT path")
    if pruned is not None:
        ax.plot(pruned[:, 0], pruned[:, 1], "-.", linewidth=2.0, label="Pruned")
    if spline is not None:
        ax.plot(spline[:, 0], spline[:, 1], linewidth=3.0, label="Spline")

    ax.plot(start[0], start[1], "o", markersize=8, label="Start")
    ax.plot(goal[0], goal[1], "o", markersize=8, label="Goal")
    ax.legend(loc="best")
    ax.set_title("RRT on OpenCV-loaded Image (with cv2.dilate) + Catmull-Rom Spline")
    plt.tight_layout()
    plt.show()


# ============================================================
# Main
# ============================================================

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--map", type=str, required=True, help="Path to occupancy image.")
    p.add_argument("--start", type=float, nargs=2, help="Start pixel (x y).")
    p.add_argument("--goal", type=float, nargs=2, help="Goal pixel (x y).")
    p.add_argument("--robot_radius_px", type=int, default=6, help="Inflation radius in pixels.")
    p.add_argument("--skip_inflation", action="store_true", help="Do not inflate (map already inflated).")
    p.add_argument("--invert", action="store_true", default=True, help="Black obstacles on white background.")
    p.add_argument("--no-invert", dest="invert", action="store_false")
    p.add_argument("--step", type=float, default=6.0)
    p.add_argument("--max_iter", type=int, default=20000)
    p.add_argument("--goal_tol", type=float, default=5.0)
    p.add_argument("--seed", type=int, default=7)
    args = p.parse_args()

    # Load occupancy with OpenCV
    occ_base = load_occupancy_from_image_cv(args.map, invert=args.invert, threshold=200)

    # Inflate with OpenCV if requested
    if args.skip_inflation or args.robot_radius_px <= 0:
        occ_infl = occ_base
    else:
        occ_infl = binary_dilate_cv(occ_base, args.robot_radius_px)

    H, W = occ_base.shape
    start = (10.0, 10.0) if args.start is None else (float(args.start[0]), float(args.start[1]))
    goal  = (W - 10.0, H - 10.0) if args.goal is None else (float(args.goal[0]), float(args.goal[1]))

    # Snap to nearest free pixels to avoid errors at borders or inside inflated obstacles
    sx, sy = nearest_free(occ_infl, *start)
    gx, gy = nearest_free(occ_infl, *goal)

    rrt = RRT(
        start_xy=(sx, sy),
        goal_xy=(gx, gy),
        occ_grid=occ_infl,
        step=args.step,
        goal_sample_rate=0.07,
        max_iter=args.max_iter,
        goal_tolerance=args.goal_tol,
        collision_step=1.0,
        rng_seed=args.seed,
    )

    path = rrt.plan()
    pruned = prune_path(path, occ_infl, 1.0) if path is not None else None
    spline = catmull_rom_spline(pruned, 20, 0.5) if pruned is not None else None

    print(f"Start snapped to: ({sx:.1f}, {sy:.1f})")
    print(f"Goal  snapped to: ({gx:.1f}, {gy:.1f})")

    plot_all(occ_base, occ_infl, rrt, path, pruned, spline, (sx, sy), (gx, gy))


if __name__ == "__main__":
    main()
