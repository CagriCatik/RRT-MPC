import math
import random
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple

# ============================================
# RRT with Catmull-Rom spline and obstacle inflation
# ============================================

# Axis-aligned rectangular obstacles
Rect = namedtuple("Rect", "xmin ymin xmax ymax")


class Node:
    __slots__ = ("x", "y", "parent")
    def __init__(self, x, y, parent=None):
        self.x = float(x)
        self.y = float(y)
        self.parent = parent


# -----------------------------
# Geometry and obstacle helpers
# -----------------------------

def inflate_rect(rect, radius, x_range, y_range):
    """Inflate a single axis-aligned rectangle by 'radius' (Minkowski sum with a disk)."""
    r = float(max(0.0, radius))
    xmin = max(x_range[0], rect.xmin - r)
    ymin = max(y_range[0], rect.ymin - r)
    xmax = min(x_range[1], rect.xmax + r)
    ymax = min(y_range[1], rect.ymax + r)
    return Rect(xmin, ymin, xmax, ymax)


def inflate_obstacles(obstacles, radius, x_range, y_range):
    """Inflate all obstacles by 'radius' and clamp to world bounds."""
    return [inflate_rect(o, radius, x_range, y_range) for o in obstacles]


def point_in_rect(x, y, r):
    return (r.xmin <= x <= r.xmax) and (r.ymin <= y <= r.ymax)


def segment_collides_aabb(a, b, rect, step=0.5):
    """Discrete collision check against an inflated AABB by sampling along the segment."""
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    length = math.hypot(dx, dy)
    if length == 0.0:
        return point_in_rect(ax, ay, rect)
    steps = max(1, int(length / max(step, 1e-9)))
    for i in range(steps + 1):
        t = i / steps
        x = ax + t * dx
        y = ay + t * dy
        if point_in_rect(x, y, rect):
            return True
    return False


# -----------------------------
# RRT planner
# -----------------------------

class RRT:
    def __init__(
        self,
        start,
        goal,
        obstacles,
        x_range=(0.0, 100.0),
        y_range=(0.0, 100.0),
        step=2.5,
        goal_sample_rate=0.05,
        max_iter=10000,
        goal_tolerance=3.0,
        collision_step=0.5,
        rng_seed=42,
    ):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles  # already inflated if desired
        self.x_range = (float(x_range[0]), float(x_range[1]))
        self.y_range = (float(y_range[0]), float(y_range[1]))
        self.step = float(step)
        self.goal_sample_rate = float(goal_sample_rate)
        self.max_iter = int(max_iter)
        self.goal_tolerance = float(goal_tolerance)
        self.collision_step = float(collision_step)
        self.nodes = [self.start]
        self.rng = random.Random(rng_seed)

    # ---- helpers ----

    @staticmethod
    def _dist(a, b):
        return math.hypot(a.x - b.x, a.y - b.y)

    def _nearest(self, point):
        px, py = point
        best = None
        best_d = float("inf")
        for n in self.nodes:
            d = (n.x - px) * (n.x - px) + (n.y - py) * (n.y - py)
            if d < best_d:
                best_d = d
                best = n
        return best

    def _collides_point(self, x, y):
        for r in self.obstacles:
            if point_in_rect(x, y, r):
                return True
        return False

    def _collides_segment(self, a, b):
        for r in self.obstacles:
            if segment_collides_aabb(a, b, r, step=self.collision_step):
                return True
        return False

    def _sample(self):
        if self.rng.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        x = self.rng.uniform(self.x_range[0], self.x_range[1])
        y = self.rng.uniform(self.y_range[0], self.y_range[1])
        return (x, y)

    def _steer(self, from_node, to_point):
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        d = math.hypot(dx, dy)
        if d == 0.0:
            return Node(from_node.x, from_node.y, from_node)
        scale = min(self.step, d) / d
        nx = from_node.x + dx * scale
        ny = from_node.y + dy * scale
        return Node(nx, ny, from_node)

    # ---- main ----

    def plan(self):
        if self._collides_point(self.start.x, self.start.y):
            raise ValueError("Start is inside an obstacle.")
        if self._collides_point(self.goal.x, self.goal.y):
            raise ValueError("Goal is inside an obstacle.")

        for _ in range(self.max_iter):
            rnd = self._sample()
            nearest = self._nearest(rnd)
            new_node = self._steer(nearest, rnd)

            if not self._collides_segment((nearest.x, nearest.y), (new_node.x, new_node.y)):
                self.nodes.append(new_node)

                if self._dist(new_node, self.goal) <= self.goal_tolerance:
                    if not self._collides_segment((new_node.x, new_node.y), (self.goal.x, self.goal.y)):
                        goal_node = Node(self.goal.x, self.goal.y, new_node)
                        self.nodes.append(goal_node)
                        return self._extract_path(goal_node)

        # Fallback: connect closest to goal if possible
        closest = min(self.nodes, key=lambda n: self._dist(n, self.goal))
        if not self._collides_segment((closest.x, closest.y), (self.goal.x, self.goal.y)):
            goal_node = Node(self.goal.x, self.goal.y, closest)
            self.nodes.append(goal_node)
            return self._extract_path(goal_node)

        return None

    @staticmethod
    def _extract_path(node):
        path = []
        cur = node
        while cur is not None:
            path.append((cur.x, cur.y))
            cur = cur.parent
        path.reverse()
        return np.array(path, dtype=float)


# -----------------------------
# Robust Catmull-Rom spline
# -----------------------------

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
    """
    Numerically robust centripetal Catmull-Rom spline.
    - Removes consecutive duplicates.
    - Guards zero denominators.
    - Falls back to linear interpolation when segments degenerate.
    """
    P = _dedupe_consecutive(points)
    n = len(P)
    if n == 0:
        return P.copy()
    if n == 1:
        return P.copy()
    if n == 2:
        t = np.linspace(0.0, 1.0, max(2, samples_per_segment + 1))
        return (1 - t)[:, None] * P[0] + t[:, None] * P[1]

    # Endpoint clamping
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

        t_vals = np.linspace(t1, t2, max(2, samples_per_segment + 1), endpoint=False)

        for t in t_vals:
            A1 = (t1 - t) / dt01 * p0 + (t - t0) / dt01 * p1
            A2 = (t2 - t) / dt12 * p1 + (t - t1) / dt12 * p2
            A3 = (t3 - t) / dt23 * p2 + (t - t2) / dt23 * p3

            B1 = (t2 - t) / dt02 * A1 + (t - t0) / dt02 * A2
            B2 = (t3 - t) / dt13 * A2 + (t - t1) / dt13 * A3

            C = (t2 - t) / dt12 * B1 + (t - t1) / dt12 * B2
            out.append(C)

    out.append(P[-1])
    return np.asarray(out)


# -----------------------------
# Path utilities
# -----------------------------

def prune_path(path, obstacles, collision_step=0.5):
    """Shortcut pruning while keeping path collision-free wrt given obstacles."""
    if path is None or len(path) <= 2:
        return path

    pts = [tuple(p) for p in path]

    def collides(a, b):
        for r in obstacles:
            if segment_collides_aabb(a, b, r, step=collision_step):
                return True
        return False

    i = 0
    pruned = [pts[0]]
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and collides(pts[i], pts[j]):
            j -= 1
        pruned.append(pts[j])
        i = j
    return np.array(pruned, dtype=float)


# -----------------------------
# Demo / Visualization
# -----------------------------

def demo():
    # World bounds
    x_range = (0.0, 100.0)
    y_range = (0.0, 100.0)

    # Base obstacles (uninflated)
    base_obstacles = [
        Rect(15, 40, 45, 60),
        Rect(55, 20, 85, 35),
        Rect(60, 65, 90, 85),
        Rect(20, 10, 35, 25),
        Rect(10, 70, 25, 90),
    ]

    # Obstacle inflation radius
    inflation_radius = 3.0  # units

    # Compute inflated obstacles
    inflated = inflate_obstacles(base_obstacles, inflation_radius, x_range, y_range)

    start = (5.0, 5.0)
    goal = (95.0, 95.0)

    # Build planner against inflated obstacles
    rrt = RRT(
        start=start,
        goal=goal,
        obstacles=inflated,
        x_range=x_range,
        y_range=y_range,
        step=3.0,
        goal_sample_rate=0.07,
        max_iter=15000,
        goal_tolerance=4.0,
        collision_step=0.75,
        rng_seed=7,
    )

    path = rrt.plan()
    if path is None:
        print("No path found.")
        return

    # Prune and smooth (both against inflated obstacles)
    pruned = prune_path(path, inflated, collision_step=0.75)
    pruned = _dedupe_consecutive(pruned, tol=1e-9)
    spline = catmull_rom_spline(pruned, samples_per_segment=20, alpha=0.5)

    # Plot
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.grid(True, linewidth=0.5, alpha=0.3)

    # Draw base obstacles
    for r in base_obstacles:
        ax.add_patch(
            plt.Rectangle(
                (r.xmin, r.ymin),
                r.xmax - r.xmin,
                r.ymax - r.ymin,
                fill=True,
                alpha=0.35,
                linewidth=0.0,
            )
        )

    # Draw inflated obstacles as outlines
    for r in inflated:
        ax.add_patch(
            plt.Rectangle(
                (r.xmin, r.ymin),
                r.xmax - r.xmin,
                r.ymax - r.ymin,
                fill=False,
                linewidth=1.5,
                linestyle="--",
            )
        )

    # RRT tree
    for n in rrt.nodes:
        if n.parent is not None:
            ax.plot([n.parent.x, n.x], [n.parent.y, n.y], linewidth=0.5, alpha=0.35)

    # Path variants
    ax.plot(path[:, 0], path[:, 1], linewidth=1.5, linestyle="--", label="RRT path")
    ax.plot(pruned[:, 0], pruned[:, 1], linewidth=2.0, linestyle="-.", label="Pruned")
    ax.plot(spline[:, 0], spline[:, 1], linewidth=3.0, label="Spline")

    # Start/Goal
    ax.plot(start[0], start[1], marker="o", markersize=8, label="Start")
    ax.plot(goal[0], goal[1], marker="o", markersize=8, label="Goal")

    ax.legend(loc="best")
    ax.set_title("RRT with Catmull-Rom Spline and Inflated Obstacles")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    demo()
