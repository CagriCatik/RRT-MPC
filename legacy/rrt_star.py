#!/usr/bin/env python3
"""
rrt_star.py
RRT* algorithm implementation (with simple rewiring).
"""

import numpy as np
import random
from math import sqrt, atan2, cos, sin


class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0.0


def distance(a, b):
    return sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


def steer(from_node, to_node, step):
    theta = atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = from_node.x + step * cos(theta)
    new_y = from_node.y + step * sin(theta)
    new_node = Node(new_x, new_y, from_node)
    new_node.cost = from_node.cost + step
    return new_node


def collision_free(p1, p2, occ):
    x1, y1, x2, y2 = int(p1.x), int(p1.y), int(p2.x), int(p2.y)
    for u in np.linspace(0, 1, int(distance(p1, p2)) + 1):
        x = int(x1 + u * (x2 - x1))
        y = int(y1 + u * (y2 - y1))
        if not (0 <= x < occ.shape[1] and 0 <= y < occ.shape[0]):
            return False
        if occ[y, x] == 0:
            return False
    return True


def get_near_nodes(nodes, new_node, radius):
    near = []
    for n in nodes:
        if distance(n, new_node) < radius:
            near.append(n)
    return near


def extract_path(goal_node):
    path = []
    n = goal_node
    while n:
        path.append((n.x, n.y))
        n = n.parent
    return path[::-1]


def rrt_star(occ, start, goal, step=10, goal_radius=15, max_iter=3000, rewire_radius=25, goal_prob=0.1):
    start_node = Node(*start)
    goal_node = Node(*goal)
    nodes = [start_node]

    for i in range(max_iter):
        rnd = Node(goal[0], goal[1]) if random.random() < goal_prob else \
              Node(random.randint(0, occ.shape[1] - 1), random.randint(0, occ.shape[0] - 1))

        nearest = min(nodes, key=lambda n: distance(n, rnd))
        new_node = steer(nearest, rnd, step)

        if 0 <= new_node.x < occ.shape[1] and 0 <= new_node.y < occ.shape[0]:
            if not collision_free(nearest, new_node, occ):
                continue

            near_nodes = get_near_nodes(nodes, new_node, rewire_radius)
            best_parent = nearest
            min_cost = nearest.cost + distance(nearest, new_node)

            for n in near_nodes:
                if collision_free(n, new_node, occ) and n.cost + distance(n, new_node) < min_cost:
                    best_parent = n
                    min_cost = n.cost + distance(n, new_node)

            new_node.parent = best_parent
            new_node.cost = min_cost
            nodes.append(new_node)

            # Rewire
            for n in near_nodes:
                new_cost = new_node.cost + distance(n, new_node)
                if new_cost < n.cost and collision_free(new_node, n, occ):
                    n.parent = new_node
                    n.cost = new_cost

            if distance(new_node, goal_node) < goal_radius and collision_free(new_node, goal_node, occ):
                goal_node.parent = new_node
                goal_node.cost = new_node.cost + distance(new_node, goal_node)
                print(f"Goal reached at iteration {i}.")
                return nodes, goal_node

    print("Failed to find path.")
    return nodes, None
