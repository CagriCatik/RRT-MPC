# RRT* Planning

The planner operates on binary occupancy grids where free cells have value `1`
and obstacles `0`. Obstacle inflation is performed beforehand to guarantee the
vehicle footprint remains within free space.

## Deterministic Sampling

A `PlannerParameters` dataclass encapsulates step size, goal radius, maximum
iterations, rewire radius, goal sampling probability and a random seed. The seed
initialises `numpy.random.default_rng`, ensuring runs are reproducible given the
same map and configuration.

## Algorithm

1. Initialises the tree with the start node.
2. At each iteration, samples the goal with probability `goal_sample_rate` or a
   uniform random free-space point otherwise.
3. Steers from the nearest node towards the sample with the configured `step`
   length, discarding samples that leave map bounds or collide with obstacles.
4. Chooses the best parent among nearby nodes (`rewire_radius`) by minimising
   cumulative cost while preserving collision-free edges.
5. Rewires neighbour nodes when routing through the new node reduces path cost.
6. Declares success when a node enters the goal radius and the straight line to
   the goal is collision-free. A dedicated goal node is appended to the tree for
   consistent path extraction.

The output `PlanResult` contains the ordered path (including start and goal),
all nodes with parent indices and metadata such as iteration count.

## Collision Checking

Edges are discretised at one-pixel increments along the straight line between
nodes. Any collision or out-of-bounds sample causes the candidate edge to be
rejected. Inflation ensures straight-line interpolation between nodes respects
safety margins.

## Tuning Guidelines

- **Step size**: smaller values increase precision but require more iterations.
  A step of 10 pixels balances computation and smoothness on the default map.
- **Rewire radius**: must be larger than the step size to allow meaningful cost
  improvements. Radii between 15–30 pixels work well for 256×256 grids.
- **Goal radius**: should be large enough to tolerate discretisation while not
  overshooting the target.
- **Goal sampling rate**: biases the tree towards the goal; 0.1–0.2 provides a
  good compromise between exploration and convergence speed.

Deterministic seeds make unit testing straightforward and allow regression tests
across environments.
