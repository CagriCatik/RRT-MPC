# MPC RRT* Motion Planning Framework

A modular Python package that combines deterministic RRT* global path planning with
a vehicle-dynamics-aware Model Predictive Controller (MPC) for tracking. The project
originates from a working prototype and has been refactored into a production-grade
package complete with documentation, configuration management, command line tools and
unit tests.

## Architecture Overview

```
src/
├── maps/          # Occupancy grid generation, IO and inflation
├── planning/      # Deterministic RRT* planner with structured outputs
├── control/       # Vehicle model, MPC controller and reference builders
├── viz/           # Headless-capable visualisation utilities
├── common/        # Shared geometry and type helpers
├── cli.py         # Click-based CLI for maps, planning and the full pipeline
└── config.py      # YAML-driven configuration dataclasses
```

The end-to-end pipeline performs the following steps:

1. Generate or load an occupancy grid, then inflate obstacles using the vehicle safety
   margin.
2. Compute a deterministic RRT* tree and extract a kinematically feasible path.
3. Build a curvature-aware reference trajectory and track it with an OSQP-backed MPC
   that enforces velocity, input and rate limits.
4. Visualise the tree, solution path, MPC predictions and tracked vehicle states.

## Theory of Operation

* **RRT\***: A deterministic variant with configurable step size, goal bias, rewire radius
  and iteration count. Collision checking operates on the inflated occupancy grid to
  guarantee clearance.
* **MPC Tracking**: A kinematic bicycle model is linearised along the reference window
  and discretised with forward Euler. The controller solves a strictly convex QP with
  quadratic slack penalties for velocity, input and rate constraints. OSQP parameters
  are tuned (``rho=0.1``, ``alpha=1.6``, ``adaptive_rho=True``) for numerical robustness.

Further theoretical details, derivations and design rationales are documented in the
[`docs/`](docs/index.md) directory.

## Installation

```bash
python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e .[dev]
```

This installs runtime dependencies (NumPy, OpenCV, matplotlib, cvxpy, click, PyYAML)
plus development tools (pytest, mypy, ruff, black).

## Command Line Interface

All capabilities are exposed via the ``mpc-rrt-star`` CLI:

```bash
python -m src.cli --config configs/pipeline.yaml run
python -m src.cli generate-map
python -m src.cli inflate-map
python -m src.cli plan-path
```

Use ``--config`` to point to a YAML configuration file. Without it the built-in defaults
are used. The CLI emits deterministic logs suitable for automation.

## Configuration

Configuration is assembled from the following schema:

```yaml
map:
  map_file: occupancy_grid.png
  inflated_map_file: occupancy_grid_inflated.png
  map_resolution: 0.2
  inflation_radius_m: 0.6
  start: [50, 50]
  goal_offset: [50, 50]
planner:
  step_size: 10
  goal_radius: 15
  max_iterations: 3000
  rewire_radius: 25
  goal_sample_rate: 0.1
  random_seed: 13
mpc:
  wheelbase_m: 2.8
  dt: 0.1
  horizon: 12
  v_px_s: 28.0
  sim_steps: 600
viz:
  backend: Agg
  prediction_pause: 0.05
  animate_tree: true
  record_frames: false
```

See [`docs/configuration.md`](docs/configuration.md) for a complete reference.

## Simulation Demo

1. Generate a map and inflated occupancy grid:
   ```bash
   python -m src.cli generate-map
   python -m src.cli inflate-map
   ```
2. Run deterministic RRT* planning and capture the tree:
   ```bash
   python -m src.cli plan-path
   ```
3. Execute the full pipeline with MPC tracking:
   ```bash
   python -m src.cli run
   ```

During the run the package renders the RRT* tree, planned path, MPC predictions and the
tracked vehicle states. For headless environments set ``viz.backend`` to ``Agg``.

## Documentation

The documentation set lives under [`docs/`](docs/index.md) and covers:

* Software architecture and module interactions
* MPC derivations and solver tuning
* Vehicle dynamics and model linearisation
* RRT* planning theory and parameter selection
* End-to-end usage walkthroughs and configuration guides

## Contact & Citation

For enquiries, please contact the maintainers via `research@example.com` and cite this
work as:

> OpenAI Prototype Team (2024). *MPC RRT* Motion Planning Framework.* Version 0.1.0.


# MPC RRT* Motion Planning Toolkit

This repository hosts the `src` Python package, a modular implementation of
deterministic RRT* path planning combined with a model-predictive controller for
kinematic bicycle models. All actively maintained source code, documentation, and tests
live under [`src/`](src/).

Legacy prototype scripts and generated artefacts have been removed in favour of the
package, which offers a clean command-line interface, reusable modules, and automated
test coverage.

## Getting started

```bash
cd src
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
```

With the environment active you can execute the full planning and control pipeline:

```bash
python -m src.cli run
```

The CLI automatically generates a deterministic test map if none is present and writes
visualisation artefacts (inflated map, RRT* tree snapshot, optional frame dumps) to the
working directory. Additional commands such as `generate-map`, `inflate-map`, and
`plan-path` are available; consult [`src/README.md`](src/README.md) for
a complete overview.
