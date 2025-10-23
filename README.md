# MPC RRT* Motion Planning Framework

A production-ready Python package that combines deterministic RRT* global path planning
with a curvature-aware Model Predictive Controller (MPC) for vehicle tracking. The code
base started as an academic prototype and has been refactored into a modular
architecture with clear interfaces, documentation, and automated tests.

## Key capabilities

- **Deterministic RRT\*** planner with configurable sampling, rewiring, and goal bias
  to guarantee reproducible global paths.
- **Bicycle-model MPC** tracker that enforces steering, rate, and velocity limits while
  remaining numerically well-conditioned through an OSQP backend.
- **Pipeline orchestrator** that composes map preparation, planning, and control stages
  into reusable building blocks for scripts, services, or notebooks.
- **Headless-friendly visualisation** utilities for generating artefacts and recording
  simulations without a display.
- **Extensive documentation** including developer guides, architecture notes, and
  theory references for MPC tuning and vehicle dynamics.

## Architecture at a glance

```mermaid
digraph Pipeline {
    rankdir=LR
    Config["PipelineConfig\n(YAML or defaults)"]
    CLI["CLI (click)"]
    API["Python API"]
    Orchestrator["PipelineOrchestrator"]
    MapStage["MapStage\noccupancy & inflation"]
    PlanningStage["PlanningStage\nRRT* planner"]
    ControlStage["TrajectoryTracker\nMPC"]
    Viz["Visualization"]

    Config -> CLI
    Config -> API
    CLI -> Orchestrator
    API -> Orchestrator
    Orchestrator -> MapStage -> PlanningStage -> ControlStage
    ControlStage -> Viz
}
```

The `PipelineOrchestrator` produces immutable artefacts describing each stage. They can
be reused to visualise the tree, analyse trajectories, or feed downstream components.
A deeper architectural discussion lives in [`docs/architecture.md`](docs/architecture.md).

## Repository layout

```
src/
├── common/           # Geometry helpers and array typing utilities
├── config.py         # Dataclasses + YAML loader for pipeline configuration
├── pipeline/         # Map, planning, and control stages with orchestrator + API
├── planning/         # Deterministic RRT* implementation and data structures
├── control/          # MPC dynamics, linearisation, and solver wrappers
├── maps/             # Occupancy grid generators and inflation helpers
├── viz/              # Matplotlib-based plotting utilities
└── cli.py            # Click-powered command line entry points
```

Additional documentation resides in [`docs/`](docs/index.md) and runnable examples in
[`examples/`](examples/).

## Installation

```bash
python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e .[dev]
```

The optional `[dev]` extras install testing, linting, and formatting tooling.

## Command line usage

All workflows are exposed via the Click-based CLI:

```bash
# Generate the base occupancy grid according to the configuration
python -m src.cli generate-map

# Inflate the map and persist the result
python -m src.cli inflate-map

# Run deterministic RRT* planning and visualise the tree
python -m src.cli plan-path

# Execute the full pipeline with MPC tracking and visualisation
python -m src.cli run
```

Pass `--config path/to/config.yaml` to override the defaults from `PipelineConfig`.
For headless environments set `viz.backend` to `Agg` in the configuration file.

## Programmatic API

Use the high-level helper for scripts or notebooks:

```python
from src import PipelineConfig, load_config, run_pipeline

config = load_config("configs/pipeline.yaml")
plan_result, tracked_states = run_pipeline(config, visualize=False)
print(f"Reached goal with {len(plan_result.path)} path points")
```

For granular control import the individual stages from `src.pipeline` and compose them
manually (see [`examples/run_pipeline.py`](examples/run_pipeline.py)).

## Testing

Run the automated test suite with:

```bash
pytest
```

The tests validate stage interfaces, pipeline orchestration, and configuration
loading. Continuous integration can extend this with linting and static analysis.

## Further reading

- [`docs/developer_guide.md`](docs/developer_guide.md) – conventions, architecture
  decisions, and extension guidelines.
- [`docs/usage_pipeline.md`](docs/usage_pipeline.md) – step-by-step tutorial for
  configuring and running the full stack.
- [`docs/architecture.md`](docs/architecture.md) – detailed diagrams and data flow.

For enquiries please contact the maintainers at `research@example.com`.
