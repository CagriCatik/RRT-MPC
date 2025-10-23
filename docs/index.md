# MPC RRT*

Welcome to the documentation for the MPC RRT* Motion Planning Framework. This
package provides a modular reference implementation of a deterministic RRT*
planner tightly coupled with a bicycle-model MPC controller. The documentation
is organised as follows:

- [Architecture](architecture.md) – software modules, data flow and
  configuration boundaries.
- [MPC Theory](mpc_theory.md) – optimal control problem formulation and solver
tuning strategies.
- [Vehicle Dynamics](vehicle_dynamics.md) – single-track model derivation and
  linearisation steps.
- [RRT* Planning](planning_rrt_star.md) – algorithmic details, heuristics and
  determinism guarantees.
- [Pipeline Usage](usage_pipeline.md) – running the end-to-end system from CLI
  and Python APIs.
- [Pipeline Integration](system_integration.md) – end-to-end sequencing, data
  contracts and MPC/RRT* interaction notes.
- [Configuration](configuration.md) – YAML schema, defaults and override tips.
- [Developer Guide](developer_guide.md) – contribution workflow and extension points.

Each document is self-contained and includes references to the relevant code
modules. All equations are provided using standard Markdown+LaTeX syntax.

## Capabilities at a Glance

- **Deterministic planning** – reproducible RRT* exploration trees driven by a
  shared random seed and deterministic collision checking on occupancy grids.
- **Model-predictive control** – OSQP-backed MPC with velocity, steering and
  rate constraints, plus automatic relaxation strategies for infeasible
  iterations.
- **Visualization toolchain** – Matplotlib utilities for RRT* trees, MPC
  predictions and frame recording that exports to PNG or GIF.
- **Headless automation** – CLI entry points and Python APIs that run the full
  pipeline while persisting artefacts under `plots/` for later inspection.

## Repository Layout

- `src/` – production code organised by domain (maps, planning, control,
  pipeline orchestration, visualisation helpers).
- `tests/` – deterministic unit tests covering geometry helpers, planning and
  MPC building blocks.
- `docs/` – this MkDocs site documenting architecture, algorithms and operator
  workflows.
- `examples/` – runnable scripts demonstrating pipeline usage and GIF
  generation.
- `assets/` & `plots/` – input maps and generated artefacts respectively.

## Execution Modes

- **Command line** – `python -m src.cli run` executes the full stack with live
  visualisation; additional sub-commands target individual stages.
- **Library** – import `PipelineOrchestrator` or `run_pipeline` to embed the
  planner/controller stack inside notebooks, experiments or services.
- **Batch** – schedule simulations headlessly by configuring `viz.backend: Agg`
  and enabling frame recording for offline review.

## Observability

The project configures structured logging via `src.logging_setup`. Core stages
emit INFO-level progress updates (map preparation, planning iteration milestones
and MPC roll-out summaries) so terminal output mirrors the high-level timeline
of a run. Adjust log levels with `logging.basicConfig` or the helper
`configure_logging()` to obtain more verbose DEBUG traces during development.
