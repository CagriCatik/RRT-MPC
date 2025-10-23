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
- [Configuration](configuration.md) – YAML schema, defaults and override tips.
- [Developer Guide](developer_guide.md) – contribution workflow and extension points.

Each document is self-contained and includes references to the relevant code
modules. All equations are provided using standard Markdown+LaTeX syntax.
