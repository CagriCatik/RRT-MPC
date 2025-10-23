# Software Architecture

The framework is organised around clearly defined module boundaries to promote
maintainability and deterministic behaviour. The diagram below summarises the
primary packages and their interactions.

```
┌────────────┐     ┌────────────┐     ┌────────────┐     ┌────────────┐
│ maps       │ ──▶ │ planning   │ ──▶ │ control    │ ──▶ │ viz        │
│ generation │     │ RRT*       │     │ MPC        │     │ rendering  │
└────────────┘     └────────────┘     └────────────┘     └────────────┘
        ▲                 ▲                 │                 │
        │                 │                 ▼                 ▼
   config.py         cli.py/run_pipeline     logging_setup.py/common/
```

## Module Responsibilities

### `maps`
- **generator.py**: Deterministic occupancy grid synthesis with optional
  obstacle templates.
- **inflate.py**: Obstacle dilation functions operating on binary grids or
  grayscale images. Returns pixel-accurate safety margins.
- **io.py**: Thin wrappers for OpenCV-based loading/saving with consistent
  grayscale semantics.

### `planning`
- **rrt_star.py**: Deterministic RRT* with parent-indexed nodes (`PlanResult`).
  The planner accepts a `PlannerParameters` dataclass to ensure reproducible
  seeds and tunable metrics.
- **plan_result.py**: Data structures for returning the final path, nodes and
  metadata (iterations, goal index).

### `control`
- **vehicle_model.py**: Kinematic bicycle model and Jacobian linearisation
  utilities.
- **ref_builder.py**: Curvature-aware reference generation from geometric
  polylines.
- **mpc_controller.py**: OSQP-backed MPC with soft constraints and recovery
  strategy. Accepts an `MPCParameters` dataclass derived from configuration.

### `viz`
- **visualization.py**: Backend selection, RRT* rendering, MPC prediction plots
  and tracked vehicle animation.
- **vehicle_draw.py**: Reusable car footprint drawing primitives.
- **record.py**: Frame capture helper for unit tests or offline rendering.

### Shared Utilities
- **common/**: Geometry resampling, yaw computation and type aliases.
- **logging_setup.py**: Centralised logging configuration for CLI and unit
  tests.
- **config.py**: Dataclasses for map, planner, MPC and visualisation settings.
  Responsible for YAML deserialisation using `yaml.safe_load`.
- **cli.py**: Orchestrates the pipeline, exposes `run_pipeline` for direct API
  consumption and provides Click commands for end users.

## Data Flow

1. **Configuration** is loaded into `PipelineConfig`, providing typed access to
   all parameters.
2. **Map Preparation**: The CLI optionally generates a synthetic map, inflates
   obstacles and produces the binary occupancy grid.
3. **Planning**: `RRTStarPlanner.plan` runs with deterministic seeds and returns
   a `PlanResult` containing the path and tree.
4. **Reference Building**: The geometric path is resampled and translated into a
   curvature-aware trajectory used by the MPC.
5. **Control**: `MPCController.solve` produces optimal control sequences, with
   slack-based recovery in the face of infeasibilities.
6. **Visualisation / Recording**: The live predictions, tree and tracked states
   are rendered using the configured matplotlib backend. Optional frame capture
   creates PNG sequences in headless scenarios.

This structure enables unit testing of each component and promotes deterministic
behaviour across runs.
