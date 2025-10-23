# Software Architecture

The refactored prototype is structured around composable stages so that map
preparation, path planning, and trajectory tracking can evolve independently.
The following diagram summarises the runtime composition.

```mermaid
flowchart LR
    cfg[(config.PipelineConfig)] --> cli[cli.run_pipeline]
    cli --> orch[pipeline.PipelineOrchestrator]
    orch --> mapStage[MapStage\n(pipeline.map_stage)]
    orch --> planStage[PlanningStage\n(pipeline.planning_stage)]
    orch --> trackStage[TrajectoryTracker\n(pipeline.control_stage)]
    mapStage --> maps[maps.*]
    planStage --> planning[planning.rrt_star]
    trackStage --> control[control.*]
    trackStage --> viz[viz.visualization]
    planning --> common[common.geometry]
    control --> common
```

## Module Responsibilities

### `pipeline`
- **map_stage.py** – encapsulates map generation, inflation and start/goal
  derivation, returning immutable `MapArtifacts`.
- **planning_stage.py** – wraps `RRTStarPlanner` execution and records the full
  `PlanResult` alongside metadata.
- **control_stage.py** – performs MPC roll-outs with a structured recovery
  strategy and optional visualisation hooks, yielding `TrackingResult`.
- **orchestrator.py** – coordinates the stages, manages Matplotlib lifecycle and
  aggregates results into a `PipelineResult` that exposes both the plan and the
  tracked states.
- **artifacts.py** – typed containers that document the data exchanged between
  stages and enforce immutability where possible.

### `maps`
- **generator.py** – deterministic occupancy grid synthesis with optional
  obstacle templates.
- **inflate.py** – obstacle dilation routines operating on binary grids or
  grayscale images to provide metric safety margins.
- **io.py** – OpenCV-backed loading/saving helpers with explicit grayscale
  semantics.

### `planning`
- **rrt_star.py** – deterministic RRT* with parent-indexed nodes (`PlanResult`).
  Accepts a `PlannerParameters` dataclass to ensure reproducible seeds and
  tunable metrics.
- **plan_result.py** – data structures for returning the final path, node list
  and metadata (iterations, goal index).

### `control`
- **vehicle_model.py** – kinematic bicycle model and Jacobian linearisation
  utilities.
- **ref_builder.py** – curvature-aware reference generation from geometric
  polylines.
- **mpc_controller.py** – OSQP-backed MPC with soft constraints and structured
  fallback via slack penalties.

### `viz`
- **visualization.py** – backend selection, RRT* rendering, MPC prediction plots
  and tracked vehicle animation.
- **vehicle_draw.py** – reusable car footprint drawing primitives.
- **record.py** – frame capture helper and GIF assembly utilities for offline
  rendering.

### Shared Utilities
- **common/** – geometry resampling, yaw computation, plot path helpers and type
  aliases.
- **logging_setup.py** – centralised logging configuration for CLI and unit
  tests.
- **config.py** – dataclasses for map, planner, MPC and visualisation settings,
  responsible for YAML deserialisation with `yaml.safe_load`.
- **cli.py** – thin façade that wires the `PipelineOrchestrator` into Click
  commands and exposes `run_pipeline` for direct API consumption.

## Data Flow

1. **Configuration** is loaded into `PipelineConfig`, providing typed access to
   all parameters.
2. **Map Preparation** uses `MapStage` to optionally generate a synthetic map,
   inflate obstacles and construct a binary occupancy grid.
3. **Planning** runs inside `PlanningStage`, calling `RRTStarPlanner.plan` and
   returning a `PlanResult` containing the path and exploration tree.
4. **Reference Building & Control** happens inside `TrajectoryTracker`, which
   converts the geometric path into an MPC reference, solves the QP with
   recovery logic, and integrates the vehicle model forward in time.
5. **Visualisation / Recording** is triggered from the control stage, reusing
   the shared Matplotlib utilities and optional frame recorder. All artefacts are
   persisted beneath `plots/` through the shared path helpers.

This layered structure keeps responsibilities narrow, simplifies testing of each
stage, and provides a clear seam for future planners or controllers.
