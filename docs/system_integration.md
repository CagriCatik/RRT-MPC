# Pipeline Integration Deep Dive

This document connects the RRT* global planner with the MPC-based tracker by
walking through the runtime hand-off points, contracts, and numerical
considerations encoded in the source code. Use it when embedding the pipeline in
research experiments or when swapping out individual components.

## End-to-End Control Flow

1. **Configuration loading** – `src.config.load_config()` deserialises YAML into
   `PipelineConfig`, exposing strongly-typed stage parameters and default seeds
   so reproducible runs do not depend on global state.
2. **Stage construction** – `pipeline.orchestrator.PipelineOrchestrator.run()`
   initialises `MapStage`, `PlanningStage`, and `TrajectoryTracker` with the
   relevant sub-configs while configuring the requested Matplotlib backend. It
   also logs per-stage timings for observability.
3. **Map preparation** – `MapStage.build()` guarantees that a grayscale map
   exists (regenerating it via `maps.generator.MapGenerator` when requested),
   inflates obstacles, and returns a `MapArtifacts` dataclass bundling the
   occupancy grid, start/goal pose, inflation masks, and workspace extents for
   planners operating in continuous space.
4. **Planning** – `PlanningStage.plan()` executes `planning.rrt_star.RRTStarPlanner`
   on the inflated occupancy grid. It wraps the resulting `PlanResult` inside
   `PlanningArtifacts` and applies optional Catmull-Rom smoothing so the control
   stage receives curvature-aware waypoints without extra pre-processing.
5. **Tracking** – `TrajectoryTracker.track()` converts the selected path into an
   MPC reference using `control.ref_builder.build_reference`, solves the MPC via
   `control.mpc_controller.MPCController`, and optionally streams prediction
   frames to the visualisation helpers. The closed-loop state history is stored in
   a `TrackingResult` for downstream analysis.
6. **Aggregation** – The orchestrator returns `PipelineResult`, exposing
   `.map`, `.plan`, and `.control` attributes so CLI commands, notebooks, or tests
   can inspect intermediate artefacts without re-running stages.

## Planner/Controller Interface

- `MapArtifacts` exposes both discrete (raw and inflated occupancy grids) and
  continuous representations (workspace bounds). RRT* consumes the inflated grid
  directly, while the inflation mask supports richer visualisation overlays.
- `PlanResult.path` is guaranteed to contain the smoothed trajectory when
  available. `PlanResult.raw_path` remains accessible, giving numerical
  experiments access to the unsmoothed exploration tree.
- `TrajectoryTracker` reads `PlanResult.success` before starting MPC tracking;
  unsuccessful runs raise immediately so experiments can retry planning or report
  the failure.

## MPC Execution Strategy

- `MPCConfig.to_parameters()` rescales the vehicle wheelbase into pixel units and
  instantiates `MPCParameters`, keeping geometry consistent with the map
  resolution provided by `MapConfig`.
- `_solve_with_relaxation()` attempts a nominal solve first. If OSQP reports
  infeasibility, the helper reduces the reference speed by 40% and widens rate
  limits before retrying. Persistent failures are logged and bubble up as `None`
  to terminate the loop gracefully.
- The main loop in `TrajectoryTracker.track()` integrates the discrete bicycle
  model (`control.vehicle_model.f_discrete`) with the chosen control input and
  records progress every 10% of the configured simulation steps. Distance-based
  path advancement ensures the reference window slides forward only after the
  vehicle leaves the current waypoint neighbourhood.
- Visualisation hooks (`viz.visualization.plot_prediction` and
  `viz.record.FrameRecorder`) are optional and do not affect the returned
  artefacts, enabling headless evaluation while still supporting offline GIF
  generation.

## Determinism & Scaling Considerations

- Planning and map generation use explicit RNG seeds (`PlannerConfig.random_seed`,
  `MapConfig.generator_seed`), ensuring identical
  trees and maps across runs when input parameters match.
- NumPy-based occupancy inflation and reference construction avoid Python loops,
  keeping per-step latency low as map resolution increases.
- OSQP warm starts combined with modest horizons (default 15 steps) strike a
  balance between responsiveness and solve time; adjust `MPCConfig.horizon` and
  penalty matrices when targeting higher-speed manoeuvres.

## Extending the Pipeline

- Replace `MapStage` with a custom implementation that still returns
  `MapArtifacts` to integrate external occupancy providers (e.g. ROS2 costmaps).
- Register new planners inside `PlanningStage.plan()` by dispatching on a config
  key and returning a populated `PlanResult`.
- Swap the MPC with an alternative controller by implementing the
  `TrajectoryTracker.track()` contract; `PipelineResult` will continue to expose
  the new state history for analysis and tooling.
- Use the artefact dataclasses as the single source of truth for new metadata
  fields (e.g. control inputs, cost metrics) so both CLI and API consumers pick
  up changes transparently.
