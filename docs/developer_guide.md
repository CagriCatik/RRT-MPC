# Developer Guide

This guide explains how to contribute new features, extend planners/controllers
and keep the prototype production-ready.

## Environment & Tooling

1. **Python version** – the project targets Python 3.11+. Use `pyproject.toml`
   to install dependencies via `pip install -e .[dev]`.
2. **Linters/tests** – execute `pytest` before submitting changes. Type hints
   are enforced through the test-suite; prefer NumPy typing (`np.ndarray`) for
   array-valued functions.
3. **Logging** – route all log statements through `logging_setup.get_logger` to
   keep formatting consistent across CLI and notebooks.

## Pipeline Extension Points

The orchestration layer is intentionally declarative. The relevant entry points
are located in `src/pipeline/`.

- `MapStage.build()` prepares `MapArtifacts` (occupancy grid, start, goal).
  Custom map sources should implement a small helper that returns the same
  dataclass to remain compatible.
- `PlanningStage.plan()` is where new planners can be inserted. Wrap your
  implementation inside a class that consumes `MapArtifacts` and returns a
  `PlanResult`.
- `TrajectoryTracker.track()` performs MPC tracking. To experiment with a new
  controller, implement the same method signature and update the orchestrator to
  inject it. The `TrackingResult` dataclass is intentionally small so it can be
  extended without breaking callers.

## Numerical Stability Tips

- Always rescale units when adjusting map resolution – the MPC operates in pixel
  coordinates, so convert wheelbase and velocity bounds accordingly.
- When modifying `MPCParameters`, prefer the provided dataclasses and avoid
  mutating controller state in-place. Construct a new controller with updated
  bounds so that warm starts remain well-conditioned.
- Use the `curvature_slowdown` helper to keep yaw-rate constraints satisfied
  when altering the reference trajectory.

## Testing Recommendations

- Add unit tests in `tests/` mirroring the module layout (maps, planning,
  control, pipeline). Each stage is deterministic and can be validated with
  fixed seeds.
- For visual components, keep assertions numeric (e.g. number of predicted
  states) rather than image-based snapshots. The `FrameRecorder` is meant for
  manual inspection only.

## Documentation Practices

- Update `docs/architecture.md` when adding new modules or changing the flow.
- Document configuration flags in `docs/configuration.md` and include default
  values.
- Examples and notebooks should import `PipelineOrchestrator` from
  `src.pipeline` to showcase the new API surface.

By following these conventions the prototype remains maintainable while still
supporting rapid algorithmic iteration.
