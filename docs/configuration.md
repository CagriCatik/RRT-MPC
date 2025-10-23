# Configuration Reference

Configuration is expressed in YAML and deserialised via `yaml.safe_load` into a
`PipelineConfig` dataclass. The schema mirrors the package structure.

The CLI accepts a `--config` flag pointing to the YAML file. When omitted, the
package falls back to `config.default_config()` which mirrors the defaults shown
below. Programmatic entry points (`load_config`, `default_config`) share the same
code path, so behaviour is consistent between notebooks, scripts and CLI usage.

All relative paths resolve beneath the repository root. Map generation and plots
use `src.common.paths` to ensure artefacts land in `plots/`, keeping workspaces
tidy even when running multiple experiments.

## Map

```yaml
map:
  map_file: occupancy_grid.png
  inflated_map_file: occupancy_grid_inflated.png
  map_resolution: 0.2   # metres per pixel
  inflation_radius_m: 0.6
  start: [50, 50]
  goal_offset: [50, 50]
  generate: false
  size_m: [80.0, 80.0]
  generator_resolution: 1.0
  generator_seed: 4
  rect_obstacles: []
  rect_inflation_radius: 0.0
```

- `start`: pixel coordinates in image space (origin bottom-left after flip).
- `goal_offset`: subtracted from the upper-right corner to place the goal.
- `generate`: when `true` creates a deterministic synthetic map if the file is
  absent.
- `size_m` / `generator_resolution`: control the procedural generator. Use a
  finer resolution when you need tighter obstacle placement; remember to update
  `map_resolution` so the MPC scaling remains valid.
- `rect_obstacles`: optional list of `[xmin, ymin, xmax, ymax]` rectangles for
  continuous-space planners. Leave empty when relying solely on occupancy
  grids.
- `rect_inflation_radius`: Minkowski inflation (in map units) applied to each
  rectangle before planning.

When `generate: true`, the generator persists the base map to disk. Subsequent
runs reuse the cached image unless `generate` stays true.

## Planner

```yaml
planner:
  algorithm: rrt_star  # or "rrt"
  step_size: 10.0
  goal_radius: 15.0
  max_iterations: 3000
  rewire_radius: 25.0
  goal_sample_rate: 0.1
  random_seed: 13
  rrt_step: 3.0
  rrt_goal_sample_rate: 0.07
  rrt_max_iterations: 15000
  rrt_goal_tolerance: 4.0
  rrt_collision_step: 0.75
  rrt_rng_seed: 7
  rrt_prune_path: true
  rrt_spline_samples: 20
  rrt_spline_alpha: 0.5
  rrt_dedupe_tolerance: 1.0e-9
```

`algorithm` selects the planner implementation. When set to `rrt_star`, the
remaining fields map directly to `PlannerParameters`. When set to `rrt`, the
`rrt_*` fields configure the rectangular-world planner and its smoothing
behaviour. Both planners honour deterministic seeds for reproducible runs.

Runtime overrides can be applied without editing YAML by constructing
`PipelineConfig.from_dict({...})` and merging dictionaries. This is useful when
sweeping hyperparameters inside notebooks.

## MPC

```yaml
mpc:
  wheelbase_m: 2.8
  dt: 0.1
  horizon: 12
  v_px_s: 28.0
  sim_steps: 600
  q: [[4,0,0,0], [0,4,0,0], [0,0,0.6,0], [0,0,0,0.1]]
  r: [[0.03,0], [0,0.25]]
  q_terminal: [[8,0,0,0], [0,8,0,0], [0,0,1,0], [0,0,0,0.2]]
  u_bounds: [[-35,35], [-0.6,0.6]]
  v_bounds: [0, 90]
  du_bounds: [[-12,12], [-0.15,0.15]]
```

`wheelbase_m` is converted to pixels using `map_resolution`. Horizon and solver
settings are passed to the MPC controller unchanged.

`sim_steps` controls the length of the closed-loop roll-out. The controller logs
progress every ~10% of the configured steps so you can monitor long simulations
without enabling debug logs.

## Visualisation

```yaml
viz:
  backend: Agg
  prediction_pause: 0.05
  animate_tree: true
  record_frames: false
  record_dir: frames
```

- `backend`: forwarded to `matplotlib.use` for headless execution.
- `record_frames`: enables PNG frame capture of MPC predictions.
- `prediction_pause`: dwell time between MPC predictions when visualising
  interactively. Increase this to slow down animations for demos.

Frame directories are timestamped when using the GIF generation CLI helper, so
multiple recordings can coexist safely. Clean-up is automatic unless the
`--keep-frames` flag is passed.

## Example Configuration File

```yaml
map:
  map_file: assets/base_map.png
  generate: true
planner:
  random_seed: 123
mpc:
  horizon: 16
viz:
  backend: Agg
```

Unspecified fields fall back to defaults defined in `config.py`.

### Loading Precedence

1. CLI flag `--config` (if provided).
2. `default_config()` values embedded in code.
3. For programmatic usage, pass an already constructed `PipelineConfig` to
   `run_pipeline` to bypass YAML entirely. This is useful in tests where inline
   fixtures keep dependencies self-contained.
