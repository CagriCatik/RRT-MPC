# Configuration Reference

Configuration is expressed in YAML and deserialised via `yaml.safe_load` into a
`PipelineConfig` dataclass. The schema mirrors the package structure.

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
```

- `start`: pixel coordinates in image space (origin bottom-left after flip).
- `goal_offset`: subtracted from the upper-right corner to place the goal.
- `generate`: when `true` creates a deterministic synthetic map if the file is
  absent.

## Planner

```yaml
planner:
  step_size: 10.0
  goal_radius: 15.0
  max_iterations: 3000
  rewire_radius: 25.0
  goal_sample_rate: 0.1
  random_seed: 13
```

These fields map directly to `PlannerParameters`. The random seed guarantees
reproducible planning.

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

## Visualisation

```yaml
viz:
  backend: auto
  prediction_pause: 0.05
  animate_tree: true
  record_frames: false
  record_dir: frames
```

- `backend`: choose the Matplotlib backend. The default `auto` mode tries to pick a GUI backend and falls back to headless when necessary.
- `record_frames`: enables PNG frame capture of MPC predictions.

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
  backend: auto
```

Unspecified fields fall back to defaults defined in `config.py`.
