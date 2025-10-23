# Pipeline Usage Guide

This guide walks through running the full map → plan → control pipeline via the
command line and Python API.

## Prerequisites

1. Install dependencies with `pip install -e .[dev]`.
2. Ensure the default configuration (or your own YAML file) points to writable
   locations for generated maps and visualisations.

## Command Line Workflow

1. **Generate Map** (optional):
   ```bash
   python -m src.cli generate-map
   ```
2. **Inflate Obstacles**:
   ```bash
   python -m src.cli inflate-map
   ```
3. **Run RRT\*** (saves tree and path plots):
   ```bash
   python -m src.cli plan-path
   ```
4. **Execute Full Pipeline**:
   ```bash
   python -m src.cli run
   ```

Pass a configuration file with `--config custom.yaml` to override defaults.

## Python API

```python
from src import PipelineConfig, run_pipeline, load_config

cfg = load_config("configs/pipeline.yaml")
plan_result, states = run_pipeline(cfg, visualize=False)
print(plan_result.success, len(states))
```

The returned `states` sequence contains the simulated vehicle trajectory in
pixel coordinates.

## Headless Rendering

Set `viz.backend: "Agg"` in the configuration or call
`configure_backend("Agg")` before plotting. Enable frame recording with
`viz.record_frames: true` to store prediction snapshots under `viz.record_dir`.
