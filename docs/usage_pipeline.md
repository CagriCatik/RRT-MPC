# Pipeline Usage Guide

This guide walks through running the full map → plan → control pipeline via the
command line and Python API.

## Prerequisites

1. Install dependencies with `pip install -e .[dev]`.
2. Ensure the default configuration (or your own YAML file) points to writable
   locations for generated maps and visualisations. Relative paths are resolved
   under the `plots/` directory automatically.

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

5. **Export Simulation GIF** (captures frames then assembles the animation):

   ```bash
   python -m src.cli generate-gif
   ```

Pass a configuration file with `--config custom.yaml` to override defaults.

## Python API

```python
from src import PipelineOrchestrator, load_config

config = load_config("configs/pipeline.yaml")
orchestrator = PipelineOrchestrator(config)
result = orchestrator.run(visualize=False)
print(result.plan.success, len(result.states))
```

The helper `run_pipeline` is still available for quick scripts and returns the
same `(PlanResult, states)` tuple as before. The `PipelineResult` object exposes
the plan, intermediate artifacts and the tracked state history.

## Headless Rendering

Set `viz.backend: "Agg"` in the configuration or call
`configure_backend("Agg")` before plotting. Enable frame recording with
`viz.record_frames: true`; the PNG frames are saved in `plots/<viz.record_dir>/`
and can be converted into GIFs via the CLI or
[`examples/generate_simulation_gif.py`](../examples/generate_simulation_gif.py).
