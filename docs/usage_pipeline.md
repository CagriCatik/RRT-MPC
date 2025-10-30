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

### CLI Options

- `--config PATH` – load parameters from a YAML file.
- `--help` – every sub-command documents its stage-specific arguments (e.g.
  `generate-gif --help`).
- `--keep-frames` (for `generate-gif`) – retain intermediate PNGs for manual
  curation or post-processing.

CLI invocations emit INFO-level logging to the terminal. Redirect to a file for
batch runs (`python -m src.cli run --config sweep.yaml > sweep.log`).

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

Wrap simulations in `with configure_logging(...):` blocks or call
`configure_logging(logging.DEBUG)` for verbose traces when debugging. The
orchestrator logs stage durations and the controller reports progress at regular
intervals, making notebook execution as observable as the CLI.

## Headless Rendering

Set `viz.backend: "Agg"` in the configuration or call
`configure_backend("Agg")` before plotting. Enable frame recording with
`viz.record_frames: true`; the PNG frames are saved in `plots/<viz.record_dir>/`
and can be converted into GIFs via the CLI or `examples/generate_simulation_gif.py`

## Troubleshooting

- If planning fails, inspect the logged iteration summaries to identify whether
  the sampler exhausted iterations or collisions prevented goal connection.
- MPC infeasibility triggers warning logs; check whether `map_resolution`
  changes demand a different `wheelbase_m` or relaxed rate bounds.
- Confirm generated maps exist under `plots/maps/` when running in headless
  environments—`MapStage` logs the resolved paths for quick verification.
