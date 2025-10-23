"""Command line interface for the MPC + RRT* package."""
from __future__ import annotations

import copy
import logging
import shutil
from datetime import datetime
from pathlib import Path
from typing import Optional

import click

from .common.paths import resolve_plot_dir, resolve_plot_path
from .config import PipelineConfig, default_config, load_config
from .logging_setup import configure_logging, get_logger
from .pipeline import MapStage, PlanningStage, run_pipeline
from .viz.record import assemble_gif
from .viz.visualization import configure_backend, plot_rrt_star

LOG = get_logger(__name__)


@click.group()
@click.option("--config", "config_path", type=click.Path(exists=False, path_type=Path), default=None, help="YAML configuration file")
@click.pass_context
def cli(ctx: click.Context, config_path: Optional[Path]) -> None:
    if config_path and config_path.exists():
        ctx.obj = load_config(config_path)
    elif config_path:
        raise click.BadParameter(f"Configuration file not found: {config_path}")
    else:
        ctx.obj = default_config()


@cli.command("generate-map")
@click.pass_obj
def generate_map_cmd(config: PipelineConfig) -> None:
    """Generate and persist the base occupancy grid."""

    configure_logging(logging.INFO)
    stage = MapStage(config.map)
    stage.ensure_base_map()
    click.echo(f"Map saved to {resolve_plot_path(config.map.map_file)}")


@cli.command("inflate-map")
@click.pass_obj
def inflate_map_cmd(config: PipelineConfig) -> None:
    """Inflate the configured map and save the result."""

    configure_logging(logging.INFO)
    stage = MapStage(config.map)
    raw_map = stage.ensure_base_map()
    stage.inflate(raw_map)
    click.echo(f"Inflated map written to {resolve_plot_path(config.map.inflated_map_file)}")


@cli.command("plan-path")
@click.pass_obj
def plan_path_cmd(config: PipelineConfig) -> None:
    """Run RRT* planning only and visualise the resulting tree/path."""

    configure_logging(logging.INFO)
    configure_backend(config.viz.backend)
    stage = MapStage(config.map)
    maps = stage.build()
    planner_stage = PlanningStage(config.planner)
    planning = planner_stage.plan(maps)
    if not planning.plan.success:
        raise click.ClickException("Planning failed")
    LOG.info("Planning succeeded with %d path points", len(planning.plan.path))
    plot_rrt_star(
        maps.occupancy,
        maps.start,
        maps.goal,
        planning.plan,
        show_tree=True,
        inflation_mask=maps.inflation_mask,
        save_path="planner/rrtstar_path.png",
    )


@cli.command("generate-gif")
@click.option(
    "--output",
    "output_path",
    type=click.Path(path_type=Path),
    default=Path("animations/simulation.gif"),
    help="Relative path under 'plots' for the resulting GIF.",
)
@click.option(
    "--frame-duration",
    type=float,
    default=0.05,
    show_default=True,
    help="Seconds per frame in the output GIF.",
)
@click.option(
    "--keep-frames/--no-keep-frames",
    default=False,
    show_default=True,
    help="Keep the intermediate PNG frames instead of deleting them.",
)
@click.pass_obj
def generate_gif_cmd(
    config: PipelineConfig,
    output_path: Path,
    frame_duration: float,
    keep_frames: bool,
) -> None:
    """Run the full simulation, record frames, and assemble a GIF."""

    cfg = copy.deepcopy(config)
    cfg.viz.backend = "Agg"
    cfg.viz.record_frames = True

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    frame_dir = resolve_plot_dir(Path(cfg.viz.record_dir) / timestamp)
    cfg.viz.record_dir = str(frame_dir)

    plan_result, _ = run_pipeline(cfg, visualize=True)
    if not plan_result.success:
        raise click.ClickException("Pipeline failed to reach the goal; GIF not created")

    frames = sorted(frame_dir.glob("*.png"))
    if not frames:
        raise click.ClickException("No frames recorded; ensure visualization is enabled")

    gif_path = assemble_gif(frames, output_path, duration=frame_duration)
    click.echo(f"GIF saved to {gif_path}")

    if not keep_frames:
        shutil.rmtree(frame_dir, ignore_errors=True)


@cli.command("run")
@click.pass_obj
def run_cmd(config: PipelineConfig) -> None:
    """Execute the full end-to-end pipeline with visualisation enabled."""

    plan_result, _ = run_pipeline(config, visualize=True)
    if not plan_result.success:
        raise click.ClickException("Pipeline failed to reach the goal")


__all__ = ["cli", "run_pipeline", "main"]


def main() -> None:
    """Entry-point for ``python -m src.cli`` execution."""

    cli()


if __name__ == "__main__":  # pragma: no cover - manual invocation
    main()
