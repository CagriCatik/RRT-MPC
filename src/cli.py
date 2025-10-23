"""Command line interface for the MPC + RRT* package."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import click

from .config import PipelineConfig, default_config, load_config
from .logging_setup import configure_logging, get_logger
from .pipeline import MapStage, PlanningStage, run_pipeline
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
    click.echo(f"Map saved to {config.map.map_file}")


@cli.command("inflate-map")
@click.pass_obj
def inflate_map_cmd(config: PipelineConfig) -> None:
    """Inflate the configured map and save the result."""

    configure_logging(logging.INFO)
    stage = MapStage(config.map)
    raw_map = stage.ensure_base_map()
    stage.inflate(raw_map)
    click.echo(f"Inflated map written to {config.map.inflated_map_file}")


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
    plot_rrt_star(maps.occupancy, maps.start, maps.goal, planning.plan, show_tree=True, save_path="rrtstar_path.png")


@cli.command("run")
@click.pass_obj
def run_cmd(config: PipelineConfig) -> None:
    """Execute the full end-to-end pipeline with visualisation enabled."""

    plan_result, _ = run_pipeline(config, visualize=True)
    if not plan_result.success:
        raise click.ClickException("Pipeline failed to reach the goal")


__all__ = ["cli", "run_pipeline"]
