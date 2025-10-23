"""Command line interface for the MPC + RRT* package."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional, Sequence, Tuple

import click
import numpy as np

from .common.types import FloatArray
from .config import PipelineConfig, default_config, load_config
from .control.mpc_controller import MPCController
from .control.ref_builder import build_reference
from .logging_setup import configure_logging, get_logger
from .maps.generator import MapGenerator
from .maps.inflate import inflate_grayscale_map, to_occupancy_grid
from .maps.io import load_grayscale, save_grayscale
from .planning.plan_result import PlanResult
from .planning.rrt_star import RRTStarPlanner
from .viz.record import FrameRecorder
from .viz.vehicle_draw import VehicleParams
from .viz.visualization import configure_backend, plot_prediction, plot_rrt_star
from .control.vehicle_model import f_discrete

LOG = get_logger(__name__)


def run_pipeline(config: PipelineConfig, *, visualize: bool = True) -> Tuple[PlanResult, Sequence[FloatArray]]:
    configure_logging(logging.INFO)
    configure_backend(config.viz.backend)

    if config.map.generate or not Path(config.map.map_file).exists():
        generator = MapGenerator(config.map.size_m, config.map.generator_resolution, seed=config.map.generator_seed)
        base_map = generator.generate()
        save_grayscale(config.map.map_file, (base_map * 255).astype(np.uint8))
        LOG.info("Generated base map at %s", config.map.map_file)

    raw_map = load_grayscale(config.map.map_file)
    inflated_map = inflate_grayscale_map(raw_map, config.map.inflation_radius_m, config.map.map_resolution)
    save_grayscale(config.map.inflated_map_file, inflated_map)
    occupancy = to_occupancy_grid(inflated_map)
    occupancy = np.flipud(occupancy)

    start = config.map.start
    goal = (occupancy.shape[1] - config.map.goal_offset[0], occupancy.shape[0] - config.map.goal_offset[1])

    planner = RRTStarPlanner(occupancy, config.planner.to_parameters())
    plan_result = planner.plan(start, goal)
    if not plan_result.success:
        raise RuntimeError("RRT* failed to find a path")

    path = plan_result.path
    if not path:
        raise RuntimeError("Empty path returned from planner")

    axis = None
    if visualize:
        import matplotlib.pyplot as plt

        plt.ion()
        axis = plot_rrt_star(
            occupancy,
            start,
            goal,
            plan_result,
            show_tree=config.viz.animate_tree,
            save_path="rrtstar_path.png" if config.viz.animate_tree else None,
            keep_open=True,
        )

    wheelbase_px = config.mpc.wheelbase_m / config.map.map_resolution
    vehicle_params = VehicleParams.from_wheelbase(wheelbase_px)
    controller = MPCController(config.mpc.to_parameters(config.map.map_resolution))

    if len(path) > 1:
        yaw0 = np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    else:
        yaw0 = 0.0
    state = np.array([start[0], start[1], yaw0, 5.0], dtype=float)
    u_prev = np.zeros(2)

    ref_global = build_reference(path, config.mpc.v_px_s, controller.params.horizon, config.mpc.dt)
    states: list[FloatArray] = []

    recorder = FrameRecorder(config.viz.record_dir) if (visualize and config.viz.record_frames) else None
    path_idx = 0
    for step in range(config.mpc.sim_steps):
        end = min(path_idx + controller.params.horizon + 1, len(ref_global))
        ref_window = ref_global[path_idx:end]
        if len(ref_window) < controller.params.horizon + 1:
            tail = np.repeat(ref_window[-1:], controller.params.horizon + 1 - len(ref_window), axis=0)
            ref_window = np.vstack((ref_window, tail))

        u0, Xp, Up = controller.solve(state, ref_window, u_prev=u_prev)
        if u0 is None:
            LOG.warning("MPC infeasible at step %d. Applying relaxed bounds and reduced speed.", step)
            relaxed = ref_window.copy()
            relaxed[:, 3] *= 0.6
            old_du = controller.params.du_bounds
            controller.params.du_bounds = (
                (old_du[0][0] - 5.0, old_du[0][1] + 5.0),
                (old_du[1][0] - 0.05, old_du[1][1] + 0.05),
            )
            u0, Xp, Up = controller.solve(state, relaxed, u_prev=u_prev)
            controller.params.du_bounds = old_du
            if u0 is None:
                LOG.error("MPC infeasible after relaxation at step %d", step)
                break

        if visualize:
            plot_prediction(
                occupancy,
                path,
                Xp,
                state,
                step,
                config.viz.prediction_pause,
                ax=axis,
                vehicle_params=vehicle_params,
            )
            if recorder and axis is not None:
                recorder.capture(axis.figure)

        state = f_discrete(state, u0, controller.params.dt, wheelbase_px)
        states.append(state.copy())
        u_prev = u0.copy()

        if path_idx < len(ref_global) - 2:
            dx = state[0] - ref_global[path_idx][0]
            dy = state[1] - ref_global[path_idx][1]
            if dx * dx + dy * dy > 25.0:
                path_idx += 1

        if np.hypot(state[0] - goal[0], state[1] - goal[1]) < 8.0:
            LOG.info("Reached goal region at step %d", step)
            break

    if visualize:
        import matplotlib.pyplot as plt

        plt.ioff()

    return plan_result, states


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
    configure_logging(logging.INFO)
    generator = MapGenerator(config.map.size_m, config.map.generator_resolution, seed=config.map.generator_seed)
    grid = generator.generate()
    save_grayscale(config.map.map_file, (grid * 255).astype(np.uint8))
    click.echo(f"Map saved to {config.map.map_file}")


@cli.command("inflate-map")
@click.pass_obj
def inflate_map_cmd(config: PipelineConfig) -> None:
    configure_logging(logging.INFO)
    raw_map = load_grayscale(config.map.map_file)
    inflated = inflate_grayscale_map(raw_map, config.map.inflation_radius_m, config.map.map_resolution)
    save_grayscale(config.map.inflated_map_file, inflated)
    click.echo(f"Inflated map written to {config.map.inflated_map_file}")


@cli.command("plan-path")
@click.pass_obj
def plan_path_cmd(config: PipelineConfig) -> None:
    configure_logging(logging.INFO)
    raw_map = load_grayscale(config.map.map_file)
    inflated = inflate_grayscale_map(raw_map, config.map.inflation_radius_m, config.map.map_resolution)
    occupancy = to_occupancy_grid(inflated)
    occupancy = np.flipud(occupancy)
    start = config.map.start
    goal = (occupancy.shape[1] - config.map.goal_offset[0], occupancy.shape[0] - config.map.goal_offset[1])
    planner = RRTStarPlanner(occupancy, config.planner.to_parameters())
    result = planner.plan(start, goal)
    if not result.success:
        raise click.ClickException("Planning failed")
    plot_rrt_star(occupancy, start, goal, result, show_tree=True, save_path="rrtstar_path.png")


@cli.command("run")
@click.pass_obj
def run_cmd(config: PipelineConfig) -> None:
    plan_result, _ = run_pipeline(config, visualize=True)
    status = "succeeded" if plan_result.success else "failed"
    click.echo(f"Pipeline {status}")


def main() -> None:
    cli()


if __name__ == "__main__":
    main()
