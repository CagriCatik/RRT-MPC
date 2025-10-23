"""Configuration dataclasses and YAML loading helpers."""
from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Any, Dict, Tuple, Type, TYPE_CHECKING

import numpy as np

from .logging_setup import get_logger
from .planning.rrt import RRTParameters
from .planning.rrt_star import PlannerParameters

if TYPE_CHECKING:  # pragma: no cover - imported only for type checking
    from .control.mpc_controller import MPCParameters

LOG = get_logger(__name__)


@dataclass
class MapConfig:
    map_file: str = "maps/occupancy_grid.png"
    inflated_map_file: str = "maps/occupancy_grid_inflated.png"
    map_resolution: float = 0.8
    inflation_radius_m: float = 0.6
    start: Tuple[int, int] = (70, 70)
    goal_offset: Tuple[int, int] = (60, 60)
    generate: bool = False
    size_m: Tuple[float, float] = (80.0, 80.0)
    generator_resolution: float = 1.0
    generator_seed: int = 4
    rect_obstacles: Tuple[Tuple[float, float, float, float], ...] = ()
    rect_inflation_radius: float = 0.0


@dataclass
class PlannerConfig:
    algorithm: str = "rrt_star"
    step_size: float = 5.0
    goal_radius: float = 10.0
    max_iterations: int = 2000
    rewire_radius: float = 15.0
    goal_sample_rate: float = 0.1
    random_seed: int = 13
    rrt_step: float = 3.0
    rrt_goal_sample_rate: float = 0.07
    rrt_max_iterations: int = 15_000
    rrt_goal_tolerance: float = 4.0
    rrt_collision_step: float = 0.75
    rrt_rng_seed: int = 7
    rrt_prune_path: bool = True
    rrt_spline_samples: int = 20
    rrt_spline_alpha: float = 0.5
    rrt_dedupe_tolerance: float = 1e-9

    def to_parameters(self) -> PlannerParameters:
        return PlannerParameters(
            step=self.step_size,
            goal_radius=self.goal_radius,
            max_iterations=self.max_iterations,
            rewire_radius=self.rewire_radius,
            goal_sample_rate=self.goal_sample_rate,
            random_seed=self.random_seed,
        )

    def to_rrt_parameters(self) -> RRTParameters:
        return RRTParameters(
            step=self.rrt_step,
            goal_sample_rate=self.rrt_goal_sample_rate,
            max_iterations=self.rrt_max_iterations,
            goal_tolerance=self.rrt_goal_tolerance,
            collision_step=self.rrt_collision_step,
            rng_seed=self.rrt_rng_seed,
            prune_path=self.rrt_prune_path,
            spline_samples=self.rrt_spline_samples,
            spline_alpha=self.rrt_spline_alpha,
            dedupe_tolerance=self.rrt_dedupe_tolerance,
        )


@dataclass
class MPCConfig:
    wheelbase_m: float = 2.8
    dt: float = 0.1
    horizon: int = 8
    v_px_s: float = 15.0
    sim_steps: int = 300
    q: Tuple[Tuple[float, float, float, float], ...] = ((4.0, 0.0, 0.0, 0.0), (0.0, 4.0, 0.0, 0.0), (0.0, 0.0, 0.6, 0.0), (0.0, 0.0, 0.0, 0.1))
    r: Tuple[Tuple[float, float], ...] = ((0.03, 0.0), (0.0, 0.25))
    q_terminal: Tuple[Tuple[float, float, float, float], ...] = ((8.0, 0.0, 0.0, 0.0), (0.0, 8.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.2))
    u_bounds: Tuple[Tuple[float, float], Tuple[float, float]] = ((-35.0, 35.0), (-0.6, 0.6))
    v_bounds: Tuple[float, float] = (0.0, 90.0)
    du_bounds: Tuple[Tuple[float, float], Tuple[float, float]] = ((-12.0, 12.0), (-0.15, 0.15))

    def to_parameters(self, map_resolution: float) -> MPCParameters:
        mpc_parameters_cls = _load_mpc_parameters()
        wb_px = self.wheelbase_m / map_resolution
        return mpc_parameters_cls(
            wheelbase_px=wb_px,
            dt=self.dt,
            horizon=self.horizon,
            q=np.array(self.q, dtype=float),
            r=np.array(self.r, dtype=float),
            q_terminal=np.array(self.q_terminal, dtype=float),
            u_bounds=self.u_bounds,
            v_bounds=self.v_bounds,
            du_bounds=self.du_bounds,
        )


@dataclass
class VizConfig:
    backend: str = "auto"
    prediction_pause: float = 0.05
    animate_tree: bool = True
    record_frames: bool = False
    record_dir: str = "frames"


@dataclass
class PipelineConfig:
    map: MapConfig
    planner: PlannerConfig
    mpc: MPCConfig
    viz: VizConfig

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PipelineConfig":
        map_cfg = MapConfig(**data.get("map", {}))
        planner_cfg = PlannerConfig(**data.get("planner", {}))
        mpc_cfg = MPCConfig(**data.get("mpc", {}))
        viz_cfg = VizConfig(**data.get("viz", {}))
        return cls(map=map_cfg, planner=planner_cfg, mpc=mpc_cfg, viz=viz_cfg)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "map": self.map.__dict__,
            "planner": self.planner.__dict__,
            "mpc": self.mpc.__dict__,
            "viz": self.viz.__dict__,
        }


def load_config(path: str | Path) -> PipelineConfig:
    try:
        yaml = import_module("yaml")
    except ModuleNotFoundError as exc:  # pragma: no cover - dependency missing only in CI
        raise RuntimeError(
            "PyYAML is required to load configuration files. Install it with 'pip install pyyaml'."
        ) from exc

    with open(path, "r", encoding="utf8") as fh:
        data = yaml.safe_load(fh) or {}
    cfg = PipelineConfig.from_dict(data)
    LOG.info("Loaded configuration from %s", path)
    return cfg


def _load_mpc_parameters() -> Type["MPCParameters"]:
    try:
        module = import_module("src.control.mpc_controller")
    except ModuleNotFoundError as exc:  # pragma: no cover - dependency missing only in CI
        raise RuntimeError(
            "cvxpy is required for the MPC controller. Install it with 'pip install cvxpy'."
        ) from exc
    return module.MPCParameters


def default_config() -> PipelineConfig:
    return PipelineConfig(
        map=MapConfig(),
        planner=PlannerConfig(),
        mpc=MPCConfig(),
        viz=VizConfig(),
    )
