"""Configuration dataclasses and YAML loading helpers."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import yaml

from .control.mpc_controller import MPCParameters
from .logging_setup import get_logger
from .planning.rrt_star import PlannerParameters

LOG = get_logger(__name__)


@dataclass
class MapConfig:
    map_file: str = "occupancy_grid.png"
    inflated_map_file: str = "occupancy_grid_inflated.png"
    map_resolution: float = 0.2
    inflation_radius_m: float = 0.6
    start: Tuple[int, int] = (50, 50)
    goal_offset: Tuple[int, int] = (50, 50)
    generate: bool = False
    size_m: Tuple[float, float] = (80.0, 80.0)
    generator_resolution: float = 1.0
    generator_seed: int = 4


@dataclass
class PlannerConfig:
    step_size: float = 10.0
    goal_radius: float = 15.0
    max_iterations: int = 3000
    rewire_radius: float = 25.0
    goal_sample_rate: float = 0.1
    random_seed: int = 13

    def to_parameters(self) -> PlannerParameters:
        return PlannerParameters(
            step=self.step_size,
            goal_radius=self.goal_radius,
            max_iterations=self.max_iterations,
            rewire_radius=self.rewire_radius,
            goal_sample_rate=self.goal_sample_rate,
            random_seed=self.random_seed,
        )


@dataclass
class MPCConfig:
    wheelbase_m: float = 2.8
    dt: float = 0.1
    horizon: int = 12
    v_px_s: float = 28.0
    sim_steps: int = 600
    q: Tuple[Tuple[float, float, float, float], ...] = ((4.0, 0.0, 0.0, 0.0), (0.0, 4.0, 0.0, 0.0), (0.0, 0.0, 0.6, 0.0), (0.0, 0.0, 0.0, 0.1))
    r: Tuple[Tuple[float, float], ...] = ((0.03, 0.0), (0.0, 0.25))
    q_terminal: Tuple[Tuple[float, float, float, float], ...] = ((8.0, 0.0, 0.0, 0.0), (0.0, 8.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.2))
    u_bounds: Tuple[Tuple[float, float], Tuple[float, float]] = ((-35.0, 35.0), (-0.6, 0.6))
    v_bounds: Tuple[float, float] = (0.0, 90.0)
    du_bounds: Tuple[Tuple[float, float], Tuple[float, float]] = ((-12.0, 12.0), (-0.15, 0.15))

    def to_parameters(self, map_resolution: float) -> MPCParameters:
        wb_px = self.wheelbase_m / map_resolution
        return MPCParameters(
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
    with open(path, "r", encoding="utf8") as fh:
        data = yaml.safe_load(fh) or {}
    cfg = PipelineConfig.from_dict(data)
    LOG.info("Loaded configuration from %s", path)
    return cfg


def default_config() -> PipelineConfig:
    return PipelineConfig(map=MapConfig(), planner=PlannerConfig(), mpc=MPCConfig(), viz=VizConfig())
