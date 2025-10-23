"""Top-level package for the modular MPC + RRT* pipeline."""
from __future__ import annotations

from .config import PipelineConfig, load_config
from .cli import run_pipeline

__all__ = ["PipelineConfig", "load_config", "run_pipeline"]

__version__ = "0.1.0"
