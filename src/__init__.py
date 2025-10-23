"""Top-level package for the modular MPC + RRT* pipeline."""
from __future__ import annotations

from .config import PipelineConfig, load_config
from .pipeline import PipelineOrchestrator
from .pipeline.api import run_pipeline

__all__ = ["PipelineConfig", "load_config", "run_pipeline", "PipelineOrchestrator"]

__version__ = "0.1.0"
