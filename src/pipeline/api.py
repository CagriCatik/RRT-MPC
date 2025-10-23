"""Public helpers for executing the MPC + RRT* pipeline programmatically."""
from __future__ import annotations

import logging
from typing import Sequence, Tuple

from ..common.types import FloatArray
from ..config import PipelineConfig
from ..logging_setup import configure_logging
from ..planning.plan_result import PlanResult
from .orchestrator import PipelineOrchestrator


def run_pipeline(
    config: PipelineConfig,
    *,
    visualize: bool = True,
) -> Tuple[PlanResult, Sequence[FloatArray]]:
    """Execute the configured pipeline and return the plan and tracked states."""

    configure_logging(logging.INFO)
    orchestrator = PipelineOrchestrator(config)
    result = orchestrator.run(visualize=visualize)
    return result.plan, result.states


__all__ = ["run_pipeline"]
