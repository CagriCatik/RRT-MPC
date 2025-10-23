"""Public helpers for executing the MPC + RRT* pipeline programmatically."""
from __future__ import annotations

import logging
from typing import Sequence, Tuple

from ..common.types import FloatArray
from ..config import PipelineConfig
from ..logging_setup import configure_logging, get_logger
from ..planning.plan_result import PlanResult
from .orchestrator import PipelineOrchestrator


LOG = get_logger(__name__)


def run_pipeline(
    config: PipelineConfig,
    *,
    visualize: bool = True,
) -> Tuple[PlanResult, Sequence[FloatArray]]:
    """Execute the configured pipeline and return the plan and tracked states."""

    configure_logging(logging.INFO)
    LOG.info("Launching pipeline (visualize=%s)", visualize)
    orchestrator = PipelineOrchestrator(config)
    result = orchestrator.run(visualize=visualize)
    LOG.info(
        "Pipeline execution completed (success=%s, tracked_states=%d)",
        result.plan.success,
        len(result.states),
    )
    return result.plan, result.states


__all__ = ["run_pipeline"]
