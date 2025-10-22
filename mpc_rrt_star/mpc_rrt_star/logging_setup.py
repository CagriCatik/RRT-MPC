"""Central logging configuration for the MPC + RRT* pipeline."""
from __future__ import annotations

import logging
from typing import Optional

_LOG_FORMAT = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"


def configure_logging(level: int = logging.INFO, *, force: bool = True) -> None:
    """Configure the root logger with a deterministic format.

    Parameters
    ----------
    level:
        Logging level passed to :func:`logging.basicConfig`.
    force:
        When ``True`` (default) existing handlers are replaced. This keeps
        repeated invocations idempotent during tests and CLI usage.
    """

    logging.basicConfig(level=level, format=_LOG_FORMAT, datefmt="%H:%M:%S", force=force)


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """Return a logger bound to ``name`` using the package defaults."""

    return logging.getLogger(name if name else "mpc_rrt_star")
