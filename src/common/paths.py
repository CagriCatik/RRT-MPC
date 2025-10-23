"""Utilities for resolving plot output locations inside ``plots``."""
from __future__ import annotations

from pathlib import Path

PLOTS_ROOT = Path("plots")


def ensure_plots_root() -> Path:
    """Ensure the root ``plots`` directory exists and return it."""

    PLOTS_ROOT.mkdir(parents=True, exist_ok=True)
    return PLOTS_ROOT


def _normalize_relative(path: Path) -> Path:
    if path.parts and path.parts[0] == PLOTS_ROOT.name:
        return Path(*path.parts[1:])
    return path


def resolve_plot_path(path: str | Path) -> Path:
    """Return an absolute path for plot outputs under ``plots``.

    ``path`` may be relative to the repository root or already rooted under the
    ``plots`` directory. Absolute paths are returned unchanged after ensuring
    their parent directories exist so callers can override the location during
    testing.
    """

    ensure_plots_root()
    target = Path(path)
    if target.is_absolute():
        target.parent.mkdir(parents=True, exist_ok=True)
        return target
    normalized = _normalize_relative(target)
    final = PLOTS_ROOT / normalized
    final.parent.mkdir(parents=True, exist_ok=True)
    return final


def resolve_plot_dir(path: str | Path) -> Path:
    """Ensure ``path`` exists under ``plots`` and return it."""

    directory = resolve_plot_path(path)
    directory.mkdir(parents=True, exist_ok=True)
    return directory


__all__ = ["PLOTS_ROOT", "ensure_plots_root", "resolve_plot_dir", "resolve_plot_path"]
