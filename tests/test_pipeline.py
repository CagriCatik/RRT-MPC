from __future__ import annotations

from pathlib import Path

from src.config import PipelineConfig, default_config
from src.pipeline import MapStage, PipelineOrchestrator


def _configure_tmp_paths(cfg: PipelineConfig, tmp_path: Path) -> None:
    cfg.map.map_file = str(tmp_path / "base.png")
    cfg.map.inflated_map_file = str(tmp_path / "inflated.png")
    cfg.map.generate = True
    cfg.viz.backend = "Agg"
    cfg.viz.animate_tree = False
    cfg.viz.record_frames = False


def test_map_stage_build_creates_artifacts(tmp_path) -> None:
    cfg = default_config()
    _configure_tmp_paths(cfg, tmp_path)

    stage = MapStage(cfg.map)
    artifacts = stage.build()

    assert artifacts.occupancy.size > 0
    assert artifacts.start == cfg.map.start
    assert Path(cfg.map.map_file).exists()
    assert Path(cfg.map.inflated_map_file).exists()


def test_orchestrator_runs_without_visualisation(tmp_path) -> None:
    cfg = default_config()
    _configure_tmp_paths(cfg, tmp_path)
    orchestrator = PipelineOrchestrator(cfg)

    result = orchestrator.run(visualize=False)

    assert result.plan.success
    assert len(result.states) > 0
    assert result.plan.smoothed_path is not None
    assert len(result.plan.smoothed_path) >= len(result.plan.raw_path)
    assert list(result.plan.smoothed_path) == result.plan.path
