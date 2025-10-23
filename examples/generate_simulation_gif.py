"""Run the full pipeline, capture frames, and assemble a GIF."""
from __future__ import annotations

from datetime import datetime
from pathlib import Path

from src.common.paths import resolve_plot_dir
from src.config import default_config
from src.pipeline import run_pipeline
from src.viz.record import assemble_gif


def main() -> None:
    cfg = default_config()
    cfg.viz.backend = "Agg"
    cfg.viz.record_frames = True

    frame_dir = resolve_plot_dir(Path(cfg.viz.record_dir) / datetime.now().strftime("%Y%m%d_%H%M%S"))
    cfg.viz.record_dir = str(frame_dir)

    plan, _ = run_pipeline(cfg, visualize=True)
    if not plan.success:
        raise RuntimeError("Pipeline failed; GIF not generated")

    frames = sorted(frame_dir.glob("*.png"))
    if not frames:
        raise RuntimeError("No frames captured during simulation")

    gif_path = assemble_gif(frames, "animations/simulation.gif")
    print(f"GIF saved to {gif_path}")


if __name__ == "__main__":
    main()
