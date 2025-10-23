"""Execute the full RRT* + MPC pipeline using default settings."""
from __future__ import annotations

from src import PipelineOrchestrator
from src.config import default_config


def main() -> None:
    cfg = default_config()
    orchestrator = PipelineOrchestrator(cfg)
    result = orchestrator.run(visualize=False)
    print(
        "Pipeline success: %s, states tracked: %d" % (result.plan.success, len(result.states))
    )


if __name__ == "__main__":
    main()
