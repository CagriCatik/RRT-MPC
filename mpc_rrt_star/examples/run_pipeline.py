"""Execute the full RRT* + MPC pipeline using default settings."""
from __future__ import annotations

from mpc_rrt_star import run_pipeline
from mpc_rrt_star.config import default_config


def main() -> None:
    cfg = default_config()
    plan_result, states = run_pipeline(cfg, visualize=False)
    print(f"Pipeline success: {plan_result.success}, states tracked: {len(states)}")


if __name__ == "__main__":
    main()
