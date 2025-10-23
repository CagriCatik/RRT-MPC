"""Generate a deterministic occupancy grid using the default configuration."""
from __future__ import annotations

from src.config import default_config
from src.maps.generator import MapGenerator
from src.maps.io import save_grayscale


def main() -> None:
    cfg = default_config()
    generator = MapGenerator(cfg.map.size_m, cfg.map.generator_resolution, seed=cfg.map.generator_seed)
    grid = generator.generate()
    save_grayscale(cfg.map.map_file, (grid * 255).astype("uint8"))
    print(f"Map saved to {cfg.map.map_file}")


if __name__ == "__main__":
    main()
