"""Map preparation stage for the navigation pipeline."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from ..config import MapConfig
from ..common.paths import resolve_plot_path
from ..logging_setup import get_logger
from ..maps.generator import MapGenerator
from ..maps.inflate import inflate_grayscale_map, to_occupancy_grid
from ..maps.io import load_grayscale, save_grayscale
from .artifacts import MapArtifacts

LOG = get_logger(__name__)


@dataclass
class MapStage:
    """Prepare occupancy grids and start/goal states from ``MapConfig``."""

    config: MapConfig

    def ensure_base_map(self) -> np.ndarray:
        """Return the raw grayscale map, generating it when required."""

        path = resolve_plot_path(self.config.map_file)
        if self.config.generate or not path.exists():
            generator = MapGenerator(
                self.config.size_m,
                self.config.generator_resolution,
                seed=self.config.generator_seed,
            )
            base = generator.generate()
            save_grayscale(path, (base * 255).astype(np.uint8))
            LOG.info("Generated base map at %s", path)
        return load_grayscale(path)

    def inflate(self, raw_map: np.ndarray) -> np.ndarray:
        """Return an inflated occupancy map saved to disk."""

        inflated = inflate_grayscale_map(
            raw_map,
            self.config.inflation_radius_m,
            self.config.map_resolution,
        )
        inflated_path = resolve_plot_path(self.config.inflated_map_file)
        save_grayscale(inflated_path, inflated)
        LOG.info("Wrote inflated map to %s", inflated_path)
        return inflated

    def build(self) -> MapArtifacts:
        """Materialize all map-related assets for downstream stages."""

        raw = self.ensure_base_map()
        inflated = self.inflate(raw)
        occupancy = to_occupancy_grid(inflated)
        occupancy = np.flipud(occupancy)
        start = self.config.start
        goal = (
            occupancy.shape[1] - self.config.goal_offset[0],
            occupancy.shape[0] - self.config.goal_offset[1],
        )
        LOG.debug("Map prepared with start=%s goal=%s", start, goal)
        return MapArtifacts(occupancy=occupancy, start=start, goal=goal)


__all__ = ["MapStage"]
