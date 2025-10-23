import numpy as np

from src.maps.inflate import inflate_binary_occupancy, inflation_radius_pixels


def test_inflation_radius_pixels() -> None:
    assert inflation_radius_pixels(0.6, 0.2) == 3


def test_binary_inflation_creates_margin() -> None:
    occ = np.ones((20, 20), dtype=np.uint8)
    occ[10, 10] = 0
    inflated = inflate_binary_occupancy(occ, 2)
    zeros = np.argwhere(inflated == 0)
    assert any(np.all(pt == [10, 10]) for pt in zeros)
    assert any(np.all(pt == [12, 10]) for pt in zeros)
