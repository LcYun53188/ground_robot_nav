"""Tests for removal of traversable surfaces from projective depth."""

import numpy as np

from oakd_perception.traversable_depth_filter import (
    filter_traversable_depth,
)


def _rays(height, width):
    columns = np.linspace(-0.2, 0.2, width, dtype=np.float32)
    rows = np.linspace(-0.2, 0.2, height, dtype=np.float32)
    return (
        np.broadcast_to(columns[None, :], (height, width)),
        np.broadcast_to(rows[:, None], (height, width)),
    )


def _filter(depth, **parameters):
    ray_x, ray_y = _rays(*depth.shape)
    return filter_traversable_depth(
        depth,
        ray_x,
        ray_y,
        np.eye(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
        **parameters,
    )


def test_low_ground_band_is_removed_at_image_border():
    """The projective artifact band must include the one-pixel border."""
    depth = np.full((7, 9), 0.10, dtype=np.float32)
    assert np.all(np.isnan(_filter(depth)))


def test_smooth_low_surface_above_ground_band_is_removed():
    """Surface-normal classification removes a low plane above the band."""
    depth = np.full((7, 9), 0.13, dtype=np.float32)
    filtered = _filter(depth, mask_dilation_pixels=0)
    assert np.all(np.isnan(filtered[1:-1, 1:-1]))
    assert np.all(np.isfinite(filtered[[0, -1], :]))


def test_tall_surface_is_preserved():
    """A wall or obstacle above the robot envelope remains in depth."""
    depth = np.full((7, 9), 1.0, dtype=np.float32)
    assert np.array_equal(_filter(depth), depth)


def test_invalid_returns_do_not_contaminate_valid_obstacle():
    """Infinite no-returns stay invalid while a tall valid patch remains."""
    depth = np.full((7, 9), np.inf, dtype=np.float32)
    depth[2:5, 3:6] = 1.0
    filtered = _filter(depth)
    assert np.all(np.isinf(filtered[np.isinf(depth)]))
    assert np.all(filtered[2:5, 3:6] == 1.0)
