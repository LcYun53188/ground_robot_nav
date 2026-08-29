"""Tests for removal of traversable surfaces from projective depth."""

import math

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


def _filter(depth, translation_z=0.0, **parameters):
    ray_x, ray_y = _rays(*depth.shape)
    return filter_traversable_depth(
        depth,
        ray_x,
        ray_y,
        np.eye(3, dtype=np.float32),
        np.asarray([0.0, 0.0, translation_z], dtype=np.float32),
        **parameters,
    )


def test_low_ground_band_is_removed_at_image_border():
    """The projective artifact band must include the one-pixel border."""
    depth = np.full((7, 9), 0.10, dtype=np.float32)
    assert np.all(_filter(depth, translation_z=-0.20) == 0.0)


def test_smooth_low_surface_above_ground_band_is_removed():
    """Surface-normal classification removes a low plane above the band."""
    depth = np.full((7, 9), 0.13, dtype=np.float32)
    filtered = _filter(depth, mask_dilation_pixels=0)
    assert np.all(filtered[1:-1, 1:-1] == 0.0)
    assert np.all(filtered[[0, -1], :] > 0.0)


def test_tall_surface_is_preserved():
    """A wall or obstacle above the robot envelope remains in depth."""
    depth = np.full((7, 9), 1.0, dtype=np.float32)
    assert np.array_equal(_filter(depth), depth)


def test_traversable_surface_above_old_height_limit_is_removed():
    """The complete arena ramp is filtered before the robot climbs onto it."""
    depth = np.full((7, 9), 0.45, dtype=np.float32)
    filtered = _filter(depth, translation_z=-0.20, mask_dilation_pixels=0)
    assert np.all(filtered[1:-1, 1:-1] == 0.0)


def test_traversable_surface_below_old_height_limit_is_removed():
    """A pitched chassis must not leave a stripe on distant support terrain."""
    depth = np.full((7, 9), 0.20, dtype=np.float32)
    filtered = _filter(depth, translation_z=-1.0, mask_dilation_pixels=0)
    assert np.all(filtered[1:-1, 1:-1] == 0.0)


def test_slope_scaled_continuity_removes_distant_ramp_pixels():
    """A 30 degree ramp remains smooth when its projected pixels are far apart."""
    height, width = 9, 11
    ray_x = np.broadcast_to(
        np.linspace(-0.3, 0.3, width, dtype=np.float32)[None, :],
        (height, width),
    )
    ray_y = np.broadcast_to(
        np.linspace(-0.4, 0.4, height, dtype=np.float32)[:, None],
        (height, width),
    )
    # With identity camera/base rotation, z is base height and y is horizontal
    # distance. Adjacent (two-pixel) rows differ by 5 cm: larger than the old
    # fixed 4 cm test but exactly a 30 degree traversable surface.
    slope = math.tan(math.radians(30.0))
    # z = 0.25 + slope * (ray_y * z), solved for camera depth. This is a
    # plane in base coordinates rather than a linear ramp in image depth.
    depth = 0.25 / (1.0 - slope * ray_y)
    filtered = filter_traversable_depth(
        depth,
        ray_x,
        ray_y,
        np.eye(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
        max_slope_deg=30.0,
        max_neighbor_height_jump_m=0.015,
        mask_dilation_pixels=0,
    )
    assert np.all(filtered[1:-1, 1:-1] == 0.0)


def test_obliquely_viewed_ramp_is_not_rejected_by_optical_depth_gradient():
    """A valid ramp can have a large z-depth gradient when viewed side-on."""
    height, width = 9, 11
    ray_x = np.broadcast_to(
        np.linspace(-0.70, 0.70, width, dtype=np.float32)[None, :],
        (height, width),
    )
    ray_y = np.broadcast_to(
        np.linspace(-0.2, 0.2, height, dtype=np.float32)[:, None],
        (height, width),
    )
    slope = math.tan(math.radians(30.0))
    # The camera is 1 m above a 30 degree ramp. Its far-side two-pixel
    # optical-depth difference exceeds the former 0.12 m fixed gate, despite
    # the surface being geometrically traversable in base_link.
    depth = 1.0 / (1.0 - slope * ray_x)
    filtered = filter_traversable_depth(
        depth,
        ray_x,
        ray_y,
        np.eye(3, dtype=np.float32),
        np.asarray([0.0, 0.0, -1.0], dtype=np.float32),
        max_slope_deg=30.0,
        max_surface_height_m=0.75,
        max_neighbor_height_jump_m=0.015,
        mask_dilation_pixels=0,
    )
    assert np.max(np.abs(depth[1:-1, 2:] - depth[1:-1, :-2])) > 0.12
    assert np.all(filtered[1:-1, 1:-1] == 0.0)


def test_five_centimeter_step_boundary_is_preserved():
    """A rejected 5 cm step must leave a closed obstacle boundary in depth."""
    depth = np.full((11, 13), 0.10, dtype=np.float32)
    depth[3:8, 4:9] = 0.15
    filtered = _filter(
        depth,
        translation_z=-0.20,
        max_neighbor_height_jump_m=0.015,
        mask_dilation_pixels=1,
    )

    # Smooth ground and the smooth top interior may be removed, but the height
    # discontinuity surrounding the top must remain for nvblox to mark it.
    assert np.all(filtered[3, 4:9] > 0.0)
    assert np.all(filtered[7, 4:9] > 0.0)
    assert np.all(filtered[3:8, 4] > 0.0)
    assert np.all(filtered[3:8, 8] > 0.0)
    assert filtered[5, 6] == 0.0


def test_twenty_centimeter_low_obstacle_keeps_closed_boundary():
    """Expanded ramp filtering must retain a low obstacle's side perimeter."""
    depth = np.full((11, 13), 0.10, dtype=np.float32)
    depth[3:8, 4:9] = 0.30
    filtered = _filter(
        depth,
        translation_z=-0.20,
        max_neighbor_height_jump_m=0.015,
        mask_dilation_pixels=1,
    )

    assert np.all(filtered[3, 4:9] > 0.0)
    assert np.all(filtered[7, 4:9] > 0.0)
    assert np.all(filtered[3:8, 4] > 0.0)
    assert np.all(filtered[3:8, 8] > 0.0)
    assert filtered[5, 6] == 0.0


def test_old_twenty_two_centimeter_cutoff_is_not_applied():
    """Points above the 3 cm ground band must not be deleted by height alone."""
    depth = np.full((7, 9), 0.20, dtype=np.float32)
    filtered = _filter(
        depth,
        translation_z=-0.20,
        max_surface_height_m=-0.01,
        mask_dilation_pixels=0,
    )
    assert np.array_equal(filtered, depth)


def test_invalid_returns_do_not_contaminate_valid_obstacle():
    """Infinite no-returns stay invalid while a tall valid patch remains."""
    depth = np.full((7, 9), np.inf, dtype=np.float32)
    depth[2:5, 3:6] = 1.0
    filtered = _filter(depth)
    assert np.all(np.isinf(filtered[np.isinf(depth)]))
    assert np.all(filtered[2:5, 3:6] == 1.0)
