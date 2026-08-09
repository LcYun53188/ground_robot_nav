"""Unit tests for local elevation discontinuity detection."""

import math

import numpy as np

from oakd_perception.cliff_detector import (
    CliffDetectionConfig,
    depth_discontinuity_mask,
    detect_cliff_edges,
    elevation_grid_points,
    terrain_height_mask,
)

import pytest


def _terrain(heights, resolution=0.05):
    points = []
    for ix, height in enumerate(heights):
        for iy in range(-2, 3):
            points.append((
                (ix + 0.5) * resolution,
                (iy + 0.5) * resolution,
                height,
            ))
    return np.asarray(points, dtype=np.float32)


def test_flat_ground_has_no_cliff():
    """A locally level surface must not create cliff points."""
    points = _terrain([-0.11] * 10)
    assert detect_cliff_edges(points, CliffDetectionConfig()).shape == (0, 3)


def test_depth_jump_marks_only_nearer_supported_edge():
    """A depth jump must mark its near side even without a height neighbor."""
    depth = np.ones((8, 10), dtype=np.float32)
    depth[:, 5:] = 1.4
    mask = depth_discontinuity_mask(depth, 0.12, 3, True)
    assert np.all(mask[2:-2, 4])
    assert not np.any(mask[:, 5])


def test_missing_depth_marks_supported_visible_boundary():
    """A coherent no-return area must preserve its visible near boundary."""
    depth = np.ones((8, 10), dtype=np.float32)
    depth[:, 5:] = np.nan
    mask = depth_discontinuity_mask(depth, 0.12, 3, True)
    assert np.all(mask[2:-2, 4])


def test_flat_depth_has_no_discontinuity():
    """A continuous depth surface must not create image-space edges."""
    depth = np.ones((8, 10), dtype=np.float32)
    mask = depth_discontinuity_mask(depth, 0.12, 3, True)
    assert not np.any(mask)


def test_elevation_grid_keeps_highest_surface():
    """Each cell must expose its highest surface for conservative clearing."""
    points = np.asarray([
        (0.01, 0.01, -0.23),
        (0.02, 0.02, -0.11),
        (0.09, 0.01, -0.12),
    ], dtype=np.float32)
    grid = elevation_grid_points(points, 0.08)
    assert grid.shape == (2, 3)
    assert np.max(grid[:, 2]) == pytest.approx(-0.11)


def test_descending_step_marks_upper_edge():
    """A 12 cm descending step must mark its upper edge."""
    points = _terrain([-0.11] * 5 + [-0.23] * 5)
    edges = detect_cliff_edges(points, CliffDetectionConfig())
    assert edges.shape[0] >= 3
    assert np.all(edges[:, 0] < 0.45)
    assert np.allclose(edges[:, 2], -0.11)


def test_five_centimeter_rising_step_marks_upper_edge():
    """A 5 cm rising step that can stop the chassis must be an obstacle."""
    points = _terrain([-0.11] * 5 + [-0.06] * 5, resolution=0.05)
    edges = detect_cliff_edges(points, CliffDetectionConfig())
    assert edges.shape[0] >= 3
    assert np.all(edges[:, 0] >= 0.20)
    assert np.allclose(edges[:, 2], -0.06)


def test_lateral_step_marks_ramp_side_edge():
    """A side drop must be detected independently of travel direction."""
    forward_step = _terrain([-0.11] * 5 + [-0.23] * 5)
    lateral_step = forward_step[:, [1, 0, 2]]
    edges = detect_cliff_edges(lateral_step, CliffDetectionConfig())
    assert edges.shape[0] >= 3
    assert np.all(edges[:, 1] < 0.45)
    assert np.allclose(edges[:, 2], -0.11)


def test_uphill_terrain_is_kept_for_side_edge_detection():
    """The terrain prefilter must retain an uphill ramp."""
    x = np.linspace(0.25, 1.0, 8, dtype=np.float32)
    z = -0.11 + np.tan(np.deg2rad(30.0)) * x
    points = np.column_stack((x, np.zeros_like(x), z))
    mask = terrain_height_mask(points, -0.11, 30.0, 0.08, 0.50, 0.75)
    assert np.all(mask)


def test_uphill_ramp_side_drop_marks_upper_edge():
    """A visible side drop beside an uphill ramp must remain an obstacle."""
    resolution = 0.05
    points = []
    for ix in range(3, 11):
        x = (ix + 0.5) * resolution
        ramp_z = -0.11 + math.tan(math.radians(30.0)) * x
        for iy in range(-4, 5):
            z = ramp_z if iy <= 0 else -0.11
            points.append((x, (iy + 0.5) * resolution, z))
    points = np.asarray(points, dtype=np.float32)
    mask = terrain_height_mask(
        points, -0.11, 30.0, 0.08, 0.50, 0.75
    )
    edges = detect_cliff_edges(
        points[mask], CliffDetectionConfig()
    )
    assert edges.shape[0] >= 3
    assert np.any(edges[:, 0] > 0.4)


def test_30_degree_ramp_is_not_a_cliff():
    """A ramp at the configured traversable limit must remain clear."""
    resolution = 0.05
    height_step = math.tan(math.radians(30.0)) * resolution
    heights = [-0.11 - index * height_step for index in range(6)]
    points = _terrain(heights, resolution)
    config = CliffDetectionConfig(height_tolerance=0.015)
    assert detect_cliff_edges(points, config).shape == (0, 3)
