"""Geometry regressions for subtractive, port-bound shell routing."""

import manifold3d as md
import numpy as np
import pytest
import trimesh
from generate_shell_groove_trial import (
    InnerWall,
    box,
    clipped_length,
    discard_zero_volume_artifacts,
    export_checked_stl,
    surface_path,
    sweep,
)


def test_inner_surface_is_the_visible_wall_not_a_hidden_exterior_fold():
    shell = box([20, 20, 2], [-10, -10, -8]) + box([20, 20, 2], [-10, -10, -3])
    assert InnerWall(shell).at(0, 0) == [-3, -1]


def test_cleanup_preserves_enclosed_voids():
    shell = box([20, 20, 20], [-10, -10, -10]) - md.Manifold.sphere(3)
    cleaned, _ = discard_zero_volume_artifacts(shell)
    assert cleaned.volume() == pytest.approx(shell.volume(), abs=1e-8)
    assert (cleaned - shell).volume() == pytest.approx(0, abs=1e-8)


def test_export_is_closed_and_preserves_enclosed_void(tmp_path):
    shell = box([20, 20, 20], [-10, -10, -10]) - md.Manifold.sphere(3)
    path = tmp_path / "shell.stl"
    report = export_checked_stl(shell, path)
    reloaded = trimesh.load(path, force="mesh")
    assert reloaded.is_watertight and reloaded.is_volume
    assert reloaded.volume == pytest.approx(shell.volume(), abs=0.01)
    assert report["reloaded_watertight"]


def test_crop_length_does_not_join_disconnected_route_intervals():
    points = [[-2, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 0]]
    assert clipped_length(points, [[-1, -1, -1], [1, 1, 1]]) == pytest.approx(3)


def test_surface_routing_avoids_hardware_instead_of_projecting_through_it():
    shell = box([24, 24, 3], [-12, -12, -5])
    bounds = np.array([[-2, -3, -4], [2, 3, 0]], float)
    ends = np.array([[0, -8, 5], [0, 8, 5]], float)
    path = surface_path(InnerWall(shell), ends, 2, 1, [bounds], md.Manifold(), [])
    assert path[0, :2] == pytest.approx(ends[0, :2])
    assert path[-1, :2] == pytest.approx(ends[-1, :2])
    obstacle = box((bounds[1] - bounds[0]).tolist(), bounds[0].tolist())
    assert (sweep(path, 1) ^ obstacle).volume() < 0.001
    assert np.max(np.abs(path[:, 0])) >= 4
