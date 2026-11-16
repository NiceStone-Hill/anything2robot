"""Generic geometry, fixed endpoints, coverage and assumption boundaries."""

import numpy as np
import pytest
import trimesh
from harness_design.geometry import (
    curve,
    largest_region,
    line_clear,
    rasterize,
    reserve,
    search,
)
from harness_design.pipeline import collision, settings
from shell_channels.geometry import solid, sweep


def test_hollow_raster_keeps_cavity():
    outer = solid(trimesh.creation.box([20, 20, 20]))
    inner = solid(trimesh.creation.box([10, 10, 10]))
    grid = rasterize(outer - inner, np.array([-12.0, -12.0, -12.0]), 2.0, (13, 13, 13))
    assert not grid[6, 6, 6]
    assert grid[10, 6, 6]
    assert not grid[0, 0, 0]


def test_obstacle_detour_keeps_endpoints_and_radius():
    free = np.ones((40, 40, 20), bool)
    free[15:25, 10:30, :] = False
    start = (5, 20, 10)
    end = (35, 20, 10)
    path = search(free, start, end, np.ones(free.shape))
    assert path is not None
    points = curve(path, free, np.zeros(3), 1.0, 2.0)
    assert np.allclose(points[0], start) and np.allclose(points[-1], end)
    assert line_clear(points, free, np.zeros(3), 1.0)


def test_disconnected_route_is_not_reported_as_success():
    free = np.ones((12, 12, 12), bool)
    free[6, :, :] = False
    assert search(free, (2, 2, 2), (9, 9, 9), np.ones(free.shape)) is None
    assert largest_region(free).sum() == 6 * 12 * 12


def test_capsule_collision_and_reservation():
    blocker = solid(trimesh.creation.box([4, 4, 4]).apply_translation([10, 10, 10]))
    tube = sweep([[1, 10, 10], [19, 10, 10]], 1.0)
    assert collision(tube, {"hardware": blocker}) == "hardware"
    free = reserve(
        np.ones((20, 20, 20), bool), [[1, 10, 10], [19, 10, 10]], np.zeros(3), 1.0, 2.0
    )
    assert not free[10, 10, 10] and free[10, 3, 10]


def test_reject_hidden_assumptions(tmp_path):
    import json

    p = tmp_path / "settings.json"
    p.write_text(
        json.dumps({"schema_version": 1, "units": "mm", "mode": "physical_release"})
    )
    with pytest.raises(ValueError, match="explicit"):
        settings(p)


def test_curve_can_avoid_greedy_short_last_leg():
    free = np.ones((35, 35, 12), bool)
    free[14:20, 5:23, :] = False
    path = search(free, (4, 14, 6), (26, 14, 6), np.ones(free.shape))
    points = curve(path, free, np.zeros(3), 1.0, 3.0)
    assert np.allclose(points[[0, -1]], [[4, 14, 6], [26, 14, 6]])


def test_rejected_channel_preserves_spatial_route_and_shell(tmp_path, monkeypatch):
    from harness_design import channels

    cube = trimesh.creation.box([20, 20, 20])
    value = solid(cube)
    bundle = {
        "parts": {"meshes/body.stl": value},
        "space": None,
        "surfaces": {
            "arbitrary_link": {
                "shell": value,
                "base": value,
                "normal": [0, 0, 1],
                "obstacles": [],
                "closed_obstacles": [],
                "outer": cube,
                "file": "meshes/body.stl",
            }
        },
    }
    route = {
        "id": "wire",
        "source": {"owner_link": "arbitrary_link", "position_mm": [0, 0, 20]},
        "target": {"owner_link": "arbitrary_link", "position_mm": [10, 0, 20]},
        "points_mm": [[0, 0, 20], [10, 0, 20]],
        "diameter_mm": 2,
        "length_mm": 10,
    }

    def reject(*args):
        raise ValueError("wall too thin")

    monkeypatch.setattr(channels, "route_one", reject)
    (tmp_path / "meshes").mkdir()
    result, options = channels.channelize(
        bundle, [route], {}, {"min_bend_radius_mm": 4, "clearance_mm": 0.5}, tmp_path
    )
    assert result == [route]
    assert options[0]["method"] == "spatial_fallback"
    assert bundle["parts"]["meshes/body.stl"].volume() == value.volume()
    assert not (tmp_path / "meshes/body.stl").exists()


def test_reservation_does_not_inflate_neighbour_cells_unnecessarily():
    free = np.ones((12, 12, 12), bool)
    result = reserve(free, [[0, 15, 15], [33, 15, 15]], np.zeros(3), 3.0, 2.5)
    assert not result[5, 5, 5]
    assert result[5, 6, 5]  # 3 mm centre separation exceeds the 2.5 mm requirement.
