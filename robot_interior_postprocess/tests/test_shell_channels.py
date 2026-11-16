"""Generic channel geometry: rotated shells, guards, transaction and contracts."""

import copy
import json

import manifold3d as md
import numpy as np
import pytest
import trimesh
from shell_channels.assembly import safe_path, validate_connections
from shell_channels.geometry import Surface, check_self_clearance, frame, rounded, sweep
from shell_channels.pipeline import DEFAULTS, route_one, run, settings_from


def fixture(normal=(0, 0, 1), translation=(0, 0, 0), thickness=4):
    world = np.linalg.inv(frame(normal))
    world[:3, 3] = translation
    shell = md.Manifold.cube([60, 60, thickness]).translate([-30, -30, -thickness])
    outer = md.Manifold.cube([60, 60, 30 + thickness]).translate([-30, -30, -thickness])
    from hardware import as_mesh64

    shell = shell.transform(world[:3])
    descriptor = {
        "shell": shell,
        "base": shell,
        "normal": normal,
        "outer": as_mesh64(outer.transform(world[:3])),
        "closed_obstacles": [],
    }
    surface = Surface(shell, shell, normal, dict(DEFAULTS))
    wire = {
        "id": "supply-A",
        "net": "example",
        "diameter_mm": 2,
        "evidence": "synthetic geometry only",
    }
    for side, point in zip(("source", "target"), ((-12, 0, 8), (12, 0, 8))):
        wire[side] = {
            "instance_id": side,
            "port_id": "P",
            "owner_link": "arbitrary-housing",
            "position_mm": trimesh.transform_points([point], world)[0].tolist(),
            "normal": (world[:3, :3] @ [0, 0, 1]).tolist(),
            "evidence": "fixture",
        }
    return surface, descriptor, wire


@pytest.mark.parametrize(
    "normal,translation",
    [
        ((0, 0, 1), (0, 0, 0)),
        ((0, -1, 0), (170, -220, 100)),
        ((0.6, 0, 0.8), (91, -37, 63)),
    ],
)
def test_route_preserves_ports_on_rotated_translated_shell(
    tmp_path, normal, translation
):
    surface, descriptor, wire = fixture(normal, translation)
    changed, _, result = route_one(surface, descriptor, wire, [], tmp_path / "part.stl")
    assert changed.volume() < surface.shell.volume()
    assert result["points_mm"][0] == pytest.approx(wire["source"]["position_mm"])
    assert result["points_mm"][-1] == pytest.approx(wire["target"]["position_mm"])
    assert result["checks"]["remaining_wall_sampled_min_mm"] > DEFAULTS["min_wall_mm"]
    assert result["checks"]["export_reloaded_and_rechecked"]
    assert result["manufacturing_cut_length_mm"] is None


def test_thin_shell_rejected_without_output(tmp_path):
    surface, descriptor, wire = fixture(thickness=1.8)
    with pytest.raises(ValueError, match="safe surface entry"):
        route_one(surface, descriptor, wire, [], tmp_path / "invalid.stl")
    assert not (tmp_path / "invalid.stl").exists()


def test_reentrant_surface_rejected():
    slab = md.Manifold.cube([30, 30, 3]).translate([-15, -15, -3])
    folded = slab + slab.translate([0, 0, -8])
    surface = Surface(folded, folded, [0, 0, 1], DEFAULTS)
    with pytest.raises(ValueError, match="single visible wall"):
        surface.wall([0, 0])


def test_curved_bowl_preserves_exterior_and_uses_actual_inner_surface(tmp_path):
    from hardware import as_mesh64

    outer = md.Manifold.sphere(40, circular_segments=64)
    shell = (outer - md.Manifold.sphere(36, circular_segments=64)) ^ md.Manifold.cube(
        [90, 90, 45]
    ).translate([-45, -45, -45])
    surface = Surface(shell, shell, [0, 0, 1], DEFAULTS)
    descriptor = {"outer": as_mesh64(outer), "closed_obstacles": []}
    wire = {
        "id": "different-shape",
        "diameter_mm": 2,
        "evidence": "synthetic curved bowl",
    }
    for side, y in (("source", -12), ("target", 12)):
        wire[side] = {
            "instance_id": side,
            "port_id": "pin",
            "owner_link": "bowl",
            "position_mm": [0, y, -26],
            "normal": [0, 0, 1],
            "evidence": "synthetic",
        }
    changed, _, result = route_one(surface, descriptor, wire, [], tmp_path / "bowl.stl")
    assert (changed - shell.transform(surface.local[:3])).volume() < 0.001
    points = np.array(result["channel_points_mm"])
    assert np.ptp(points[:, 2]) > 1  # follows curvature, not a flat projected line
    assert result["checks"]["remaining_wall_sampled_min_mm"] > DEFAULTS["min_wall_mm"]


def test_true_circle_fillet_and_radius_not_silently_reduced():
    path = rounded([[0, 0], [20, 0], [20, 20]], 4, 0.2)
    arc = path[(path[:, 0] > 16.01) & (path[:, 1] < 3.99)]
    assert np.linalg.norm(arc - [16, 4], axis=1) == pytest.approx(
        np.full(len(arc), 4), abs=0.002
    )
    with pytest.raises(ValueError, match="Insufficient"):
        rounded([[0, 0], [1, 0], [1, 1]], 4, 0.2)


def test_closed_obstacle_rejects_lead_collision(tmp_path):
    surface, descriptor, wire = fixture()
    descriptor["closed_obstacles"] = [sweep([[0, -12, 3], [0, -12, 9]], 2)]
    with pytest.raises(ValueError, match="intersects installed"):
        route_one(surface, descriptor, wire, [], tmp_path / "invalid.stl")


def test_structural_feature_at_entry_not_cut(tmp_path):
    surface, descriptor, wire = fixture()
    # Use the actual opening frame rather than a fixed world-axis assumption.
    anchor = np.array(wire["source"]["position_mm"])
    post = md.Manifold.cube([6, 6, 6], center=True).translate([anchor[0], anchor[1], 2])
    surface = Surface(
        descriptor["shell"] + post, descriptor["base"], [0, 0, 1], DEFAULTS
    )
    changed, _, result = route_one(
        surface, descriptor, wire, [], tmp_path / "preserved.stl"
    )
    assert (post.transform(surface.local[:3]) - changed).volume() < 0.001
    assert result["checks"]["protected_features_preserved"]


def test_self_contact_and_reversing_lead_are_rejected():
    with pytest.raises(ValueError, match="doubles back"):
        check_self_clearance([[0, 0, 0], [0, 0, 8], [0, 0, -4]], 2)
    with pytest.raises(ValueError, match="self-contact"):
        check_self_clearance(
            [[0, 0, 0], [20, 0, 0], [20, 20, 0], [10, 20, 0], [10, -10, 0]], 2
        )


def test_duplicate_ids_unknown_settings_and_nonfinite_data_rejected(tmp_path):
    _, _, wire = fixture()
    data = {"schema_version": 1, "units": "mm", "cables": [wire, copy.deepcopy(wire)]}
    with pytest.raises(ValueError, match="unique"):
        validate_connections(data)
    data["cables"] = [wire]
    wire["diameter_mm"] = float("nan")
    with pytest.raises(ValueError, match="diameter"):
        validate_connections(data)
    config = tmp_path / "config.json"
    config.write_text(
        json.dumps({"schema_version": 1, "units": "mm", "settings": {"made_up": 1}})
    )
    with pytest.raises(ValueError, match="Unknown"):
        settings_from(config)


def test_output_cannot_overwrite_or_nest_in_source(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    marker = source / "keep.txt"
    marker.write_text("original")
    with pytest.raises(ValueError, match="non-nested"):
        run(source, source / "new")
    with pytest.raises(ValueError, match="non-nested"):
        run(source, source)
    out = tmp_path / "occupied"
    out.mkdir()
    (out / "keep.txt").write_text("existing output")
    with pytest.raises(ValueError, match="never overwritten"):
        run(source, out)
    assert marker.read_text() == "original"


def test_bundle_paths_cannot_escape_source(tmp_path):
    source = tmp_path / "bundle"
    source.mkdir()
    (tmp_path / "outside.stl").write_text("not geometry")
    with pytest.raises(ValueError, match="outside"):
        safe_path(source, "../outside.stl")
