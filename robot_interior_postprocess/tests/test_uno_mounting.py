"""The cactus Uno must use the pinned board holes, never a side-contact fallback."""

import json
from pathlib import Path

import manifold3d as md
import numpy as np
import pytest
import trimesh
from build_uno_interface import build, rotation
from hardware import cylinder_at, load_interface, transform_solid
from shell_partition.geometry import split_shell
from shell_partition.supports import (
    DEFAULTS,
    intersection_volume,
    solid,
    support_options,
    union,
)

ROOT = Path(__file__).resolve().parents[1]


def binding():
    config = json.loads((ROOT / "configs/cactus_uno_mounts.json").read_text())
    return next(r for r in config["targets"] if r["target_id"] == "U1_1")


def test_official_uno_footprints_and_leads_are_reproducible():
    actual = build(ROOT / "hardware_catalog/sources/UNO-TH_Rev3e.brd")
    saved = json.loads(
        (ROOT / "hardware_catalog/arduino_uno_th_rev3e_footprints.json").read_text()
    )
    assert actual == saved
    assert {h["id"]: h["center_mm"] for h in actual["holes"]} == {
        "H1": [66.04, 35.56],
        "H2": [66.04, 7.62],
        "H3": [15.24, 50.8],
        "H4": [13.97, 2.54],
    }
    assert any(k["id"] == "ICSP1:lead:5" for k in actual["keepouts"])
    assert all(not k["id"].startswith("FRAME") for k in actual["keepouts"])


def test_cactus_uno_requires_real_holes_and_has_no_legacy_side_pad():
    request = binding()
    assert not {"bearing_pad", "tray"} & set(request["templates"])
    interface = load_interface(
        request["mounting"], ROOT / "configs", {"id": "U1_1"}, {}
    )
    assert [h["id"] for h in interface["holes"]] == ["H1", "H2", "H4"]
    expected = [[31.75, 10.39, 38.5], [31.75, -17.55, 38.5], [-20.32, -22.63, 38.5]]
    actual = [
        interface["transform"] @ [*h["center_mm"], 0, 1] for h in interface["holes"]
    ]
    assert np.allclose(np.array(actual)[:, :3], expected)
    # All four PCB holes remain present in the displayed physical board slab.
    assert len(interface["record"]["holes"]) == 4
    assert np.isclose(interface["display_mesh"].extents[2], 1.6)


def test_uno_unused_hole_is_blocked_by_real_neighbouring_footprints():
    request = binding()
    interface = load_interface(
        request["mounting"], ROOT / "configs", {"id": "U1_1"}, {}
    )
    h3 = next(h for h in interface["record"]["holes"] if h["id"] == "H3")
    boss = transform_solid(
        cylinder_at(h3["center_mm"], 2.6, -6, 0), interface["transform"]
    )
    assert intersection_volume(boss, interface["body"]) > 0.1
    for hole in interface["holes"]:
        value = transform_solid(
            cylinder_at(hole["center_mm"], 2.6, -6, 0), interface["transform"]
        )
        assert intersection_volume(value, interface["body"]) < 0.01


def test_uno_support_bosses_land_on_pcb_underside_and_keep_bores_open():
    request = binding()
    interface = load_interface(
        request["mounting"], ROOT / "configs", {"id": "U1_1"}, {}
    )
    shell = trimesh.boolean.difference(
        [trimesh.creation.box([180, 160, 160]), trimesh.creation.box([170, 150, 150])],
        engine="manifold",
    )
    plane = {"id": "test:upper", "normal": [0, 0, 1], "offset_mm": 65.0}
    pieces, _ = split_shell(shell, plane, 0.4)
    target = {
        "id": "U1_1",
        "owners": ["base"],
        "kind": "electronics",
        "mesh": interface["mesh"],
    }
    options, trace = support_options(
        target,
        {**request, "templates": ["ribbed_shelf"]},
        solid(pieces[0]),
        solid(pieces[1]),
        interface["body"],
        md.Manifold(),
        plane,
        0,
        0.4,
        [0, 0, 1],
        DEFAULTS,
        interface,
    )
    assert options, trace
    feature = options[0]
    assert len(feature["record"]["holes"]) == 3
    final = union([solid(pieces[0]), feature["solid"]])
    assert intersection_volume(final, interface["body"]) < 0.01
    for hole in interface["holes"]:
        probe = transform_solid(
            cylinder_at(hole["center_mm"], 1.4, -3.9, -0.1), interface["transform"]
        )
        assert intersection_volume(final, probe) < 0.01
    assert all(np.isclose(h["center_mm"][2], 38.5) for h in feature["record"]["holes"])


def test_eagle_mirrored_rotation_preserves_geometry_without_resizing():
    assert np.allclose(rotation("MR90") @ [1, 2], [-2, -1])
    assert np.allclose(rotation("R90") @ [1, 2], [-2, 1])


@pytest.mark.parametrize("cyclic", [False, True])
def test_removable_carriers_record_local_order_and_reject_cycles(cyclic):
    from shell_partition.joint import mounting_order

    left = solid(trimesh.creation.box([2, 2, 2]).apply_translation([-5, 0, 0]))
    right = solid(trimesh.creation.box([2, 2, 2]).apply_translation([5, 0, 0]))
    paths = {"board": (right, right), "battery": (left, left)}
    features = [{"record": {"target_id": "board"}, "detachable": left}]
    if cyclic:
        features.append({"record": {"target_id": "battery"}, "detachable": right})
    order = mounting_order(features, ["board", "battery"], paths)
    if cyclic:
        assert order["status"] == "cyclic_mounting_dependencies"
        assert not order["ordered_target_ids"]
    else:
        assert order["ordered_target_ids"] == ["battery", "board"]
        assert {"before": "battery", "after": "board"} in order["must_precede"]


@pytest.mark.parametrize("crosses_mounting_land", [False, True])
def test_route_relief_preserves_boss_lands_and_source_shell(
    tmp_path, crosses_mounting_land
):
    from shell_partition.joint import evaluate_joint
    from shell_partition.supports import load_support_plan, rod
    from test_mounting_interfaces import PLANE, fixture, room

    _, raw = fixture(tmp_path, "ribbed_shelf")
    config = tmp_path / "supports.json"
    config.write_text(json.dumps(raw))
    shell = room()
    original = shell.vertices.copy()
    targets = [
        {
            "id": "board",
            "owners": ["room"],
            "kind": "electronics",
            "mesh": trimesh.creation.box([20, 16, 12]),
        }
    ]
    plan = load_support_plan(config, targets, {"room": shell}, {})
    x, y = (7, 5) if crosses_mounting_land else (0, 0)
    wires = rod([x, y, -10], [x, y, -4], 1.5)
    plan["wire_relief"] = rod([x, y, -10], [x, y, -4], 2.0)
    _, report, features = evaluate_joint(
        shell,
        PLANE,
        targets,
        plan,
        wires,
        "protected_fixed_tubes",
        part_id="room",
        probe_radius_mm=2.0,
    )
    assert np.array_equal(shell.vertices, original)
    if crosses_mounting_land:
        assert not features
        assert any(
            a["reason"] == "wire_relief_would_cut_mounting_land_or_root"
            for row in report["support_design"]["generation_trace"]
            for a in row["attempts"]
        )
    else:
        assert report["support_design"]["all_required_generated"]
        assert features[0]["record"]["wire_relief"]["applied"]
        assert features[0]["record"]["wire_relief"]["removed_mm3"] > 1
        assert intersection_volume(features[0]["solid"], plan["wire_relief"]) < 0.01
