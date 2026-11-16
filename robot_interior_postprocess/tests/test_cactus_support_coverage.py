"""Exercise every saved cactus electronic target together, with real wire routes."""

from pathlib import Path

import numpy as np
import trimesh
from hardware import interface_targets
from shell_partition.geometry import candidates
from shell_partition.io import fingerprint, read_layout
from shell_partition.joint import evaluate_joint, mounting_order
from shell_partition.supports import (
    intersection_volume,
    load_support_plan,
    route_obstacles,
    solid,
    union,
)

ROOT = Path(__file__).resolve().parents[1]


def test_hardware_tool_dependencies_reject_reverse_insertion_cycle():
    left = solid(trimesh.creation.box([2, 2, 2]).apply_translation([-5, 0, 0]))
    right = solid(trimesh.creation.box([2, 2, 2]).apply_translation([5, 0, 0]))
    paths = {"lower": (left, left), "upper": (right, right)}
    features = [{"record": {"target_id": "lower"}, "tools": right}]
    order = mounting_order(features, paths, paths, {"lower": left, "upper": right})
    assert order["ordered_target_ids"] == ["lower", "upper"]
    # The lower part cannot be installed after an upper part that blocks its tool.
    paths["upper"] = (left, right)
    assert (
        mounting_order(features, paths, paths, {"lower": left, "upper": right})[
            "status"
        ]
        == "cyclic_mounting_dependencies"
    )


def test_saved_cactus_generates_all_four_electronic_supports():
    layout = ROOT / "outputs/cactus"
    shells, targets, _, _, sources, _ = read_layout(layout)
    plan = load_support_plan(
        ROOT / "configs/cactus_interface_supports.json", targets, shells, sources
    )
    targets = interface_targets(targets, plan)
    wires, wire_status = route_obstacles(layout / "routing.json", sources, 1.5)
    plan["wire_relief"] = route_obstacles(layout / "routing.json", {}, 2.0)[0]
    plane = next(
        c for c in candidates("base", shells["base"], []) if c["id"] == "base:n2:f0.65"
    )
    pieces, report, features = evaluate_joint(
        shells["base"],
        plane,
        [t for t in targets if "base" in t["owners"]],
        plan,
        wires,
        wire_status,
        other_shells=[v for k, v in shells.items() if k != "base"],
        all_targets=targets,
        part_id="base",
    )
    expected = {"BATTERY_1", "REGULATOR_1", "U1_1", "S1_1"}
    design = report["support_design"]
    assert design["all_required_generated"], design["generation_trace"]
    assert {f["record"]["target_id"] for f in features} == expected
    assert design["missing_target_ids"] == []
    order = design["local_mounting_order"]["ordered_target_ids"]
    assert order.index("BATTERY_1") < order.index("REGULATOR_1") < order.index("U1_1")
    hardware = union([solid(t["mesh"]) for t in targets])
    for feature in features:
        occupied = feature.get("occupancy", feature["solid"])
        assert intersection_volume(occupied, hardware) < 0.01
        assert intersection_volume(occupied, wires) < 0.01
        if "fastener_solid" in feature:
            assert intersection_volume(feature["fastener_solid"], hardware) < 0.01
    for target_id, count in [("U1_1", 3), ("REGULATOR_1", 2)]:
        feature = next(f for f in features if f["record"]["target_id"] == target_id)
        interface = plan["interfaces"][target_id]
        assert len(feature["record"]["holes"]) == count
        for actual, hole in zip(feature["record"]["holes"], interface["holes"]):
            expected_point = interface["transform"] @ [*hole["center_mm"], 0, 1]
            assert np.allclose(actual["center_mm"], expected_point[:3])
    assert all(fingerprint(Path(p)) == sha for p, sha in sources.items())
    assert len(pieces) == 2
