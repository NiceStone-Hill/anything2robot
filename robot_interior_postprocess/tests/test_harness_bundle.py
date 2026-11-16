"""Shared corridors must preserve wire identity, capacity, and continuity."""

import numpy as np
import pytest
from harness_bundle.network import (
    cable_segment_path,
    pack,
    rooted_grid_tree,
    shared_segments,
    tree_path,
)


def test_packing_contains_every_insulated_cable_without_overlap():
    sizes = (2.0, 2.0, 2.0, 3.0, 3.0)
    radius, layout = pack(sizes, 0.2, 0.5)
    assert sorted(2 * r for x, y, r in layout) == sorted(sizes)
    for i, (x, y, r) in enumerate(layout):
        assert np.hypot(x, y) + r + 0.5 <= radius + 1e-8
        for a, b, s in layout[:i]:
            assert np.hypot(x - a, y - b) >= r + s + 0.2 - 1e-8
    assert radius > pack((3.0,), 0.2, 0.5)[0]


def test_shared_trunk_keeps_distinct_cable_membership_and_continuity():
    paths = {"power": [0, 1, 2, 3], "signal": [4, 1, 2, 5], "local": [0, 1, 4]}
    segments = shared_segments(paths, {0, 3, 4, 5})
    for i, s in enumerate(segments):
        s["id"] = str(i)
    trunk = next(s for s in segments if s["nodes"] == [1, 2])
    assert trunk["cables"] == ["power", "signal"]
    for key, path in paths.items():
        mapped = cable_segment_path(segments, key, path[0], path[-1])
        restored = []
        for item in mapped:
            s = next(s for s in segments if s["id"] == item["segment_id"])
            nodes = s["nodes"][::-1] if item["reverse"] else s["nodes"]
            if restored:
                assert restored[-1] == nodes[0]
            restored += nodes if not restored else nodes[1:]
        assert restored == path
    assert "local" not in trunk["cables"]


def test_grid_tree_avoids_obstacle_and_rejects_disconnected_endpoints():
    free = np.ones((12, 12, 5), bool)
    free[5, 2:10, :] = False
    cells, ids, _dist, previous = rooted_grid_tree(
        free, np.full(free.shape, 8.0), (1, 6, 2)
    )
    path = tree_path(ids[1, 6, 2], ids[10, 6, 2], previous)
    assert free[tuple(cells[path].T)].all()
    assert np.abs(np.diff(cells[path], axis=0)).sum(axis=1).max() == 1
    free[5, :, :] = False
    cells, ids, _dist, previous = rooted_grid_tree(
        free, np.full(free.shape, 8.0), (1, 6, 2)
    )
    with pytest.raises(ValueError, match="Disconnected"):
        tree_path(ids[1, 6, 2], ids[10, 6, 2], previous)


def test_capacity_not_based_on_collapsed_visual_line():
    small = pack((2.0,), 0.2, 0.5)[0]
    full = pack((2.0,) * 29, 0.2, 0.5)[0]
    assert full > 4 * small
    with pytest.raises(ValueError):
        pack((0.0,))


def test_missing_cable_branch_is_not_a_success():
    segments = [{"id": "A", "nodes": [0, 1], "cables": ["power"]}]
    with pytest.raises(ValueError, match="no continuous"):
        cable_segment_path(segments, "signal", 0, 1)


def test_junction_consolidation_keeps_terminals_and_shared_membership():
    from harness_bundle.network import consolidate_junctions

    coords = np.array(
        [[0, 0, 0], [8, 0, 0], [10, 0, 0], [20, 0, 0], [10, 10, 0], [8, -10, 0]], float
    )
    paths = {"one": [0, 1, 2, 3], "two": [0, 1, 2, 4], "three": [0, 1, 5]}
    result = consolidate_junctions(paths, {0, 3, 4, 5}, coords, 3, lambda a, b: True)
    assert result == {"one": [0, 1, 3], "two": [0, 1, 4], "three": [0, 1, 5]}
    segments = shared_segments(result, {0, 3, 4, 5})
    assert next(s for s in segments if s["nodes"] == [0, 1])["cables"] == [
        "one",
        "three",
        "two",
    ]
    assert (
        consolidate_junctions(paths, {0, 3, 4, 5}, coords, 3, lambda a, b: False)
        == paths
    )


def test_growing_tree_attaches_to_existing_trunk_without_cycles():
    from harness_bundle.network import growing_tree

    free = np.ones((20, 8, 4), bool)
    clearance = np.full(free.shape, 10.0)
    cells, ids, _d, _p = rooted_grid_tree(free, clearance, (1, 4, 2))
    terminals = [int(ids[p]) for p in [(1, 4, 2), (18, 4, 2), (10, 6, 2)]]
    pred = growing_tree(free, clearance, terminals, terminals[0])
    for end in terminals[1:]:
        path = tree_path(terminals[0], end, pred)
        assert len(path) == len(set(path))
        assert np.abs(np.diff(cells[path], axis=0)).sum(axis=1).max() == 1


def test_motor_audit_uses_explicit_identity_and_detects_missing_motor_wire():
    from harness_bundle.details import motor_coverage

    motor = {
        "id": "joint_alpha",
        "axial_start_mm": [0, 0, 0],
        "axial_end_mm": [0, 0, 10],
        "radius_mm": 2,
    }
    connections = [
        {
            "id": key,
            "net": key,
            "source": {"instance_id": "board", "port_id": pin},
            "target": {"instance_id": "actuator_Q", "port_id": pin},
            "bundle_id": None,
            "protocol": None,
        }
        for key, pin in [("supply", "V+"), ("return", "V-")]
    ]
    # Device identities need not contain SERVO or motor numbers.
    plan = {"cables": connections}
    cables = [{"id": "supply", "segments": [{"segment_id": "H1"}]}]
    audit = motor_coverage(
        plan,
        cables,
        {"actuator_Q": {"position_mm": [5, 0, 5]}},
        {"actuator_Q": {"motor_id": "joint_alpha"}},
        {"motors": [motor]},
    )
    assert audit["expected_motor_cables"] == 2
    assert not audit["logical_coverage_complete"]
    assert audit["motors"][0]["missing_cables"] == ["return"]
    assert audit["motors"][0]["unresolved_gap_distance_mm"] == 3
    assert not audit["real_connector_connection_complete"]


def test_motor_gap_reference_stays_on_cylinder_not_at_centre():
    from harness_bundle.details import cylinder_reference

    motor = {"axial_start_mm": [0, 0, 0], "axial_end_mm": [0, 0, 10], "radius_mm": 2}
    assert np.allclose(cylinder_reference([5, 0, 5], motor), [2, 0, 5])
    assert np.allclose(cylinder_reference([1, 0, 15], motor), [1, 0, 10])


def test_cable_colours_are_distinct_and_independent_of_input_order():
    from copy import deepcopy

    from harness_bundle.details import annotate_cables

    cables = [
        {
            "id": f"wire-{i}",
            "conductors": [
                {
                    "id": "p",
                    "source": {"port_id": "V+"},
                    "target": {"port_id": "V+"},
                    "protocol": None,
                }
            ],
        }
        for i in range(29)
    ]
    reverse = deepcopy(cables[::-1])
    annotate_cables(cables)
    annotate_cables(reverse)
    assert len({c["color"] for c in cables}) == 29
    assert {c["id"]: c["color"] for c in cables} == {
        c["id"]: c["color"] for c in reverse
    }
    assert all(c["category"] == "power" for c in cables)


def test_smoothing_rejects_moved_endpoints_without_overwriting_mesh(
    tmp_path, monkeypatch
):
    from harness_bundle import smoothing

    original = [[2.0, 2.0, 2.0], [12.0, 2.0, 2.0]]
    segment = {
        "id": "H1",
        "nodes": [0, 1],
        "diameter_mm": 2.0,
        "points_mm": original,
        "assumed_bend_radius_mm": 2.0,
        "file": "meshes/H1.stl",
    }
    (tmp_path / "meshes").mkdir()
    (tmp_path / segment["file"]).write_text("original artifact")
    monkeypatch.setattr(
        smoothing, "curve", lambda *args: np.array([[3.0, 2.0, 2.0], [12.0, 2.0, 2.0]])
    )
    result = smoothing.refine(
        [segment],
        np.asarray(original),
        np.ones((20, 20, 20), bool),
        np.full((20, 20, 20), 20.0),
        np.zeros(3),
        1.0,
        {},
        {"preferred_bend_radius_factor": 4.0, "clearance_mm": 0.5},
        tmp_path,
        {},
    )
    assert segment["points_mm"] == original
    assert (tmp_path / segment["file"]).read_text() == "original artifact"
    assert result[0]["status"] == "kept_previous_checked_geometry"
    assert all(
        a["reason"] == "endpoint changed" for a in result[0]["rejected_attempts"]
    )


def test_smoothing_rejects_new_collision_without_overwriting_mesh(
    tmp_path, monkeypatch
):
    from harness_bundle import smoothing

    original = [[2.0, 2.0, 2.0], [12.0, 2.0, 2.0]]
    segment = {
        "id": "H1",
        "nodes": [0, 1],
        "diameter_mm": 2.0,
        "points_mm": original,
        "assumed_bend_radius_mm": 2.0,
        "file": "meshes/H1.stl",
    }
    (tmp_path / "meshes").mkdir()
    (tmp_path / segment["file"]).write_text("original artifact")
    monkeypatch.setattr(smoothing, "curve", lambda *args: np.asarray(original))
    monkeypatch.setattr(smoothing, "collision", lambda *args: "carrier")
    result = smoothing.refine(
        [segment],
        np.asarray(original),
        np.ones((20, 20, 20), bool),
        np.full((20, 20, 20), 20.0),
        np.zeros(3),
        1.0,
        {},
        {"preferred_bend_radius_factor": 4.0, "clearance_mm": 0.5},
        tmp_path,
        {},
    )
    assert segment["points_mm"] == original
    assert (tmp_path / segment["file"]).read_text() == "original artifact"
    assert result[0]["rejected_attempts"][0]["reason"] == "collision: carrier"
