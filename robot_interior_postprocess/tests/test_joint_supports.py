"""Ownership, imported mount preservation and final-piece assembly regressions."""

import json
from argparse import Namespace

import manifold3d as md
import numpy as np
import pytest
import trimesh
from partition_shells import run
from shell_partition.geometry import split_shell
from shell_partition.joint import evaluate_joint
from shell_partition.supports import (
    DEFAULTS,
    attach_existing,
    connected_material,
    intersection_volume,
    load_support_plan,
    rod,
    route_obstacles,
    solid,
    within_half,
)


def shell():
    return trimesh.boolean.difference(
        [trimesh.creation.box([60, 60, 60]), trimesh.creation.box([50, 50, 50])],
        engine="manifold",
    )


def target(name="board", owners=None, kind="electronics"):
    return {
        "id": name,
        "owners": owners or ["room"],
        "kind": kind,
        "mesh": trimesh.creation.box([10, 10, 10]),
    }


def plan(settings=None, templates=None):
    return {
        "settings": {**DEFAULTS, "beam_width": 2, **(settings or {})},
        "existing": [],
        "targets": [
            {
                "target_id": "board",
                "owner_link": "room",
                "templates": templates or ["bearing_pad", "tray"],
            }
        ],
    }


PLANE = {"id": "room:y", "normal": [0, 1, 0], "offset_mm": 0}


def joint(support_plan=None, wires=None):
    return evaluate_joint(
        shell(),
        PLANE,
        [target()],
        support_plan or plan(),
        md.Manifold() if wires is None else wires,
        "protected_fixed_tubes",
        part_id="room",
        probe_radius_mm=3,
    )


def test_supports_belong_to_one_piece_and_final_paths_remain_clear():
    pieces, report, features = joint()
    design = report["support_design"]
    assert design["all_required_generated"]
    assert report["selected_opening"]["all_local_proxy_tasks_clear"]
    assert len(features) == 1
    feature = features[0]
    record = feature["record"]
    assert record["owner_link"] == "room"
    assert len(record["roots"]) == 4
    assert record["fixation"].startswith("pending")
    assert within_half(feature["solid"], PLANE, record["owner_piece_index"], 0.4)
    assert intersection_volume(feature["solid"], solid(target()["mesh"])) < 0.01
    assert all(connected_material(solid(p)) for p in pieces)
    assert sum(p.volume for p in pieces) > shell().volume - report["seam_removed_mm3"]


def test_tray_is_an_explicit_template_and_is_not_clipped_at_seam():
    _, report, features = joint(plan(templates=["tray"]))
    assert report["support_design"]["all_required_generated"]
    assert features[0]["record"]["template"] == "tray"
    assert within_half(
        features[0]["solid"], PLANE, features[0]["record"]["owner_piece_index"], 0.4
    )


def test_fixed_wire_forces_a_different_owner_piece():
    # Whichever side the unconstrained search chooses, place a wire through its
    # required bearing face. The opposite opening should remain available.
    _, plain, _ = joint(plan(templates=["bearing_pad"]))
    first_owner = plain["support_design"]["installation_piece_index"]
    sign = -1 if first_owner == 0 else 1
    wire = rod([-8, sign * 6, 0], [8, sign * 6, 0], 1)
    _, report, features = joint(plan(templates=["bearing_pad"]), wire)
    assert report["support_design"]["all_required_generated"]
    assert report["support_design"]["installation_piece_index"] != first_owner
    assert all(intersection_volume(f["solid"], wire) < 0.01 for f in features)
    reasons = [
        a.get("reason")
        for r in report["support_design"]["generation_trace"]
        for a in r["attempts"]
    ]
    assert "support_platform_blocks_fixed_route" in reasons


def test_no_roots_remains_unresolved_even_when_bare_shell_access_is_clear():
    _, report, features = joint(plan(settings={"max_anchor_mm": 2}))
    assert not features
    assert report["selected_opening"]["all_local_proxy_tasks_clear"]
    assert not report["support_design"]["all_required_generated"]
    assert report["support_design"]["missing_target_ids"] == ["board"]
    assert report["status"] == "partial_support_or_access"


def test_existing_bridge_is_rejected_instead_of_cut_into_support_fragments():
    pieces, _ = split_shell(shell(), PLANE, 0.4)
    bridge = trimesh.creation.box([6, 60, 3])
    with pytest.raises(ValueError, match="crosses_seam"):
        attach_existing(
            {"id": "mount", "solid": solid(bridge)},
            [solid(p) for p in pieces],
            PLANE,
            0.4,
        )


def test_valid_existing_mount_is_kept_whole_with_original_geometry():
    support_mesh = trimesh.creation.box([10, 8, 10]).apply_translation([0, -25, 0])
    value = solid(support_mesh)
    p = {
        "settings": DEFAULTS.copy(),
        "targets": [],
        "existing": [
            {
                "id": "known_mount",
                "target_id": "board",
                "owner_link": "room",
                "solid": value,
                "mesh_source": "explicit_test_geometry.stl",
            }
        ],
    }
    _, report, features = joint(p)
    assert report["support_design"]["imported_count"] == 1
    assert np.isclose(features[0]["solid"].volume(), support_mesh.volume)
    assert features[0]["record"]["owner_piece_index"] == 0


@pytest.mark.parametrize(
    "failure",
    [
        "cross_link",
        "motor_without_interface",
        "unknown_template",
        "unknown_setting",
        "nonfinite",
        "duplicate",
    ],
)
def test_support_contract_rejects_unsafe_or_invented_input(tmp_path, failure):
    raw = {
        "schema_version": 1,
        "units": "mm",
        "targets": [
            {"target_id": "board", "owner_link": "room", "templates": ["bearing_pad"]}
        ],
    }
    targets = [target()]
    if failure == "cross_link":
        raw["targets"][0]["owner_link"] = "other_link"
    elif failure == "motor_without_interface":
        targets[0]["kind"] = "motor"
    elif failure == "unknown_template":
        raw["targets"][0]["templates"] = ["imaginary_motor_holes"]
    elif failure == "unknown_setting":
        raw["settings"] = {"disable_wire_check": True}
    elif failure == "nonfinite":
        raw["settings"] = {"embed_mm": float("nan")}
    elif failure == "duplicate":
        raw["targets"].append(raw["targets"][0])
    path = tmp_path / "supports.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError):
        load_support_plan(path, targets, {"room": shell()}, {})


def test_imported_fused_feature_without_root_is_rejected(tmp_path):
    trimesh.creation.box([5, 5, 5]).export(tmp_path / "floating.stl")
    raw = {
        "schema_version": 1,
        "units": "mm",
        "targets": [],
        "existing_supports": [
            {
                "id": "floating",
                "target_id": "board",
                "owner_link": "room",
                "mesh": "floating.stl",
                "attachment": "fused",
            }
        ],
    }
    path = tmp_path / "supports.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="no positive-volume root"):
        load_support_plan(path, [target()], {"room": shell()}, {})


def test_joint_generation_is_not_tied_to_world_axes_or_gravity():
    transform = trimesh.transformations.rotation_matrix(np.deg2rad(33), [0, 0, 1])
    body = target()
    body["mesh"].apply_transform(transform)
    plane = {
        **PLANE,
        "normal": (transform[:3, :3] @ np.array(PLANE["normal"])).tolist(),
    }
    _, report, features = evaluate_joint(
        shell().apply_transform(transform),
        plane,
        [body],
        plan(),
        md.Manifold(),
        "pending_no_source_routing",
        part_id="room",
        probe_radius_mm=3,
    )
    assert report["support_design"]["all_required_generated"]
    assert report["selected_opening"]["all_local_proxy_tasks_clear"]
    assert not features[0]["record"]["wire_clear"]


def test_each_support_preserves_every_requested_body_path():
    targets = [target(), target("board2")]
    targets[0]["mesh"].apply_translation([-9, 0, 0])
    targets[1]["mesh"].apply_translation([9, 0, 0])
    spec = plan(templates=["bearing_pad"])
    spec["targets"].append(
        {"target_id": "board2", "owner_link": "room", "templates": ["bearing_pad"]}
    )
    _, report, features = evaluate_joint(
        shell(),
        PLANE,
        targets,
        spec,
        md.Manifold(),
        "pending_no_source_routing",
        part_id="room",
        probe_radius_mm=3,
    )
    assert len(features) == 2
    assert report["support_design"]["all_required_generated"]
    assert all(t["body_path_clear"] for t in report["selected_opening"]["targets"])


@pytest.mark.parametrize("find_roots", [True, False])
def test_joint_cli_preserves_sources_and_does_not_hide_missing_supports(
    tmp_path, find_roots
):
    source = tmp_path / "source"
    (source / "meshes").mkdir(parents=True)
    shell().export(source / "meshes/room_hollow_assembly_mm.stl")
    target()["mesh"].export(source / "meshes/board_proxy_mm.stl")
    for name, data in {
        "manifest.json": {
            "config": {
                "placement_part": "room",
                "motor_mapping": {},
                "voxel_tool_source": str(tmp_path / "unused"),
                "wire_radius_mm": 1,
            }
        },
        "mechanical_scene.json": {
            "parts": ["room"],
            "motors": [],
            "units": "millimeter",
            "frame": "test world",
        },
        "placement.json": {
            "units": "millimeter",
            "placements": [{"instance_id": "board", "component_id": "board"}],
        },
        "routing.json": {
            "units": "mm",
            "routes": [{"points_mm": [[18, 18, -10], [18, 18, 10]]}],
        },
    }.items():
        (source / name).write_text(json.dumps(data))
    proposal = tmp_path / "proposal.json"
    proposal.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "unknowns": [],
                "parts": [
                    {
                        "part_id": "room",
                        "ranked_candidate_ids": [f"room:n{i}:f0.50" for i in range(3)],
                        "rationale": "Synthetic joint-search integration input, not a model response",
                        "assembly_notes": [],
                        "unknowns": [],
                    }
                ],
            }
        )
    )
    config = tmp_path / "supports.json"
    config.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "units": "mm",
                "settings": {"max_anchor_mm": 250 if find_roots else 2},
                "targets": plan(templates=["bearing_pad"])["targets"],
            }
        )
    )
    before = {str(p): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    args = Namespace(
        layout=source,
        output=tmp_path / "result",
        gap_mm=0.4,
        grip_radius_mm=3,
        support_plan=config,
        llm_env=tmp_path / "missing.env",
        proposal=proposal,
        prepare_only=False,
    )
    assert run(args) == (0 if find_roots else 2)
    assert before == {str(p): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    ownership = json.loads((args.output / "support_ownership.json").read_text())
    assert len(ownership["supports"]) == (1 if find_roots else 0)
    if find_roots:
        record = ownership["supports"][0]
        assert (args.output / record["mesh_file"]).is_file()
        assert (args.output / record["owner_piece_file"]).is_file()
        final = trimesh.load(args.output / record["owner_piece_file"], force="mesh")
        support = trimesh.load(args.output / record["mesh_file"], force="mesh")
        assert (
            intersection_volume(solid(final), solid(support)) > support.volume * 0.999
        )
    else:
        assert (
            ownership["target_coverage"][0]["status"]
            == "support_unresolved_or_not_requested"
        )
    hashes = json.loads((args.output / "input_hashes.json").read_text())
    assert str(source / "routing.json") in hashes
    assert str(config) in hashes


def test_missing_routing_is_explicit_not_a_wire_clearance_certificate(tmp_path):
    _, status = route_obstacles(tmp_path / "absent.json", {}, 1.5)
    assert status == "pending_no_source_routing"
