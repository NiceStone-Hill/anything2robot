"""Regression cases for real hole geometry, attachment and staged assembly."""

import copy
import json

import manifold3d as md
import numpy as np
import pytest
import trimesh
from hardware import eagle_board_geometry, load_interface
from shell_partition.geometry import split_shell
from shell_partition.joint import evaluate_joint
from shell_partition.supports import (
    DEFAULTS,
    intersection_volume,
    load_support_plan,
    rod,
    solid,
    union,
)

PROFILE = {
    "thread": "M2",
    "shank_diameter_mm": 2.0,
    "length_mm": 6.0,
    "head_diameter_mm": 3.6,
    "head_height_mm": 2.0,
    "tool_radius_mm": 2.0,
    "boss_diameter_mm": 6.0,
    "standoff_height_mm": 6.0,
    "min_wall_mm": 1.0,
    "bottom_mm": 1.0,
    "min_engagement_mm": 2.0,
    "insert": {
        "id": "synthetic-M2-test-insert",
        "outer_diameter_mm": 3.6,
        "hole_diameter_mm": 3.2,
        "length_mm": 4.0,
    },
    "process_status": "prototype_uncalibrated",
}
PLANE = {"id": "room:z", "normal": [0, 0, 1], "offset_mm": 15.0}


def room():
    return trimesh.boolean.difference(
        [trimesh.creation.box([100] * 3), trimesh.creation.box([90] * 3)],
        engine="manifold",
    )


def fixture(tmp_path, template="ribbed_shelf", rotation=None):
    definition = {
        "schema_version": 1,
        "units": "mm",
        "id": "test-board",
        "manufacturer": "synthetic",
        "model": "PCB",
        "revision": "test",
        "source": {"url": "test://explicit-holes", "evidence": "synthetic_test"},
        "board": {
            "outline_mm": [[-10, -8], [10, -8], [10, 8], [-10, 8]],
            "thickness_mm": 1.6,
        },
        "holes": [
            {"id": str(i), "center_mm": p, "diameter_mm": 2.2}
            for i, p in enumerate([[-7, -5], [7, -5], [7, 5], [-7, 5]])
        ],
        "keepouts": [
            {"id": "top", "min_mm": [-3, -2, 1.6], "max_mm": [3, 2, 7]},
            {"id": "bottom", "min_mm": [-3, -2, -1], "max_mm": [3, 2, 0]},
        ],
    }
    path = tmp_path / "board.json"
    path.write_text(json.dumps(definition))
    raw = {
        "schema_version": 1,
        "units": "mm",
        "settings": {},
        "targets": [
            {
                "target_id": "board",
                "owner_link": "room",
                "templates": [template],
                "mounting": {
                    "definition": "board.json",
                    "world_from_interface": (
                        np.eye(4) if rotation is None else rotation
                    ).tolist(),
                    "registration": {
                        "status": "explicit_candidate",
                        "note": "Synthetic unit fixture",
                    },
                    "fastener": copy.deepcopy(PROFILE),
                },
            }
        ],
    }
    return path, raw


def evaluate(tmp_path, template="ribbed_shelf", edit=None, wires=None, rotation=None):
    _, raw = fixture(tmp_path, template, rotation)
    if edit:
        edit(raw)
    config = tmp_path / "plan.json"
    config.write_text(json.dumps(raw))
    shell = room()
    plane = dict(PLANE)
    if rotation is not None:
        shell.apply_transform(rotation)
        plane["normal"] = (rotation[:3, :3] @ np.array([0, 0, 1.0])).tolist()
        plane["offset_mm"] = 15 + np.dot(plane["normal"], rotation[:3, 3])
    targets = [
        {
            "id": "board",
            "kind": "electronics",
            "owners": ["room"],
            "mesh": trimesh.creation.box([20, 16, 12]),
        }
    ]
    sources = {}
    plan = load_support_plan(config, targets, {"room": shell}, sources)
    pieces, report, features = evaluate_joint(
        shell,
        plane,
        targets,
        plan,
        wires or md.Manifold(),
        "protected_fixed_tubes",
        part_id="room",
        probe_radius_mm=2.0,
    )
    return pieces, report, features, plan, sources


@pytest.mark.parametrize(
    "template", ["hole_standoffs", "ribbed_shelf", "removable_carrier"]
)
def test_registered_supports_have_open_bores_and_clear_assembly(tmp_path, template):
    pieces, report, features, plan, _ = evaluate(tmp_path, template)
    assert report["support_design"]["all_required_generated"], report["support_design"][
        "generation_trace"
    ]
    assert report["selected_opening"]["all_local_proxy_tasks_clear"]
    feature = features[0]
    assert (
        feature["record"]["contact_model"]
        == "registered_pcb_underside_and_mounting_holes"
    )
    assert len(feature["record"]["holes"]) == 4
    final = union([solid(p) for p in pieces])
    if "detachable" in feature:
        final += feature["detachable"]
        assert intersection_volume(feature["detachable"], feature["solid"]) < 0.01
    assert intersection_volume(final, feature["fastener_solid"]) < 0.01
    for hole in plan["interfaces"]["board"]["holes"]:
        p = hole["center_mm"]
        probe = rod([*p, -3.9], [*p, -0.1], 1.4)
        assert intersection_volume(final, probe) < 0.01
    assert feature["record"]["checks"]["tool_access"]


def test_hole_count_controls_posts_instead_of_four_root_rule(tmp_path):
    path, raw = fixture(tmp_path, "hole_standoffs")
    data = json.loads(path.read_text())
    data["holes"] = data["holes"][:2]
    path.write_text(json.dumps(data))
    target = {"id": "board", "owners": ["room"], "kind": "electronics"}
    interface = load_interface(raw["targets"][0]["mounting"], tmp_path, target, {})
    from shell_partition.supports import support_options

    pieces, _ = split_shell(room(), PLANE, 0.4)
    options, trace = support_options(
        {**target, "mesh": interface["mesh"]},
        raw["targets"][0],
        solid(pieces[0]),
        solid(pieces[1]),
        interface["body"],
        md.Manifold(),
        PLANE,
        0,
        0.4,
        [0, 0, 1],
        DEFAULTS,
        interface,
    )
    assert options, trace
    assert len(options[0]["record"]["roots"]) == 2


def test_rotated_and_translated_holes_follow_single_rigid_frame(tmp_path):
    transform = trimesh.transformations.rotation_matrix(0.63, [1, 2, 3])
    transform[:3, 3] = [8, -4, 12]
    _, report, features, _, _ = evaluate(tmp_path, rotation=transform)
    assert report["support_design"]["all_required_generated"]
    for h, p in zip(
        features[0]["record"]["holes"], [[-7, -5], [7, -5], [7, 5], [-7, 5]]
    ):
        assert np.allclose(
            h["center_mm"], transform[:3, :3] @ [*p, 0] + transform[:3, 3]
        )
        assert np.allclose(h["axis"], transform[:3, 2])


@pytest.mark.parametrize(
    "problem",
    [
        "scaled",
        "reflected",
        "nan",
        "long_screw",
        "short_screw",
        "thin_boss",
        "no_mounting",
        "fallback",
    ],
)
def test_unsafe_registration_and_fasteners_fail_before_generation(tmp_path, problem):
    _, raw = fixture(tmp_path)
    request = raw["targets"][0]
    mounting = request["mounting"]
    if problem == "scaled":
        mounting["world_from_interface"][0][0] = 2
    if problem == "reflected":
        mounting["world_from_interface"][0][0] = -1
    if problem == "nan":
        mounting["world_from_interface"][0][3] = float("nan")
    if problem == "long_screw":
        mounting["fastener"]["length_mm"] = 20
    if problem == "short_screw":
        mounting["fastener"]["length_mm"] = 2
    if problem == "thin_boss":
        mounting["fastener"]["boss_diameter_mm"] = 4
    if problem == "no_mounting":
        del request["mounting"]
    if problem == "fallback":
        request["templates"].append("bearing_pad")
    config = tmp_path / "plan.json"
    config.write_text(json.dumps(raw))
    with pytest.raises(ValueError):
        load_support_plan(
            config,
            [{"id": "board", "owners": ["room"], "kind": "electronics"}],
            {"room": room()},
            {},
        )


def test_wire_and_tool_obstacles_do_not_silently_remove_requested_mount(tmp_path):
    # A routed wire across the entire PCB screw approach blocks every template.
    wires = solid(trimesh.creation.box([80, 80, 2]).apply_translation([0, 0, 12]))
    _, report, features, _, _ = evaluate(tmp_path, wires=wires)
    assert not features
    assert report["support_design"]["missing_target_ids"] == ["board"]
    assert any(
        a["reason"] == "mount_tool_access_blocked"
        for t in report["support_design"]["generation_trace"]
        for a in t["attempts"]
    )


def test_source_and_binding_are_fingerprinted(tmp_path):
    _, _, _, plan, sources = evaluate(tmp_path)
    assert str(tmp_path / "board.json") in sources
    assert str(tmp_path / "plan.json") in sources
    assert (
        plan["interfaces"]["board"]["record"]["definition_sha256"]
        == sources[str(tmp_path / "board.json")]
    )


def test_eagle_import_reads_board_holes_not_vias_or_package_drills(tmp_path):
    path = tmp_path / "test.brd"
    path.write_text("""<eagle><drawing><board><plain>
      <wire x1="0" y1="0" x2="20" y2="0" layer="20"/>
      <wire x1="20" y1="20" x2="20" y2="0" layer="20"/>
      <wire x1="20" y1="20" x2="0" y2="20" layer="20"/>
      <wire x1="0" y1="20" x2="0" y2="0" layer="20"/>
      <hole x="2" y="2" drill="2.2"/><hole x="18" y="18" drill="2.2"/>
      </plain><signals><signal><via x="5" y="5" drill="1"/></signal></signals>
      <libraries><library><packages><package><hole x="0" y="0" drill="4"/>
      </package></packages></library></libraries></board></drawing></eagle>""")
    geometry = eagle_board_geometry(path)
    assert len(geometry["holes"]) == 2
    assert len(geometry["outline_mm"]) == 4


def test_official_uno_revision_is_not_replaced_by_community_hole_array():
    from pathlib import Path

    catalog = Path(__file__).resolve().parents[1] / "hardware_catalog"
    geometry = eagle_board_geometry(catalog / "sources/UNO-TH_Rev3e.brd")
    definition = json.loads((catalog / "arduino_uno_th_rev3e.json").read_text())
    assert geometry["holes"] == definition["holes"]
    assert [15.24, 50.8] in [h["center_mm"] for h in geometry["holes"]]
    assert [13.97, 50.8] not in [h["center_mm"] for h in geometry["holes"]]


def test_source_hash_mismatch_and_untrusted_mount_holes_are_rejected(tmp_path):
    path, raw = fixture(tmp_path)
    definition = json.loads(path.read_text())
    definition["source"].update(file="drawing.txt", sha256="incorrect")
    (tmp_path / "drawing.txt").write_text("changed source")
    path.write_text(json.dumps(definition))
    with pytest.raises(ValueError, match="SHA256"):
        load_interface(raw["targets"][0]["mounting"], tmp_path, {"id": "board"}, {})
    del definition["source"]["file"]
    del definition["source"]["sha256"]
    definition["holes"][0]["center_mm"] = [100, 100]
    path.write_text(json.dumps(definition))
    with pytest.raises(ValueError, match="outside"):
        load_interface(raw["targets"][0]["mounting"], tmp_path, {"id": "board"}, {})


def test_carrier_cli_exports_separate_parts_tools_and_hardware_bom(tmp_path):
    from argparse import Namespace

    from partition_shells import run

    _, raw = fixture(tmp_path, "removable_carrier")
    config = tmp_path / "plan.json"
    config.write_text(json.dumps(raw))
    source = tmp_path / "source"
    (source / "meshes").mkdir(parents=True)
    room().export(source / "meshes/room_hollow_assembly_mm.stl")
    trimesh.creation.box([20, 16, 12]).export(source / "meshes/board_proxy_mm.stl")
    for name, data in {
        "manifest.json": {
            "config": {
                "placement_part": "room",
                "motor_mapping": {},
                "voxel_tool_source": str(tmp_path),
            }
        },
        "mechanical_scene.json": {
            "parts": ["room"],
            "motors": [],
            "units": "millimeter",
            "frame": "test",
        },
        "placement.json": {
            "units": "millimeter",
            "placements": [{"instance_id": "board", "component_id": "board"}],
        },
    }.items():
        (source / name).write_text(json.dumps(data))
    before = {str(p): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    proposal = tmp_path / "proposal.json"
    proposal.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "unknowns": [],
                "parts": [
                    {
                        "part_id": "room",
                        "ranked_candidate_ids": [
                            "room:n2:f0.65",
                            "room:n2:f0.50",
                            "room:n2:f0.35",
                        ],
                        "rationale": "Synthetic integration test",
                        "assembly_notes": [],
                        "unknowns": [],
                    }
                ],
            }
        )
    )
    output = tmp_path / "result"
    assert (
        run(
            Namespace(
                layout=source,
                output=output,
                support_plan=config,
                gap_mm=0.4,
                grip_radius_mm=2.0,
                llm_env=tmp_path / "absent",
                proposal=proposal,
                prepare_only=False,
            )
        )
        == 0
    )
    assert before == {str(p): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    record = json.loads((output / "mounting_plan.json").read_text())["supports"][0]
    assert (output / record["detachable_file"]).is_file()
    assert (output / record["tools_file"]).is_file()
    assert (output / "hardware_interfaces.json").is_file()
    import csv

    with (output / "fasteners_bom.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["count"] == "6"  # four PCB screws, two carrier screws
    assert record["checks"]["carrier_insertion"]


@pytest.mark.parametrize(
    "template", ["hole_standoffs", "ribbed_shelf", "removable_carrier"]
)
def test_official_pololu_cad_keeps_underside_clearance_during_insertion(
    tmp_path, template
):
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    configured = json.loads(
        (root / "configs/cactus_interface_supports.json").read_text()
    )
    profile = next(
        r["mounting"]["fastener"]
        for r in configured["targets"]
        if r["target_id"] == "REGULATOR_1"
    )

    def bind(raw):
        mounting = raw["targets"][0]["mounting"]
        mounting["definition"] = str(root / "hardware_catalog/pololu_d24v50f5.json")
        mounting["fastener"] = profile
        matrix = np.eye(4)
        matrix[:3, 3] = [-8.89, -10.16, 0]
        mounting["world_from_interface"] = matrix.tolist()

    _, report, features, plan, _ = evaluate(tmp_path, template, edit=bind)
    assert report["support_design"]["all_required_generated"], report["support_design"][
        "generation_trace"
    ]
    assert report["selected_opening"]["all_local_proxy_tasks_clear"]
    assert len(features[0]["record"]["holes"]) == 2
    assert plan["interfaces"]["board"]["record"]["sweep_part_count"] > 3
