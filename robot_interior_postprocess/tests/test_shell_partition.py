"""Failures that terminal-pose and sampled-lift checks can miss."""

import copy
import json
from argparse import Namespace

import numpy as np
import pytest
import trimesh
from partition_shells import run
from shell_partition.geometry import convex_sweep, evaluate, overlap, split_shell
from shell_partition.model import validate_proposal


def hollow_box():
    return trimesh.boolean.difference(
        [
            trimesh.creation.box([60, 60, 60]),
            trimesh.creation.box([50, 50, 50]),
        ],
        engine="manifold",
    )


def test_continuous_sweep_finds_collision_between_clear_endpoints():
    body = trimesh.creation.box([2, 2, 2])
    obstruction = trimesh.creation.box([0.2, 4, 4]).apply_translation([5, 0, 0])
    assert overlap(body, obstruction) == 0
    assert overlap(body.copy().apply_translation([10, 0, 0]), obstruction) == 0
    assert overlap(convex_sweep(body, [1, 0, 0], 10), obstruction) > 0


def test_plane_through_motor_space_allows_two_sided_clamshell():
    shell = hollow_box()
    motor = trimesh.creation.cylinder(radius=8, height=20, sections=24)
    _, report = evaluate(
        shell,
        {"id": "p:y", "normal": [0, 1, 0], "offset_mm": 0},
        [{"id": "motor", "kind": "motor", "mesh": motor}],
        probe_radius_mm=6,
    )
    assert report["status"] == "local_proxy_access_clear"
    assert report["selected_opening"]["all_local_proxy_tasks_clear"]


def test_deep_cup_rejects_grip_even_when_small_body_fits():
    shell = hollow_box()
    body = trimesh.creation.box([4, 4, 4]).apply_translation([22, 0, -12])
    _, report = evaluate(
        shell,
        {"id": "p:z", "normal": [0, 0, 1], "offset_mm": 15},
        [{"id": "board", "kind": "electronics", "mesh": body}],
        probe_radius_mm=10,
    )
    upward = report["opening_options"][1]["targets"][0]
    assert upward["body_path_clear"]
    assert not upward["assumed_grip_clear"]
    # The planner can rescue this case by removing the large lower half instead.
    assert report["selected_opening"]["panel_index"] == 0


def test_reject_disconnected_shell_fragments():
    a = hollow_box().apply_translation([-45, 0, 0])
    b = hollow_box().apply_translation([45, 0, 0])
    with pytest.raises(ValueError, match="disconnected"):
        split_shell(
            trimesh.util.concatenate([a, b]), {"normal": [0, 0, 1], "offset_mm": 0}, 0.4
        )


def test_sealed_inner_cavity_is_not_a_disconnected_material_fragment():
    shell = hollow_box()
    # The cavity remains entirely in the lower half, leaving two boundary
    # surfaces there, but just one connected region of shell material.
    pieces, _ = split_shell(shell, {"normal": [0, 0, 1], "offset_mm": 27}, 0.4)
    assert len(pieces) == 2
    assert len(pieces[0].split(only_watertight=False)) == 2
    assert pieces[0].is_volume


def test_baseline_proxy_overlap_is_not_an_access_success():
    body = trimesh.creation.box([10, 10, 10]).apply_translation([26, 0, 0])
    _, report = evaluate(
        hollow_box(),
        {"id": "p:y", "normal": [0, 1, 0], "offset_mm": 0},
        [{"id": "unknown_motor", "kind": "motor", "mesh": body}],
        probe_radius_mm=6,
    )
    assert report["status"] == "partial_access"
    assert (
        report["selected_opening"]["targets"][0]["task_status"]
        == "input_proxy_overlap_unresolved"
    )


def test_offline_result_is_reproducible_and_source_layout_is_immutable(tmp_path):
    source = tmp_path / "source"
    (source / "meshes").mkdir(parents=True)
    hollow_box().export(source / "meshes/room_hollow_assembly_mm.stl")
    trimesh.creation.box([10, 10, 10]).export(source / "meshes/board_1_proxy_mm.stl")
    (source / "manifest.json").write_text(
        json.dumps(
            {
                "config": {
                    "placement_part": "room",
                    "motor_mapping": {},
                    "voxel_tool_source": str(tmp_path / "unused"),
                }
            }
        )
    )
    (source / "mechanical_scene.json").write_text(
        json.dumps(
            {
                "parts": ["room"],
                "motors": [],
                "units": "millimeter",
                "frame": "explicit test frame",
            }
        )
    )
    (source / "placement.json").write_text(
        json.dumps(
            {
                "units": "millimeter",
                "placements": [
                    {
                        "instance_id": "board_1",
                        "component_id": "board",
                        "center_mm": [0, 0, 0],
                    }
                ],
            }
        )
    )
    proposal = tmp_path / "synthetic.json"
    proposal.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "unknowns": [],
                "parts": [
                    {
                        "part_id": "room",
                        "ranked_candidate_ids": [f"room:n{i}:f0.50" for i in range(3)],
                        "rationale": "Synthetic integration test, not VLM output",
                        "assembly_notes": [],
                        "unknowns": [],
                    }
                ],
            }
        )
    )
    before = {str(p): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    args = Namespace(
        layout=source,
        output=tmp_path / "output",
        gap_mm=0.4,
        grip_radius_mm=6,
        llm_env=tmp_path / "absent.env",
        proposal=proposal,
        prepare_only=False,
    )
    assert run(args) == 0
    assert before == {str(p): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    result = json.loads((args.output / "partition_report.json").read_text())
    assert result["source_inputs_unchanged"]
    assert result["status"] == "local_proxy_access_clear"
    assert len(list((args.output / "meshes").glob("*.stl"))) == 2
    provenance = json.loads((args.output / "model_provenance.json").read_text())
    assert not provenance["live_model_called"]
    assert (args.output / "index.html").is_file()
    with pytest.raises(ValueError, match="empty"):
        run(args)


def strategy():
    return {
        "schema_version": 1,
        "parts": [
            {
                "part_id": "arm",
                "ranked_candidate_ids": ["a", "b", "c"],
                "rationale": "Motor access",
                "assembly_notes": [],
                "unknowns": [],
            },
            {
                "part_id": "body",
                "ranked_candidate_ids": ["d", "e", "f"],
                "rationale": "Board access",
                "assembly_notes": [],
                "unknowns": [],
            },
        ],
        "unknowns": [],
    }


@pytest.mark.parametrize(
    "failure",
    ["missing_motor_link", "invented_candidate", "duplicate_part", "nonfinite_gap"],
)
def test_model_cannot_omit_links_or_invent_geometry(failure):
    catalog = {"arm": [{"id": s} for s in "abc"], "body": [{"id": s} for s in "def"]}
    raw = copy.deepcopy(strategy())
    if failure == "missing_motor_link":
        raw["parts"] = raw["parts"][1:]
    elif failure == "invented_candidate":
        raw["parts"][0]["ranked_candidate_ids"][0] = "imagined"
    elif failure == "duplicate_part":
        raw["parts"][1] = raw["parts"][0]
    else:
        with pytest.raises(ValueError):
            split_shell(hollow_box(), {"normal": [0, 0, 1], "offset_mm": 0}, np.nan)
        return
    with pytest.raises(ValueError):
        validate_proposal(raw, catalog)
