"""Rejected cuts need their own geometry; model attribution must remain honest."""

import numpy as np
import pytest
import trimesh
from carrier_layout.candidate_preview import candidate_info, cut_diagnostic
from carrier_layout.process_trace import ProcessRecorder


def u_shape():
    return trimesh.boolean.union(
        [
            trimesh.creation.box([4, 10, 20]).apply_translation([-8, 0, 0]),
            trimesh.creation.box([4, 10, 20]).apply_translation([8, 0, 0]),
            trimesh.creation.box([20, 10, 4]).apply_translation([0, 0, -8]),
        ],
        engine="manifold",
    )


def test_diagnostic_exposes_disconnected_regions_without_changing_source():
    mesh = u_shape()
    vertices = mesh.vertices.copy()
    sides, report = cut_diagnostic(mesh, {"normal": [0, 0, 1], "offset_mm": 0}, 0.4)
    assert [len(parts) for parts in sides] == [1, 2]
    assert report["total_material_regions"] == 3
    assert not report["manufacturing_approved"]
    assert all(part.is_volume for side in sides for part in side)
    assert np.array_equal(mesh.vertices, vertices)


def test_rejected_candidates_keep_distinct_offsets_meshes_and_comparison_records(
    tmp_path,
):
    mesh = u_shape()
    recorder = ProcessRecorder(tmp_path)
    previews = []
    for index, offset in enumerate([0, 5]):
        plane = {
            "id": f"torso:assembly:{index}",
            "normal": [0, 0, 1],
            "offset_mm": offset,
        }
        info = candidate_info(
            "torso",
            index,
            {"fraction": 0.5 + offset / 20},
            plane,
            mesh,
            f"source:{index}",
            "configured_architecture_candidates",
        )
        info["reference_asset"] = recorder.shells({"torso": mesh})[0]["asset"]
        info["plane_visual"] = recorder.planes({"torso": mesh}, {"torso": plane})[0]
        recorder.capture(
            "opening_attempt",
            f"Candidate {index + 1}",
            ["Inspect the actual attempted plane"],
            recorder.shells({"torso": mesh}) + [info["plane_visual"]],
            inspection=info,
        )
        instances, report = recorder.diagnostic_candidate(
            {"torso": mesh}, "torso", plane, 0.4
        )
        recorder.capture(
            "candidate_rejected",
            "Rejected",
            ["Disconnected material"],
            instances,
            state="rejected",
            inspection={**info, "diagnostic": report, "status": "rejected"},
        )
        previews.append({i["asset"] for i in instances})
    assert previews[0] != previews[1]
    assert recorder.data["events"][1]["inspection"]["coordinate_mm"] == 0
    assert recorder.data["events"][3]["inspection"]["coordinate_mm"] == 5
    assert len(recorder.data["candidates"]) == 2
    for index, info in enumerate(recorder.data["candidates"].values()):
        assert info["attempt_event_index"] == index * 2
        assert info["result_event_index"] == index * 2 + 1
    assert any(i["role"] == "fragment" for i in recorder.data["events"][1]["instances"])


@pytest.mark.parametrize(
    "provenance,kind,called",
    [
        (
            {
                "source": "saved_schema_replay",
                "live_model_called": False,
                "original_run": {"model": "historical-model"},
            },
            "vlm_replay",
            False,
        ),
        (
            {"source": "live_llm", "live_model_called": True, "model": "current-model"},
            "vlm_live",
            True,
        ),
        (
            {
                "source": "configured_architecture_candidates",
                "live_model_called": False,
            },
            "local",
            False,
        ),
    ],
)
def test_vlm_replay_live_and_algorithm_steps_are_distinguished(
    tmp_path, provenance, kind, called
):
    recorder = ProcessRecorder(tmp_path)
    recorder.set_model_provenance(provenance)
    recorder.capture("architecture_ranking", "Rank cuts", [], [])
    recorder.capture("eps_move", "Move a carrier", [], [])
    first, second = recorder.data["events"]
    assert first["compute"]["kind"] == kind
    assert first["compute"]["live_model_call_at_this_step"] == called
    assert second["compute"]["kind"] == "local"
    assert not second["compute"]["live_model_call_at_this_step"]
