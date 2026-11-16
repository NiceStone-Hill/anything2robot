"""A presentation condenses execution history without fabricating solver states."""

from copy import deepcopy

import numpy as np
import pytest
from carrier_layout.process_presentation import pipeline_presentation


def example_trace():
    sequence = [
        ("mechanical_input", None),
        ("hollow_geometry", None),
        ("carrier_profiles", None),
        ("architecture_ranking", None),
        ("opening_attempt", "body"),
        ("opening_candidate", "body"),
        ("opening_attempt", "c1"),
        ("candidate_rejected", "c1"),
        ("opening_attempt", "c2"),
        ("candidate_rejected", "c2"),
        ("opening_attempt", "c3"),
        ("opening_candidate", "c3"),
        ("eps_seed", "c3"),
        ("eps_move", "c3"),
        ("eps_move", "c3"),
        ("eps_move", "c3"),
        ("candidate_supports", "c3"),
        ("opening_attempt", "c4"),
        ("opening_candidate", "c4"),
        ("candidate_rejected", "c4"),
        ("opening_attempt", "right"),
        ("opening_candidate", "right"),
        ("opening_attempt", "left"),
        ("opening_candidate", "left"),
        ("selected_layout", None),
        ("routing", None),
        ("post_duct_split", None),
        ("shell_fastening", None),
        ("final_validation", None),
    ]
    items = [
        {"id": f"p{i}", "asset": "box", "transform": np.eye(4).tolist()}
        for i in range(3)
    ]
    events = []
    move = 0
    for index, (stage, key) in enumerate(sequence):
        if stage == "eps_move":
            items[move]["transform"][0][3] = (move + 1) * 6
            move += 1
        events.append(
            {
                "index": index,
                "stage": stage,
                "title": stage,
                "changes": [stage],
                "state": "recorded",
                "coordinate_frame": "world_assembly_mm",
                "instances": deepcopy(items),
                "metrics": {},
                "compute": {
                    "kind": "vlm_replay"
                    if stage == "architecture_ranking"
                    else "local",
                    "live_model_call_at_this_step": False,
                },
                "inspection": {
                    "key": key,
                    "link": key,
                    "reason": "disconnected regions",
                }
                if key
                else None,
            }
        )
    return {
        "events": events,
        "assets": {"box": {"source_volume_mm3": 1}},
        "candidates": {
            key: {"selected": key in {"c3", "body", "right", "left"}}
            for key in {key for _, key in sequence if key}
        },
    }


def test_overview_has_fourteen_chapters_and_every_record_is_accounted_for():
    trace = example_trace()
    result = pipeline_presentation(trace)
    assert len(result["events"]) == 14
    assert sorted(
        i for e in result["events"] for i in e["source_event_indices"]
    ) == list(range(29))
    assert [e["representative_event_index"] for e in result["events"]] == sorted(
        e["representative_event_index"] for e in result["events"]
    )
    assert result["events"][5]["title"] == "Open the electronics-bearing shell"
    assert result["events"][9]["stage"] == "selected_layout"
    assert len([e for e in result["events"] if e["stage"] == "candidate_rejected"]) == 1


def test_overview_keeps_raw_trace_unchanged_and_uses_only_recorded_scenes():
    trace = example_trace()
    original = deepcopy(trace)
    result = pipeline_presentation(trace)
    assert trace == original
    for chapter in result["events"]:
        event = trace["events"][chapter["representative_event_index"]]
        assert chapter["instances"] == event["instances"]
        assert chapter["compute"] == event["compute"]
    result["events"][0]["instances"][0]["transform"][0][3] = 999
    assert trace == original


def test_condensed_eps_delta_includes_all_accepted_moves():
    overview = pipeline_presentation(example_trace())
    chapter = next(e for e in overview["events"] if e["stage"] == "eps_move")
    assert chapter["delta"]["moved"] == ["p0", "p1", "p2"]
    assert len(chapter["source_event_indices"]) == 3
    assert chapter["compute"]["kind"] == "local"
    assert not chapter["compute"]["live_model_call_at_this_step"]


@pytest.mark.parametrize("length", [0, 4, 10, 16, 21, 29])
def test_partial_and_failed_runs_keep_actual_events_without_filling_missing_steps(
    length,
):
    trace = example_trace()
    trace["events"] = trace["events"][:length]
    trace["candidates"] = {}
    overview = pipeline_presentation(trace)
    assert len(overview["events"]) <= 16
    assert sorted(
        i for e in overview["events"] for i in e["source_event_indices"]
    ) == list(range(length))
    assert all(e["representative_event_index"] < length for e in overview["events"])


def test_schema_and_seed_gallery_are_separate_real_chapters_before_eps():
    trace = example_trace()
    schema = deepcopy(trace["events"][3])
    schema["stage"] = "layout_constraints"
    gallery = deepcopy(trace["events"][12])
    gallery["stage"] = "layout_candidates"
    trace["events"].insert(4, schema)
    trace["events"].insert(13, gallery)
    for i, e in enumerate(trace["events"]):
        e["index"] = i
    result = pipeline_presentation(trace)
    assert len(result["events"]) == 16
    stages = [e["stage"] for e in result["events"]]
    assert (
        stages.index("architecture_ranking")
        < stages.index("layout_constraints")
        < stages.index("layout_candidates")
        < stages.index("eps_move")
    )
