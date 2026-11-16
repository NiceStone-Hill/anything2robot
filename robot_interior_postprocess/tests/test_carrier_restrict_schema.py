"""VLM policies must affect seeds and EPS, and cannot override physical guards."""

from copy import deepcopy

import numpy as np
import pytest
from carrier_layout.pipeline import settings_for
from carrier_layout.profiles import build_profile
from carrier_layout.restrict_schema import (
    CompiledRestrictions,
    local_schema,
    prepare_restrictions,
    restriction_context,
    validate_schema,
)
from carrier_layout.search import Cavity, feasible, make_pose, objective, plan_link
from test_carrier_layout import profile_spec, space_fixture


def inputs(tmp_path):
    profile = build_profile(profile_spec(), tmp_path, {})
    space = space_fixture()
    context = restriction_context(
        [profile],
        space,
        {"parts": ["torso"], "motors": []},
        {"torso": [{"normal": [0, 0, 1]}, {"normal": [0, 1, 0]}]},
        {"world_m_s2": [0, 0, -9.8], "evidence": "synthetic explicit gravity"},
        1,
    )
    return profile, space, context, local_schema(context)


@pytest.mark.parametrize(
    "change,match",
    [
        (lambda r: r["components"][0].update(instance_id="motor_fixed"), "Unknown"),
        (lambda r: r["components"][0].update(owner_link="another_link"), "ownership"),
        (lambda r: r["components"][0].update(allowed_rolls_deg=[45]), "rolls"),
        (
            lambda r: r["components"][0].update(allowed_opening_normals=[[1, 0, 0]]),
            "normals",
        ),
        (
            lambda r: r["components"][0].update(
                allowed_center_normalized=[[0, 1], [0, 1], [0.8, 0.2]]
            ),
            "bounds",
        ),
        (
            lambda r: r["components"][0].update(preference_weight=float("nan")),
            "preference",
        ),
        (lambda r: r["components"][0].update(preference_weight=True), "preference"),
        (lambda r: r["components"].append(deepcopy(r["components"][0])), "repeated"),
        (lambda r: r.update(components=[]), "every"),
        (lambda r: r.update(geometry_transform=[]), "requires"),
        (
            lambda r: r.update(
                relations=[
                    {
                        "type": "near",
                        "a": "module_7",
                        "b": "motor_fixed",
                        "weight": 1,
                        "evidence": "test",
                    }
                ]
            ),
            "relation",
        ),
    ],
)
def test_untrusted_schema_cannot_invent_hardware_or_unbounded_rules(
    tmp_path, change, match
):
    _, _, context, raw = inputs(tmp_path)
    change(raw)
    with pytest.raises(ValueError, match=match):
        validate_schema(raw, context)


def test_schema_clearance_cannot_reduce_local_geometry_floor(tmp_path):
    profile, space, context, raw = inputs(tmp_path)
    raw["components"][0]["min_clearance_mm"] = 0
    compiled = CompiledRestrictions(raw, context)
    assert compiled.schema["components"][0]["min_clearance_mm"] == 1
    raw["components"][0]["min_clearance_mm"] = 10
    strict = CompiledRestrictions(raw, context)
    pose = make_pose(profile, [0, 0, 1], 0, [6, 0, 0])
    assert feasible(pose, [], Cavity(space, 1), np.array([0, 0, 1]), 35, 1, 3)
    assert not feasible(
        pose, [], Cavity(space, 1), np.array([0, 0, 1]), 35, 1, 3, strict
    )


def test_initial_seeds_and_every_eps_move_obey_schema_domains_and_atomic_poses(
    tmp_path,
):
    profile, space, context, raw = inputs(tmp_path)
    policy = raw["components"][0]
    policy.update(
        allowed_center_normalized=[[0, 0.5], [0, 1], [0, 1]],
        allowed_rolls_deg=[90],
        allowed_opening_normals=[[0, 0, 1]],
        vertical_preference="low",
        preference_weight=5,
    )
    compiled = CompiledRestrictions(raw, context)
    observed = []
    poses, report = plan_link(
        [profile],
        space,
        1,
        [0, 0, 1],
        35,
        settings_for({"max_evaluations": 60}),
        restrictions=compiled,
        on_progress=lambda kind, poses, detail: observed.extend(
            [*poses, *[p for seed in detail.get("_seed_candidates", []) for p in seed]]
        ),
    )
    assert poses and observed
    assert all(not compiled.violations(p) for p in observed)
    assert all(p["center"][0] <= 0 and p["roll"] == 90 for p in observed)
    assert report["layout_restrictions_applied"]
    assert report["semantic_validation"]["all_hard_constraints_pass"]
    assert report["final_cost"] <= report["initial_cost"]
    excluded, _failure = plan_link(
        [profile], space, 1, [0, 1, 0], 35, settings_for({}), restrictions=compiled
    )
    assert excluded is None  # no silent fallback to a forbidden normal


def test_gravity_preference_changes_objective_using_supplied_gravity(tmp_path):
    profile, _, context, raw = inputs(tmp_path)
    raw["components"][0].update(vertical_preference="low", preference_weight=5)
    context["gravity"]["world_m_s2"] = [-9.8, 0, 0]
    compiled = CompiledRestrictions(raw, context)
    low = make_pose(profile, [0, 0, 1], 0, [-10, 0, 0])
    high = make_pose(profile, [0, 0, 1], 0, [10, 0, 0])
    weights = {"compactness": 0, "coplanarity": 0, "projection_overlap": 0}
    assert objective([low], np.array([0, 0, 1]), weights, compiled) < objective(
        [high], np.array([0, 0, 1]), weights, compiled
    )


def test_replay_compiles_saved_constraints_without_network_and_keeps_provenance(
    tmp_path,
):
    import json

    _, _, context, raw = inputs(tmp_path)
    source = tmp_path / "schema.json"
    source.write_text(json.dumps(raw))
    out = tmp_path / "out"
    out.mkdir()
    sources = {}
    compiled, provenance = prepare_restrictions(
        context, [], out, mode="llm", replay=source, sources=sources
    )
    assert provenance["source"] == "saved_schema_replay"
    assert not provenance["live_model_called"]
    assert str(source) in sources
    assert (out / "layout_constraints.json").is_file()
    assert compiled.schema["components"][0]["instance_id"] == "module_7"


def test_final_report_requires_complete_unique_hardware_and_checks_all_hard_domains(
    tmp_path,
):
    profile, _, context, raw = inputs(tmp_path)
    compiled = CompiledRestrictions(raw, context)
    pose = make_pose(profile, [0, 0, 1], 0, [0, 0, 0])
    assert compiled.report([pose], require_all=True)["all_hard_constraints_pass"]
    assert not compiled.report([], require_all=True)["all_hard_constraints_pass"]
    assert not compiled.report([pose, pose], require_all=True)[
        "all_hard_constraints_pass"
    ]
    invalid = make_pose(profile, [0, 0, 1], 0, [100, 0, 0])
    assert not compiled.report([invalid], require_all=True)["all_hard_constraints_pass"]


def test_bounded_relations_change_soft_objective_and_duplicates_are_rejected(tmp_path):
    _, _, context, raw = inputs(tmp_path)
    second = deepcopy(context["components"][0])
    second["instance_id"] = "second"
    context["components"].append(second)
    raw = local_schema(context)
    raw["relations"] = [
        {
            "type": "near",
            "a": "module_7",
            "b": "second",
            "weight": 2,
            "evidence": "short local lead",
        }
    ]
    compiled = CompiledRestrictions(raw, context)
    profile = build_profile(profile_spec(), tmp_path, {})
    a = make_pose(profile, [0, 0, 1], 0, [0, 0, 0])
    b = make_pose(profile, [0, 0, 1], 0, [5, 0, 0])
    b["id"] = "second"
    far = deepcopy(b)
    far["center"] = np.array([30, 0, 0])
    assert compiled.soft_cost([a, b], np.array([0, 0, 1])) < compiled.soft_cost(
        [a, far], np.array([0, 0, 1])
    )
    raw["relations"].append(deepcopy(raw["relations"][0]))
    with pytest.raises(ValueError, match="Duplicate"):
        validate_schema(raw, context)


def test_layout_gallery_uses_real_seed_geometry_and_offline_png(tmp_path):
    from carrier_layout.layout_gallery import render_seed
    from carrier_layout.process_trace import ProcessRecorder

    profile, _, _, _ = inputs(tmp_path)
    recorder = ProcessRecorder(tmp_path)
    first = make_pose(profile, [0, 0, 1], 0, [-10, 0, 0])
    second = make_pose(profile, [0, 0, 1], 90, [10, 0, 0])
    images = []
    for i, pose in enumerate([first, second]):
        items = recorder.bundles([profile], [pose])
        url = render_seed(
            items,
            recorder.data["assets"],
            [[-60, -60, -60], [60, 60, 60]],
            [0, 0, 1],
            tmp_path / f"seed{i}.png",
            f"Seed {i}",
            "Synthetic fixture",
        )
        assert url.startswith("data:image/png;base64,")
        images.append((tmp_path / f"seed{i}.png").read_bytes())
    assert images[0] != images[1]
