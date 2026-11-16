"""Regression failures for carrier layout, atomic transforms and release gating."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import trimesh
from carrier_layout.fabrication import build_link, export_solid
from carrier_layout.loads import load_report
from carrier_layout.pipeline import import_mechanical, run_pipeline, settings_for
from carrier_layout.profiles import build_profile, pose_matrix, world_bounds
from carrier_layout.search import (
    Cavity,
    feasible,
    make_pose,
    objective,
    plan_link,
    projection_metrics,
)
from carrier_layout.wiring import blocked_route_cells, explicit_route
from hardware import as_mesh
from shell_partition.supports import intersection_volume, solid


def profile_spec():
    return {
        "id": "module_7",
        "component_id": "controller",
        "owner_link": "torso",
        "mass_kg": None,
        "envelope": {
            "size_mm": [20, 10, 6],
            "evidence": "synthetic_test",
            "retention": "pending",
        },
        "carrier": {
            "plate_mm": 3,
            "margin_mm": 3,
            "ear_width_mm": 12,
            "dock_depth_mm": 6,
            "max_anchor_mm": 100,
        },
    }


def space_fixture():
    axis = np.arange(-60, 61, 3)
    xyz = np.stack(np.meshgrid(axis, axis, axis, indexing="ij"), -1)
    cavity = (abs(xyz) < 42).all(-1)
    return {
        "origin": np.array([-60.0] * 3),
        "pitch": 3.0,
        "usable": cavity,
        "part_ids": np.ones(cavity.shape, np.uint8),
        "material": ~cavity,
        "motor_block": np.zeros(cavity.shape, bool),
        "protected_material": np.zeros(cavity.shape, bool),
    }


def test_independent_carrier_is_connected_and_does_not_enter_component(tmp_path):
    p = build_profile(profile_spec(), tmp_path, {})
    assert as_mesh(p["carrier"]).is_volume
    assert intersection_volume(p["carrier"], p["body"]) < 0.01
    assert "component_retention" in p["pending"]
    assert "component_mass_and_centre_of_mass" in p["pending"]


def test_rotation_moves_holes_ports_and_carrier_in_one_frame(tmp_path):
    p = build_profile(profile_spec(), tmp_path, {})
    transform = pose_matrix([0, 1, 0], 90, [8, 4, -3], p)
    assert np.isclose(np.linalg.det(transform[:3, :3]), 1)
    assert np.allclose(transform[:3, :3] @ [0, 0, 1], [0, 1, 0])
    bounds = world_bounds(p["bounds"], transform)
    assert np.allclose(bounds.mean(0), [8, 4, -3])
    local = np.r_[p["ears"][0], p["ear_z"], 1]
    world = transform @ local
    assert np.allclose(np.linalg.inv(transform) @ world, local)


def test_projection_overlap_is_penalized_but_staged_layers_are_not_forbidden(tmp_path):
    p = build_profile(profile_spec(), tmp_path, {})
    cavity = Cavity(space_fixture(), 1)
    front = make_pose(p, [0, 0, 1], 0, [0, 0, 15])
    rear = make_pose(p, [0, 0, 1], 0, [0, 0, -15])
    assert front["bounds"][0, 2] > rear["bounds"][1, 2]
    assert feasible(rear, [front], cavity, np.array([0, 0, 1]), 35, 0, 3)
    separated = make_pose(p, [0, 0, 1], 0, [0, 25, -15])
    weights = {"coplanarity": 0, "compactness": 0, "projection_overlap": 40}
    assert objective([front, rear], np.array([0, 0, 1]), weights) > objective(
        [front, separated], np.array([0, 0, 1]), weights
    )


def test_cavity_prefix_detects_internal_material_not_just_corners():
    s = space_fixture()
    c = Cavity(s, 1)
    bounds = np.array([[-10] * 3, [10] * 3])
    assert c.clear(bounds)
    s["usable"][20, 20, 20] = False
    assert not Cavity(s, 1).clear(bounds)


def test_eps_returns_complete_atomic_bundles_and_monotone_improvements(tmp_path):
    a = build_profile(profile_spec(), tmp_path, {})
    spec = profile_spec()
    spec["id"] = "different_name"
    b = build_profile(spec, tmp_path, {})
    settings = settings_for(
        {"seed_pitch_mm": 9, "max_seed_candidates": 240, "max_evaluations": 60}
    )
    poses, report = plan_link([a, b], space_fixture(), 1, [0, 0, 1], 35, settings)
    observed = []
    recorded, recorded_report = plan_link(
        [a, b],
        space_fixture(),
        1,
        [0, 0, 1],
        35,
        settings,
        on_progress=lambda kind, state, detail: observed.append(
            (kind, [p["transform"].copy() for p in state], detail)
        ),
    )
    assert observed[0][0] == "initial_seed"
    assert len(observed) == 1 + len(report["accepted_moves"])
    assert recorded_report["accepted_moves"] == report["accepted_moves"]
    assert all(
        np.array_equal(a["transform"], b["transform"]) for a, b in zip(poses, recorded)
    )
    assert poses is not None, report
    assert {p["id"] for p in poses} == {"module_7", "different_name"}
    assert report["final_cost"] <= report["initial_cost"]
    assert all(m["after"] < m["before"] for m in report["accepted_moves"])
    assert (
        projection_metrics(poses, np.array([0, 0, 1]))["projected_overlap_pairs"] == []
    )


def test_exact_shell_docks_remain_fused_but_carrier_remains_separate(tmp_path):
    p = build_profile(profile_spec(), tmp_path, {})
    shell = trimesh.boolean.difference(
        [trimesh.creation.box([100] * 3), trimesh.creation.box([90] * 3)],
        engine="manifold",
    )
    pose = make_pose(p, [0, 0, 1], 0, [0, 0, 0])
    result = build_link(
        shell, {"normal": [0, 0, 1], "offset_mm": 25}, [p], [pose], settings_for({})
    )
    assert result["all_paths_clear"], result["checks"]
    assert len(result["records"][0]["roots"]) == 2
    assert (
        intersection_volume(solid(result["pieces"][0]), result["carriers"]["module_7"])
        < 0.01
    )
    assert result["records"][0]["carrier_attachment"] == "removable_two_bolts"


def test_too_short_anchor_budget_is_rejected(tmp_path):
    spec = profile_spec()
    spec["carrier"]["max_anchor_mm"] = 10
    p = build_profile(spec, tmp_path, {})
    shell = trimesh.boolean.difference(
        [trimesh.creation.box([100] * 3), trimesh.creation.box([90] * 3)],
        engine="manifold",
    )
    with pytest.raises(ValueError, match="anchor"):
        build_link(
            shell,
            {"normal": [0, 0, 1], "offset_mm": 25},
            [p],
            [make_pose(p, [0, 0, 1], 0, [0, 0, 0])],
            settings_for({}),
        )


def test_other_link_blocks_a_path_that_local_shell_alone_accepts(tmp_path):
    p = build_profile(profile_spec(), tmp_path, {})
    shell = trimesh.boolean.difference(
        [trimesh.creation.box([100] * 3), trimesh.creation.box([90] * 3)],
        engine="manifold",
    )
    obstruction = solid(trimesh.creation.box([60, 60, 5]).apply_translation([0, 0, 70]))
    result = build_link(
        shell,
        {"normal": [0, 0, 1], "offset_mm": 25},
        [p],
        [make_pose(p, [0, 0, 1], 0, [0, 0, 0])],
        settings_for({}),
        [obstruction],
    )
    assert not result["all_paths_clear"]


def test_normalized_through_hole_input_preserves_source_exterior(tmp_path):
    source = tmp_path / "mechanical"
    (source / "meshes").mkdir(parents=True)
    outer = trimesh.creation.box([100] * 3)
    shell = trimesh.boolean.difference(
        [
            outer,
            trimesh.creation.box([90] * 3),
            trimesh.creation.cylinder(radius=4, height=120),
        ],
        engine="manifold",
    )
    shell.export(source / "meshes/torso_hollow_assembly_mm.stl")
    exterior = source / "meshes/torso_solid_assembly_mm.stl"
    outer.export(exterior)
    (source / "mechanical_scene.json").write_text(
        json.dumps({"units": "millimeter", "parts": ["torso"], "motors": []})
    )
    np.savez_compressed(source / "space_masks.npz", **space_fixture())
    sources = {}
    out = tmp_path / "out"
    import_mechanical(
        {"mechanical": {"adapter": "normalized", "directory": "mechanical"}},
        tmp_path,
        out,
        sources,
    )
    assert (out / "meshes" / exterior.name).read_bytes() == exterior.read_bytes()
    assert str(exterior) in sources


def test_normalized_end_to_end_is_robot_name_independent_and_never_hides_missing_data(
    tmp_path,
):
    source = tmp_path / "mechanical"
    (source / "meshes").mkdir(parents=True)
    shell = trimesh.boolean.difference(
        [trimesh.creation.box([100] * 3), trimesh.creation.box([90] * 3)],
        engine="manifold",
    )
    shell.export(source / "meshes/torso_hollow_assembly_mm.stl")
    (source / "mechanical_scene.json").write_text(
        json.dumps({"units": "millimeter", "parts": ["torso"], "motors": []})
    )
    np.savez_compressed(source / "space_masks.npz", **space_fixture())
    config = {
        "schema_version": 1,
        "units": "mm",
        "mechanical": {"adapter": "normalized", "directory": "mechanical"},
        "gravity": {"world_m_s2": [0, 0, -9.81], "evidence": "test fixture"},
        "components": [profile_spec()],
        "openings": {"torso": [{"normal": [0, 0, 1], "fraction": 0.75}]},
        "search": {"max_evaluations": 24, "max_seed_candidates": 80},
    }
    file = tmp_path / "config.json"
    file.write_text(json.dumps(config))
    original = (source / "meshes/torso_hollow_assembly_mm.stl").read_bytes()
    out = tmp_path / "result"
    assert run_pipeline(file, out) == 2
    manifest = json.loads((out / "manifest.json").read_text())
    assert manifest["status"] == "partial" and not manifest["complete_assemblable"]
    assert manifest["all_carrier_paths_clear"]
    assert manifest["all_shell_pieces_fastened"]
    coverage = json.loads((out / "shell_connections.json").read_text())[
        "piece_coverage"
    ]
    assert len(coverage) == 2 and all(len(ids) >= 3 for ids in coverage.values())
    assert manifest["source_inputs_unchanged"]
    assert (source / "meshes/torso_hollow_assembly_mm.stl").read_bytes() == original
    assert (out / "meshes/module_7_carrier.stl").is_file()
    assert (out / "index.html").is_file()
    process = json.loads((out / "process_trace.json").read_text())
    assert process["status"] == "partial"
    stages = [e["stage"] for e in process["events"]]
    assert stages[0] == "mechanical_input" and stages[-1] == "final_validation"
    assert {
        "carrier_profiles",
        "eps_seed",
        "selected_layout",
        "routing",
        "shell_fastening",
    } <= set(stages)
    assert process["events"][-1]["metrics"]["all_shell_pieces_fastened"]
    assert all(
        not any("\u4e00" <= c <= "\u9fff" for c in e["title"] + "".join(e["changes"]))
        for e in process["events"]
    )
    assert 'href="process.html"' in (out / "index.html").read_text()
    assert "Generation process" in (out / "index.html").read_text()
    baseline = tmp_path / "no_trace"
    assert run_pipeline(file, baseline, record_process=False) == 2
    assert not (baseline / "process.html").exists()
    assert json.loads((baseline / "carrier_placement.json").read_text()) == json.loads(
        (out / "carrier_placement.json").read_text()
    )
    for mesh_file in (out / "meshes").glob("*piece_*.stl"):
        a = solid(trimesh.load(mesh_file, force="mesh"))
        b = solid(trimesh.load(baseline / "meshes" / mesh_file.name, force="mesh"))
        assert (a - b).volume() + (b - a).volume() < 0.001
    with pytest.raises(ValueError, match="empty"):
        run_pipeline(file, out)


def test_real_pinned_pcb_uses_source_holes_without_saved_world_placement():
    root = Path(__file__).resolve().parents[1]
    config = json.loads((root / "configs/cactus_carrier_assembly.json").read_text())
    spec = next(
        s
        for s in config["components"]
        if "arduino" in s.get("mounting", {}).get("definition", "")
    )
    p = build_profile(spec, root / "configs", {})
    assert p["interface"]["record"]["selected_hole_ids"] == ["H1", "H2", "H4"]
    assert np.array_equal(p["interface"]["transform"], np.eye(4))


@pytest.mark.parametrize(
    "settings", [{"beam_width": 1.5}, {"max_evaluations": float("nan")}, {"unknown": 3}]
)
def test_invalid_search_controls_fail_closed(settings):
    with pytest.raises(ValueError):
        settings_for(settings)


def test_explicit_wiring_routes_supplied_ports_and_refuses_a_blocked_terminal(tmp_path):
    spec = profile_spec()
    spec["ports"] = {
        "A": {
            "position_mm": [-12, 24, 15],
            "normal": [0, 1, 0],
            "evidence": "synthetic cable terminal",
        },
        "B": {
            "position_mm": [12, 24, 15],
            "normal": [0, 1, 0],
            "evidence": "synthetic cable terminal",
        },
    }
    profile = build_profile(spec, tmp_path, {})
    pose = make_pose(profile, [0, 0, 1], 0, [0, 0, 0])
    path = tmp_path / "netlist.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "units": "mm",
                "nets": [
                    {
                        "id": "test",
                        "terminals": [
                            {"instance_id": spec["id"], "port_id": "A"},
                            {"instance_id": spec["id"], "port_id": "B"},
                        ],
                    }
                ],
            }
        )
    )
    electrical = {"file": "netlist.json", "wire_radius_mm": 0.5}
    sources = {}
    out = tmp_path / "route"
    out.mkdir()
    result = explicit_route(
        electrical, tmp_path, space_fixture(), [profile], [pose], {}, out, sources
    )
    assert result["status"] == "passed" and result["branches"] == 1
    assert str(path) in sources
    # A terminal inside occupied hardware must not be snapped to nearby air.
    profile["spec"]["ports"]["A"]["position_mm"] = [0, 0, 3]
    result = explicit_route(
        electrical, tmp_path, space_fixture(), [profile], [pose], {}, out, {}
    )
    assert result["status"] == "failed"
    assert (
        result["unrouted"][0]["reason"] == "explicit_port_not_in_certified_routing_air"
    )


def test_known_gravity_reports_force_but_cannot_invent_load_capacity(tmp_path):
    spec = profile_spec()
    spec.update(mass_kg=0.1, centre_of_mass_local_mm=[0, 0, 3])
    p = build_profile(spec, tmp_path, {})
    pose = make_pose(p, [0, 0, 1], 0, [0, 0, 0])
    design = {
        "records": [
            {
                "target_id": spec["id"],
                "roots": [{"anchor_mm": [-10, 0, -45]}, {"anchor_mm": [10, 0, -45]}],
            }
        ]
    }
    result = load_report(
        [p], [pose], {"torso": design}, {"world_m_s2": [0, 0, -10], "evidence": "test"}
    )
    row = result["components"][0]
    assert np.allclose(row["component_gravity_force_n"], [0, 0, -1])
    assert row["mount_plane_angle_to_horizontal_deg"] == 0
    assert row["status"] == "pending" and not row["capacity_validated"]


def test_finite_wire_radius_detects_grazing_that_a_clear_centreline_misses():
    shell = solid(
        trimesh.boolean.difference(
            [trimesh.creation.box([100] * 3), trimesh.creation.box([90] * 3)],
            engine="manifold",
        )
    )
    route = SimpleNamespace(points_mm=((-30, 44.5, 0), (30, 44.5, 0)))
    origin = np.array([-60] * 3)
    # Centreline is strictly inside the cavity; a 1 mm wire still grazes the wall.
    assert blocked_route_cells([route], shell, 1.0, origin, 3.0)
    clear = SimpleNamespace(points_mm=((-30, 42, 0), (30, 42, 0)))
    assert not blocked_route_cells([clear], shell, 1.0, origin, 3.0)


def test_stl_roundtrip_removes_zero_area_faces_without_changing_material(tmp_path):
    mesh = trimesh.creation.box([10, 12, 14])
    mesh.faces = np.vstack([mesh.faces, [0, 0, 0]])
    result = export_solid(mesh, tmp_path / "part.stl")
    assert result.is_volume and np.isclose(result.volume, 10 * 12 * 14)
    assert trimesh.load(tmp_path / "part.stl", force="mesh").is_volume


def test_stl_export_does_not_fill_a_real_hole_to_claim_success(tmp_path):
    mesh = trimesh.creation.box([10] * 3)
    mesh.update_faces(np.arange(len(mesh.faces) - 1))
    with pytest.raises(ValueError, match="Exported STL"):
        export_solid(mesh, tmp_path / "invalid.stl")
