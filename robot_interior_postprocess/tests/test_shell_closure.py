"""Shell fastening must exist on both pieces and preserve real operation paths."""

import json

import numpy as np
import pytest
import trimesh
from carrier_layout.closure import (
    ClosureSearchError,
    fasten_shell_pair,
    local_frame,
    outer_envelope,
    prepare_pair,
    settings_for,
    try_candidate,
)
from carrier_layout.fabrication import export_solid
from carrier_layout.fastener_access import export_access_guide
from carrier_layout.fastener_patterns import discover_groups, fasten_group, policy_for
from hardware import cylinder_at, transform_solid
from shell_partition.geometry import split_shell
from shell_partition.supports import intersection_volume, solid, union
from verify_shell_fasteners import verify


@pytest.fixture
def box_pair():
    outer = trimesh.creation.box([100] * 3)
    shell = trimesh.boolean.difference(
        [outer, trimesh.creation.box([90] * 3)], engine="manifold"
    )
    plane = {"normal": [0, 0, 1], "offset_mm": 25}
    pieces, _ = split_shell(shell, plane, 0.4)
    return outer, shell, plane, pieces


def test_clearance_counterbore_blind_insert_and_both_roots_are_real_geometry(box_pair):
    outer, shell, plane, pieces = box_pair
    result = fasten_shell_pair(pieces, plane, shell, outer)
    assert result["report"]["connection_count"] >= 3
    final = [solid(m) for m in result["pieces"]]
    transform = local_frame(plane)
    for record in result["report"]["connections"]:
        xy = (np.linalg.inv(transform) @ [*record["axis_origin_world_mm"], 1])[:2]
        seat = record["head_seat_local_z_mm"]
        shaft_air = transform_solid(
            cylinder_at(xy, 1.45, -0.2 - record["engagement_mm"], seat), transform
        )
        head_air = transform_solid(
            cylinder_at(xy, 3.0, seat + 0.01, seat + 3), transform
        )
        assert intersection_volume(shaft_air + head_air, union(final)) < 0.02
        # Insert pocket has a material bottom: do not drill through the owner.
        bottom = transform_solid(cylinder_at(xy, 1.0, -7.5, -6.5), transform)
        assert intersection_volume(bottom, final[0]) > 2
        assert 3 <= record["engagement_mm"] <= 5
        assert record["geometry_checks"]["external_tool_clear"]
        # Both pieces meet a physical shoulder at their declared assembled pose.
        # The nominal seam must not be a floating gap that collapses on tightening.
        stop = transform_solid(
            cylinder_at(xy, 5.0, 0.19, 0.21) - cylinder_at(xy, 4.0, 0.18, 0.22),
            transform,
        )
        assert intersection_volume(stop, final[0]) > 0.1
        assert intersection_volume(stop, final[1]) > 0.1
        assert record["axial_stop_bridges_gap_mm"] == 0.4


@pytest.mark.parametrize("normal", [[1, 0, 0], [0, -1, 0], [0, 0, 1]])
def test_axis_transform_and_locating_socket_release_without_undercuts(normal):
    outer = trimesh.creation.box([100] * 3)
    shell = trimesh.boolean.difference(
        [outer, trimesh.creation.box([90] * 3)], engine="manifold"
    )
    plane = {"normal": normal, "offset_mm": 25}
    pieces, _ = split_shell(shell, plane, 0.4)
    result = fasten_shell_pair(pieces, plane, shell, outer)
    owner, cover = map(solid, result["pieces"])
    for distance in (0, 0.2, 1, 2, 3, 12, 80):
        moved = cover.translate((np.array(normal) * distance).tolist())
        assert intersection_volume(owner, moved) < 0.02
    for record in result["report"]["connections"]:
        assert np.allclose(record["screw_axis_world"], normal)
        assert record["locator"]["radial_clearance_mm"] > 0
        assert record["locator"]["exit_clearance_mm"] > 0


def test_external_tool_obstruction_rejects_closed_but_unfastenable_shell(box_pair):
    outer, shell, plane, pieces = box_pair
    barrier = solid(trimesh.creation.box([130, 130, 5]).apply_translation([0, 0, 80]))
    with pytest.raises(ClosureSearchError) as error:
        fasten_shell_pair(
            pieces,
            plane,
            shell,
            outer,
            obstacles=[barrier],
            settings={"contour_samples": 24, "max_candidates": 72},
        )
    assert error.value.report["connection_count"] == 0
    assert any(
        "access_hits" in r["reason"] for r in error.value.report["candidate_rejections"]
    )


def test_joint_search_avoids_an_existing_wire_instead_of_cutting_it(box_pair):
    outer, shell, plane, pieces = box_pair
    reference = fasten_shell_pair(pieces, plane, shell, outer)
    origin = np.array(reference["report"]["connections"][0]["axis_origin_world_mm"])
    wire = solid(
        trimesh.creation.cylinder(
            radius=6, segment=np.array([origin - [0, 0, 20], origin + [0, 0, 70]])
        )
    )
    result = fasten_shell_pair(pieces, plane, shell, outer, obstacles=[wire])
    for value in result["screws"]:
        assert intersection_volume(value, wire) < 0.02
    assert all(
        np.linalg.norm(np.array(r["axis_origin_world_mm"]) - origin) > 10
        for r in result["report"]["connections"]
    )


def test_existing_cross_seam_feature_cannot_reuse_the_old_half_space_proof(box_pair):
    outer, shell, plane, pieces = box_pair
    pin = trimesh.creation.cylinder(radius=3, height=10).apply_translation([0, 0, 25])
    pieces[0] = trimesh.boolean.union([pieces[0], pin], engine="manifold")
    with pytest.raises(ValueError, match="cross-seam"):
        fasten_shell_pair(pieces, plane, shell, outer)


def test_single_boundary_through_hole_shell_requires_actual_exterior(box_pair):
    outer, shell, _, _ = box_pair
    duct = trimesh.creation.cylinder(radius=4, height=120)
    opened = trimesh.boolean.difference([shell, duct], engine="manifold")
    with pytest.raises(ValueError, match="explicit exterior"):
        outer_envelope(opened)
    assert outer_envelope(opened, outer).is_volume


@pytest.mark.parametrize(
    "profile",
    [
        {"clearance_diameter_mm": 2.9},
        {"counterbore_diameter_mm": 5},
        {"boss_depth_mm": 5},
        {"socket_diameter_mm": 7},
        {"socket_depth_mm": 1},
        {"min_connections": 1},
        {"contour_samples": float("nan")},
    ],
)
def test_invalid_fastener_or_alignment_profiles_fail_closed(profile):
    with pytest.raises((TypeError, ValueError)):
        settings_for(profile)


def test_real_screw_head_and_full_loading_port_survive_stl_export(box_pair, tmp_path):
    outer, shell, plane, pieces = box_pair
    result = fasten_shell_pair(pieces, plane, shell, outer)
    files = [f"shell_{i}.stl" for i in range(2)]
    for file, mesh in zip(files, result["pieces"]):
        export_solid(mesh, tmp_path / file)
    report = result["report"]
    for i, r in enumerate(report["connections"]):
        r["id"] = f"shell:screw_{i + 1}"
        a = r["access"]
        assert a["port_diameter_mm"] >= 8
        assert 3.5 <= a["mouth_to_seat_mm"] <= 12
        assert r["fastener_spec"]["standard"] == "ISO 4762"
        assert r["fastener_spec"]["hex_key_mm"] == 2.5
    (tmp_path / "shell_connections.json").write_text(
        json.dumps(
            {
                "links": {"shell": report},
                "piece_coverage": {
                    f: [r["id"] for r in report["connections"]] for f in files
                },
            }
        )
    )
    (tmp_path / "manufacturing_manifest.json").write_text(
        json.dumps({"printed_parts": [{"file": f, "kind": "shell"} for f in files]})
    )
    assert verify(tmp_path)["status"] == "passed"
    export_access_guide(tmp_path, {"shell": result})
    assert "ISO 4762 M3" in (tmp_path / "shell_fasteners.csv").read_text()
    # A filled port can look plausible in metadata; exported-material checking
    # must catch it even when the nominal smaller screw shaft would pass.
    r = report["connections"][0]
    seat = r["head_seat_local_z_mm"]
    export_solid(
        trimesh.boolean.union(
            [
                result["pieces"][1],
                trimesh.creation.annulus(
                    r_min=3.4, r_max=4.1, height=1
                ).apply_transform(
                    local_frame(plane)
                    @ trimesh.transformations.translation_matrix(
                        [*r["local_cut_xy_mm"], seat + 4.5]
                    )
                ),
            ],
            engine="manifold",
        ),
        tmp_path / files[1],
    )
    with pytest.raises(ValueError, match="tunnel blocked"):
        verify(tmp_path)


def test_driver_handle_obstruction_is_rejected_even_when_its_shaft_fits(box_pair):
    outer, shell, plane, pieces = box_pair
    result = fasten_shell_pair(pieces, plane, shell, outer)
    r = result["report"]["connections"][0]
    p = settings_for()
    point = np.asarray(r["local_cut_xy_mm"])
    z = r["head_seat_local_z_mm"] + 3 + p["driver_shaft_length_mm"] + 5
    transform = local_frame(plane)
    barrier = solid(
        trimesh.creation.box([2, 2, 2])
        .apply_translation([*(point + [8, 0]), z])
        .apply_transform(transform)
    )
    ctx = prepare_pair(pieces, plane, shell, outer, obstacles=[barrier])
    with pytest.raises(ValueError, match="access_hits"):
        try_candidate(ctx, point, r["screw_length_mm"])


def mirrored_contexts():
    outer = trimesh.boolean.difference(
        [
            trimesh.creation.box([120, 100, 100]),
            trimesh.creation.box([30, 40, 120]).apply_translation([55, 45, 0]),
        ],
        engine="manifold",
    )
    shell = trimesh.boolean.difference(
        [outer, trimesh.creation.box([110, 90, 90])], engine="manifold"
    )
    plane = {"normal": [0, 0, 1], "offset_mm": 25}
    matrix = np.diag([-1.0, 1.0, 1.0, 1.0])
    matrix[0, 3] = 180
    contexts = {}
    for name, transform in [("front_leg", np.eye(4)), ("rear_leg", matrix)]:
        mesh = shell.copy().apply_transform(transform)
        envelope = outer.copy().apply_transform(transform)
        pieces, _ = split_shell(mesh, plane, 0.4)
        contexts[name] = prepare_pair(pieces, plane, mesh, envelope)
    return contexts


def test_mirrored_structures_jointly_share_hole_positions_and_exact_screw_specs():
    contexts = mirrored_contexts()
    groups = discover_groups(contexts, policy_for())
    assert len(groups) == 1
    result = fasten_group(contexts, groups[0])
    a, b = [r["report"] for r in result.values()]
    assert a["connection_count"] == b["connection_count"] >= 3
    assert np.linalg.det(b["shared_pattern"]["normalized_matrix_from_reference"]) == -1
    for x, y in zip(a["connections"], b["connections"]):
        assert x["pattern_normalized_xy"] == y["pattern_normalized_xy"]
        assert x["fastener_spec"] == y["fastener_spec"]
        assert np.allclose(
            y["axis_origin_world_mm"],
            [180 - x["axis_origin_world_mm"][0], *x["axis_origin_world_mm"][1:]],
        )


def test_shared_pattern_fails_instead_of_silently_using_different_holes():
    contexts = mirrored_contexts()
    for ctx in contexts.values():
        ctx["points"] = ctx["points"][::24]
        ctx["p"]["screw_lengths_mm"] = [20.0]
    group = discover_groups(contexts, policy_for())[0]
    # Block only one member's entire outside path; another member alone is
    # feasible, but the family must not silently fall back to different holes.
    contexts["rear_leg"]["blockers"] += solid(
        trimesh.creation.box([170, 150, 4]).apply_translation([180, 0, 100])
    )
    with pytest.raises(ClosureSearchError) as error:
        fasten_group(contexts, group)
    assert error.value.report["status"] == "failed_shared_pattern"


def test_unlike_outline_sizes_are_not_forced_into_one_pattern():
    contexts = mirrored_contexts()
    contexts["rear_leg"]["outer"] = contexts["rear_leg"]["outer"].scale([2, 1, 1])
    assert len(discover_groups(contexts, policy_for())) == 2
