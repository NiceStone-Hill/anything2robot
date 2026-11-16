"""Transactional shell-channel stage on an immutable carrier assembly bundle."""

from __future__ import annotations

import csv
import json
import shutil
from pathlib import Path

import numpy as np
import trimesh
from hardware import as_mesh64

from .assembly import load_bundle, read, snapshot
from .geometry import (
    Surface,
    check_self_clearance,
    open_sweep,
    simplify_curve,
    solid,
    sweep,
    union,
)

DEFAULTS = {
    "grid_mm": 2.0,
    "sample_mm": 0.75,
    "clearance_mm": 0.25,
    "depth_mm": 1.0,
    "min_wall_mm": 1.5,
    "wall_guard_mm": 0.2,
    "bend_radius_mm": 4.0,
    "lead_mm": 5.0,
    "lay_in_mm": 15.0,
    "wire_gap_mm": 0.5,
    "max_grid_cells": 100000,
    "max_expansions": 20000,
    "entry_search_radius_mm": 40.0,
    "entry_candidates": 6,
    "max_entry_pairs": 16,
    "max_shortcut_nodes": 48,
    "max_shortcut_checks": 4000,
}
TOL = 0.001


def dump(path, value):
    Path(path).write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )


def settings_from(path=None):
    raw = read(path) if path else {"schema_version": 1, "units": "mm"}
    if (
        set(raw) - {"schema_version", "units", "settings"}
        or raw.get("schema_version") != 1
        or raw.get("units") != "mm"
    ):
        raise ValueError("Channel config requires schema_version=1, units=mm, settings")
    settings = {**DEFAULTS, **raw.get("settings", {})}
    if set(settings) != set(DEFAULTS):
        raise ValueError("Unknown channel setting")
    for name, value in settings.items():
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not np.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{name} must be finite and positive")
    for name in (
        "max_grid_cells",
        "max_expansions",
        "entry_candidates",
        "max_entry_pairs",
        "max_shortcut_nodes",
        "max_shortcut_checks",
    ):
        if int(settings[name]) != settings[name]:
            raise ValueError(f"{name} must be an integer")
    if settings["max_shortcut_nodes"] < 3:
        raise ValueError("At least three shortcut nodes are required")
    if settings["sample_mm"] > settings["grid_mm"] / 2:
        raise ValueError("Sampling must be at most half the surface grid spacing")
    return settings


def validate_candidate(
    surface, changed, tool, cable, centerline, outer, closed_obstacles, diameter
):
    """Exact finite-volume checks plus explicitly labelled sampled wall evidence."""
    removed = surface.shell ^ tool
    if removed.volume() <= TOL:
        raise ValueError("Path does not create a groove in this shell")
    if (removed ^ surface.protected).volume() > TOL:
        raise ValueError("Groove cuts an existing support or fastening feature")
    if sum(p.volume() > 1e-6 for p in changed.decompose()) != 1:
        raise ValueError("Groove disconnects shell material")
    if (cable ^ changed).volume() > TOL:
        raise ValueError("Lead or cable intersects remaining shell material")
    world_cable = cable.transform(surface.world[:3])
    for obstacle in closed_obstacles:
        if (world_cable ^ obstacle).volume() > TOL:
            raise ValueError(
                "Cable intersects installed hardware, carrier, screw or another shell"
            )
    lay_in = open_sweep(centerline, diameter / 2, surface.settings["lay_in_mm"])
    if (lay_in ^ (changed + surface.obstacles)).volume() > TOL:
        raise ValueError("Groove cable cannot be laid in with carriers installed")
    removed_mesh = as_mesh64(removed)
    samples = np.vstack((removed_mesh.vertices, removed_mesh.triangles_center))
    if surface.mask_hits(samples):
        raise ValueError("Removed material enters protected structural/motor cells")
    world_samples = trimesh.transform_points(samples, surface.world)
    # Chunking avoids a large proximity-query temporary for fine meshes.
    minimum = min(
        float(trimesh.proximity.closest_point(outer, p)[1].min())
        for p in np.array_split(world_samples, max(1, len(world_samples) // 4000))
    )
    if minimum < surface.settings["min_wall_mm"]:
        raise ValueError(f"Sampled remaining wall {minimum:.3f} mm is too thin")
    return {
        "removed_volume_mm3": float(removed.volume()),
        "remaining_wall_sampled_min_mm": minimum,
        "wall_samples": len(samples),
        "wall_scope": "Distances of removed-surface vertices/face centres to original exterior; not continuous thickness or strength certification",
        "protected_features_preserved": True,
        "closed_material_clear": True,
        "local_lay_in_clear": True,
        "lay_in_distance_mm": surface.settings["lay_in_mm"],
        "lay_in_scope": "Local straight sweep with carriers installed; full cable/connector insertion and flexible manipulation pending",
    }


def route_one(surface, descriptor, wire, previous, output_file):
    diameter = wire["diameter_mm"]
    if surface.settings["depth_mm"] >= diameter:
        raise ValueError(
            "First version requires an open, partially recessed cable groove"
        )
    anchors_world = np.array(
        [wire[side]["position_mm"] for side in ("source", "target")]
    )
    normals = np.array([wire[side]["normal"] for side in ("source", "target")])
    anchors = trimesh.transform_points(anchors_world, surface.local)
    leads = trimesh.transform_points(
        anchors_world + normals * surface.settings["lead_mm"], surface.local
    )
    entries = leads.copy()
    # If a port points along the opening direction, dropping vertically from
    # its stub would retrace the cable through its own endpoint. Give that
    # lead an explicit lateral approach; final obstacle checks still apply.
    delta = leads[1, :2] - leads[0, :2]
    lateral = np.array([-delta[1], delta[0]])
    lateral = (
        lateral / np.linalg.norm(lateral)
        if np.linalg.norm(lateral) > 1e-6
        else np.array([1.0, 0.0])
    )
    local_normals = normals @ surface.local[:3, :3].T
    for i, normal in enumerate(local_normals):
        if abs(normal[2]) > 0.8:
            entries[i, :2] += lateral * max(
                2 * surface.settings["lead_mm"], 3 * diameter
            )
    from .entries import EntrySearchError, entry_candidates
    from .search import SurfaceField

    field = SurfaceField(surface, entries, diameter, previous)
    closed = union(
        o.transform(surface.local[:3]) for o in descriptor["closed_obstacles"]
    )
    candidates, entry_evidence = [], []
    for i in range(2):
        try:
            choices, evidence = entry_candidates(
                surface,
                field,
                anchors[i],
                leads[i],
                entries[i],
                diameter,
                closed,
                previous,
            )
        except EntrySearchError as exc:
            exc.diagnostics.update(
                endpoint=("source", "target")[i], other_endpoint_search=entry_evidence
            )
            raise
        candidates.append(choices)
        entry_evidence.append(evidence)
    pairs = sorted(
        ((x, y) for x in candidates[0] for y in candidates[1]),
        key=lambda pair: (
            pair[0]["length"]
            + pair[1]["length"]
            + np.linalg.norm(pair[0]["point"] - pair[1]["point"])
        ),
    )[: surface.settings["max_entry_pairs"]]
    attempts = []
    for margin in (
        0,
        surface.settings["bend_radius_mm"] / 2,
        surface.settings["bend_radius_mm"],
    ):
        for pair_index, (a, b) in enumerate(pairs):
            try:
                print(
                    f"  candidate {pair_index + 1}/{len(pairs)}, clearance +{margin:g} mm",
                    flush=True,
                )
                endpoints = np.array([a["point"], b["point"]])
                # Share unpadded geometry samples across candidate pairs.
                center, expanded = surface.plan(
                    endpoints,
                    diameter,
                    previous,
                    search_margin=margin,
                    field=field if margin == 0 else None,
                )
                result = finish_candidate(
                    surface,
                    descriptor,
                    wire,
                    previous,
                    output_file,
                    center,
                    a["lead"],
                    b["lead"],
                    expanded,
                )
                result[2].update(
                    search_rejected_attempts=attempts,
                    entry_search=entry_evidence,
                    selected_entry_pair=pair_index,
                    tested_entry_pairs=pair_index + 1,
                    entry_offsets_mm=[a["offset_mm"], b["offset_mm"]],
                    extra_surface_clearance_mm=margin,
                )
                return result
            except ValueError as exc:
                output_file.unlink(missing_ok=True)
                print("    rejected:", str(exc), flush=True)
                attempts.append(
                    {
                        "pair": pair_index,
                        "extra_surface_clearance_mm": margin,
                        "reason": str(exc),
                    }
                )
    raise EntrySearchError(
        "No candidate entry pair passed surface, bend and delivered-geometry checks",
        {"entries": entry_evidence, "attempts": attempts},
    )


def finish_candidate(
    surface,
    descriptor,
    wire,
    previous,
    output_file,
    center,
    source_lead,
    target_lead,
    expanded,
):
    diameter = wire["diameter_mm"]
    radius = diameter / 2 + surface.settings["clearance_mm"]
    # Use a small, explicit geometric error budget instead of thousands of
    # redundant CSG hulls. Exact cutter/cable/material checks follow this step.
    center = simplify_curve(center, min(0.03, surface.settings["clearance_mm"] / 4))
    whole = np.vstack((source_lead[:-1], center, target_lead[-2::-1]))
    check_self_clearance(whole, diameter)
    cable = sweep(whole, diameter / 2)
    # Reserve both leads and channel portions. Shared terminals need an explicit
    # physical junction design; independent wires must not silently overlap.
    if previous and (cable ^ union(previous)).volume() > TOL:
        raise ValueError(
            "Cable crosses a reserved cable; shared junction requires an explicit connector design"
        )
    cutter = open_sweep(center, radius, surface.travel)
    changed = surface.shell - cutter
    checks = validate_candidate(
        surface,
        changed,
        cutter,
        cable,
        center,
        descriptor["outer"],
        descriptor["closed_obstacles"],
        diameter,
    )
    world_changed = changed.transform(surface.world[:3])
    # Always retain double coordinates: a binary STL can preserve total volume
    # while moving a rotated boundary enough to violate a subtractive-only cut.
    mesh = as_mesh64(world_changed)
    output_file.write_text(trimesh.exchange.stl.export_stl_ascii(mesh))
    # Re-read exact STL bytes and repeat checks against the delivered geometry.
    from .assembly import load_mesh

    delivered_world = solid(load_mesh(output_file))
    if abs(delivered_world.volume() - world_changed.volume()) > TOL:
        raise ValueError("STL export changed material volume")
    delivered = delivered_world.transform(surface.local[:3])
    if (delivered - surface.shell).volume() > TOL:
        raise ValueError("STL export added unexpected material")
    export_difference = float(((delivered - changed) + (changed - delivered)).volume())
    if export_difference > TOL:
        raise ValueError("STL export changed the channel boundary")
    checks = validate_candidate(
        surface,
        delivered,
        cutter,
        cable,
        center,
        descriptor["outer"],
        descriptor["closed_obstacles"],
        diameter,
    )
    checks["export_reloaded_and_rechecked"] = True
    checks["reserved_interwire_clearance_mm"] = surface.settings["wire_gap_mm"]
    checks["export_symmetric_difference_mm3"] = export_difference
    points = trimesh.transform_points(whole, surface.world)
    channel = trimesh.transform_points(center, surface.world)
    length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    channel_length = float(np.linalg.norm(np.diff(channel, axis=0), axis=1).sum())
    return (
        delivered,
        sweep(whole, diameter / 2 + surface.settings["wire_gap_mm"]),
        {
            **wire,
            "status": "static_geometry_passed",
            "points_mm": points.tolist(),
            "channel_points_mm": channel.tolist(),
            "channel_length_mm": channel_length,
            "geometric_length_mm": length,
            "lead_length_mm": length - channel_length,
            "search_expansions": expanded,
            "checks": checks,
            "bend_scope": "Radius-aware surface shortcuts and rounded port leads; spatial curvature at the lead/channel transition and actual cable bend specification remain pending",
            "retention_status": "pending_clamps_and_strain_relief",
            "manufacturing_cut_length_mm": None,
        },
    )


def presentation(bundle, out, routes, records, modified):
    from PIL import Image, ImageDraw, ImageFont
    from shell_partition.viewer import export_viewer

    pieces, reports, targets = {}, {}, []
    for owner, surface in bundle["surfaces"].items():
        files = [
            r["file"]
            for r in bundle["inventory"]["printed_parts"]
            if r.get("owner_link") == owner
        ]
        meshes = [as_mesh64(modified.get(f, bundle["parts"][f])) for f in files]
        pieces[owner] = meshes
        reports[owner] = {
            "status": "partial",
            "selected_opening": {
                "panel_index": next(
                    (i for i, f in enumerate(files) if f != surface["file"]), 1
                ),
                "direction": surface["normal"],
                "clear_tasks": 0,
                "total_tasks": 0,
                "targets": [],
            },
            "pending": [
                "Cable retention, real ports, bends, dynamics and physical fit"
            ],
            "strategy": {
                "rationale": "New static channels; cable retention, connectors and motion remain pending."
            },
        }
    for part in bundle["inventory"]["printed_parts"]:
        if part["kind"] == "independent_removable_carrier":
            targets.append(
                {
                    "id": part["target_id"],
                    "kind": "carrier",
                    "owners": list(pieces),
                    "mesh": as_mesh64(bundle["parts"][part["file"]]),
                }
            )
    for route in routes:
        targets.append(
            {
                "id": "NEW " + route["id"],
                "kind": "wire",
                "owners": [route["source"]["owner_link"]],
                "mesh": as_mesh64(sweep(route["points_mm"], route["diameter_mm"] / 2)),
            }
        )
    dump(out / "partition_report.json", {"status": "partial", "parts": reports})
    export_viewer(out, pieces, targets, reports)
    image = Image.new("RGB", (1400, 800), "#f4f7fa")
    draw = ImageDraw.Draw(image)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = (
        ImageFont.truetype(font_path, 16)
        if Path(font_path).exists()
        else ImageFont.load_default()
    )
    draw.text(
        (25, 15),
        f"Static prototype: {len(routes)}/{len(records)} branches accepted; unresolved branches remain pending",
        fill="#182b40",
        font=font,
    )
    palettes = ["#1b998b", "#e25b45", "#7259bd", "#b57a08", "#147cc1"]
    projected = []
    for route in routes:
        from .geometry import frame

        local = frame(bundle["surfaces"][route["source"]["owner_link"]]["normal"])
        projected.append(
            {
                key: trimesh.transform_points(route[key], local)[:, :2]
                for key in ("baseline_points_mm", "points_mm", "channel_points_mm")
                if key in route
            }
        )
    if projected:
        points = np.vstack([p for item in projected for p in item.values()])
        low, high = points.min(axis=0), points.max(axis=0)
        scale = min(580 / max(high[0] - low[0], 1), 580 / max(high[1] - low[1], 1))
        for side, key in enumerate(("baseline_points_mm", "points_mm")):
            origin = np.array([55 + side * 700, 680])
            draw.text(
                (25 + side * 700, 60),
                (
                    "Original paths (same branches)"
                    if side == 0
                    else "New channels (thick) + leads"
                ),
                fill="#182b40",
                font=font,
            )
            for i, item in enumerate(projected):
                if key not in item:
                    continue

                def screen(values, origin=origin):
                    return [tuple(origin + (p - low) * [scale, -scale]) for p in values]

                color = palettes[i % len(palettes)]
                draw.line(screen(item[key]), fill=color, width=3)
                if side:
                    draw.line(screen(item["channel_points_mm"]), fill=color, width=7)
                for x, y in screen(item[key][[0, -1]]):
                    draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color)
                draw.text(
                    (30 + side * 700, 710 + i * 20),
                    routes[i]["id"],
                    fill=color,
                    font=font,
                )
    draw.text(
        (25, 775),
        "Opening-frame projection, equal scale. Endpoint positions preserved. This is not a certified wiring/cut-length plan.",
        fill="#526477",
        font=font,
    )
    image.save(out / "routing_comparison.png")
    html = (out / "index.html").read_text()
    html = html.replace(
        "Robot shells · Assembly review", "Shell channels · Static prototype"
    )
    html = html.replace(
        '<a href="proposal_raw.json">VLM proposal</a>',
        '<a href="channel_report.json">Routing results</a><a href="routing_comparison.png">Before / after</a><a href="baseline/index.html">Original assembly</a>',
    )
    (out / "index.html").write_text(html)


def run(source, output, config=None, connections=None, motor_registration=None):
    if connections and motor_registration:
        raise ValueError(
            "Use either complete --connections or legacy mapping plus --motor-terminals"
        )
    source, out = Path(source).resolve(), Path(output).resolve()
    if out == source or out.is_relative_to(source) or source.is_relative_to(out):
        raise ValueError("Output and source must be separate, non-nested directories")
    if out.exists() and any(out.iterdir()):
        raise ValueError(
            "Output must be new or empty; existing results are never overwritten"
        )
    settings = settings_from(config)
    hashes = snapshot(source)
    bundle = load_bundle(source, connections, motor_registration)
    out.mkdir(parents=True, exist_ok=True)
    dump(out / "source_snapshot.json", hashes)
    # Full read-only-input baseline copied before generating any modified files.
    shutil.copytree(source, out / "baseline")
    shutil.copytree(source / "meshes", out / "meshes")
    dump(
        out / "channel_config.json",
        {"schema_version": 1, "units": "mm", "settings": settings},
    )
    dump(out / "channel_inputs.json", bundle["connections"])
    dump(out / "motor_registration_report.json", bundle["motor_registration"])
    manifest = {
        "pipeline": "shell_channels_v2",
        "status": "running",
        "complete_assemblable": False,
        "source": str(source),
        "baseline": "baseline",
        "units": "mm",
    }
    dump(out / "manifest.json", manifest)
    try:
        surfaces, previous, modified, accepted, records = {}, {}, {}, [], []
        for wire in bundle["connections"]["cables"]:
            print("SHELL_CHANNEL", wire["id"], flush=True)
            a, b = wire["source"].get("owner_link"), wire["target"].get("owner_link")
            if not a or not b:
                records.append(
                    {
                        "id": wire["id"],
                        "status": "pending",
                        "reason": (
                            "motor_terminal_registration_required"
                            if any(
                                wire[side].get("motor_binding")
                                for side in ("source", "target")
                            )
                            else "missing_rigid_link_registration"
                        ),
                        "motor_bindings": [
                            wire[side]["motor_binding"]
                            for side in ("source", "target")
                            if wire[side].get("motor_binding")
                        ],
                    }
                )
                continue
            if a != b:
                records.append(
                    {
                        "id": wire["id"],
                        "status": "pending",
                        "reason": "cross_joint_flexible_harness_required",
                    }
                )
                continue
            if a not in bundle["surfaces"]:
                records.append(
                    {
                        "id": wire["id"],
                        "status": "pending",
                        "reason": "no_supported_opening_surface",
                    }
                )
                continue
            shared = [
                r["id"]
                for r in accepted
                if any(
                    wire[x]["instance_id"] == r[y]["instance_id"]
                    and wire[x]["port_id"] == r[y]["port_id"]
                    and np.allclose(
                        wire[x]["position_mm"], r[y]["position_mm"], atol=1e-6, rtol=0
                    )
                    for x in ("source", "target")
                    for y in ("source", "target")
                )
            ]
            if shared:
                records.append(
                    {
                        "id": wire["id"],
                        "status": "pending",
                        "reason": "shared_terminal_requires_physical_junction",
                        "shares_with": shared,
                    }
                )
                continue
            descriptor = bundle["surfaces"][a]
            if a not in surfaces:
                surfaces[a] = Surface(
                    descriptor["shell"],
                    descriptor["base"],
                    descriptor["normal"],
                    settings,
                    descriptor["obstacles"],
                    bundle["space"],
                )
                previous[a] = []
            surface = surfaces[a]
            temporary = out / "meshes" / "candidate_channel.stl"
            try:
                changed, reservation, result = route_one(
                    surface, descriptor, wire, previous[a], temporary
                )
                # Never publish a failed candidate: only commit after all checks pass.
                temporary.replace(out / descriptor["file"])
                surface.shell = changed
                modified[descriptor["file"]] = changed.transform(surface.world[:3])
                previous[a].append(reservation)
                accepted.append(result)
                records.append(
                    {
                        "id": wire["id"],
                        "status": result["status"],
                        "file": descriptor["file"],
                    }
                )
            except ValueError as exc:
                temporary.unlink(missing_ok=True)
                records.append(
                    {
                        "id": wire["id"],
                        "status": "failed",
                        "reason": str(exc),
                        "diagnostics": getattr(exc, "diagnostics", {}),
                    }
                )
                print("  rejected:", exc, flush=True)
        dump(
            out / "channel_routes.json",
            {"schema_version": 1, "units": "mm", "routes": accepted},
        )
        pending = [
            "Physical cable/connector dimensions and pin registration",
            "Removable clamps, strain relief and end-to-end connector installation",
            "Spatial bend radius and lead transition certification",
            "Cross-joint motion, electrical compatibility and real robot testing",
            "Structural capacity and printed fit after material removal",
        ]
        report = {
            "status": "partial",
            "mode": "shell_channels",
            "branches": records,
            "accepted": len(accepted),
            "requested": len(records),
            "unresolved": [
                r for r in records if r["status"] != "static_geometry_passed"
            ],
            "unrouted_input": bundle["connections"].get("unrouted_input", []),
            "pending": pending,
            "complete_assemblable": False,
            "baseline_routes_reused_as_new_routes": False,
        }
        dump(out / "channel_report.json", report)
        with (out / "wire_lengths.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "cable",
                    "status",
                    "channel_mm",
                    "lead_mm",
                    "geometric_total_mm",
                    "manufacturing_cut_mm",
                ]
            )
            by_id = {r["id"]: r for r in accepted}
            for row in records:
                r = by_id.get(row["id"], {})
                writer.writerow(
                    [
                        row["id"],
                        row["status"],
                        r.get("channel_length_mm"),
                        r.get("lead_length_mm"),
                        r.get("geometric_length_mm"),
                        "PENDING",
                    ]
                )
        inventory = {
            **bundle["inventory"],
            "release": "blocked_pending_channel_validation",
            "fasteners": "source_fasteners_bom.csv",
            "baseline": "baseline/manufacturing_manifest.json",
            "modified_shell_files": sorted(modified),
            "pending": pending,
        }
        dump(out / "manufacturing_manifest.json", inventory)
        # Historical assembly certificates stay exclusively in baseline/.
        dump(
            out / "assembly_validation.json",
            {
                "status": "partial",
                "complete_assemblable": False,
                "inherited_checks_not_recertified": "baseline/assembly_validation.json",
                "channel_checks": "channel_routes.json",
                "pending": pending,
            },
        )
        for name in (
            "fasteners_bom.csv",
            "shell_connections.json",
            "assembly_sequence.json",
        ):
            # Reference files are labelled as source-derived, not new certifications.
            shutil.copy2(source / name, out / ("source_" + name))
        presentation(bundle, out, accepted, records, modified)
        unchanged = snapshot(source) == hashes and snapshot(out / "baseline") == hashes
        if not unchanged:
            raise RuntimeError("Source/baseline integrity changed during routing")
        manifest.update(
            status="partial",
            stage="completed_with_explicit_blockers",
            accepted=len(accepted),
            requested=len(records),
            unresolved=len(report["unresolved"]),
            source_unchanged=True,
            baseline_exact_copy=True,
            modified_shells=sorted(modified),
            artifacts={
                "viewer": "index.html",
                "report": "channel_report.json",
                "routes": "channel_routes.json",
                "comparison": "routing_comparison.png",
                "lengths": "wire_lengths.csv",
            },
        )
        dump(out / "manifest.json", manifest)
        return manifest
    except Exception as exc:
        manifest.update(status="failed", error=str(exc))
        dump(out / "manifest.json", manifest)
        raise
