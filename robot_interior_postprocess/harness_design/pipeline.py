"""Plan every logical cable in real assembly space under explicit design assumptions.

These proposals deliberately never produce a physical_wiring routing contract.
Unknown real terminals remain unresolved even if candidate centreline coverage is full.
"""

from __future__ import annotations

import csv
import itertools
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh
from hardware import as_mesh64
from physical_wiring.catalog import read
from physical_wiring.pipeline import assembly_context
from physical_wiring.registration import cable_groups
from scipy.ndimage import distance_transform_edt
from shell_channels.assembly import load_bundle, snapshot
from shell_channels.geometry import check_self_clearance, dense, solid
from shell_channels.pipeline import dump

from .geometry import curve, guarded_free, largest_region, rasterize, reserve, search
from .geometry import wire_solid as sweep


def collision(value, obstacles):
    bounds = np.array(value.bounding_box()).reshape(2, 3)
    for key, other in obstacles.items():
        b = np.array(other.bounding_box()).reshape(2, 3)
        if np.any(bounds[1] < b[0]) or np.any(b[1] < bounds[0]):
            continue
        if (value ^ other).volume() > 0.001:
            return key
    return None


def settings(path):
    data = read(path)
    if (
        data.get("schema_version") != 1
        or data.get("units") != "mm"
        or data.get("mode") != "explicit_design_assumptions_not_physical_registration"
    ):
        raise ValueError("Design assumptions must be explicit and in millimetres")
    for key in (
        "wire_diameter_mm",
        "bus_pair_diameter_mm",
        "min_bend_radius_mm",
        "clearance_mm",
        "port_spacing_mm",
        "max_terminal_extension_mm",
        "max_route_attempts",
        "support_spacing_mm",
    ):
        v = data[key]
        if (
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not np.isfinite(v)
            or v <= 0
        ):
            raise ValueError(f"{key} must be positive and finite")
    if int(data["max_route_attempts"]) != data["max_route_attempts"]:
        raise ValueError("max_route_attempts must be an integer")
    for size in data["new_component_envelopes_mm"].values():
        if len(size) != 3 or not np.isfinite(size).all() or min(size) <= 0:
            raise ValueError("Invalid candidate envelope")
    if not data.get("assumptions"):
        raise ValueError("Assumption provenance is required")
    return data


def propose_components(
    plan, context, bundle, free, origin, pitch, obstacles, config, out
):
    assembly = read(bundle["source"] / "carrier_placement.json")["placements"]
    existing = {p["id"]: p for p in assembly}
    owner = context["owners"][context["controller"]]
    # Source adapter identifies a power source by configured component semantics,
    # never by a robot-specific coordinate or motor count.
    legacy_config = read(bundle["source"] / "assembly_config_snapshot.json")
    batteries = [
        p["id"]
        for p in legacy_config["components"]
        if p.get("component_id") == "BATTERY"
    ]
    if len(batteries) != 1:
        raise ValueError(
            "Design adapter needs one explicitly identified battery envelope"
        )
    existing["PW_MOTOR_SUPPLY"] = existing[batteries[0]]
    scene = read(bundle["source"] / "mechanical_scene.json")
    motors = {m["id"]: m for m in scene["motors"]}
    available = free.copy()
    proposals = {}
    target = np.mean(np.argwhere(available), axis=0) * pitch + origin
    # Larger unselected modules get space first. Every pose is newly proposed.
    ordered = sorted(
        plan["components"],
        key=lambda c: (
            -np.prod(config["new_component_envelopes_mm"].get(c["id"], [0, 0, 0]))
        ),
    )
    for component in ordered:
        key = component["id"]
        if key in existing:
            bounds = np.asarray(existing[key]["bounds_mm"])
            proposals[key] = {
                "id": key,
                "owner_link": owner,
                "bounds_mm": bounds.tolist(),
                "center_mm": bounds.mean(axis=0).tolist(),
                "geometry": "existing_envelope",
                "physical_registration_confirmed": False,
            }
            continue
        if key in context["motor_candidates"]:
            motor = motors[context["motor_candidates"][key]["motor_id"]]
            adjacent = context["motor_adjacent_links"][key]
            chosen = owner if owner in adjacent else motor["parent"]
            proposals[key] = {
                "id": key,
                "owner_link": chosen,
                "center_mm": motor["center_mm"],
                "bounds_mm": trimesh.creation.cylinder(
                    radius=motor["radius_mm"],
                    segment=[motor["axial_start_mm"], motor["axial_end_mm"]],
                ).bounds.tolist(),
                "geometry": "motor_envelope",
                "physical_registration_confirmed": False,
                "mounting_side_status": "design_assumption_requires_confirmation",
            }
            continue
        if key not in config["new_component_envelopes_mm"]:
            raise ValueError(f"No explicit assumed envelope for {key}")
        print("HARNESS: propose module", key, flush=True)
        candidates = np.argwhere(available)
        if not len(candidates):
            raise ValueError("No space for remaining candidate components")
        score = np.linalg.norm(origin + candidates * pitch - target, axis=1)
        candidates = candidates[np.argsort(score)[::13]][:1200]
        found = None
        for size in dict.fromkeys(
            itertools.permutations(config["new_component_envelopes_mm"][key])
        ):
            half = np.asarray(size) / 2 + config["clearance_mm"]
            for index in candidates:
                center = origin + index * pitch
                lo = np.maximum(
                    0, np.floor((center - half - origin) / pitch).astype(int)
                )
                hi = np.minimum(
                    free.shape,
                    np.ceil((center + half - origin) / pitch).astype(int) + 1,
                )
                slices = tuple(slice(a, b) for a, b in zip(lo, hi))
                if not available[slices].all():
                    continue
                mesh = trimesh.creation.box(size).apply_translation(center)
                value = solid(mesh)
                if collision(value, obstacles):
                    continue
                found = (mesh, value, slices)
                break
            if found:
                break
        if found is None:
            raise ValueError(
                f"No collision-free placement for assumed module {key}; resize/replan carrier explicitly"
            )
        mesh, value, slices = found
        available[slices] = False
        obstacles["proposed:" + key] = value
        mesh.export(out / "meshes" / f"{key}_assumed_envelope.stl")
        proposals[key] = {
            "id": key,
            "owner_link": owner,
            "bounds_mm": mesh.bounds.tolist(),
            "center_mm": mesh.bounds.mean(axis=0).tolist(),
            "geometry": "assumed_module_space_reservation",
            "physical_registration_confirmed": False,
            "retention": "new_mount_required",
            "mesh": f"meshes/{key}_assumed_envelope.stl",
        }
    return proposals, available


def propose_endpoints(plan, proposals, free, origin, pitch, config, obstacles):
    groups = cable_groups(plan)
    endpoint_groups = defaultdict(list)
    for key, members in groups.items():
        for side in ("source", "target"):
            ids = {m[side]["instance_id"] for m in members}
            if len(ids) != 1:
                raise ValueError("Bundle cannot span different source/target devices")
            endpoint_groups[next(iter(ids))].append(
                (key, side, [m[side]["port_id"] for m in members])
            )
    exit_clearance = distance_transform_edt(free, sampling=pitch)
    candidates = np.argwhere(free & (exit_clearance >= 1.5 * pitch))
    xyz = origin + candidates * pitch
    candidate_clearance = exit_clearance[tuple(candidates.T)]
    occupied = []
    ends = {}
    failures = []
    # Reserve all proposed exits before routing to prevent a cable blocking a later endpoint.
    for instance, items in sorted(
        endpoint_groups.items(), key=lambda item: -len(item[1])
    ):
        proposal = proposals[instance]
        center = np.asarray(proposal["center_mm"])
        if "bounds_mm" in proposal:
            lo, hi = np.asarray(proposal["bounds_mm"])
            delta = np.maximum(np.maximum(lo - xyz, xyz - hi), 0)
            distances = np.linalg.norm(delta, axis=1)
        else:
            # Distance to the original motor envelope centre, not an inferred pin.
            distances = np.linalg.norm(xyz - center, axis=1)
        score = (
            distances
            + 0.03 * np.linalg.norm(xyz - center, axis=1)
            - 0.6 * np.minimum(candidate_clearance, 9.0)
        )
        order = np.argsort(score)
        for key, side, ports in items:
            selected = None
            for i in order[:5000]:
                if distances[i] > config["max_terminal_extension_mm"]:
                    continue
                point = xyz[i]
                if (
                    occupied
                    and np.min(np.linalg.norm(np.asarray(occupied) - point, axis=1))
                    < config["port_spacing_mm"]
                ):
                    continue
                import manifold3d as md

                probe = md.Manifold.sphere(
                    max(config["wire_diameter_mm"], config["bus_pair_diameter_mm"]) / 2
                    + config["clearance_mm"],
                    16,
                ).translate(point)
                if collision(probe, obstacles):
                    continue
                selected = i
                break
            if selected is None:
                failures.append(
                    {
                        "cable_id": key,
                        "side": side,
                        "reason": "no_space_for_assumed_terminal_exit",
                    }
                )
                continue
            point = xyz[selected]
            occupied.append(point)
            ends[key, side] = {
                "instance_id": instance,
                "logical_ports": ports,
                "owner_link": proposal["owner_link"],
                "position_mm": point.tolist(),
                "grid_index": candidates[selected].tolist(),
                "evidence": "DESIGN ASSUMPTION: proposed cable-tail exit in checked free air; not a measured connector",
                "device_to_exit_connection": "UNRESOLVED real pin, connector body, tail geometry and mounting side",
                "distance_to_reference_envelope_mm": float(distances[selected]),
            }
    return ends, failures


def plan_routes(
    plan,
    proposals,
    ends,
    free,
    origin,
    pitch,
    obstacles,
    config,
    out,
    priority=(),
    iteration=1,
):
    groups = cable_groups(plan)
    routes = []
    failures = []
    occupied = free.copy()
    wall_distance = distance_transform_edt(free, sampling=pitch)
    maximum_radius = max(config["wire_diameter_mm"], config["bus_pair_diameter_mm"]) / 2
    # Short local connections first. Multi-pass reordering can be added without changing input identity.
    order = sorted(
        groups,
        key=lambda key: np.linalg.norm(
            np.asarray(ends.get((key, "source"), {}).get("position_mm", [0, 0, 0]))
            - ends.get((key, "target"), {}).get("position_mm", [0, 0, 0])
        ),
    )
    priority_index = {key: i for i, key in enumerate(priority)}
    order.sort(key=lambda key: priority_index.get(key, len(priority_index)))
    for key in order:
        print(f"HARNESS pass {iteration}: route", key, flush=True)
        if any((key, side) not in ends for side in ("source", "target")):
            failures.append({"id": key, "reason": "terminal_proposal_unavailable"})
            continue
        source, target = ends[key, "source"], ends[key, "target"]
        if source["owner_link"] != target["owner_link"]:
            failures.append(
                {
                    "id": key,
                    "reason": "cross_joint_requires_explicit_motion_and_flexible_harness",
                }
            )
            continue
        working = occupied.copy()
        # Other cable exits are not transit nodes.
        for (other, _), endpoint in ends.items():
            if other != key:
                working = reserve(
                    working,
                    [endpoint["position_mm"]] * 2,
                    origin,
                    pitch,
                    2 * maximum_radius + config["clearance_mm"],
                )
        start, end = np.asarray(source["grid_index"]), np.asarray(target["grid_index"])
        radius = (
            config["bus_pair_diameter_mm"] / 2
            if len(groups[key]) > 1
            else config["wire_diameter_mm"] / 2
        )
        reasons = []
        accepted = None
        for attempt in range(int(config["max_route_attempts"])):
            path = search(working, start, end, wall_distance)
            if path is None:
                reasons.append("no_free_space_path")
                break
            try:
                points = curve(
                    path, working, origin, pitch, config["min_bend_radius_mm"]
                )
                check_self_clearance(points, 2 * radius)
                tube = sweep(points, radius)
                clash = collision(tube, obstacles)
                if clash:
                    raise ValueError("exact_solid_collision:" + clash)
                mesh = as_mesh64(tube)
                filename = (
                    f"meshes/pass_{iteration:02d}_cable_{len(routes) + 1:03d}.stl"
                )
                mesh.export(out / filename, file_type="stl_ascii")
                from shell_channels.assembly import load_mesh

                reloaded = solid(load_mesh(out / filename))
                if collision(reloaded, obstacles):
                    raise ValueError("exported_tube_collision")
                accepted = (points, reloaded, filename)
                break
            except ValueError as error:
                reasons.append(str(error))
                # Force a different corridor without changing either terminal.
                mid = origin + path[len(path) // 2] * pitch
                if (
                    min(
                        np.linalg.norm(mid - (origin + start * pitch)),
                        np.linalg.norm(mid - (origin + end * pitch)),
                    )
                    < 2 * pitch
                ):
                    break
                working = reserve(working, [mid, mid], origin, pitch, pitch)
        if accepted is None:
            failures.append(
                {
                    "id": key,
                    "reason": "no_valid_rounded_collision_free_candidate",
                    "attempts": reasons,
                }
            )
            continue
        points, tube, filename = accepted
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        routes.append(
            {
                "id": key,
                "source": source,
                "target": target,
                "conductors": groups[key],
                "points_mm": points.tolist(),
                "length_mm": length,
                "diameter_mm": 2 * radius,
                "assumed_min_bend_radius_mm": config["min_bend_radius_mm"],
                "file": filename,
                "routing_method": "interior_wall_biased_spatial_route_with_retention_to_design",
                "checks": {
                    "solid_collision": False,
                    "export_reloaded": True,
                    "endpoints_preserved": True,
                    "tangent_fillet_radius_mm": config["min_bend_radius_mm"],
                },
                "status": "candidate_geometry_passed_real_interfaces_pending",
            }
        )
        obstacles["cable:" + key] = sweep(points, radius + config["clearance_mm"])
        occupied = reserve(
            occupied,
            points,
            origin,
            pitch,
            radius + maximum_radius + config["clearance_mm"],
        )
    return routes, failures


def presentation(source, out, bundle, proposals, routes, summary):
    from shell_partition.viewer import export_viewer

    pieces = defaultdict(list)
    reports = {}
    targets = []
    inventory = bundle["inventory"]["printed_parts"]
    for record in inventory:
        if record.get("kind") == "independent_removable_carrier":
            targets.append(
                {
                    "id": record["target_id"] + " carrier",
                    "kind": "carrier",
                    "owners": [
                        next(
                            s["owner_link"]
                            for s in read(source / "support_ownership.json")["supports"]
                            if s["target_id"] == record["target_id"]
                        )
                    ],
                    "mesh": as_mesh64(bundle["parts"][record["file"]]),
                }
            )
        else:
            pieces[record["owner_link"]].append(
                as_mesh64(bundle["parts"][record["file"]])
            )
    openings = read(source / "carrier_placement.json")["openings"]
    for owner in pieces:
        reports[owner] = {
            "status": "design_candidate",
            "selected_opening": {
                "panel_index": 1,
                "direction": openings[owner]["normal"],
                "clear_tasks": 0,
                "total_tasks": 0,
                "targets": [],
            },
            "pending": ["Real interfaces, retention and motion remain unverified"],
            "strategy": {
                "rationale": "Complete logical cable inventory with explicit candidate geometry status"
            },
        }
    for key, record in proposals.items():
        if "bounds_mm" in record:
            lo, hi = np.asarray(record["bounds_mm"])
            mesh = trimesh.creation.box(hi - lo).apply_translation((hi + lo) / 2)
            targets.append(
                {
                    "id": key + " [assumed/unconfirmed]",
                    "kind": "component",
                    "owners": [record["owner_link"]],
                    "mesh": mesh,
                }
            )
    for route in routes:
        targets.append(
            {
                "id": route["id"],
                "kind": "wire",
                "owners": [route["source"]["owner_link"]],
                "mesh": as_mesh64(sweep(route["points_mm"], route["diameter_mm"] / 2)),
            }
        )
    dump(
        out / "partition_report.json",
        {"parts": reports, "status": "candidate_not_physical_release"},
    )
    export_viewer(out, pieces, targets, reports)
    (out / "index.html").rename(out / "assembly_view.html")
    from .dashboard import write

    write(out)


def run(source, plan_path, output, config_path):
    source, out = Path(source).resolve(), Path(output).resolve()
    if source == out or out.is_relative_to(source) or source.is_relative_to(out):
        raise ValueError("Design output must be separate from the source assembly")
    if out.exists() and any(out.iterdir()):
        raise ValueError("Use a new empty design output directory")
    config = settings(config_path)
    plan = read(plan_path)
    if config["scenario"] != plan["protocol"]:
        raise ValueError("Assumed scenario must match the logical plan")
    from physical_wiring.checks import review

    if review(plan)["errors"]:
        raise ValueError("Logical circuit contains errors")
    before = snapshot(source)
    context = assembly_context(source)
    bundle = load_bundle(source, geometry_only=True)
    out.mkdir(parents=True, exist_ok=True)
    (out / "meshes").mkdir()
    dump(out / "source_snapshot.json", before)
    dump(out / "design_assumptions.json", config)
    dump(out / "logical_plan.json", plan)
    with np.load(source / "space_masks.npz") as data:
        origin = data["origin"].copy()
        pitch = float(data["pitch"])
        domain = data["usable"].copy()
        owner = context["owners"][context["controller"]]
        domain &= data["part_ids"] == context["links"].index(owner) + 1
        occupied = (
            data["material"].copy() | data["motor_block"] | data["protected_material"]
        )
    obstacles = {key: value for key, value in bundle["parts"].items()}
    descriptor = next(iter(bundle["surfaces"].values()))
    part_objects = {id(value) for value in bundle["parts"].values()}
    for i, value in enumerate(descriptor["closed_obstacles"]):
        if id(value) not in part_objects:
            obstacles["assembly_obstacle_" + str(i)] = value
    # Original wall/material masks already exist; rasterize added carriers and
    # final shell features so the graph also sees actual assembly details.
    print("HARNESS: classify final assembly solids", flush=True)
    for key, value in obstacles.items():
        occupied |= rasterize(value, origin, pitch, domain.shape)
    radius = max(config["wire_diameter_mm"], config["bus_pair_diameter_mm"]) / 2
    free = largest_region(
        guarded_free(domain, occupied, pitch, radius + config["clearance_mm"])
    )
    proposals, free = propose_components(
        plan, context, bundle, free, origin, pitch, obstacles, config, out
    )
    # Added component boxes are blocked with an explicit wire clearance guard.
    for key, value in obstacles.items():
        if key.startswith("proposed:"):
            occupied |= rasterize(value, origin, pitch, domain.shape)
    free = largest_region(
        guarded_free(domain, occupied, pitch, radius + config["clearance_mm"])
    )
    dump(
        out / "component_proposals.json",
        {"components": list(proposals.values()), "all_hardware_confirmed": False},
    )
    ends, endpoint_failures = propose_endpoints(
        plan, proposals, free, origin, pitch, config, obstacles
    )
    dump(
        out / "terminal_proposals.json",
        {
            "terminals": [
                {"cable_id": key, "side": side, **value}
                for (key, side), value in ends.items()
            ],
            "failures": endpoint_failures,
            "physical_registration": False,
        },
    )
    iterations = []
    best = None
    priority = []
    for iteration in range(1, 7):
        trial_routes, trial_failures = plan_routes(
            plan,
            proposals,
            ends,
            free,
            origin,
            pitch,
            dict(obstacles),
            config,
            out,
            priority,
            iteration,
        )
        iterations.append(
            {
                "iteration": iteration,
                "routed": len(trial_routes),
                "unrouted": [f["id"] for f in trial_failures],
            }
        )
        if best is None or len(trial_routes) > len(best[0]):
            best = (trial_routes, trial_failures)
        if not trial_failures:
            break
        priority = [f["id"] for f in trial_failures] + priority
        priority = list(dict.fromkeys(priority))
        if iteration % 3 == 0:
            priority = list(reversed(list(cable_groups(plan))))
    routes, failures = best
    dump(out / "routing_iterations.json", iterations)
    _channel_options = []
    if config.get("try_wall_channels", True):
        from .channels import channelize

        routes, _channel_options = channelize(bundle, routes, obstacles, config, out)
    dump(
        out / "routes.json",
        {"units": "mm", "routes": routes, "physical_release": False},
    )
    groups = cable_groups(plan)
    summary = {
        "pipeline": "harness_design_v1",
        "source": str(source),
        "scenario": plan["protocol"],
        "required_cables": len(groups),
        "routed_candidates": len(routes),
        "wall_channel_candidates": sum(
            r["routing_method"] == "wall_channel" for r in routes
        ),
        "spatial_fallback_candidates": sum(
            r["routing_method"] != "wall_channel" for r in routes
        ),
        "candidate_path_coverage_complete": len(routes) == len(groups),
        "unrouted": failures,
        "real_interface_registration_complete": False,
        "physical_routing_complete": False,
        "complete_assemblable": False,
        "source_unchanged": snapshot(source) == before,
        "unresolved_scope": [
            "real motor and module selection",
            "device-to-proposed-exit tails and connectors",
            "new module carriers",
            "bus loom internal trunk and stub lengths",
            "cable retention and groove manufacture",
            "connector insertion and removal",
            "joint motion and flex endurance",
            "power and electrical release",
        ],
    }
    dump(out / "routing_report.json", summary)
    retention = []
    for route in routes:
        points = np.asarray(route["points_mm"])
        samples = dense(points, config["support_spacing_mm"])
        indices = np.rint(
            np.linspace(
                0,
                len(samples) - 1,
                max(2, int(np.ceil(route["length_mm"] / config["support_spacing_mm"]))),
            )
        ).astype(int)
        retention.append(
            {
                "cable_id": route["id"],
                "candidate_support_centres_mm": samples[indices].tolist(),
                "preferred_method": "open_shell_channel_with_removable_cover",
                "fallback": "removable_inner_wall_clips_or_standoff_raceway",
                "status": "support_and_groove_attachment_geometry_not_yet_validated",
            }
        )
    dump(out / "retention_plan.json", {"items": retention, "print_ready": False})
    with (out / "wire_schedule.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "id",
                "source",
                "target",
                "conductors",
                "diameter_assumed_mm",
                "geometry_length_mm",
                "cut_length_mm",
                "status",
            ]
        )
        for r in routes:
            writer.writerow(
                [
                    r["id"],
                    r["source"]["instance_id"],
                    r["target"]["instance_id"],
                    len(r["conductors"]),
                    r["diameter_mm"],
                    round(r["length_mm"], 3),
                    "PENDING connector and service allowance",
                    r["status"],
                ]
            )
        for r in failures:
            writer.writerow([r["id"], "", "", "", "", "", "UNROUTED: " + r["reason"]])
    presentation(source, out, bundle, proposals, routes, summary)
    if not summary["source_unchanged"]:
        raise RuntimeError("Source assembly changed during planning")
    dump(out / "manifest.json", summary)
    return summary
