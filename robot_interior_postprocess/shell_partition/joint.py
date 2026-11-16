"""Bounded per-link split/support co-search, with fixed hardware and routes."""

from graphlib import CycleError, TopologicalSorter

import numpy as np
from hardware import interface_targets

from .geometry import (
    access_probe,
    convex_sweep,
    evaluate_pieces,
    split_shell,
    target_sweep,
)
from .supports import (
    attach_existing,
    connected_material,
    intersection_volume,
    mesh_of,
    solid,
    support_options,
    union,
)


def mounting_order(features, target_ids, path_checks, hardware_bodies=None):
    """Removable carriers may be installed after the hardware they obstruct.

    Fused material stays present for every step. Later hardware and detachable
    carriers may obstruct earlier corridors; each obstruction becomes an
    explicit dependency checked for cycles. Inserts precede hardware/wiring.
    """
    nodes = set(target_ids)
    edges = set()
    hardware_bodies = hardware_bodies or {}
    installation = {
        f["record"]["target_id"]: f.get(
            "installation_sweep", path_checks[f["record"]["target_id"]][0]
        )
        for f in features
    }
    # Hardware is absent until its own step; permanent shell material is never deferred.
    for other_id, body in hardware_bodies.items():
        for target_id in nodes - {other_id}:
            if (
                intersection_volume(
                    body, installation.get(target_id, path_checks[target_id][0])
                )
                > 0.01
            ):
                edges.add((target_id, other_id))
        for feature in features:
            target_id = feature["record"]["target_id"]
            if (
                target_id != other_id
                and "tools" in feature
                and intersection_volume(body, feature["tools"]) > 0.01
            ):
                edges.add((target_id, other_id))
    for feature in features:
        carrier = feature.get("detachable")
        if carrier is None:
            continue
        target_id = feature["record"]["target_id"]
        for other_id in nodes - {target_id}:
            if any(
                intersection_volume(carrier, volume) > 0.01
                for volume in (
                    installation.get(other_id, path_checks[other_id][0]),
                    path_checks[other_id][1],
                )
            ):
                edges.add((other_id, target_id))
        for prior in features:
            other_id = prior["record"]["target_id"]
            if other_id != target_id and any(
                intersection_volume(carrier, prior[k]) > 0.01
                for k in ("tools", "insert_tools")
                if k in prior
            ):
                edges.add((other_id, target_id))
    graph = {node: set() for node in sorted(nodes)}
    for before, after in edges:
        graph.setdefault(after, set()).add(before)
    try:
        ordered = list(TopologicalSorter(graph).static_order())
        status = "acyclic_local_mounting_order"
    except CycleError:
        ordered, status = [], "cyclic_mounting_dependencies"
    return {
        "status": status,
        "ordered_target_ids": ordered,
        "must_precede": [{"before": a, "after": b} for a, b in sorted(edges)],
        "wiring": "Install saved wiring after hardware/carriers; final printed material must clear protected routes",
        "scope": "Local finite-template sequence, not whole-robot assembly validation",
    }


def evaluate_joint(
    shell,
    candidate,
    targets,
    plan,
    wires,
    wire_status,
    other_shells=(),
    all_targets=(),
    gap_mm=0.4,
    probe_radius_mm=12.0,
    part_id=None,
):
    """Try either piece as installation base, then finite support templates/faces.

    A feature must stay wholly on its owner side of the seam. Imported fused
    features are never bisected. This preserves the separating-plane removal
    argument; cross-seam brackets require an explicit future release interface.
    """
    targets = interface_targets(targets, plan)
    all_targets = interface_targets(all_targets, plan)
    pieces, removed = split_shell(shell, candidate, gap_mm)
    settings = plan["settings"]
    requests = [r for r in plan["targets"] if r["owner_link"] == part_id]
    lookup = {t["id"]: t for t in targets}
    original = [solid(p) for p in pieces]
    initial = list(original)
    imported = []
    outside = union([solid(m) for m in other_shells])
    for feature in plan["existing"]:
        if feature["owner_link"] != part_id:
            continue
        index = attach_existing(feature, original, candidate, gap_mm)
        if intersection_volume(feature["solid"], wires) > 0.01:
            raise ValueError(f"existing_support_blocks_fixed_route:{feature['id']}")
        if intersection_volume(feature["solid"], outside) > 0.01:
            raise ValueError(f"existing_support_crosses_rigid_links:{feature['id']}")
        initial[index] = union([initial[index], feature["solid"]])
        if not connected_material(initial[index]):
            raise ValueError(f"existing_support_disconnected:{feature['id']}")
        imported.append(
            {
                "solid": feature["solid"],
                "record": {
                    "id": feature["id"],
                    "target_id": feature["target_id"],
                    "owner_link": part_id,
                    "owner_piece_index": index,
                    "attachment": "fused",
                    "template": "imported_indivisible",
                    "status": "imported_geometry_preserved",
                    "mesh_source": feature["mesh_source"],
                    "release": "Moves with owner piece; never cut by this stage",
                    "crosses_split": False,
                    "fixation": "pending_source_interface_review",
                    "strength": "pending",
                    "volume_mm3": float(feature["solid"].volume()),
                    "wire_clear": wire_status == "protected_fixed_tubes",
                },
            }
        )
    hardware = union([solid(t["mesh"]) for t in (all_targets or targets)])
    requested_ids = {r["target_id"] for r in requests}
    permanent_hardware = union(
        [
            solid(t["mesh"])
            for t in (all_targets or targets)
            if t["id"] not in requested_ids
        ]
    )
    all_bounds = np.array([shell.bounds, *[t["mesh"].bounds for t in targets]])
    travel = float(
        np.linalg.norm(all_bounds[:, 1].max(0) - all_bounds[:, 0].min(0)) * 2 + 20
    )
    designs, generation_trace = [], []
    for panel_index, sign in ((0, -1), (1, 1)):
        owner_index = 1 - panel_index
        direction = np.asarray(candidate["normal"]) * sign
        path_checks = {
            t["id"]: (
                solid(target_sweep(t, direction, travel)),
                solid(access_probe(t["mesh"], direction, probe_radius_mm, travel)),
            )
            for t in targets
        }
        states = [
            {
                "owner": initial[owner_index],
                "features": [],
                "missing": [],
                "volume": 0.0,
            }
        ]
        for request in requests:
            expanded = []
            target = lookup[request["target_id"]]
            for state in states:
                blockers = union(
                    [
                        hardware,
                        outside,
                        *[f.get("occupancy", f["solid"]) for f in state["features"]],
                        *[
                            f["fastener_solid"]
                            for f in state["features"]
                            if "fastener_solid" in f
                        ],
                        *[f["solid"] for f in imported],
                    ]
                )
                options, attempts = support_options(
                    target,
                    request,
                    state["owner"],
                    initial[panel_index],
                    blockers,
                    wires,
                    candidate,
                    owner_index,
                    gap_mm,
                    direction,
                    settings,
                    interface=plan.get("interfaces", {}).get(target["id"]),
                    wire_relief=plan.get("wire_relief"),
                    access_obstacles=union(
                        [
                            outside,
                            permanent_hardware,
                            *[f["solid"] for f in state["features"]],
                            *[f["solid"] for f in imported],
                        ]
                    ),
                )
                accepted = []
                for option in options:
                    option["record"]["wire_protection"] = wire_status
                    option["record"]["wire_clear"] = (
                        wire_status == "protected_fixed_tubes"
                    )
                    blocked = None
                    for required in requests:
                        swept, corridor = path_checks[required["target_id"]]
                        # A detachable carrier is installed with its own PCB;
                        # check that combined motion in mounting_options instead.
                        checked = option["solid"]
                        if intersection_volume(checked, swept) > 0.01:
                            blocked = (
                                "support_blocks_body_insertion",
                                required["target_id"],
                            )
                            break
                        if intersection_volume(checked, corridor) > 0.01:
                            blocked = (
                                "support_blocks_assumed_grip_corridor",
                                required["target_id"],
                            )
                            break
                    if blocked:
                        attempts.append(
                            {
                                "template": option["record"]["template"],
                                "reason": blocked[0],
                                "blocked_target_id": blocked[1],
                            }
                        )
                        continue
                    owner = union([state["owner"], option["solid"]])
                    # Recheck earlier tool corridors after every added support.
                    # Tool/fastener volumes are stage-specific; they are not fused material.
                    for prior in state["features"]:
                        occupied = option.get("occupancy", option["solid"])
                        if any(
                            intersection_volume(
                                prior[k],
                                occupied if k == "fastener_solid" else option["solid"],
                            )
                            > 0.01
                            for k in ("tools", "insert_tools", "fastener_solid")
                            if k in prior
                        ):
                            blocked = (
                                "mount_blocks_prior_fastener_or_tool",
                                prior["record"]["target_id"],
                            )
                            break
                    if blocked:
                        attempts.append(
                            {
                                "template": option["record"]["template"],
                                "reason": blocked[0],
                                "blocked_target_id": blocked[1],
                            }
                        )
                        continue
                    swept, corridor = path_checks[target["id"]]
                    if intersection_volume(owner, swept) > 0.01:
                        attempts.append(
                            {
                                "template": option["record"]["template"],
                                "reason": "support_blocks_body_insertion",
                            }
                        )
                        continue
                    if intersection_volume(owner, corridor) > 0.01:
                        attempts.append(
                            {
                                "template": option["record"]["template"],
                                "reason": "support_blocks_assumed_grip_corridor",
                            }
                        )
                        continue
                    accepted.append(
                        {
                            "owner": owner,
                            "features": state["features"] + [option],
                            "missing": list(state["missing"]),
                            "volume": state["volume"] + option["record"]["volume_mm3"],
                        }
                    )
                generation_trace.append(
                    {
                        "panel_index": panel_index,
                        "target_id": target["id"],
                        "prior_supports": [
                            f["record"]["id"] for f in state["features"]
                        ],
                        "attempts": attempts,
                        "accepted_options": len(accepted),
                    }
                )
                if accepted:
                    expanded.extend(accepted[: settings["max_options_per_target"]])
                else:
                    expanded.append(
                        {**state, "missing": state["missing"] + [target["id"]]}
                    )

            # Preserve alternatives instead of committing greedily to one post layout.
            def score(state, path_checks=path_checks):
                clear = sum(
                    intersection_volume(state["owner"], path_checks[t["id"]][0]) < 0.01
                    for t in targets
                    if t["kind"] != "motor"
                )
                return (
                    sum("holes" in f["record"] for f in state["features"]),
                    len(state["features"]),
                    clear,
                    -state["volume"],
                )

            states = sorted(expanded, key=score, reverse=True)[: settings["beam_width"]]
        for state in states:
            final = list(initial)
            final[owner_index] = state["owner"]
            final_meshes = [mesh_of(value) for value in final]
            _, report = evaluate_pieces(
                shell, final_meshes, candidate, targets, removed, probe_radius_mm
            )
            opening = report["opening_options"][panel_index]
            local_order = mounting_order(
                state["features"],
                [r["target_id"] for r in requests],
                path_checks,
                {
                    t["id"]: solid(t["mesh"])
                    for t in targets
                    if t["id"] in requested_ids
                },
            )
            if local_order["status"] == "cyclic_mounting_dependencies":
                opening["all_local_proxy_tasks_clear"] = False
            mount_closure = []
            for feature in state["features"]:
                installed = union(
                    [
                        feature[k]
                        for k in ("detachable", "fastener_solid")
                        if k in feature
                    ]
                )
                if not installed.is_empty():
                    sweep = solid(convex_sweep(mesh_of(installed), -direction, travel))
                    overlap = intersection_volume(final[panel_index], sweep)
                    mount_closure.append(
                        {
                            "target_id": feature["record"]["target_id"],
                            "panel_translation_overlap_mm3": overlap,
                            "clear": overlap < 0.01,
                        }
                    )
            opening["installed_mount_closure"] = mount_closure
            if any(not c["clear"] for c in mount_closure):
                opening["all_local_proxy_tasks_clear"] = False
            report["selected_opening"] = opening
            records = [f["record"] for f in imported + state["features"]]
            # Missing targets remain explicit. Partial geometry isn't an alternative
            # way to satisfy a required support request.
            report["support_design"] = {
                "status": "concept_geometry_checked"
                if not state["missing"]
                else "unresolved",
                "required_count": len(requests),
                "generated_count": len(state["features"]),
                "imported_count": len(imported),
                "missing_target_ids": state["missing"],
                "all_required_generated": not state["missing"],
                "ownership": records,
                "registered_interface_count": sum("holes" in r for r in records),
                "local_mounting_order": local_order,
                "installation_piece_index": owner_index,
                "wire_protection": wire_status,
                "standalone_support_volume_mm3": state["volume"],
                "added_material_volume_mm3": float(
                    sum(v.volume() for v in final) - sum(v.volume() for v in original)
                ),
                "unplanned_target_ids": [
                    t["id"]
                    for t in targets
                    if t["id"]
                    not in {r["target_id"] for r in requests}
                    | {f["record"]["target_id"] for f in imported}
                ],
                "mount_interfaces": "explicit_registered_interfaces"
                if any("holes" in r for r in records)
                else "pending_measured_hardware_data",
                "scope": "Final support geometry and local insertion; registered interfaces include screw/tool checks; strength and whole-robot order pending",
            }
            report["status"] = (
                "joint_local_proxy_access_clear"
                if not state["missing"] and opening["all_local_proxy_tasks_clear"]
                else "partial_support_or_access"
            )
            report["piece_separation"] = (
                "Continuous separating-plane proof; imported/generated fused supports remain on their owner side"
            )
            key = (
                not state["missing"] and opening["all_local_proxy_tasks_clear"],
                sum("holes" in r for r in records),
                len(state["features"]),
                opening["clear_tasks"],
                sum(t["panel_clear"] for t in opening["targets"]),
                -state["volume"],
            )
            designs.append((key, final_meshes, report, imported + state["features"]))
    _, final_meshes, report, features = max(designs, key=lambda d: d[0])
    report["support_design"]["generation_trace"] = generation_trace
    return final_meshes, report, features
