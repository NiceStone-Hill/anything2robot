"""Plan a shared harness corridor from an existing assembly and logical circuit.

An independent alternative to per-cable grooves. Device aggregation exits are
new explicit assumptions, never substituted for measured connector positions.
"""

from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh
from hardware import as_mesh64
from harness_design.geometry import (
    curve,
    largest_region,
    line_clear,
    rasterize,
    simplify,
    wire_solid,
)
from harness_design.pipeline import collision
from physical_wiring.catalog import read
from physical_wiring.pipeline import assembly_context
from physical_wiring.registration import cable_groups
from scipy.ndimage import distance_transform_edt
from shell_channels.assembly import load_bundle, load_mesh, snapshot
from shell_channels.geometry import solid
from shell_channels.pipeline import dump

from .network import (
    cable_segment_path,
    consolidate_junctions,
    growing_tree,
    pack,
    rooted_grid_tree,
    shared_segments,
    tree_path,
)


def scene(source, candidate):
    bundle = load_bundle(source, geometry_only=True)
    context = assembly_context(source)
    owner = context["owners"][context["controller"]]
    proposals = read(candidate / "component_proposals.json")["components"]
    with np.load(source / "space_masks.npz") as data:
        origin = data["origin"].copy()
        pitch = float(data["pitch"])
        domain = data["usable"] & (
            data["part_ids"] == context["links"].index(owner) + 1
        )
        occupied = data["material"] | data["motor_block"] | data["protected_material"]
    obstacles = dict(bundle["parts"])
    known = {id(v) for v in obstacles.values()}
    for i, value in enumerate(
        next(iter(bundle["surfaces"].values()))["closed_obstacles"]
    ):
        if id(value) not in known:
            obstacles["assembly:" + str(i)] = value
    for p in proposals:
        if p["geometry"] == "assumed_module_space_reservation":
            lo, hi = np.asarray(p["bounds_mm"])
            obstacles["module:" + p["id"]] = solid(
                trimesh.creation.box(hi - lo).apply_translation((lo + hi) / 2)
            )
    print(
        "BUNDLE: classify original assembly and proposed module envelopes", flush=True
    )
    for value in obstacles.values():
        occupied |= rasterize(value, origin, pitch, domain.shape)
    return (
        bundle,
        proposals,
        owner,
        origin,
        pitch,
        domain,
        distance_transform_edt(~occupied, sampling=pitch),
        obstacles,
    )


def run(candidate, output, config_path):
    candidate, out = Path(candidate).resolve(), Path(output).resolve()
    source = Path(read(candidate / "routing_report.json")["source"]).resolve()
    for protected in (candidate, source):
        if (
            out == protected
            or out.is_relative_to(protected)
            or protected.is_relative_to(out)
        ):
            raise ValueError("Use an independent output directory")
    if out.exists() and any(out.iterdir()):
        raise ValueError("Use a new empty output directory")
    config = read(config_path)
    if (
        config.get("mode") != "explicit_bundle_corridor_assumptions"
        or config.get("units") != "mm"
    ):
        raise ValueError("Bundle assumptions must be explicit")
    for key in (
        "sleeve_wall_mm",
        "cable_gap_mm",
        "clearance_mm",
        "max_exit_distance_mm",
        "exit_spacing_mm",
        "bend_radius_factor",
        "min_bend_radius_mm",
        "junction_merge_distance_mm",
    ):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError("Invalid " + key)
    preferred = config.get("preferred_bend_radius_factor", 4.0)
    if (
        isinstance(preferred, bool)
        or not isinstance(preferred, (int, float))
        or not np.isfinite(preferred)
        or preferred <= 0
    ):
        raise ValueError("Invalid preferred_bend_radius_factor")
    plan = read(candidate / "logical_plan.json")
    groups = cable_groups(plan)
    old_routes = {r["id"]: r for r in read(candidate / "routes.json")["routes"]}
    if set(old_routes) != set(groups):
        raise ValueError("Source must describe every candidate cable")
    diameters = {k: float(r["diameter_mm"]) for k, r in old_routes.items()}
    before = {"assembly": snapshot(source), "candidate": snapshot(candidate)}
    out.mkdir(parents=True, exist_ok=True)
    (out / "meshes").mkdir()
    bundle, proposals, owner, origin, pitch, domain, clearance, obstacles = scene(
        source, candidate
    )
    by_id = {p["id"]: p for p in proposals}
    devices = defaultdict(set)
    for key, members in groups.items():
        for side in ("source", "target"):
            endpoints = {m[side]["instance_id"] for m in members}
            if len(endpoints) != 1:
                raise ValueError("Cable spans multiple devices on one end")
            device = next(iter(endpoints))
            if by_id[device]["owner_link"] != owner:
                raise ValueError(
                    "Cross-joint harness requires a motion-aware adapter: " + device
                )
            devices[device].add(key)

    def packing(cables):
        return pack(
            tuple(sorted(diameters[k] for k in cables)),
            config["cable_gap_mm"],
            config["sleeve_wall_mm"],
        )

    # Reserve the conservative all-cables cross-section while choosing the tree.
    # Each exported segment is then sized from its own actual cable membership.
    envelope_radius, _ = packing(groups)
    free = largest_region(
        domain & (clearance > envelope_radius + config["clearance_mm"] + pitch * 0.87)
    )
    cells = np.argwhere(free)
    xyz = origin + cells * pitch
    center = np.mean([by_id[k]["center_mm"] for k in devices], axis=0)
    root_index = int(
        np.argmin(
            np.linalg.norm(xyz - center, axis=1) - 0.4 * clearance[tuple(cells.T)]
        )
    )
    cells, _ids, distances, predecessors = rooted_grid_tree(
        free, clearance, cells[root_index]
    )
    xyz = origin + cells * pitch
    ports = {}
    chosen = []
    print(
        f"BUNDLE: reserve {2 * envelope_radius:.1f} mm corridor; group {len(groups)} cables at {len(devices)} devices",
        flush=True,
    )
    for key, cables in sorted(devices.items(), key=lambda item: -len(item[1])):
        proposal = by_id[key]
        lo, hi = np.asarray(proposal["bounds_mm"])
        distance = np.linalg.norm(np.maximum(np.maximum(lo - xyz, xyz - hi), 0), axis=1)
        score = (
            distance
            + 0.05 * np.linalg.norm(xyz - np.asarray(proposal["center_mm"]), axis=1)
            + 0.02 * distances
        )
        order = np.argsort(score)
        chosen_index = None
        for i in order:
            if (
                not np.isfinite(distances[i])
                or distance[i] > config["max_exit_distance_mm"]
            ):
                continue
            if (
                chosen
                and np.min(np.linalg.norm(np.asarray(chosen) - xyz[i], axis=1))
                < config["exit_spacing_mm"]
            ):
                continue
            # Verify the largest shared envelope, not only a thin lead.
            import manifold3d as md

            if collision(
                md.Manifold.sphere(
                    envelope_radius + config["clearance_mm"], 24
                ).translate(xyz[i]),
                obstacles,
            ):
                continue
            chosen_index = int(i)
            break
        if chosen_index is None:
            raise ValueError("No shared harness exit near " + key)
        chosen.append(xyz[chosen_index])
        ports[key] = {
            "node": chosen_index,
            "position_mm": xyz[chosen_index].tolist(),
            "cables": sorted(cables),
            "distance_to_envelope_mm": float(distance[chosen_index]),
            "owner_link": owner,
            "status": "proposed_aggregation_exit_connector_fanout_unresolved",
        }
    predecessors = growing_tree(
        free,
        clearance,
        {p["node"] for p in ports.values()},
        ports[max(devices, key=lambda k: len(devices[k]))]["node"],
    )
    paths = {}
    for key, members in groups.items():
        a = ports[members[0]["source"]["instance_id"]]["node"]
        b = ports[members[0]["target"]["instance_id"]]["node"]
        paths[key] = tree_path(a, b, predecessors)
        if len(paths[key]) < 2:
            raise ValueError("Different device exits collapsed to one node")
    paths = consolidate_junctions(
        paths,
        {p["node"] for p in ports.values()},
        xyz,
        config["junction_merge_distance_mm"],
        lambda a, b: line_clear([a, b], free, origin, pitch),
    )
    segments = shared_segments(paths, {p["node"] for p in ports.values()})
    tube_solids = {}
    for i, segment in enumerate(segments, 1):
        segment["id"] = f"H{i:02d}"
        radius, layout = packing(segment["cables"])
        segment["diameter_mm"] = 2 * radius
        segment["packing_circles_xy_radius_mm"] = layout
        segment["packing_cable_order"] = sorted(
            segment["cables"], key=lambda key: -diameters[key]
        )
        bend = max(config["min_bend_radius_mm"], config["bend_radius_factor"] * radius)
        local_free = domain & (
            clearance > radius + config["clearance_mm"] + pitch * 0.87
        )
        path = cells[segment["nodes"]]
        bend_pass = False
        try:
            points = curve(path, local_free, origin, pitch, bend)
            bend_pass = True
        except ValueError:
            points = simplify(origin + path * pitch, local_free, origin, pitch)
        tube = wire_solid(points, radius + config["clearance_mm"])
        hit = collision(tube, obstacles)
        if hit:
            # Rounded shortcuts cannot turn a blocked segment into a claimed pass.
            points = origin + path * pitch
            bend_pass = False
            hit = collision(
                wire_solid(points, radius + config["clearance_mm"]), obstacles
            )
        segment.update(
            points_mm=points.tolist(),
            length_mm=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            kind="shared_trunk"
            if len(segment["cables"]) > 1
            else "single_cable_branch",
            assumed_bend_radius_mm=bend,
            local_bend_passed=bend_pass,
            assembly_collision=hit,
            branch_transition_validated=False,
        )
        filename = f"meshes/{segment['id']}_bundle_envelope.stl"
        value = wire_solid(points, radius)
        as_mesh64(value).export(out / filename, file_type="stl_ascii")
        reloaded = solid(load_mesh(out / filename))
        segment["exported_solid_collision"] = collision(reloaded, obstacles)
        segment["file"] = filename
        tube_solids[segment["id"]] = reloaded
        print(
            f"BUNDLE: {segment['id']}, {len(segment['cables'])} cables, diameter {2 * radius:.1f} mm, collision {hit}, bend {bend_pass}",
            flush=True,
        )
    from .smoothing import refine

    smoothing = refine(
        segments,
        cells,
        domain,
        clearance,
        origin,
        pitch,
        obstacles,
        config,
        out,
        tube_solids,
    )
    # Segments may meet at the same tree node; other overlaps are unresolved.
    # Even intended branch junctions still require cable-level fan-out geometry.
    overlaps = []
    for i, a in enumerate(segments):
        for b in segments[:i]:
            if collision(tube_solids[a["id"]], {b["id"]: tube_solids[b["id"]]}):
                shared = {a["nodes"][0], a["nodes"][-1]} & {
                    b["nodes"][0],
                    b["nodes"][-1],
                }
                overlaps.append(
                    {
                        "a": a["id"],
                        "b": b["id"],
                        "shared_tree_node": bool(shared),
                        "status": "junction_volume_requires_fanout_validation"
                        if shared
                        else "unresolved_nonjunction_envelope_overlap",
                    }
                )
    cable_records = []
    for key, members in groups.items():
        source_device = members[0]["source"]["instance_id"]
        target_device = members[0]["target"]["instance_id"]
        segment_path = cable_segment_path(
            segments, key, ports[source_device]["node"], ports[target_device]["node"]
        )
        cable_records.append(
            {
                "id": key,
                "source_device": source_device,
                "target_device": target_device,
                "conductors": members,
                "segments": segment_path,
                "status": "between_proposed_aggregation_exits_only",
                "original_proposed_exits": {
                    side: old_routes[key][side] for side in ("source", "target")
                },
            }
        )
    from .details import annotate_cables, motor_coverage

    annotate_cables(cable_records)
    motor_audit = motor_coverage(
        plan,
        cable_records,
        ports,
        assembly_context(source)["motor_candidates"],
        read(source / "mechanical_scene.json"),
    )
    dump(out / "motor_coverage.json", motor_audit)
    dump(
        out / "smoothing_report.json",
        {"segments": smoothing, "junction_transitions_validated": False},
    )
    failed = [
        s["id"]
        for s in segments
        if s["assembly_collision"] or s["exported_solid_collision"]
    ]
    unexpected = [o for o in overlaps if not o["shared_tree_node"]]
    report = {
        "pipeline": "shared_harness_corridor_v2",
        "motor_logical_coverage_complete": motor_audit["logical_coverage_complete"],
        "motor_cables_mapped": motor_audit["mapped_motor_cables"],
        "larger_bend_segments": sum(
            s["status"] == "larger_collision_checked_fillets" for s in smoothing
        ),
        "assembly_source": str(source),
        "previous_candidate": str(candidate),
        "required_cables": len(groups),
        "mapped_cables": len(cable_records),
        "logical_conductor_connections": sum(len(g) for g in groups.values()),
        "device_aggregation_exits": len(ports),
        "shared_segments": sum(len(s["cables"]) > 1 for s in segments),
        "single_cable_segments": sum(len(s["cables"]) == 1 for s in segments),
        "total_segments": len(segments),
        "corridor_total_length_mm": sum(s["length_mm"] for s in segments),
        "max_segment_diameter_mm": max(s["diameter_mm"] for s in segments),
        "assembly_collision_segments": failed,
        "nonjunction_envelope_overlaps": unexpected,
        "local_bend_pending_segments": [
            s["id"] for s in segments if not s["local_bend_passed"]
        ],
        "candidate_connectivity_complete": len(cable_records) == len(groups),
        "corridor_static_geometry_passed": not failed and not unexpected,
        "physical_routing_complete": False,
        "print_ready": False,
        "electrical_topology_changed": False,
        "source_unchanged": before
        == {"assembly": snapshot(source), "candidate": snapshot(candidate)},
        "pending": [
            "real hardware and conductor sizes",
            "new aggregation exits replace old proposed exits; connector fan-out not routed",
            "per-cable positions through junctions and bends",
            "power/signal grouping compatibility",
            "clips, removable raceway, shell attachment and assembly access",
            "joint motion and cable flex",
            "electrical release",
        ],
        "scope": "Replan mechanical harness corridors on unchanged assembly; no fresh assembly or hardware test in this comparison",
    }
    dump(out / "report.json", report)
    dump(
        out / "network.json",
        {
            "units": "mm",
            "segments": segments,
            "device_exits": ports,
            "cables": cable_records,
            "overlap_checks": overlaps,
        },
    )
    dump(out / "assumptions.json", config)
    dump(out / "source_snapshot.json", before)
    from .report import write

    write(out, bundle, proposals)
    return report
