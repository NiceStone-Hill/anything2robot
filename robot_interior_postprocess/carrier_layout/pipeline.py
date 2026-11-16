"""Read-only mechanical adapter -> carrier EPS -> fabrication -> assembly dossier."""

from __future__ import annotations

import csv
import itertools
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
from common import ROOT, dump, geometry_signature, read
from hardware import as_mesh64 as as_mesh
from shell_partition.geometry import candidates, split_shell
from shell_partition.io import fingerprint, observations, render_views
from shell_partition.supports import intersection_volume, rod, solid, union
from shell_partition.viewer import export_viewer

from .fabrication import build_link, export_solid
from .profiles import build_profile, safe_id, vector
from .search import objective, plan_link, projection_metrics

DEFAULTS = {
    "seed_pitch_mm": 12.0,
    "max_seed_candidates": 320,
    "beam_width": 8,
    "max_evaluations": 240,
    "step_mm": [12.0, 6.0, 3.0],
    "clearance_mm": 1.0,
    "gap_mm": 0.4,
    "seam_keepout_mm": 3.0,
    "max_architectures": 12,
    "max_route_attempts": 3,
    "weights": {"coplanarity": 2.0, "compactness": 0.1, "projection_overlap": 40.0},
}


def settings_for(raw):
    if raw.keys() - DEFAULTS.keys():
        raise ValueError("Unknown assembly search setting")
    value = {**DEFAULTS, **raw}
    for key in DEFAULTS.keys() - {"step_mm", "weights"}:
        if (
            isinstance(value[key], bool)
            or not isinstance(value[key], (int, float))
            or not np.isfinite(value[key])
            or value[key] <= 0
        ):
            raise ValueError(f"Invalid positive setting: {key}")
    for key in (
        "max_seed_candidates",
        "beam_width",
        "max_evaluations",
        "max_architectures",
        "max_route_attempts",
    ):
        if int(value[key]) != value[key]:
            raise ValueError(f"Integer setting required: {key}")
        value[key] = int(value[key])
    if (
        value["beam_width"] > 32
        or value["max_architectures"] > 64
        or value["max_evaluations"] > 10000
    ):
        raise ValueError("Search budget exceeds prototype limits")
    if not value["step_mm"] or any(
        not np.isfinite(v) or v <= 0 for v in value["step_mm"]
    ):
        raise ValueError("Positive finite EPS step schedule required")
    if set(value["weights"]) != {
        "coplanarity",
        "compactness",
        "projection_overlap",
    } or any(not np.isfinite(v) or v < 0 for v in value["weights"].values()):
        raise ValueError("Invalid objective weights")
    return value


def import_mechanical(config, base, out, sources, cache=None):
    """Raw Anything2robot files are the authoritative input, not old placement.

    A normalized mechanical adapter also accepts other robot/circuit instances;
    the search and assembly kernel has no Eagle or cactus identifier rules.
    """
    mechanical = config["mechanical"]
    kind = mechanical["adapter"]
    if kind == "anything2robot":
        from mechanical import load_scene
        from space import build_space

        path = (base / mechanical["config"]).resolve()
        sources[str(path)] = fingerprint(path)
        raw = read(path)
        sys.path.insert(0, raw["voxel_tool_source"])
        signature = geometry_signature(raw)
        sources.update(signature["input_sha256"])
        scene, meshes = load_scene(raw, out)
        if cache:
            cache = Path(cache).resolve()
            if read(cache / "geometry_signature.json") != signature:
                raise ValueError(
                    "Geometry cache does not match raw mechanical inputs/config"
                )
            integrity_path = cache / "geometry_cache_manifest.json"
            if not integrity_path.exists():
                raise ValueError(
                    "Geometry cache lacks artifact integrity hashes; rebuild once without --geometry-cache"
                )
            integrity = read(integrity_path)
            sources[str(integrity_path)] = fingerprint(integrity_path)
            names = ["space_masks.npz", "space_report.json"]
            names += [
                f"meshes/{name}_hollow_assembly_mm.stl" for name in scene["parts"]
            ]
            for name in names:
                if integrity.get("artifacts", {}).get(name) != fingerprint(
                    cache / name
                ):
                    raise ValueError("Geometry cache artifact hash mismatch: " + name)
                sources[str(cache / name)] = fingerprint(cache / name)
                shutil.copy2(cache / name, out / name)
            with np.load(out / "space_masks.npz") as data:
                space = {k: data[k].copy() for k in data.files}
            space["pitch"] = float(space["pitch"])
            hollows = {
                name: trimesh.load(
                    out / "meshes" / f"{name}_hollow_assembly_mm.stl", force="mesh"
                )
                for name in scene["parts"]
            }
        else:
            space, hollows = build_space(meshes, scene, raw, out)
        dump(out / "geometry_signature.json", signature)
        cache_files = ["space_masks.npz", "space_report.json"] + [
            f"meshes/{name}_hollow_assembly_mm.stl" for name in scene["parts"]
        ]
        dump(
            out / "geometry_cache_manifest.json",
            {"artifacts": {name: fingerprint(out / name) for name in cache_files}},
        )
        return scene, space, hollows, raw
    if kind != "normalized":
        raise ValueError("Unknown mechanical adapter")
    if cache:
        raise ValueError(
            "--geometry-cache applies only to the anything2robot mechanical adapter"
        )
    path = (base / mechanical["directory"]).resolve()
    files = [path / "mechanical_scene.json", path / "space_masks.npz"]
    scene = read(files[0])
    if scene.get("units") != "millimeter":
        raise ValueError("Normalized scene requires explicit millimeter units")
    if not scene.get("parts") or len(set(scene["parts"])) != len(scene["parts"]):
        raise ValueError("Unique rigid-link IDs required")
    hollows = {}
    for name in scene["parts"]:
        safe_id(name)
        mesh_path = path / "meshes" / f"{name}_hollow_assembly_mm.stl"
        files.append(mesh_path)
        hollows[name] = trimesh.load(mesh_path, force="mesh")
        # A shell with through holes cannot safely recover its original exterior.
        # Preserve an optional source solid for clipping the fastening features.
        exterior_path = path / "meshes" / f"{name}_solid_assembly_mm.stl"
        if exterior_path.exists():
            exterior = trimesh.load(exterior_path, force="mesh")
            if not exterior.is_volume:
                raise ValueError("Normalized exterior must be a closed solid")
            files.append(exterior_path)
            (out / "meshes").mkdir(parents=True, exist_ok=True)
            shutil.copy2(exterior_path, out / "meshes" / exterior_path.name)
    if any(not m.is_volume for m in hollows.values()):
        raise ValueError("Closed oriented shell materials required")
    with np.load(files[1]) as data:
        space = {k: data[k].copy() for k in data.files}
    for file in files:
        sources[str(file)] = fingerprint(file)
    space["pitch"] = float(space["pitch"])
    dump(out / "mechanical_scene.json", scene)
    return scene, space, hollows, None


def voxel_bounds(bounds, space):
    lo = np.floor((bounds[0] - space["origin"]) / space["pitch"]).astype(int)
    hi = np.ceil((bounds[1] - space["origin"]) / space["pitch"]).astype(int) + 1
    return tuple(
        slice(max(0, int(a)), min(int(b), n))
        for a, b, n in zip(lo, hi, space["material"].shape)
    )


def validate_space(space, scene):
    masks = {"usable", "material", "motor_block", "protected_material", "part_ids"}
    if not masks <= space.keys():
        raise ValueError("Mechanical adapter is missing required space masks")
    shape = space["usable"].shape
    if (
        len(shape) != 3
        or any(n < 2 for n in shape)
        or any(space[k].shape != shape for k in masks)
    ):
        raise ValueError("All mechanical masks must share one three-dimensional grid")
    if any(space[k].dtype != bool for k in masks - {"part_ids"}):
        raise ValueError("Material and air masks must be explicit booleans")
    if (
        not np.isfinite(space["pitch"])
        or space["pitch"] <= 0
        or np.asarray(space["origin"]).shape != (3,)
        or not np.isfinite(space["origin"]).all()
    ):
        raise ValueError("Finite grid origin and positive pitch required")
    if (
        not np.issubdtype(space["part_ids"].dtype, np.integer)
        or np.any(space["part_ids"] < 0)
        or np.any(space["part_ids"] > len(scene["parts"]))
    ):
        raise ValueError("Grid part IDs do not match the rigid-link index")
    if np.any(space["usable"] & (space["material"] | space["motor_block"])):
        raise ValueError("Mechanical adapter marks blocked material as usable air")


def route_candidate(
    config,
    raw,
    scene,
    space,
    hollows,
    profiles,
    poses,
    designs,
    out,
    sources,
    base,
    tail_variant=0,
):
    """Optional existing electrical adapter; all material is reserved before A*.

    This adapter is deliberately outside the general placement kernel. Its port
    assumptions remain explicit and prevent complete-hardware certification.
    """
    adapter = config.get("electrical", {}).get("adapter", "none")
    if adapter == "none":
        dump(
            out / "routing.json",
            {"routes": [], "status": "pending_no_electrical_adapter"},
        )
        return {"status": "pending", "reason": "no_electrical_adapter"}, hollows
    if adapter == "normalized":
        from .wiring import explicit_route

        return explicit_route(
            config["electrical"],
            base,
            space,
            profiles,
            poses,
            designs,
            out,
            sources,
            config.get("voxel_tool_source"),
        ), hollows
    if adapter != "legacy_eagle" or raw is None:
        raise ValueError(
            "Unsupported electrical adapter; use legacy_eagle only with its declared input config"
        )
    from circuit import component_schema, import_circuit
    from layout import fixed_motors
    from routing import route_layout
    from voxel_tool.stages.placement.planner import (
        Placement,
        PlacementResult,
        save_placement_result,
    )

    for file in (ROOT / raw["circuit_directory"]).iterdir():
        if file.suffix.lower() in {".brd", ".sch", ".csv"}:
            sources[str(file.resolve())] = fingerprint(file)
    topology, connectivity = import_circuit(raw, out, scene)
    local = {k: v.copy() if isinstance(v, np.ndarray) else v for k, v in space.items()}
    fixed = fixed_motors(scene, raw, local)
    schema = component_schema(raw, connectivity, fixed, out, scene)
    placement = list(fixed)
    lookup = {p["spec"]["id"]: p for p in profiles}
    for pose in poses:
        p = lookup[pose["id"]]
        bounds = pose["bounds"]
        start = np.floor((bounds[0] - local["origin"]) / local["pitch"]).astype(int)
        stop = np.ceil((bounds[1] - local["origin"]) / local["pitch"]).astype(int) + 1
        placement.append(
            Placement(
                pose["id"],
                p["spec"]["component_id"],
                pose["id"],
                tuple(pose["transform"][:3, 3]),
                tuple(bounds[1] - bounds[0]),
                tuple(int(v) for v in start),
                tuple(int(v) for v in stop - start),
                (0.2, 0.6, 0.8),
                tuple(tuple(r) for r in pose["transform"][:3, :3]),
            )
        )
        component = next(
            c for c in schema["components"] if c["id"] == p["spec"]["component_id"]
        )
    expected = {c["id"] for c in connectivity["components"]}
    if {p.component_id for p in placement} != expected:
        raise ValueError(
            "Electrical components do not exactly match all carrier/fixed-motor instances"
        )
    for design in designs.values():
        for dock in design["docks"].values():
            # Conservative block of the whole bracket, including empty bore.
            slices = voxel_bounds(as_mesh(dock).bounds, local)
            local["usable"][slices] = False
            local["material"][slices] = True
            local["protected_material"][slices] = True
    # The legacy circuit has no measured connector frames. Design explicitly
    # provisional cable-tail locations in existing air, rather than retaining
    # the old arbitrary +Y port when the whole carrier has changed orientation.
    occupied = np.zeros(local["usable"].shape, bool)
    for pose in poses:
        occupied[voxel_bounds(pose["bounds"], local)] = True
    tail_air = local["usable"] & ~occupied
    tails = []
    for pose in poses:
        p = lookup[pose["id"]]
        component = next(
            c for c in schema["components"] if c["id"] == p["spec"]["component_id"]
        )
        lo, hi = p["bounds"]
        options = []
        for axis in range(3):
            tangent = max(
                (i for i in range(3) if i != axis), key=lambda i: hi[i] - lo[i]
            )
            for sign in (-1, 1):
                for distance in (1.0, 2.0, 3.0):
                    points = []
                    for i in range(len(component["interfaces"])):
                        point = (lo + hi) / 2
                        point[axis] = (
                            lo[axis] if sign < 0 else hi[axis]
                        ) + sign * local["pitch"] * distance
                        point[tangent] += (
                            i - (len(component["interfaces"]) - 1) / 2
                        ) * min(
                            2.54,
                            (hi[tangent] - lo[tangent])
                            / (len(component["interfaces"]) + 1),
                        )
                        points.append(point)
                    normal = np.eye(3)[axis] * sign
                    checks = []
                    for lead in (0, 1, 2):
                        world = np.array(
                            [
                                (
                                    pose["transform"]
                                    @ np.r_[point + normal * local["pitch"] * lead, 1]
                                )[:3]
                                for point in points
                            ]
                        )
                        cells = np.rint(
                            (world - local["origin"]) / local["pitch"]
                        ).astype(int)
                        checks.append(
                            not (np.any(cells < 0) or np.any(cells >= tail_air.shape))
                            and bool(tail_air[tuple(cells.T)].all())
                        )
                    if all(checks):
                        options.append((distance, axis, sign, points, normal))
        if not options:
            raise ValueError("no_planning_cable_tail_in_existing_air:" + pose["id"])
        options.sort(key=lambda row: row[:3])
        distance, axis, sign, points, normal = options[
            min(tail_variant, len(options) - 1)
        ]
        for port, point in zip(component["interfaces"], points):
            port["position_mm"], port["normal"] = point.tolist(), normal.tolist()
        tails.append(
            {
                "instance_id": pose["id"],
                "local_axis": axis,
                "sign": sign,
                "offset_grid_steps": distance,
                "status": "provisional_cable_tail_not_measured_pin",
                "clear_outward_steps": 2,
            }
        )
    dump(out / "planning_tail_selection.json", tails)
    result = PlacementResult(tuple(placement), (), 0.0)
    save_placement_result(result, out / "placement.json")
    _grid, routing = route_layout(
        raw, local, scene, hollows.copy(), result, schema, topology, out
    )
    updated = {}
    for name, shell in hollows.items():
        path = out / "meshes" / f"{name}_hollow_with_access_mm.stl"
        updated[name] = trimesh.load(path, force="mesh") if path.exists() else shell
    # A grid path can be clear while a finite-radius segment grazes a mesh.
    # Block those cells and reroute; never raise the collision acceptance limit.
    from routing import validate_routes
    from voxel_tool.domain.labels import VoxelLabel
    from voxel_tool.stages.routing.router import (
        load_netlist_mapping,
        route_topology,
        save_routing_result,
    )

    from .wiring import blocked_route_cells

    shell_material = union([solid(mesh) for mesh in updated.values()])
    refinements = []
    for iteration in range(3):
        blocked_cells = blocked_route_cells(
            routing.routes,
            shell_material,
            raw["wire_radius_mm"] + 0.2,
            _grid.origin,
            _grid.pitch,
        )
        if not blocked_cells:
            break
        refinements.append(
            {
                "iteration": iteration + 1,
                "blocked_cells": len(blocked_cells),
                "reason": "finite_radius_route_near_shell",
                "radial_guard_mm": 0.2,
            }
        )
        for cell in blocked_cells:
            _grid.labels[cell] = int(VoxelLabel.SHELL_MATERIAL)
        routing = route_topology(
            _grid,
            result,
            topology,
            load_netlist_mapping(out / "circuit_mapping.json"),
            component_schema_path=out / "components.json",
            terminal_access_mode="strict",
        )
        save_routing_result(routing, out / "routing.json")
        validate_routes(_grid, result, routing, out)
    dump(out / "finite_radius_refinement.json", refinements)
    routing_json = read(out / "routing.json")
    tubes = []
    for route in routing_json["routes"]:
        points = route["points_mm"]
        tubes.extend(
            rod(np.array(a), np.array(b), raw["wire_radius_mm"])
            for a, b in itertools.pairwise(points)
            if np.linalg.norm(np.array(a) - b) > 1e-6
        )
    wire = union(tubes)
    return {
        "status": "passed" if not routing.unrouted else "failed",
        "branches": len(routing.routes),
        "unrouted": list(routing.unrouted),
        "wire_solid": wire,
        "radius_mm": raw["wire_radius_mm"],
        "ports": "provisional planning cable tails; measured pins and electrical compatibility pending",
    }, updated


def sequence_and_validation(profiles, designs, scene, gravity, closures=None):
    closures = closures or {}
    tasks, dependencies, checks = [], [], []
    profile_by_id = {p["spec"]["id"]: p for p in profiles}
    for owner, design in designs.items():
        previous_carrier = None
        all_insert_tasks = [
            f"{key}:install_dock_inserts" for key in design["carrier_order"]
        ]
        all_fixed_tasks = [
            f"{key}:fasten_carrier_to_shell" for key in design["carrier_order"]
        ]
        for carrier_id in design["carrier_order"]:
            record = next(r for r in design["records"] if r["target_id"] == carrier_id)
            check = next(r for r in design["checks"] if r["target_id"] == carrier_id)
            key = record["target_id"]
            previous = None
            for action, stage in [
                ("install_dock_inserts", "open_shell_without_carriers"),
                ("fasten_component_to_carrier", "bench"),
                ("insert_carrier_assembly", "open_shell"),
                ("fasten_carrier_to_shell", "open_shell"),
                ("connect_wires", "open_shell"),
            ]:
                task = {
                    "id": f"{key}:{action}",
                    "target_id": key,
                    "owner_link": owner,
                    "action": action,
                    "stage": stage,
                    "status": "pending"
                    if action == "connect_wires"
                    else "geometry_checked",
                    "requires": [previous] if previous else [],
                }
                if (
                    action == "fasten_component_to_carrier"
                    and not profile_by_id[key]["interface"]
                ):
                    task["status"] = "pending_measured_component_retention"
                if action == "insert_carrier_assembly":
                    task["requires"].extend(all_insert_tasks)
                if action == "connect_wires":
                    task["requires"] = all_fixed_tasks.copy()
                if action == "insert_carrier_assembly" and previous_carrier:
                    task["requires"].append(previous_carrier)
                tasks.append(task)
                for requirement in task["requires"]:
                    dependencies.append([requirement, task["id"]])
                previous = task["id"]
            previous_carrier = f"{key}:fasten_carrier_to_shell"
            checks.append(check)
        closure_id = f"{owner}:close_cover"
        tasks.append(
            {
                "id": closure_id,
                "owner_link": owner,
                "action": "close_and_fasten_cover",
                "requires": [
                    f"{r['target_id']}:connect_wires" for r in design["records"]
                ],
                "status": "pending_closure_fasteners",
            }
        )
    pending = [
        {"target_id": p["spec"]["id"], "code": code}
        for p in profiles
        for code in p["pending"]
    ]
    for name, closure in closures.items():
        insert_task = f"{name}:install_shell_inserts"
        tasks.append(
            {
                "id": insert_task,
                "owner_link": name,
                "action": "install_shell_inserts_before_carriers",
                "requires": [],
                "status": "geometry_checked",
                "physical_fit": "pending_prototype_calibration",
            }
        )
        for task in tasks:
            if (
                task.get("owner_link") == name
                and task["action"] == "insert_carrier_assembly"
            ):
                task["requires"].append(insert_task)
        close_id = f"{name}:close_cover"
        task = next((t for t in tasks if t["id"] == close_id), None)
        if task is None:
            task = {"id": close_id, "owner_link": name, "requires": [insert_task]}
            tasks.append(task)
        task.update(
            action="align_locating_spigots_and_close_cover", status="geometry_checked"
        )
        task["requires"] = list(dict.fromkeys([*task["requires"], insert_task]))
        tasks.append(
            {
                "id": f"{name}:tighten_shell_screws",
                "owner_link": name,
                "action": "insert_and_tighten_external_shell_screws",
                "requires": [close_id],
                "status": "geometry_checked",
                "connection_count": closure["report"]["connection_count"],
                "physical_fit_and_torque": "pending",
            }
        )
    dependencies = [
        [requirement, task["id"]]
        for task in tasks
        for requirement in task.get("requires", [])
    ]
    pending += [
        {"owner_link": name, "code": "cover_locating_and_fastening_interfaces"}
        for name in scene["parts"]
        if name not in closures
    ]
    pending += [
        {"owner_link": name, "code": "shell_fastener_process_fit_and_strength"}
        for name in closures
    ]
    pending += [
        {
            "motor_id": m["id"],
            "code": "motor_mount_output_calibration_and_wiring_interfaces",
        }
        for m in scene.get("motors", [])
    ]
    pending += [
        {"code": "assembly_fixture_and_intermediate_stability"},
        {"code": "connector_insertion_and_harness_sequence"},
        {"code": "physical_print_fit_and_load_test"},
    ]
    return {
        "tasks": tasks,
        "dependencies": dependencies,
        "scope": "Carrier and shell-closure geometry; bench fastening; actual fit/torque, fixtures, motor tasks and electrical assembly remain pending",
    }, {
        "status": "partial",
        "complete_assemblable": False,
        "carrier_path_checks": checks,
        "all_carrier_paths_clear": bool(checks)
        and all(all(v for k, v in c.items() if k.endswith("_clear")) for c in checks),
        "pending": pending,
        "shell_closure_geometry_checked": bool(closures)
        and set(closures) == set(scene["parts"]),
        "gravity": gravity,
        "structural_validation": {
            "status": "pending",
            "reason": "Explicit gravity is recorded; unknown masses/material/fastener capacities prohibit a load certificate",
        },
        "manufacturing_release": "blocked_until_all_required_evidence_and_validation_pass",
    }


def run_pipeline(
    config_path,
    out,
    cache=None,
    proposal=None,
    live_vlm=False,
    *,
    record_process=True,
    schema_proposal=None,
):
    config_path, out = Path(config_path).resolve(), Path(out).resolve()
    if out.exists() and any(out.iterdir()):
        raise ValueError("Output must be a new empty directory")
    out.mkdir(parents=True, exist_ok=True)
    (out / "meshes").mkdir()
    config = read(config_path)
    if (
        config.get("schema_version") != 1
        or isinstance(config.get("schema_version"), bool)
        or config.get("units") != "mm"
    ):
        raise ValueError("Assembly configuration requires schema_version=1, units=mm")
    sources = {str(config_path): fingerprint(config_path)}
    manifest = {
        "status": "running",
        "complete_assemblable": False,
        "input": "raw Anything2robot mechanical output or normalized mechanical bundle",
        "units": "mm",
        "pipeline": "carrier_assembly_v1",
    }
    dump(out / "manifest.json", manifest)
    dump(out / "assembly_config_snapshot.json", config)
    from .process_trace import ProcessRecorder

    recorder = ProcessRecorder(out) if record_process else None
    electrical_review = None
    try:
        review_path = config.get("electrical", {}).get("review_plan")
        if review_path:
            from electrical_review import review

            review_path = (config_path.parent / review_path).resolve()
            sources[str(review_path)] = fingerprint(review_path)
            electrical_review = review(read(review_path))
            dump(out / "electrical_review.json", electrical_review)
            if electrical_review["errors"]:
                raise ValueError("Electrical preflight failed; inspect electrical_review.json")
        settings = settings_for(config.get("search", {}))
        gravity = config["gravity"]
        vector(gravity["world_m_s2"], "gravity")
        if not gravity.get("evidence"):
            raise ValueError("Gravity frame evidence must be explicit")
        scene, space, hollows, raw = import_mechanical(
            config, config_path.parent, out, sources, cache
        )
        validate_space(space, scene)
        if recorder:
            original = {}
            for name in scene["parts"]:
                source = out / "meshes" / f"{name}_solid_assembly_mm.stl"
                original[name] = (
                    trimesh.load(source, force="mesh")
                    if source.exists()
                    else hollows[name]
                )
            recorder.capture(
                "mechanical_input",
                "Import robot mechanical output",
                [
                    "Load shells and fixed servo envelopes in assembly coordinates. Recording starts at the Anything2robot output.",
                    "For normalized input without an original solid, show its supplied hollow material; do not reconstruct unknown upstream stages.",
                ],
                recorder.shells(original) + recorder.motors(scene),
                {"adapter": config["mechanical"]["adapter"], "links": scene["parts"]},
            )
            recorder.capture(
                "hollow_geometry",
                "Load hollow shell geometry",
                [
                    "Replace the original solid outlines with the hollow shells used by the solver.",
                    "Reuse a hollow-geometry cache verified by input signatures and artifact hashes."
                    if cache
                    else "Obtain hollow shells and spatial masks from the mechanical adapter.",
                ],
                recorder.shells(hollows) + recorder.motors(scene),
                {
                    "geometry_cache_used": bool(cache),
                    "usable_voxels": int(space["usable"].sum()),
                },
            )
        print("ASSEMBLY: build independent carriers", flush=True)
        profiles = [
            build_profile(p, config_path.parent, sources) for p in config["components"]
        ]
        ids = [p["spec"]["id"] for p in profiles]
        if not ids or len(set(ids)) != len(ids):
            raise ValueError("Every hardware instance needs a unique carrier profile")
        if any(p["spec"]["owner_link"] not in scene["parts"] for p in profiles):
            raise ValueError("Carrier assigned to unknown rigid link")
        if recorder:
            recorder.capture(
                "carrier_profiles",
                "Generate independent component carriers",
                [
                    "Read component geometry, mounting interfaces or explicit planning envelopes; generate carriers and mounting ears.",
                    "Display local models side by side on the bench; their robot placement has not been selected.",
                ],
                recorder.bundles(profiles),
                {
                    "carrier_count": len(profiles),
                    "pending_by_component": {
                        p["spec"]["id"]: p["pending"] for p in profiles
                    },
                },
                frame="bench_local_mm",
            )
        groups = {
            name: [p for p in profiles if p["spec"]["owner_link"] == name]
            for name in scene["parts"]
        }
        catalog = {
            name: candidates(
                name,
                shell,
                [
                    m
                    for m in scene.get("motors", [])
                    if name in (m["parent"], m["link"])
                ],
            )
            for name, shell in hollows.items()
        }
        # Candidate directions are supplied explicitly; no semantic world-axis assumptions.
        openings = config["openings"]
        if set(openings) != set(scene["parts"]):
            raise ValueError(
                "Every link requires opening candidates, including motor-only links"
            )
        for name, rows in openings.items():
            if not rows:
                raise ValueError("Empty opening list")
            for row in rows:
                normal = vector(row["normal"], "opening normal", unit=True)
                if (
                    not np.isclose(abs(normal).max(), 1)
                    or not 0.1 <= row["fraction"] <= 0.9
                ):
                    raise ValueError(
                        "First carrier search supports cardinal opening normals and fractions .1.. .9"
                    )
        images = []
        provenance = {
            "source": "configured_architecture_candidates",
            "live_model_called": False,
        }
        if proposal or live_vlm:
            from shell_partition.model import propose

            if proposal:
                proposal_path = Path(proposal).resolve()
                sources[str(proposal_path)] = fingerprint(proposal_path)

            context = {
                "schema_version": 1,
                "units": "mm",
                "gravity": gravity,
                "parts": [
                    {
                        "part_id": name,
                        "bounds_mm": m.bounds.tolist(),
                        "candidates": catalog[name],
                    }
                    for name, m in hollows.items()
                ],
                "carrier_design": [
                    {
                        "id": p["spec"]["id"],
                        "owner_link": p["spec"]["owner_link"],
                        "bounds_local_mm": p["bounds"].tolist(),
                        "pending": p["pending"],
                    }
                    for p in profiles
                ],
                "preserved_workflow": "Fixed motors/joints; movable independent carriers; translations and atomic rotations; no invented holes",
            }
            images = observations(hollows, [], out)
            # Existing client and environment, no separate key/model configuration.
            env = ROOT / "robot_interior_postprocess/.env"
            proposals = propose(context, catalog, images, out, env, replay=proposal)
            provenance = read(out / "model_provenance.json")
            for row in proposals["parts"]:
                name = row["part_id"]
                ranked = [
                    next(c for c in catalog[name] if c["id"] == key)
                    for key in row["ranked_candidate_ids"]
                ]
                openings[name] = [
                    {"normal": c["normal"], "fraction": c["fraction"]}
                    for c in ranked
                    if np.isclose(max(abs(np.array(c["normal"]))), 1)
                ]
        dump(out / "model_provenance.json", provenance)
        if recorder:
            if proposal:
                origin_path = Path(proposal).resolve().parent / "model_provenance.json"
                if origin_path.is_file():
                    origin = read(origin_path)
                    sources[str(origin_path)] = fingerprint(origin_path)
                    provenance["original_run"] = {
                        "model": origin.get("model", "not recorded"),
                        "source": origin.get("source"),
                        "live_model_called": origin.get("live_model_called", False),
                        "evidence_file": str(origin_path),
                        "sha256": sources[str(origin_path)],
                    }
                    dump(out / "model_provenance.json", provenance)
            recorder.set_model_provenance(provenance)
            recorder.set_opening_catalog(catalog)
            recorder.capture(
                "architecture_ranking",
                "Rank opening candidates",
                [
                    "Try openings in configured or VLM-ranked order. The local solver computes placement and geometry.",
                    "Replay a saved real-model ranking; no model call is made during this run."
                    if provenance["source"] == "saved_schema_replay"
                    else "The ranking source is recorded in the evidence below.",
                ],
                recorder.shells(hollows) + recorder.motors(scene),
                {
                    "model_provenance": provenance,
                    "openings": openings,
                    "ranking_explanations": proposals["parts"]
                    if proposal or live_vlm
                    else [],
                },
            )
        from .restrict_schema import prepare_restrictions, restriction_context

        schema_settings = config.get("layout_constraints", {"mode": "rules"})
        schema_mode = schema_settings.get("mode", "rules")
        schema_images = []
        if schema_mode == "llm" or schema_proposal:
            if not images:
                images = observations(hollows, [], out)
            schema_directory = out / "layout_observation"
            schema_directory.mkdir()
            from semantic_bridge import _cutaway

            schema_images = [
                *images,
                _cutaway(space, space["usable"], schema_directory),
            ]
            bench_meshes, bench_x = [], 0.0
            for p in profiles:
                for body in (p["body"], p["carrier"]):
                    mesh = as_mesh(body)
                    mesh.apply_translation([bench_x - p["bounds"][0, 0], 0, 0])
                    bench_meshes.append(mesh)
                bench_x += float(np.ptp(p["bounds"], axis=0)[0]) + 25
            bench_image = schema_directory / "component_carriers.png"
            render_views(
                bench_meshes,
                [],
                bench_image,
                "Independent carrier bundles; bench order: " + ", ".join(ids),
            )
            schema_images.append(bench_image)
        restrict_context = restriction_context(
            profiles, space, scene, catalog, gravity, settings["clearance_mm"]
        )
        restrictions, schema_provenance = prepare_restrictions(
            restrict_context,
            schema_images,
            out,
            mode=schema_mode,
            replay=schema_proposal,
            env_file=ROOT / "robot_interior_postprocess/.env",
            sources=sources,
        )
        if recorder:
            recorder.set_model_provenance(schema_provenance, stage="layout_constraints")
            recorder.capture(
                "layout_constraints",
                "Generate and validate the carrier restrict schema",
                [
                    "Generate bundle-center domains, legal mounting normals/rolls and bounded soft preferences before candidate generation.",
                    "Compile and validate the schema. The same hard restrictions gate initial seeds, every EPS move and final selected placements.",
                    "The VLM defines constraints; local algorithms generate all candidate poses and validate assembly geometry.",
                ],
                recorder.shells(hollows) + recorder.motors(scene),
                {
                    "schema_provenance": schema_provenance,
                    "schema": restrictions.schema,
                    "guardrails": restrictions.guards,
                    "input_images": [str(p.relative_to(out)) for p in schema_images],
                },
            )
        selected, poses_all, designs, trace = {}, [], {}, []
        for name, shell in hollows.items():
            best = None
            for index, opening in enumerate(
                openings[name][: settings["max_architectures"]]
            ):
                normal = np.asarray(opening["normal"], float)
                projection = shell.vertices @ normal
                offset = float(
                    projection.min() + opening["fraction"] * np.ptp(projection)
                )
                plane = {
                    "id": f"{name}:assembly:{index}",
                    "normal": normal.tolist(),
                    "offset_mm": offset,
                }
                candidate_meta = None
                if recorder:
                    from .candidate_preview import candidate_info

                    source_id = next(
                        (
                            c["id"]
                            for c in catalog[name]
                            if np.allclose(c["normal"], normal)
                            and np.isclose(c["fraction"], opening["fraction"])
                        ),
                        f"configured:{name}:{index + 1}",
                    )
                    candidate_meta = candidate_info(
                        name,
                        index,
                        opening,
                        plane,
                        shell,
                        source_id,
                        provenance["source"],
                    )
                    reference = recorder.shells({name: shell})[0]
                    candidate_meta["reference_asset"] = reference["asset"]
                    candidate_meta["plane_visual"] = recorder.planes(
                        {name: shell}, {name: plane}
                    )[0]
                    recorder.capture(
                        "opening_attempt",
                        f"{name}: inspect candidate {index + 1}",
                        [
                            f"Try {candidate_meta['axis']} = {candidate_meta['coordinate_mm']:.2f} mm, fraction {opening['fraction']:.2f} of the projected extent.",
                            "Cyan marks the actual attempted cut. Geometry validation has not passed yet.",
                        ],
                        recorder.shells(hollows)
                        + recorder.motors(scene)
                        + recorder.planes({name: shell}, {name: plane}),
                        {"link": name, "plane": plane, "candidate": index + 1},
                        state="candidate",
                        inspection=candidate_meta,
                    )
                try:
                    if not groups[name]:
                        preview_pieces, _ = split_shell(
                            shell, plane, settings["gap_mm"]
                        )
                        if recorder:
                            recorder.capture(
                                "opening_candidate",
                                f"{name}: split a shell without electronics",
                                [
                                    "This link has no electronic carriers. Keep both shell pieces and generate their fasteners later."
                                ],
                                recorder.shells(
                                    {**hollows, name: preview_pieces}, {name: plane}
                                )
                                + recorder.motors(scene),
                                {"link": name, "plane": plane},
                                state="candidate",
                                inspection={**candidate_meta, "status": "split_valid"},
                            )
                        selected[name] = plane
                        break
                    print(
                        f"ASSEMBLY: carrier EPS {name} architecture {index + 1}",
                        flush=True,
                    )
                    preview_pieces, _ = split_shell(shell, plane, settings["gap_mm"])
                    if recorder:
                        recorder.capture(
                            "opening_candidate",
                            f"{name}: try opening candidate {index + 1}",
                            [
                                "Split the current shell at the candidate plane. This is a candidate, not the selected design."
                            ],
                            recorder.shells(
                                {**hollows, name: preview_pieces}, {name: plane}
                            )
                            + recorder.motors(scene)
                            + recorder.planes({name: shell}, {name: plane}),
                            {"link": name, "plane": plane},
                            state="candidate",
                            inspection={**candidate_meta, "status": "split_valid"},
                        )

                    def record_search(
                        kind,
                        trial_poses,
                        details,
                        *,
                        name=name,
                        index=index,
                        plane=plane,
                        preview_pieces=preview_pieces,
                        candidate_meta=candidate_meta,
                        normal=normal,
                        shell=shell,
                    ):
                        if not recorder:
                            return
                        changes = (
                            [
                                "Start from a feasible seed and optimize complete component-and-carrier bundles."
                            ]
                            if kind == "initial_seed"
                            else [
                                f"Accept {'atomic rotation' if details['kind'] == 'atomic_roll' else 'translation'} for {details['id']}; translation step size {details['step_mm']:g} mm.",
                                f"Objective decreases from {details['before']:.4f} to {details['after']:.4f}; exact support and operation checks are still required.",
                            ]
                        )
                        details = dict(details)
                        seeds = details.pop("_seed_candidates", [])
                        if kind == "initial_seed" and seeds:
                            from .layout_gallery import render_seed

                            gallery = []
                            for seed_index, seed_poses in enumerate(seeds):
                                instances = recorder.shells(
                                    {name: shell}
                                ) + recorder.bundles(profiles, seed_poses)
                                cost = objective(
                                    seed_poses,
                                    normal,
                                    settings["weights"],
                                    restrictions,
                                )
                                image_path = (
                                    out
                                    / "observations"
                                    / "layout_candidates"
                                    / f"{safe_id(name)}_{index + 1}_{seed_index + 1}.png"
                                )
                                gallery.append(
                                    {
                                        "id": f"{plane['id']}:seed:{seed_index + 1}",
                                        "title": f"Layout seed {seed_index + 1}",
                                        "instances": instances,
                                        "cost": cost,
                                        "schema_hard_checks": restrictions.report(
                                            seed_poses
                                        ),
                                        "image_file": str(image_path.relative_to(out)),
                                        "image_data_url": render_seed(
                                            instances,
                                            recorder.data["assets"],
                                            shell.bounds,
                                            plane["normal"],
                                            image_path,
                                            f"Layout seed {seed_index + 1} | local generation",
                                            f"{name} / opening {index + 1} | objective {cost:.3f} | schema hard checks passed",
                                        ),
                                        "status": "coarse_feasible_exact_assembly_pending",
                                    }
                                )
                            recorder.capture(
                                "layout_candidates",
                                "Compare local layout seeds under the restrict schema",
                                [
                                    f"Generate {len(gallery)} distinct coarse-feasible bundle layouts under the validated schema.",
                                    "These are locally computed alternatives, not VLM-generated numeric poses. Images use the same view and scale.",
                                    "The best seed enters EPS; alternate seeds remain available for later exact assembly checks.",
                                ],
                                recorder.shells(
                                    {**hollows, name: preview_pieces}, {name: plane}
                                )
                                + recorder.motors(scene)
                                + recorder.bundles(
                                    profiles, [*poses_all, *trial_poses]
                                ),
                                {
                                    "link": name,
                                    "candidate": index + 1,
                                    "seed_count": len(gallery),
                                    "schema_file": "layout_constraints.json",
                                },
                                state="candidate",
                                inspection={**candidate_meta, "status": "layout_seeds"},
                                gallery=gallery,
                            )
                        recorder.capture(
                            "eps_seed" if kind == "initial_seed" else "eps_move",
                            f"{name} / candidate {index + 1}: {'initial layout' if kind == 'initial_seed' else 'EPS accepted improvement'}",
                            changes,
                            recorder.shells(
                                {**hollows, name: preview_pieces}, {name: plane}
                            )
                            + recorder.motors(scene)
                            + recorder.bundles(profiles, [*poses_all, *trial_poses]),
                            {"link": name, "candidate": index + 1, **details},
                            state="candidate",
                            inspection={**candidate_meta, "status": "layout_search"},
                        )

                    poses, report = plan_link(
                        groups[name],
                        space,
                        scene["parts"].index(name) + 1,
                        normal,
                        offset,
                        settings,
                        on_progress=record_search if recorder else None,
                        restrictions=restrictions,
                    )
                    if poses is None:
                        trace.append({"link": name, "plane": plane, **report})
                        if recorder:
                            recorder.capture(
                                "candidate_rejected",
                                f"{name}: candidate {index + 1} has no feasible layout",
                                [
                                    "Record the actual failure reason and try the next candidate."
                                ],
                                metrics={"link": name, **report},
                                state="rejected",
                                inspection={
                                    **candidate_meta,
                                    "status": "rejected",
                                    "reason": report["reason"],
                                },
                            )
                        continue
                    alternates = report.pop("_alternate_poses", [])
                    neighbors = [solid(m) for key, m in hollows.items() if key != name]
                    # Fixed proxy envelopes obstruct installation; initial shell/proxy overlap remains a separate missing-interface issue.
                    for motor in scene.get("motors", []):
                        neighbors.append(
                            solid(
                                trimesh.creation.cylinder(
                                    radius=motor["radius_mm"],
                                    segment=np.array(
                                        [motor["axial_start_mm"], motor["axial_end_mm"]]
                                    ),
                                    sections=24,
                                )
                            )
                        )
                    design = None
                    for seed, trial_poses in enumerate([poses, *alternates]):
                        try:
                            trial = build_link(
                                shell,
                                plane,
                                groups[name],
                                trial_poses,
                                settings,
                                neighbors,
                            )
                            if not trial["all_paths_clear"]:
                                trace.append(
                                    {
                                        "link": name,
                                        "plane": plane,
                                        "seed": seed,
                                        "reason": "exact_assembly_path_failed",
                                        "checks": trial["checks"],
                                    }
                                )
                                if recorder:
                                    recorder.capture(
                                        "candidate_rejected",
                                        f"{name}: exact assembly check failed",
                                        [
                                            "The coarse layout fits, but exact supports or operation paths are blocked. Try another seed."
                                        ],
                                        metrics={
                                            "link": name,
                                            "seed": seed,
                                            "checks": trial["checks"],
                                        },
                                        state="rejected",
                                        inspection={
                                            **candidate_meta,
                                            "status": "exact_path_rejected",
                                            "reason": "exact_assembly_path_failed",
                                        },
                                    )
                                continue
                            design, poses = trial, trial_poses
                            if recorder:
                                recorder.capture(
                                    "candidate_supports",
                                    f"{name} / candidate {index + 1}: generate supports and check paths",
                                    [
                                        "Generate support docks for this layout candidate and fuse them into the owner shell.",
                                        "Carrier insertion, screw and tool paths pass for this candidate; other architectures are still compared.",
                                    ],
                                    recorder.shells(
                                        {**hollows, name: trial["pieces"]},
                                        {name: plane},
                                    )
                                    + recorder.motors(scene)
                                    + recorder.bundles(
                                        profiles, [*poses_all, *trial_poses]
                                    )
                                    + recorder.supports({name: trial}),
                                    {
                                        "link": name,
                                        "seed": seed,
                                        "checks": trial["checks"],
                                    },
                                    state="geometry_candidate_passed",
                                    inspection={
                                        **candidate_meta,
                                        "status": "geometry_passed",
                                    },
                                )
                            break
                        except ValueError as exc:
                            trace.append(
                                {
                                    "link": name,
                                    "plane": plane,
                                    "seed": seed,
                                    "reason": str(exc),
                                }
                            )
                    if design is None:
                        continue
                    metric = projection_metrics(poses, normal)
                    trace.append(
                        {
                            "link": name,
                            "plane": plane,
                            "status": "geometry_candidate_passed",
                            "eps": report,
                            "metrics": metric,
                        }
                    )
                    cost = objective(poses, normal, settings["weights"], restrictions)
                    if best is None or cost < best[0]:
                        best = cost, plane, poses, design
                except ValueError as exc:
                    trace.append({"link": name, "plane": plane, "reason": str(exc)})
                    if recorder:
                        candidate_meta = {
                            **candidate_meta,
                            "status": "rejected",
                            "reason": str(exc),
                        }
                        try:
                            diagnostic, details = recorder.diagnostic_candidate(
                                hollows, name, plane, settings["gap_mm"]
                            )
                            candidate_meta["diagnostic"] = details
                            diagnostic += recorder.motors(scene) + recorder.planes(
                                {name: shell}, {name: plane}
                            )
                        except (ValueError, RuntimeError) as diagnostic_error:
                            diagnostic = (
                                recorder.shells(hollows)
                                + recorder.motors(scene)
                                + recorder.planes({name: shell}, {name: plane})
                            )
                            candidate_meta["diagnostic_error"] = str(diagnostic_error)
                        recorder.capture(
                            "candidate_rejected",
                            f"{name}: candidate {index + 1} rejected",
                            [
                                str(exc),
                                "Showing a diagnostic clip of this rejected candidate. Red marks detached material regions; blue/orange distinguish the two intended sides. This is not an approved manufacturing split.",
                            ],
                            instances=diagnostic,
                            metrics={"link": name, "plane": plane},
                            state="rejected",
                            inspection=candidate_meta,
                        )
            if groups[name] and best:
                _, selected[name], poses, designs[name] = best
                poses_all.extend(poses)
            elif groups[name] or name not in selected:
                dump(out / "candidate_evaluations.json", trace)
                raise ValueError(
                    f"No assembly-feasible carrier architecture for {name}; inspect candidate_evaluations.json"
                )
        dump(out / "candidate_evaluations.json", trace)
        schema_validation = restrictions.report(poses_all, require_all=True)
        dump(out / "semantic_final_validation.json", schema_validation)
        if not schema_validation["all_hard_constraints_pass"]:
            raise ValueError(
                "Selected layout violates the restrict schema; no constraints were relaxed"
            )
        if recorder:
            for info in recorder.data["candidates"].values():
                info["selected"] = selected[info["link"]]["id"] == info["key"]
            recorder.capture(
                "selected_layout",
                "Select openings, carrier layouts and supports",
                [
                    "Complete candidate comparison and retain layouts that passed local geometric checks.",
                    "Earlier EPS and support scenes were candidates. This snapshot shows the selected combination.",
                ],
                recorder.shells(
                    {
                        name: designs[name]["pieces"]
                        if name in designs
                        else split_shell(shell, selected[name], settings["gap_mm"])[0]
                        for name, shell in hollows.items()
                    },
                    selected,
                )
                + recorder.motors(scene)
                + recorder.bundles(profiles, poses_all)
                + recorder.supports(designs),
                {
                    "openings": selected,
                    "placements": [
                        {"id": p["id"], "world_from_interface": p["transform"]}
                        for p in poses_all
                    ],
                },
                state="selected",
            )
        dump(
            out / "carrier_placement.json",
            {
                "units": "mm",
                "placements": [
                    {
                        "id": p["id"],
                        "world_from_interface": p["transform"],
                        "bounds_mm": p["bounds"],
                    }
                    for p in poses_all
                ],
                "openings": selected,
                "gravity": gravity,
            },
        )
        # Routing gets a new mutable snapshot; original inputs remain read-only.
        print("ASSEMBLY: route around carriers and fixed docking brackets", flush=True)
        route_trials, best_routing = [], None
        for attempt in range(settings["max_route_attempts"]):
            route_out = out / "electrical" / f"attempt_{attempt + 1}"
            (route_out / "meshes").mkdir(parents=True)
            try:
                trial_routing, trial_hollows = route_candidate(
                    config,
                    raw,
                    scene,
                    space,
                    hollows,
                    profiles,
                    poses_all,
                    designs,
                    route_out,
                    sources,
                    config_path.parent,
                    tail_variant=attempt,
                )
                route_trials.append(
                    {
                        "attempt": attempt + 1,
                        "status": trial_routing["status"],
                        "branches": trial_routing.get("branches", 0),
                        "unrouted": trial_routing.get("unrouted", []),
                    }
                )
                rank = (
                    trial_routing["status"] == "passed",
                    trial_routing.get("branches", 0),
                )
                if best_routing is None or rank > best_routing[0]:
                    trial_routing["artifacts_directory"] = str(
                        route_out.relative_to(out)
                    )
                    best_routing = rank, trial_routing, trial_hollows
                if trial_routing["status"] in {"passed", "pending"}:
                    break
            except ValueError as exc:
                route_trials.append(
                    {"attempt": attempt + 1, "status": "failed", "reason": str(exc)}
                )
        dump(out / "routing_attempts.json", route_trials)
        if best_routing:
            _, routing, final_hollows = best_routing
        else:
            routing, final_hollows = (
                {"status": "failed", "reason": "all_bounded_routing_attempts_failed"},
                hollows,
            )
        wire = routing.pop("wire_solid", None)
        if wire is not None:
            clashes = []
            for name, d in designs.items():
                for key, dock in d["docks"].items():
                    if intersection_volume(dock, wire) > 0.02:
                        clashes.append(key)
            routing["dock_route_clashes"] = clashes
            if clashes:
                routing["status"] = "failed"
        dump(out / "routing_summary.json", routing)
        if recorder:
            recorder.capture(
                "routing",
                "Generate access ducts and route wires",
                [
                    "Route around carriers and fixed docks, and update the shell material after access-duct generation.",
                    "Wire solids must still be checked against rebuilt final materials.",
                ],
                recorder.shells(final_hollows)
                + recorder.motors(scene)
                + recorder.bundles(profiles, poses_all)
                + recorder.supports(designs)
                + (
                    [recorder.item("wire:harness", wire, "wire", "wiring")]
                    if wire is not None
                    else []
                ),
                routing,
                state=routing["status"],
            )
        # Access ducts can change shell material. Rebuild final docks and recheck
        # geometry rather than carrying an old manufacturing certificate forward.
        for name in designs:
            poses = [
                p
                for p in poses_all
                if p["id"] in {q["spec"]["id"] for q in groups[name]}
            ]
            external = [solid(m) for key, m in final_hollows.items() if key != name]
            for other_link, other_design in designs.items():
                if other_link != name:
                    external.extend(
                        [other_design["all_hardware"], *other_design["docks"].values()]
                    )
            for motor in scene.get("motors", []):
                external.append(
                    solid(
                        trimesh.creation.cylinder(
                            radius=motor["radius_mm"],
                            segment=np.array(
                                [motor["axial_start_mm"], motor["axial_end_mm"]]
                            ),
                            sections=24,
                        )
                    )
                )
            designs[name] = build_link(
                final_hollows[name],
                selected[name],
                groups[name],
                poses,
                settings,
                external,
            )
        from .closure import export_connection_map, scene_closures

        unfixed_pieces = {
            name: designs[name]["pieces"]
            if name in designs
            else split_shell(shell, selected[name], settings["gap_mm"])[0]
            for name, shell in final_hollows.items()
        }
        process_extras = []
        if recorder:
            process_extras = (
                recorder.motors(scene)
                + recorder.bundles(profiles, poses_all)
                + recorder.supports(designs)
            )
            if wire is not None:
                process_extras.append(
                    recorder.item("wire:harness", wire, "wire", "wiring")
                )
            recorder.capture(
                "post_duct_split",
                "Rebuild supports after ducts and prepare all shell pieces",
                [
                    "Rebuild and recheck supports after duct cuts. Retain both shell pieces on every link.",
                    "Shell closure holes are not present yet; matching connections are generated next.",
                ],
                recorder.shells(unfixed_pieces, selected) + process_extras,
                {
                    "shell_pieces": sum(map(len, unfixed_pieces.values())),
                    "all_carrier_paths_clear": all(
                        d["all_paths_clear"] for d in designs.values()
                    ),
                },
            )
        closures = scene_closures(
            out,
            scene,
            final_hollows,
            unfixed_pieces,
            selected,
            designs,
            profiles,
            poses_all,
            wire,
            settings["gap_mm"],
            config.get("shell_fastening"),
        )
        if recorder:
            process_fasteners = [
                recorder.item(f"{name}:shell_screw:{i + 1}", mesh, "fastener", name)
                for name, closure in closures.items()
                for i, mesh in enumerate(closure["screws"])
            ]
            recorder.capture(
                "shell_fastening",
                "Generate shell fasteners, loading tunnels and locating stops",
                [
                    "Add bosses and locating stops to the actual shells; subtract clearance holes, head seats, loading tunnels and blind insert pockets.",
                    "Jointly search matching hole patterns and screw lengths for similar structures; check complete screw and driver sweeps.",
                ],
                recorder.shells(
                    {
                        name: closures[name]["pieces"] if name in closures else pair
                        for name, pair in unfixed_pieces.items()
                    },
                    selected,
                )
                + process_extras
                + process_fasteners,
                {
                    "connections": {
                        name: {
                            "count": d["report"]["connection_count"],
                            "shared_pattern": d["report"].get("shared_pattern"),
                        }
                        for name, d in closures.items()
                    }
                },
            )
        final_pieces, reports, shown, targets, records = {}, {}, [], [], []
        for name, shell in final_hollows.items():
            if name in designs:
                design = designs[name]
                pieces = (
                    closures[name]["pieces"] if name in closures else design["pieces"]
                )
            else:
                pieces = (
                    closures[name]["pieces"]
                    if name in closures
                    else unfixed_pieces[name]
                )
                design = None
            files = []
            for i, mesh in enumerate(pieces):
                file = f"meshes/{name}_piece_{i + 1}.stl"
                pieces[i] = export_solid(mesh, out / file)
                files.append(file)
            final_pieces[name] = pieces
            reports[name] = {
                "status": "partial",
                "piece_files": files,
                "selected_opening": {
                    "panel_index": 1,
                    "direction": selected[name]["normal"],
                    "clear_tasks": len(groups[name]),
                    "total_tasks": len(groups[name]),
                    "targets": [],
                },
                "strategy": {
                    "rationale": "Independent carrier bundles, parallel opening, single-layer projection access; whole robot evidence incomplete."
                },
            }
            if name in closures:
                closure = closures[name]
                closure["report"]["piece_files"] = files
                for i, record in enumerate(closure["report"]["connections"]):
                    record.update(
                        id=f"{name}:shell_screw_{i + 1}",
                        owner_piece_file=files[0],
                        cover_piece_file=files[1],
                    )
                reports[name]["shell_connection"] = closure["report"]
                for side, added in enumerate(closure["added"]):
                    shown.append(
                        {
                            "id": f"{name}:shell_bosses_{side + 1}",
                            "owners": [name],
                            "owner_piece_index": side,
                            "kind": "support",
                            "display_role": "shell_connection",
                            "mesh": as_mesh(added),
                        }
                    )
                for i, (screw_value, tool_value) in enumerate(
                    zip(closure["screws"], closure["tools"])
                ):
                    for role, value in (
                        ("fastener_solid", screw_value),
                        ("tools", tool_value),
                    ):
                        shown.append(
                            {
                                "id": f"{name}:shell_screw_{i + 1}:{role}",
                                "owners": [name],
                                "owner_piece_index": 1,
                                "kind": "support",
                                "display_role": role,
                                "mesh": as_mesh(value),
                            }
                        )
            if not design:
                continue
            for record in design["records"]:
                key = record["target_id"]
                record["carrier_file"] = f"meshes/{key}_carrier.stl"
                record["owner_piece_file"] = files[0]
                design["carriers"][key] = solid(
                    export_solid(
                        as_mesh(design["carriers"][key]), out / record["carrier_file"]
                    )
                )
                records.append(record)
                for role, value in [
                    ("dock", design["docks"][key]),
                    ("detachable", design["carriers"][key]),
                    ("fastener_solid", design["screws"][key]),
                    ("tools", design["tools"][key]),
                ]:
                    shown.append(
                        {
                            "id": f"{key}:{role}",
                            "owners": [name],
                            "owner_piece_index": 0,
                            "kind": "support",
                            "display_role": role,
                            "mesh": as_mesh(value),
                        }
                    )
                p = next(p for p in profiles if p["spec"]["id"] == key)
                pose = next(p for p in poses_all if p["id"] == key)
                targets.append(
                    {
                        "id": key,
                        "owners": [name],
                        "kind": "electronics",
                        "mesh": p["display"].copy().apply_transform(pose["transform"]),
                    }
                )
        for motor in scene.get("motors", []):
            targets.append(
                {
                    "id": motor["id"],
                    "owners": list(dict.fromkeys([motor["parent"], motor["link"]])),
                    "kind": "motor",
                    "mesh": trimesh.creation.cylinder(
                        radius=motor["radius_mm"],
                        segment=np.array(
                            [motor["axial_start_mm"], motor["axial_end_mm"]]
                        ),
                        sections=24,
                    ),
                }
            )
        from .loads import load_report

        sequence, validation = sequence_and_validation(
            profiles, designs, scene, gravity, closures
        )
        closure_report = {name: data["report"] for name, data in closures.items()}
        piece_coverage = {
            file: [] for report in reports.values() for file in report["piece_files"]
        }
        for data in closure_report.values():
            for joint in data["connections"]:
                piece_coverage[joint["owner_piece_file"]].append(joint["id"])
                piece_coverage[joint["cover_piece_file"]].append(joint["id"])
        validation["shell_piece_connections"] = piece_coverage
        validation["all_shell_pieces_fastened"] = bool(piece_coverage) and all(
            piece_coverage.values()
        )
        dump(
            out / "shell_connections.json",
            {
                "status": "geometry_checked"
                if validation["all_shell_pieces_fastened"]
                else "partial",
                "links": closure_report,
                "piece_coverage": piece_coverage,
            },
        )
        export_connection_map(out, closures)
        from .fastener_access import export_access_guide

        export_access_guide(out, closures)
        dump(
            out / "shell_disassembly.json",
            {
                "scope": "Local shell removal; connected-robot fixtures, motion and motor interfaces remain separately pending",
                "links": [
                    {
                        "owner_link": name,
                        "steps": [
                            {
                                "action": "remove_all_external_shell_screws",
                                "connection_ids": [
                                    r["id"] for r in data["report"]["connections"]
                                ],
                            },
                            {
                                "action": "withdraw_cover_along_axis_to_release_locators",
                                "direction_world": data["report"][
                                    "cover_translation_direction_world"
                                ],
                                "minimum_locator_release_mm": data["report"][
                                    "minimum_locator_release_mm"
                                ],
                            },
                            {"action": "remove_cover_before_carrier_service"},
                        ],
                    }
                    for name, data in closures.items()
                ],
            },
        )
        loads = load_report(profiles, poses_all, designs, gravity)
        dump(out / "load_report.json", loads)
        validation["structural_validation"] = loads
        validation["routing"] = routing
        if electrical_review is not None:
            validation["electrical_review"] = electrical_review
            if electrical_review["status"] != "limited_checks_passed":
                validation["pending"].append(
                    {"code": "electrical_review_incomplete", "evidence": "electrical_review.json"}
                )
        if routing["status"] != "passed":
            validation["pending"].append(
                {"code": "routing_incomplete", "evidence": "routing_attempts.json"}
            )
        else:
            validation["pending"].append(
                {"code": "measured_connectors_and_electrical_compatibility"}
            )
        # Recheck routed cable cylinders against the actual post-duct final docks.
        if wire is not None:
            clashes = [
                r["target_id"]
                for d in designs.values()
                for r in d["records"]
                if intersection_volume(wire, d["docks"][r["target_id"]]) > 0.02
            ]
            validation["final_dock_wire_clear"] = not clashes
            validation["final_dock_wire_clashes"] = clashes
            material_hits = []
            for name, pieces in final_pieces.items():
                volume = intersection_volume(wire, union([solid(m) for m in pieces]))
                if volume > 0.02:
                    material_hits.append(
                        {
                            "object": name,
                            "kind": "final_shell_and_docks",
                            "overlap_mm3": volume,
                        }
                    )
            for design in designs.values():
                for key in design["bodies"]:
                    volume = intersection_volume(
                        wire,
                        union(
                            [
                                design["bodies"][key],
                                design["carriers"][key],
                                design["screws"][key],
                            ]
                        ),
                    )
                    if volume > 0.02:
                        material_hits.append(
                            {
                                "object": key,
                                "kind": "hardware_carrier_fasteners",
                                "overlap_mm3": volume,
                            }
                        )
            for name, closure in closures.items():
                volume = intersection_volume(wire, union(closure["screws"]))
                if volume > 0.02:
                    material_hits.append(
                        {"object": name, "kind": "shell_screws", "overlap_mm3": volume}
                    )
            validation["final_wire_material_hits"] = material_hits
            validation["final_wire_material_clear"] = not material_hits
            if material_hits:
                routing["status"] = "failed"
                validation["pending"].append(
                    {
                        "code": "final_route_material_intersection",
                        "evidence": material_hits,
                    }
                )
        dump(out / "assembly_sequence.json", sequence)
        dump(out / "assembly_validation.json", validation)
        dump(out / "routing_summary.json", routing)
        dump(
            out / "support_ownership.json",
            {"schema_version": 2, "units": "mm", "supports": records},
        )
        dump(
            out / "carrier_placement.json",
            {
                "units": "mm",
                "placements": [
                    {
                        "id": p["id"],
                        "world_from_interface": p["transform"],
                        "bounds_mm": p["bounds"],
                    }
                    for p in poses_all
                ],
                "openings": selected,
                "gravity": gravity,
            },
        )
        dump(out / "partition_report.json", {"status": "partial", "parts": reports})
        dump(
            out / "manufacturing_manifest.json",
            {
                "units": "mm",
                "frame": "world assembly coordinates",
                "release": "blocked_pending_validation",
                "printed_parts": [
                    *[
                        {
                            "file": file,
                            "kind": "shell_with_fused_docks",
                            "owner_link": name,
                        }
                        for name, report in reports.items()
                        for file in report["piece_files"]
                    ],
                    *[
                        {
                            "file": record["carrier_file"],
                            "kind": "independent_removable_carrier",
                            "target_id": record["target_id"],
                        }
                        for record in records
                    ],
                ],
                "fasteners": "fasteners_bom.csv",
                "note": "Fixed docks are already fused into shell files. Carriers are separate parts. Source hardware and fasteners are not printable structure.",
            },
        )
        dump(
            out / "hardware_dossiers.json",
            [
                {"id": p["spec"]["id"], "profile": p["spec"], "pending": p["pending"]}
                for p in profiles
            ],
        )
        with (out / "fasteners_bom.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["assembly", "item", "quantity", "status"])
            for p in profiles:
                writer.writerow(
                    [
                        p["spec"]["id"],
                        "M3 carrier screw + blind insert (prototype dimensions)",
                        2,
                        "select_part_and_calibrate",
                    ]
                )
                if p["interface"]:
                    writer.writerow(
                        [
                            p["spec"]["id"],
                            p["interface"]["fastener"]["thread"]
                            + " PCB screw + insert",
                            len(p["interface"]["holes"]),
                            "prototype_uncalibrated",
                        ]
                    )
            for name, closure in closures.items():
                for joint in closure["report"]["connections"]:
                    writer.writerow(
                        [
                            name,
                            joint["fastener_spec"]["designation"],
                            1,
                            "prototype_uncalibrated",
                        ]
                    )
                writer.writerow(
                    [
                        name,
                        "shell blind insert; select supplier and calibrate pocket",
                        closure["report"]["connection_count"],
                        "prototype_uncalibrated",
                    ]
                )
        export_viewer(out, final_pieces, targets, reports, shown)
        render_views(
            [],
            [
                *[t for t in targets if t["kind"] != "motor"],
                *[s for s in shown if s["display_role"] != "tools"],
            ],
            out / "carrier_detail.png",
            "Independent carriers and fixed docking brackets; physical validation pending",
        )
        html = (out / "index.html").read_text()
        html = html.replace(
            '<a href="proposal_raw.json">VLM proposal</a>',
            '<a href="assembly_sequence.json">Assembly sequence</a><a href="assembly_validation.json">Validation</a>',
        )
        if recorder:
            html = html.replace(
                "</header>", '<a href="process.html">Generation process</a></header>'
            )
        html = html.replace(
            "</a></header>",
            '</a><a href="shell_connections.json">Shell connections</a><a href="shell_fastener_assembly.html">Fastener specifications and loading guide</a></header>',
        )
        (out / "index.html").write_text(html)
        changed = [
            path for path, digest in sources.items() if fingerprint(path) != digest
        ]
        if changed:
            raise ValueError("Input changed during pipeline: " + ", ".join(changed))
        dump(out / "input_hashes.json", sources)
        manifest.update(
            status="partial",
            stage="completed_with_explicit_blockers",
            carrier_count=len(profiles),
            exported_shell_pieces=sum(map(len, final_pieces.values())),
            all_carrier_paths_clear=validation["all_carrier_paths_clear"],
            all_shell_pieces_fastened=validation["all_shell_pieces_fastened"],
            shell_connection_count=sum(
                data["report"]["connection_count"] for data in closures.values()
            ),
            routing=routing,
            source_inputs_unchanged=True,
            geometry_cache_used=bool(cache),
            model_provenance=provenance,
            schema_provenance=schema_provenance,
            layout_restrictions_applied=True,
            pending_count=len(validation["pending"]),
            artifacts={
                "viewer": "index.html",
                "validation": "assembly_validation.json",
                "sequence": "assembly_sequence.json",
                "placement": "carrier_placement.json",
                "ownership": "support_ownership.json",
            },
        )
        # This first release deliberately cannot certify an untested physical robot.
        dump(out / "manifest.json", manifest)
        if recorder:
            recorder.capture(
                "final_validation",
                "Export printable parts and finish geometric checks",
                [
                    "Export and reload final STLs; record carrier paths, wire/material intersections, shell connection coverage and the manufacturing list.",
                    "Geometry generation is complete. Physical fit, actual mounting interfaces and load validation remain pending.",
                ],
                recorder.shells(final_pieces, selected)
                + process_extras
                + process_fasteners,
                {
                    "printed_parts": len(
                        read(out / "manufacturing_manifest.json")["printed_parts"]
                    ),
                    "all_carrier_paths_clear": validation["all_carrier_paths_clear"],
                    "schema_hard_constraints_pass": schema_validation[
                        "all_hard_constraints_pass"
                    ],
                    "all_shell_pieces_fastened": validation[
                        "all_shell_pieces_fastened"
                    ],
                    "final_wire_material_clear": validation.get(
                        "final_wire_material_clear"
                    ),
                    "routing": routing,
                    "pending_count": len(validation["pending"]),
                    "complete_assemblable": False,
                },
                state="partial",
            )
            recorder.finish(
                "partial", {"model_provenance": provenance, "manifest": "manifest.json"}
            )
            manifest["artifacts"].update(
                process="process.html",
                process_trace="process_trace.json",
                changes="process_changes.md",
            )
            dump(out / "manifest.json", manifest)
        print("ASSEMBLY RESULT: partial", out, flush=True)
        return 2
    except Exception as exc:
        manifest.update(status="failed", error=str(exc))
        dump(out / "manifest.json", manifest)
        if recorder:
            recorder.capture(
                "failed",
                "Generation stopped: retain recorded steps",
                [str(exc)],
                state="failed",
            )
            recorder.finish("failed", {"error": str(exc)})
        raise
