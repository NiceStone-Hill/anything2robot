#!/usr/bin/env python3
"""Run placement, routing and visualization in a new directory; check their outputs."""

import argparse
import os
import shutil
import subprocess
import sys
import time
import webbrowser
from collections import Counter
from datetime import datetime
from pathlib import Path

from common import ROOT, dump, geometry_signature, read

HERE = Path(__file__).resolve().parent


def seed_geometry(config, cache, output):
    """Only reuse matching hollow geometry, never previous placements or routes."""
    signature = cache / "geometry_signature.json"
    if (
        not signature.is_file()
        or not (cache / "mechanical_scene.json").is_file()
        or read(signature) != geometry_signature(config)
    ):
        return False
    scene = read(cache / "mechanical_scene.json")
    files = ["geometry_signature.json", "space_masks.npz", "space_report.json"]
    for part in scene["parts"]:
        files += [
            f"meshes/{part}_hollow_assembly_mm.stl",
            f"meshes/{part}_cavity_tool_mm.stl",
        ]
    if not all((cache / name).is_file() for name in files):
        return False
    for name in files:
        destination = output / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(cache / name, destination)
    return True


def check_result(output, config):
    """Check actual solver output, including exact fixed-motor poses and net coverage."""
    import numpy as np

    manifest = read(output / "manifest.json")
    placement = read(output / "placement.json")
    routing = read(output / "routing.json")
    scene = read(output / "mechanical_scene.json")
    connectivity = read(output / "connectivity.json")
    validation = read(output / "geometry_validation.json")
    route_validation = read(output / "routing_validation.json")
    checks = {}
    checks["solver_complete"] = manifest["status"] == "concept_complete"
    checks["all_components_placed"] = not placement["unplaced"] and Counter(
        p["component_id"] for p in placement["placements"]
    ) == Counter(c["id"] for c in connectivity["components"])
    expected = Counter(
        {
            n["name"]: len(n["terminals"]) - 1
            for n in connectivity["nets"]
            if len(n["terminals"]) > 1
        }
    )
    actual = Counter(r["net_name"] for r in routing["routes"])
    checks["all_net_branches_routed"] = not routing["unrouted"] and actual == expected
    by_ref = {p["component_id"]: p for p in placement["placements"]}
    mechanical = {m["id"]: m for m in scene["motors"]}
    checks["fixed_motors_unchanged"] = all(
        ref in by_ref
        and np.array_equal(by_ref[ref]["center_mm"], mechanical[motor]["center_mm"])
        for ref, motor in config["motor_mapping"].items()
    )
    with np.load(output / "space_masks.npz") as masks:
        usable = masks["usable"]
        movable = [
            p
            for p in placement["placements"]
            if p["component_id"] not in config["motor_mapping"]
        ]
        checks["movable_components_inside_cavity"] = all(
            usable[
                tuple(
                    slice(a, a + n) for a, n in zip(p["voxel_start"], p["voxel_shape"])
                )
            ].all()
            for p in movable
        )
    checks["movable_components_avoid_other_components_and_motors"] = all(
        not np.all(
            np.asarray(p["voxel_start"])
            < np.asarray(q["voxel_start"]) + q["voxel_shape"]
        )
        or not np.all(
            np.asarray(q["voxel_start"])
            < np.asarray(p["voxel_start"]) + p["voxel_shape"]
        )
        for p in movable
        for q in placement["placements"]
        if p["instance_id"] != q["instance_id"]
    )
    checks["components_clear_of_shell"] = validation["all_components_clear"]
    checks["route_samples_clear_of_shell"] = validation["all_routes_clear_of_shell"]
    checks["all_route_voxels_clear"] = all(
        b["all_voxel_segments_clear"] for b in route_validation["branches"]
    ) and len(route_validation["branches"]) == len(routing["routes"])
    checks["no_endpoint_fallback"] = not route_validation["endpoint_fallback_used"]
    if {"BATTERY", "REGULATOR"} <= {c["id"] for c in config["components"]}:
        checks["installed_power_source_and_wiring_present"] = read(
            output / "power_report.json"
        )["power_source_included"]
    if config.get("layout_constraints", {}).get("mode", "rules") != "rules":
        initial_semantics = read(output / "semantic_initial_validation.json")
        final_semantics = read(output / "semantic_final_validation.json")
        provenance = read(output / "schema_provenance.json")
        checks["initial_and_eps_final_satisfy_schema"] = (
            initial_semantics["feasible"] and final_semantics["feasible"]
        )
        checks["schema_source_matches_requested_mode"] = provenance["source"] == (
            "live_llm"
            if config["layout_constraints"]["mode"] == "llm"
            else "saved_schema_replay"
        )
    if (output / "eps_report.json").is_file():
        eps = read(output / "eps_report.json")
        before = {
            b["branch_id"]: b
            for b in read(output / "routing_before_polish.json")["routes"]
        }
        checks["eps_objective_not_increased"] = (
            eps["final_score"]["total"] <= eps["initial_score"]["total"] + 1e-6
        )
        checks["polish_preserves_lengths_and_endpoints"] = all(
            np.isclose(b["length_mm"], before[b["branch_id"]]["length_mm"])
            and b["voxel_path"][0] == before[b["branch_id"]]["voxel_path"][0]
            and b["voxel_path"][-1] == before[b["branch_id"]]["voxel_path"][-1]
            and b["bends"] <= before[b["branch_id"]]["bends"]
            for b in routing["routes"]
        )
        checks["routes_have_no_repeated_voxels"] = all(
            len(set(map(tuple, b["voxel_path"]))) == len(b["voxel_path"])
            for b in routing["routes"]
        )
    artifacts = [
        "index.html",
        "layout_canvas.html",
        "assembly.glb",
        "mechanical_scene.json",
    ]
    checks["visualization_exported"] = (
        all(
            (output / name).is_file() and (output / name).stat().st_size > 0
            for name in artifacts
        )
        and (output / "assembly.glb").read_bytes()[:4] == b"glTF"
    )
    checks["viewer_has_scene_data"] = (
        "const scene =" in (output / "layout_canvas.html").read_text()
        and 'src="layout_canvas.html"' in (output / "index.html").read_text()
    )
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "counts": {
            "shell_parts": len(scene["parts"]),
            "fixed_motors": len(mechanical),
            "placed_components": len(placement["placements"]),
            "routed_branches": len(routing["routes"]),
        },
        "viewer": str(output / "index.html"),
        "scope": "Static concept envelopes, fixed mechanical poses and routing; not electrical or motion certification",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=HERE / "configs/cactus.json")
    parser.add_argument("--output", type=Path, help="New or empty output directory")
    parser.add_argument(
        "--llm-env", type=Path, help="Local-only vision-agent .env path"
    )
    parser.add_argument(
        "--schema-proposal",
        type=Path,
        help="Replay a saved schema; does not call the LLM",
    )
    parser.add_argument(
        "--geometry-cache", type=Path, help="Matching geometry from an earlier run"
    )
    parser.add_argument(
        "--rebuild-shell", action="store_true", help="Also rebuild hollow shells"
    )
    parser.add_argument(
        "--open", action="store_true", help="Open the generated viewer in your browser"
    )
    parser.add_argument(
        "--real-models",
        action="store_true",
        help="Display downloaded CAD at the solved poses; keep proxy-based planning unchanged",
    )
    args = parser.parse_args()
    if args.real_models:
        from cad_models import ASSETS

        if not (ASSETS / "converted/models.json").is_file():
            parser.error(
                "Convert CAD first: python robot_interior_postprocess/cad_models.py --convert"
            )
    config = read(args.config.resolve())
    if args.llm_env:
        config.setdefault("layout_constraints", {})["env_file"] = str(
            args.llm_env.resolve()
        )
    if args.schema_proposal:
        config["layout_constraints"] = {
            "mode": "replay",
            "proposal": str(args.schema_proposal.resolve()),
        }
    if config.get("layout_constraints", {}).get("mode") == "llm":
        sys.path.insert(0, config["voxel_tool_source"])
        from semantic_bridge import DEFAULT_ENV
        from voxel_tool.application.llm_agent import load_agent_config

        try:
            load_agent_config(config["layout_constraints"].get("env_file", DEFAULT_ENV))
        except ValueError:
            parser.error(
                "Vision agent configuration is incomplete; fill robot_interior_postprocess/.env (API key is never printed), or explicitly use --schema-proposal for replay"
            )
    output = (
        args.output
        or HERE / "outputs" / datetime.now().strftime("layout_test_%Y%m%d_%H%M%S_%f")
    ).resolve()
    if output.exists() and any(output.iterdir()):
        parser.error(
            "Output must be new or empty; previous results will not be overwritten"
        )
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    cache = (args.geometry_cache or ROOT / config["output"]).resolve()
    reused = not args.rebuild_shell and seed_geometry(config, cache, output)
    dump(output / "test_config.json", config)
    print("OUTPUT:", output, flush=True)
    print("LOG:", output / "run.log", flush=True)
    print(
        "Geometry:",
        "matching hollow shells reused" if reused else "rebuild",
        flush=True,
    )
    print(
        "Recompute: candidate poses -> placement -> access ducts -> routing -> EPS -> bend polish -> visualization",
        flush=True,
    )
    command = [
        sys.executable,
        "-u",
        str(HERE / "run.py"),
        "--config",
        str(output / "test_config.json"),
        "--output",
        str(output),
    ]
    if reused:
        command.append("--reuse-geometry")
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    with (output / "run.log").open("w") as log:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
            log.flush()
        code = process.wait()
    if code:
        result = {
            "passed": False,
            "solver_exit_code": code,
            "log": str(output / "run.log"),
        }
    else:
        try:
            result = check_result(output, config)
        except (ValueError, KeyError, OSError) as exc:
            result = {"passed": False, "validation_error": str(exc)}
    result.update(
        geometry_reused=reused,
        placement_and_routing_recomputed=code == 0,
        elapsed_seconds=round(time.monotonic() - started, 2),
    )
    viewer = output / ("cad_preview.html" if args.real_models else "index.html")
    result["viewer"] = str(viewer)
    result["visualization_mode"] = (
        "downloaded_cad_reference" if args.real_models else "planning_proxies"
    )
    if args.real_models and code == 0:
        required = [
            "cad_preview.html",
            "cad_layout_canvas.html",
            "cad_models.html",
            "cad_registration.json",
        ]
        cad_ok = all((output / name).is_file() for name in required)
        if cad_ok:
            registration = read(output / "cad_registration.json")["registration"]
            cad_ok = bool(registration) and all(
                (output / "cad_meshes" / (entry["component"] + ".ply")).is_file()
                for entry in registration
            )
        result.setdefault("checks", {})["real_cad_visualization_exported"] = cad_ok
        result["passed"] = bool(result["passed"] and cad_ok)
    dump(output / "test_result.json", result)
    print("\nTEST:", "PASS" if result["passed"] else "FAIL", flush=True)
    print("CHECKS:", output / "test_result.json", flush=True)
    if viewer.exists():
        print("VIEWER:", viewer, flush=True)
        if args.open:
            webbrowser.open(viewer.as_uri())
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
