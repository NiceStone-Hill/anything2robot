#!/usr/bin/env python3
"""Standalone Anything2robot -> physical hollow cavity -> voxel_tool layout bridge."""

import argparse
import sys
import time
import traceback
from pathlib import Path

from common import ROOT, dump, geometry_signature, read


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path(__file__).parent / "configs/cactus.json"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--stage", choices=["geometry", "all"], default="all")
    parser.add_argument("--reuse-geometry", action="store_true")
    args = parser.parse_args()
    config = read(args.config)
    import resource

    limit = int(config.get("memory_limit_mib", 4500)) * 1024**2
    resource.setrlimit(resource.RLIMIT_AS, (limit, limit))
    sys.path.insert(0, config["voxel_tool_source"])
    out = (args.output or ROOT / config["output"]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "meshes").mkdir(exist_ok=True)
    import importlib.metadata
    import subprocess

    versions = {
        name: importlib.metadata.version(name)
        for name in ["numpy", "scipy", "trimesh", "manifold3d", "scikit-image"]
    }
    commit = subprocess.run(
        [
            "git",
            "-C",
            str(Path(config["voxel_tool_source"]).parent),
            "rev-parse",
            "HEAD",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    manifest = {
        "status": "running",
        "started_epoch": time.time(),
        "stage": "inputs",
        "config": config,
        "artifacts": {},
        "dependencies": versions,
        "voxel_tool_commit": commit.stdout.strip(),
    }

    def stage(name):
        manifest["stage"] = name
        dump(out / "manifest.json", manifest)
        print(name, flush=True)

    try:
        import numpy as np
        import trimesh
        from circuit import component_schema, import_circuit
        from layout import fixed_motors, place_components
        from mechanical import load_scene
        from space import build_space
        from voxel_tool.storage.voxel import save_voxel_grid

        stage("mechanical_scene")
        scene, meshes = load_scene(config, out)
        topology, connectivity = import_circuit(config, out, scene)
        stage("hollow_geometry")
        signature = geometry_signature(config)
        if args.reuse_geometry:
            saved = out / "geometry_signature.json"
            if not saved.is_file() or read(saved) != signature:
                raise ValueError(
                    "Geometry cache does not match current algorithm/config/inputs; rebuild without --reuse-geometry"
                )
            with np.load(out / "space_masks.npz") as data:
                space = {k: data[k].copy() for k in data.files}
            space["pitch"] = float(space["pitch"])
            hollows = {
                name: trimesh.load(
                    out / "meshes" / f"{name}_hollow_assembly_mm.stl", force="mesh"
                )
                for name in meshes
            }
        else:
            space, hollows = build_space(meshes, scene, config, out)
            dump(out / "geometry_signature.json", signature)
        fixed = fixed_motors(scene, config, space)
        schema = component_schema(config, connectivity, fixed, out, scene)
        if args.stage == "geometry":
            manifest.update(
                status="geometry_complete",
                artifacts={
                    "space_report": "space_report.json",
                    "scene": "mechanical_scene.json",
                },
            )
            dump(out / "manifest.json", manifest)
            return 0
        stage("placement")
        placement, regions = place_components(config, schema, space, scene, fixed, out)
        stage("routing")
        from routing import route_layout

        grid, routing = route_layout(
            config, space, scene, hollows, placement, schema, topology, out
        )
        save_voxel_grid(grid, out / "planning_domain.npz")
        if not placement.unplaced and not routing.unrouted:
            stage("eps_and_bend_polish")
            from optimization import optimize_layout

            placement, routing = optimize_layout(
                config, scene, space, grid, placement, routing, topology, out
            )
        stage("report")
        from report import export_report

        validation = export_report(
            config, scene, hollows, grid, placement, regions, routing, out
        )
        manifest.update(
            status="concept_complete"
            if not placement.unplaced
            and not routing.unrouted
            and validation["all_components_clear"]
            and validation["all_routes_clear_of_shell"]
            else "partial",
            unplaced=list(placement.unplaced),
            unrouted=list(routing.unrouted),
            routes=len(routing.routes),
            validation_scope="Static proxy placement and voxel path validation; electrical, assembly access, motion and manufacturing certification pending",
            artifacts={
                "viewer": "index.html",
                "layout": "layout_canvas.html",
                "scene": "mechanical_scene.json",
                "placement": "placement.json",
                "routing": "routing.json",
                "circuit": "circuit_audit.json",
                "space": "space_report.json",
            },
        )
        dump(out / "manifest.json", manifest)
        print("RESULT", manifest["status"], out, flush=True)
        return 0 if manifest["status"] == "concept_complete" else 2
    except Exception as exc:  # noqa: BLE001 - CLI boundary records the failed stage.
        manifest.update(status="failed", error=str(exc))
        dump(out / "manifest.json", manifest)
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
