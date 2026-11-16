"""Independent assembly-aware shell splitting of an existing layout result."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import UTC, datetime
from pathlib import Path

from shell_partition.io import fingerprint, observations, read_layout, write_json


def run(args):
    layout = args.layout.resolve()
    out = args.output.resolve()
    if out == layout or out.is_relative_to(layout):
        raise ValueError("Output must be outside the source layout directory")
    if out.exists() and any(out.iterdir()):
        raise ValueError("Output directory must be empty; use a new run directory")
    if not 0 < args.gap_mm <= 3 or not 0 < args.grip_radius_mm <= 100:
        raise ValueError("Require 0 < gap_mm <= 3 and 0 < grip_radius_mm <= 100")
    out.mkdir(parents=True, exist_ok=True)
    manifest = {
        "status": "running",
        "source_layout": str(layout),
        "units": "mm",
        "started_utc": datetime.now(UTC).isoformat(),
        "seam_gap_mm": args.gap_mm,
        "assumed_grip_radius_mm": args.grip_radius_mm,
    }
    write_json(out / "manifest.json", manifest)
    try:
        from shell_partition.geometry import evaluate
        from shell_partition.model import propose
        from shell_partition.viewer import export_viewer

        shells, targets, catalog, context, sources, config = read_layout(layout)
        support_plan = None
        if getattr(args, "support_plan", None):
            from shell_partition.supports import load_support_plan, route_obstacles

            support_plan = load_support_plan(
                args.support_plan, targets, shells, sources
            )
            from hardware import interface_targets

            targets = interface_targets(targets, support_plan)
            write_json(
                out / "hardware_interfaces.json",
                {
                    "schema_version": 1,
                    "units": "mm",
                    "interfaces": [
                        i["record"] for i in support_plan["interfaces"].values()
                    ],
                    "unregistered_target_ids": [
                        t["id"]
                        for t in targets
                        if t["id"] not in support_plan["interfaces"]
                    ],
                },
            )
            wire_radius = max(
                support_plan["settings"]["wire_radius_mm"],
                config.get("wire_radius_mm", 1.0),
            )
            wires, wire_status = route_obstacles(
                layout / "routing.json", sources, wire_radius
            )
            support_plan["settings"]["wire_radius_mm"] = wire_radius
            if support_plan["interfaces"]:
                support_plan["wire_relief"], _ = route_obstacles(
                    layout / "routing.json",
                    sources,
                    wire_radius + support_plan["settings"]["wire_relief_mm"],
                )
            context["support_design"] = support_plan["summary"]
            context["support_design"]["wire_protection"] = wire_status
            write_json(out / "support_plan_snapshot.json", support_plan["summary"])
        write_json(out / "input_hashes.json", sources)
        write_json(out / "candidate_catalog.json", catalog)
        print("Rendering whole-robot and per-link observations", flush=True)
        observed_targets = targets + (
            [
                {
                    "id": e["id"],
                    "kind": "support",
                    "owners": [e["owner_link"]],
                    "mesh": e["mesh"],
                }
                for e in support_plan["existing"]
            ]
            if support_plan
            else []
        )
        images = observations(shells, observed_targets, out)
        context["image_files_in_request_order"] = [
            str(p.relative_to(out)) for p in images
        ]
        context["assumed_grip_radius_mm"] = args.grip_radius_mm
        context["seam_gap_mm"] = args.gap_mm
        write_json(out / "observation_context.json", context)
        if args.prepare_only:
            manifest["status"] = "observations_ready"
            write_json(out / "manifest.json", manifest)
            return 0
        sys.path.insert(0, config["voxel_tool_source"])
        proposal = propose(context, catalog, images, out, args.llm_env, args.proposal)
        selected, reports, trace, selected_supports = {}, {}, [], {}
        (out / "meshes").mkdir()
        for strategy in proposal["parts"]:
            part = strategy["part_id"]
            local_targets = [t for t in targets if part in t["owners"]]
            lookup = {c["id"]: c for c in catalog[part]}
            best = None
            for rank, candidate_id in enumerate(strategy["ranked_candidate_ids"]):
                print(f"Evaluating {candidate_id}", flush=True)
                try:
                    features = []
                    if support_plan is None:
                        pieces, report = evaluate(
                            shells[part],
                            lookup[candidate_id],
                            local_targets,
                            args.gap_mm,
                            args.grip_radius_mm,
                        )
                    else:
                        from shell_partition.joint import evaluate_joint

                        pieces, report, features = evaluate_joint(
                            shells[part],
                            lookup[candidate_id],
                            local_targets,
                            support_plan,
                            wires,
                            wire_status,
                            other_shells=[
                                m for name, m in shells.items() if name != part
                            ],
                            all_targets=targets,
                            gap_mm=args.gap_mm,
                            probe_radius_mm=args.grip_radius_mm,
                            part_id=part,
                        )
                except ValueError as exc:
                    trace.append(
                        {
                            "part_id": part,
                            "candidate_id": candidate_id,
                            "status": "rejected",
                            "reason": str(exc),
                        }
                    )
                    continue
                trace.append({"part_id": part, **report})
                opening = report["selected_opening"]
                score = (
                    report.get("support_design", {}).get(
                        "all_required_generated", True
                    ),
                    report.get("support_design", {}).get(
                        "registered_interface_count", 0
                    ),
                    report.get("support_design", {}).get("generated_count", 0),
                    opening["clear_tasks"],
                    sum(c["panel_clear"] for c in opening["targets"]),
                    sum(c["body_path_clear"] for c in opening["targets"]),
                    -rank,
                )
                if best is None or score > best[0]:
                    best = score, pieces, report, features
            if best is None:
                reports[part] = {
                    "status": "unresolved",
                    "strategy": strategy,
                    "reason": "No ranked candidate forms two connected valid solids",
                }
            else:
                _, pieces, report, features = best
                report["strategy"] = strategy
                report["piece_files"] = []
                for side, mesh in enumerate(pieces):
                    filename = (
                        f"meshes/part_{list(shells).index(part)}_half_{side + 1}.stl"
                    )
                    mesh.export(out / filename)
                    report["piece_files"].append(filename)
                selected[part] = pieces
                selected_supports[part] = features
                reports[part] = report
            write_json(out / "candidate_evaluations.json", trace)
        unchanged = all(fingerprint(path) == digest for path, digest in sources.items())
        if not unchanged:
            raise RuntimeError(
                "Source artifacts changed during partition run; results are stale"
            )
        unresolved = [p for p, r in reports.items() if r["status"] == "unresolved"]
        all_access = not unresolved and all(
            r["selected_opening"]["all_local_proxy_tasks_clear"]
            and r.get("support_design", {}).get("all_required_generated", True)
            for r in reports.values()
        )
        support_displays = []
        if support_plan is not None:
            from shell_partition.supports import mesh_of

            records = []
            fasteners = []
            (out / "supports").mkdir()
            for part, features in selected_supports.items():
                for feature in features:
                    filename = f"supports/support_{len(records)}.stl"
                    mesh = mesh_of(feature["solid"])
                    mesh.export(out / filename)
                    record = {
                        **feature["record"],
                        "mesh_file": filename,
                        "owner_piece_file": reports[part]["piece_files"][
                            feature["record"]["owner_piece_index"]
                        ],
                    }
                    for key, folder in (
                        ("detachable", "carriers"),
                        ("fastener_solid", "fasteners"),
                        ("tools", "tools"),
                    ):
                        if key in feature:
                            (out / folder).mkdir(exist_ok=True)
                            artifact = f"{folder}/support_{len(records)}.stl"
                            artifact_mesh = mesh_of(feature[key])
                            artifact_mesh.export(out / artifact)
                            record[f"{key}_file"] = artifact
                            support_displays.append(
                                {
                                    "id": f"{record['id']}:{key}",
                                    "kind": "support",
                                    "owners": [part],
                                    "owner_piece_index": record["owner_piece_index"],
                                    "mesh": artifact_mesh,
                                    "display_role": key,
                                }
                            )
                    if "fastener" in record:
                        fasteners.append(
                            {
                                "target_id": record["target_id"],
                                "thread": record["fastener"]["thread"],
                                "length_mm": record["fastener"]["length_mm"],
                                "count": record["fastener_count"],
                                "insert_id": record["fastener"]["insert"]["id"],
                                "process_status": record["fastener"]["process_status"],
                            }
                        )
                    records.append(record)
                    support_displays.append(
                        {
                            "id": record["id"],
                            "kind": "support",
                            "owners": [part],
                            "owner_piece_index": record["owner_piece_index"],
                            "mesh": mesh,
                        }
                    )
            covered = {(r["target_id"], r["owner_link"]) for r in records}
            coverage = [
                {
                    "target_id": t["id"],
                    "owner_link": part,
                    "status": "support_geometry_present"
                    if (t["id"], part) in covered
                    else "pending_motor_interface"
                    if t["kind"] == "motor"
                    else "support_unresolved_or_not_requested",
                }
                for t in targets
                for part in t["owners"]
            ]
            write_json(
                out / "support_ownership.json",
                {
                    "schema_version": 1,
                    "units": "mm",
                    "supports": records,
                    "target_coverage": coverage,
                    "wire_protection": wire_status,
                    "fixation_and_strength": "pending",
                    "release_policy": "Fused features belong to one piece; cross-seam or cross-link features are rejected, never clipped",
                },
            )
            write_json(
                out / "mounting_plan.json",
                {
                    "schema_version": 1,
                    "units": "mm",
                    "supports": records,
                    "scope": "PCB mounting geometry and local tasks; strength and physical fit pending",
                    "local_mounting_orders": {
                        part: r.get("support_design", {}).get("local_mounting_order")
                        for part, r in reports.items()
                    },
                    "sequences": [
                        {"target_id": r["target_id"], "steps": r["assembly_steps"]}
                        for r in records
                        if "assembly_steps" in r
                    ],
                },
            )
            import csv

            with (out / "fasteners_bom.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=[
                        "target_id",
                        "thread",
                        "length_mm",
                        "count",
                        "insert_id",
                        "process_status",
                    ],
                )
                writer.writeheader()
                writer.writerows(fasteners)
            all_access &= all(
                c["status"] == "support_geometry_present" for c in coverage
            )
            manifest["support_geometry_summary"] = {
                "required": sum(
                    r.get("support_design", {}).get("required_count", 0)
                    for r in reports.values()
                ),
                "generated": sum(
                    r.get("support_design", {}).get("generated_count", 0)
                    for r in reports.values()
                ),
                "imported": sum(
                    r.get("support_design", {}).get("imported_count", 0)
                    for r in reports.values()
                ),
                "pending_target_link_pairs": sum(
                    c["status"] != "support_geometry_present" for c in coverage
                ),
                "wire_protection": wire_status,
            }
        result = {
            "status": "local_proxy_access_clear" if all_access else "partial",
            "parts": reports,
            "unresolved_parts": unresolved,
            "source_inputs_unchanged": unchanged,
            "scope": "Per-link joint split/support search and final-piece proxy paths"
            if support_plan
            else "Independent per-link, unfastened planar shell splitting and continuous convex-proxy path checks",
            "pending": [
                "Measured motor body, mount, shaft and connector coordinate frames",
                "Preservation/strength of motor seats, load paths and new seam connections",
                "Shell fastening, alignment features, real screws and actual tool paths",
                "Hand dexterity: corridor is an assumed radial proxy, not a hand model",
                "Whole-robot joint/subassembly order, fixture stability and hardware-to-hardware collision",
                "Existing wires/connectors during opening and closing; routes were not changed",
                "Full joint motion, printability and physical assembly testing",
            ],
            "interpretation": "local_proxy_access_clear does not mean the robot is assembly-validated. Partial candidates are review artifacts only.",
        }
        write_json(out / "partition_report.json", result)
        # Viewer can show all successfully split links even if another remains unresolved.
        export_viewer(
            out,
            selected,
            targets,
            {p: reports[p] for p in selected},
            supports=support_displays,
        )
        write_json(
            out / "assembly_tasks.json",
            {
                "status": "provisional_local_tasks_only",
                "global_sequence": "pending",
                "links": [
                    {
                        "part_id": part,
                        "panel_index": r["selected_opening"]["panel_index"],
                        "opening_direction": r["selected_opening"]["direction"],
                        "tasks": [
                            "Separate unfastened shell pieces",
                            "Build internal subassembly with local body/grip access",
                            "Fasten real mounts and connect wiring (pending interface data)",
                            "Connect neighboring links (global order pending)",
                            "Close shell and install externally accessible fasteners (not generated)",
                        ],
                        "vlm_notes": r["strategy"]["assembly_notes"],
                    }
                    for part, r in reports.items()
                    if part in selected
                ],
            },
        )
        manifest.update(
            status=result["status"],
            split_links=len(selected),
            total_links=len(shells),
            exported_pieces=sum(len(p) for p in selected.values()),
            joint_support_search=support_plan is not None,
            generated_or_imported_supports=len(support_displays),
            source_inputs_unchanged=unchanged,
            model_provenance=json.loads((out / "model_provenance.json").read_text()),
            artifacts={
                "viewer": "index.html",
                "report": "partition_report.json",
                "candidate_trace": "candidate_evaluations.json",
                "tasks": "assembly_tasks.json",
            },
        )
        if support_plan is not None:
            manifest["artifacts"].update(
                support_ownership="support_ownership.json",
                support_plan="support_plan_snapshot.json",
                hardware_interfaces="hardware_interfaces.json",
                mounting_plan="mounting_plan.json",
                fasteners_bom="fasteners_bom.csv",
            )
        write_json(out / "manifest.json", manifest)
        print(
            json.dumps(
                {
                    "status": result["status"],
                    "split_links": len(selected),
                    "pieces": manifest["exported_pieces"],
                    "viewer": str(out / "index.html"),
                }
            ),
            flush=True,
        )
        return 0 if all_access else 2
    except Exception as exc:
        manifest.update(status="failed", error=str(exc))
        write_json(out / "manifest.json", manifest)
        raise


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, default=root / "outputs/cactus")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--llm-env", type=Path, default=root / ".env")
    parser.add_argument(
        "--proposal",
        type=Path,
        help="Explicit replay of a saved proposal; no model call",
    )
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--gap-mm", type=float, default=0.4)
    parser.add_argument("--grip-radius-mm", type=float, default=12)
    parser.add_argument(
        "--support-plan",
        type=Path,
        help="Opt-in support ownership and split/support co-search specification",
    )
    return run(parser.parse_args())


if __name__ == "__main__":
    sys.exit(main())
