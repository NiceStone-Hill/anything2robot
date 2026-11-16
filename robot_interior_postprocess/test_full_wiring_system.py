"""Rebuild assembly from mechanical inputs, then produce all-cable candidate routing.

No previous placements, routes or hollow geometry are reused. Saved restricted
model proposals are replayed explicitly. This does not rerun upstream shape
annotation/motor optimization or certify unknown physical hardware.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path

from harness_design.pipeline import run as harness_design
from physical_wiring.catalog import read
from physical_wiring.pipeline import run as physical_wiring
from shell_channels.pipeline import dump

ROOT = Path(__file__).resolve().parent


def verify(out):
    report = read(out / "routing_report.json")
    plan = read(out / "logical_plan.json")
    import numpy as np
    from harness_design.pipeline import collision
    from physical_wiring.registration import cable_groups
    from shell_channels.assembly import load_mesh, snapshot
    from shell_channels.geometry import solid

    expected = set(cable_groups(plan))
    routes = read(out / "routes.json")["routes"]
    failures = report["unrouted"]
    ids = [r["id"] for r in routes] + [r["id"] for r in failures]
    if len(ids) != len(set(ids)) or set(ids) != expected:
        raise ValueError("Every required cable must have exactly one outcome")
    if report["physical_routing_complete"] or report["complete_assemblable"]:
        raise ValueError("Unknown hardware must not be released by design mode")
    if snapshot(Path(report["source"])) != read(out / "source_snapshot.json"):
        raise ValueError("Source assembly changed")
    tubes = {}
    for route in routes:
        if not np.allclose(
            route["points_mm"][0], route["source"]["position_mm"]
        ) or not np.allclose(route["points_mm"][-1], route["target"]["position_mm"]):
            raise ValueError("Proposed endpoint changed during routing")
        value = solid(load_mesh(out / route["file"]))
        if collision(value, tubes):
            raise ValueError("Delivered candidate cables intersect each other")
        tubes[route["id"]] = value
    for name in (
        "index.html",
        "assembly_view.html",
        "wire_schedule.csv",
        "retention_plan.json",
        "terminal_proposals.json",
    ):
        if not (out / name).is_file():
            raise ValueError("Missing deliverable " + name)
    return {
        "software_checks": "PASS",
        "candidate_path_coverage_complete": len(routes) == len(expected),
        "routed_candidates": len(routes),
        "required_cables": len(expected),
        "physical_routing_complete": False,
        "assembly_rebuilt_without_geometry_cache": read(
            Path(report["source"]) / "manifest.json"
        ).get("geometry_cache_used")
        is False,
        "scope": "Mechanical result -> hollowing -> carrier layout -> assembly -> candidate circuit -> all-cable path planning -> exported geometry checks",
        "not_rerun": [
            "upstream shape annotation",
            "upstream motor optimization",
            "physical hardware tests",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--assembly-config",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--schema-proposal",
        type=Path,
        default=ROOT
        / "outputs/cactus_restrict_schema_live_verified/layout_constraints_raw.json",
    )
    parser.add_argument(
        "--opening-proposal",
        type=Path,
        default=ROOT
        / "outputs/cactus_carrier_assembly_verified_live/proposal_raw.json",
    )
    parser.add_argument("--design-config", type=Path, default=None)
    parser.add_argument(
        "--bundle",
        action="store_true",
        help="Also generate a shared harness corridor alternative",
    )
    parser.add_argument(
        "--bundle-config", type=Path, default=ROOT / "configs/harness_bundle.json"
    )
    parser.add_argument("--controller", choices=("pico2", "uno"), default="pico2")
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()
    args.assembly_config = args.assembly_config or ROOT / (
        "configs/cactus_pico2_wiring.json"
        if args.controller == "pico2"
        else "configs/cactus_carrier_assembly.json"
    )
    args.design_config = args.design_config or ROOT / (
        "configs/harness_design_pico2.json"
        if args.controller == "pico2"
        else "configs/harness_design.json"
    )
    out = (
        args.output
        or ROOT
        / "outputs"
        / ("full_wiring_" + datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ"))
    ).resolve()
    if out.exists() and any(out.iterdir()):
        parser.error("Use a new empty output directory")
    out.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}
    tests = [
        "test_harness_design.py",
        "test_pico_wiring.py",
        "test_physical_wiring.py",
        "test_electrical_review.py",
        "test_shell_channels.py",
        "test_shell_channels_v2.py",
    ]
    if args.bundle:
        tests.append("test_harness_bundle.py")
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            *[str(ROOT / "tests" / f) for f in tests],
        ],
        env=env,
        check=True,
    )
    command = [
        sys.executable,
        "-u",
        str(ROOT / "assemble_robot.py"),
        "--config",
        str(args.assembly_config.resolve()),
        "--output",
        str(out / "carrier_assembly"),
        "--proposal",
        str(args.opening_proposal.resolve()),
    ]
    if read(args.assembly_config).get("layout_constraints", {}).get("mode") == "llm":
        command.extend(
            ["--layout-schema-proposal", str(args.schema_proposal.resolve())]
        )
    print(
        "STAGE 1: rebuild geometry and assembly from mechanical inputs; no geometry/placement/routing cache",
        flush=True,
    )
    process = subprocess.run(command, env=env, check=False)
    if process.returncode not in (0, 2):
        raise RuntimeError("Assembly generation failed")
    physical_wiring(out / "carrier_assembly", out / "physical_wiring")
    bus = read(args.design_config)["scenario"]
    harness_design(
        out / "carrier_assembly",
        out / "physical_wiring" / bus / "logical_plan.json",
        out / "harness_candidate",
        args.design_config,
    )
    result = verify(out / "harness_candidate")
    report_path = out / "harness_candidate/index.html"
    successful = result["candidate_path_coverage_complete"]
    if args.bundle:
        from harness_bundle.pipeline import run as bundled_harness
        from harness_bundle.validation import verify as verify_bundle

        bundle_report = bundled_harness(
            out / "harness_candidate", out / "bundled_harness", args.bundle_config
        )
        result["bundled_harness"] = verify_bundle(out / "bundled_harness")
        successful = successful and bundle_report["corridor_static_geometry_passed"]
        report_path = out / "bundled_harness/index.html"
    dump(out / "test_result.json", result)
    print(
        f"CANDIDATE ROUTING: {result['routed_candidates']}/{result['required_cables']}"
    )
    print(
        "PHYSICAL RELEASE: pending real hardware/interfaces and manufacturing/motion verification"
    )
    print("REPORT:", report_path)
    if args.open:
        subprocess.run(["xdg-open", str(report_path)], check=False)
    # Never return success solely because a small subset routed.
    return 0 if successful else 2


if __name__ == "__main__":
    raise SystemExit(main())
