"""Reproducible carrier assembly -> shell channels -> exported-result checks.

Exit 0 means the software checks passed, never that a robot is manufacture-ready.
Default uses the saved cactus Uno bundle because it has explicit legacy anchors.
--rebuild-assembly reruns carrier assembly from mechanical inputs with saved model
proposals; no live model call is made. Other robot bundles use --assembly and
optional --connections, with identical channel code.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from shell_channels.assembly import load_mesh, read, snapshot
from shell_channels.pipeline import dump, run

HERE = Path(__file__).resolve().parent


def verify_output(out, minimum):
    manifest = read(out / "manifest.json")
    report = read(out / "channel_report.json")
    inputs = read(out / "channel_inputs.json")["cables"]
    routes = read(out / "channel_routes.json")["routes"]
    hashes = read(out / "source_snapshot.json")
    source = Path(manifest["source"])
    ids = {c["id"] for c in inputs}
    if len(ids) != len(inputs) or ids != {r["id"] for r in report["branches"]}:
        raise ValueError("Every input cable must have exactly one outcome")
    if snapshot(source) != hashes or snapshot(out / "baseline") != hashes:
        raise ValueError("Baseline/source restoration checks failed")
    if report["accepted"] < minimum or len(routes) != report["accepted"]:
        raise ValueError(
            f"Only {report['accepted']} routes passed; test requires {minimum}"
        )
    if manifest["complete_assemblable"] or report["complete_assemblable"]:
        raise ValueError("Prototype incorrectly claims full assembly certification")
    import numpy as np

    for route in routes:
        if not np.allclose(
            route["points_mm"][0], route["source"]["position_mm"]
        ) or not np.allclose(route["points_mm"][-1], route["target"]["position_mm"]):
            raise ValueError("Physical terminal identity moved")
        if not route["checks"]["export_reloaded_and_rechecked"]:
            raise ValueError("Accepted channel lacks delivered-geometry evidence")
    # Recheck all delivered cables together, independently of their routing order.
    from shell_channels.geometry import sweep
    from shell_channels.pipeline import TOL

    gap = read(out / "channel_config.json")["settings"]["wire_gap_mm"]
    for i, first in enumerate(routes):
        reserved = sweep(first["points_mm"], first["diameter_mm"] / 2 + gap)
        for second in routes[i + 1 :]:
            cable = sweep(second["points_mm"], second["diameter_mm"] / 2)
            if (reserved ^ cable).volume() > TOL:
                raise ValueError(
                    "Delivered cables violate configured interwire clearance"
                )
    inventory = read(out / "manufacturing_manifest.json")
    for record in inventory["printed_parts"]:
        if not load_mesh(out / record["file"]).is_volume:
            raise ValueError(f"Invalid delivered printable solid: {record['file']}")
    for name in ("index.html", "routing_comparison.png", "wire_lengths.csv"):
        if not (out / name).is_file():
            raise ValueError(f"Missing deliverable: {name}")
    result = {
        "test_status": "PASS",
        "scope": "Software integration and sampled/exact static geometry only",
        "input_branches": len(inputs),
        "accepted_static_branches": len(routes),
        "routing_coverage_status": "complete"
        if len(routes) == len(inputs)
        else "incomplete",
        "required_minimum_static_branches": minimum,
        "unresolved_branches": len(report["unresolved"]),
        "baseline_exact_copy": True,
        "delivered_interwire_clearance_checked": True,
        "complete_assemblable": False,
        "physical_test": "not_performed",
    }
    dump(out / "test_result.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--assembly",
        type=Path,
        default=HERE / "outputs/cactus_restrict_schema_live_verified",
    )
    parser.add_argument(
        "--config", type=Path, default=HERE / "configs/shell_channels.json"
    )
    parser.add_argument("--connections", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rebuild-assembly", action="store_true")
    parser.add_argument(
        "--assembly-config",
        type=Path,
        default=HERE / "configs/cactus_carrier_assembly.json",
    )
    parser.add_argument(
        "--schema-proposal",
        type=Path,
        default=HERE
        / "outputs/cactus_restrict_schema_live_verified/layout_constraints_raw.json",
    )
    parser.add_argument(
        "--opening-proposal",
        type=Path,
        default=HERE
        / "outputs/cactus_carrier_assembly_verified_live/proposal_raw.json",
    )
    parser.add_argument(
        "--geometry-cache",
        type=Path,
        default=HERE / "outputs/cactus_carrier_assembly_raw",
    )
    parser.add_argument("--min-accepted", type=int, default=1)
    parser.add_argument("--open", action="store_true")
    parser.add_argument(
        "--motor-terminals",
        type=Path,
        help="Explicit motor-body ownership and terminal positions in assembly millimetres",
    )
    args = parser.parse_args()
    if args.min_accepted < 1:
        parser.error(
            "--min-accepted must be positive; zero routes is not a successful integration test"
        )
    root = (
        args.output
        or Path(tempfile.mkdtemp(prefix="shell_channel_test_", dir=HERE / "outputs"))
    ).resolve()
    if root.exists() and any(root.iterdir()):
        parser.error("Use a new/empty --output directory")
    root.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}
    tests = [
        HERE / "tests" / name
        for name in (
            "test_shell_channels.py",
            "test_shell_channels_v2.py",
            "test_shell_groove_trial.py",
            "test_stl_precision.py",
        )
    ]
    subprocess.run(
        [sys.executable, "-m", "pytest", "-q", *map(str, tests)],
        cwd=HERE.parent,
        env=env,
        check=True,
    )
    source = args.assembly.resolve()
    if args.rebuild_assembly:
        source = root / "carrier_assembly"
        command = [
            sys.executable,
            "-u",
            str(HERE / "assemble_robot.py"),
            "--config",
            str(args.assembly_config.resolve()),
            "--output",
            str(source),
            "--geometry-cache",
            str(args.geometry_cache.resolve()),
            "--layout-schema-proposal",
            str(args.schema_proposal.resolve()),
            "--proposal",
            str(args.opening_proposal.resolve()),
        ]
        print(
            "Rebuilding carrier assembly from mechanical data; model proposals are explicit replay.",
            flush=True,
        )
        completed = subprocess.run(command, cwd=HERE.parent, env=env, check=False)
        if completed.returncode not in (0, 2):
            raise RuntimeError(
                f"Carrier assembly failed ({completed.returncode}); inspect {source}"
            )
    out = root / "shell_channels"
    run(source, out, args.config, args.connections, args.motor_terminals)
    result = verify_output(out, args.min_accepted)
    dump(
        root / "test_result.json",
        {**result, "assembly_rebuilt": args.rebuild_assembly, "output": str(out)},
    )
    print(
        f"TEST: PASS (software/static geometry); {result['accepted_static_branches']}/{result['input_branches']} branches accepted"
    )
    print(
        "ROBOT STATUS: partial; retention, real interfaces, full wiring and motion pending"
    )
    print("VIEWER:", out / "index.html")
    print("REPORT:", out / "channel_report.json")
    if args.open:
        subprocess.run(["xdg-open", str(out / "index.html")], check=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
