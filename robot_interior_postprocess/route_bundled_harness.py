"""Generate an independent shared-harness alternative; preserve earlier outputs."""

import argparse
import subprocess
from datetime import UTC, datetime
from pathlib import Path

from harness_bundle.pipeline import run
from harness_bundle.validation import verify
from shell_channels.pipeline import dump

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--candidate",
        type=Path,
        default=ROOT / "outputs/pico2_full_wiring_20261115/harness_candidate",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--config", type=Path, default=ROOT / "configs/harness_bundle.json"
    )
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()
    out = (
        args.output
        or ROOT
        / "outputs"
        / ("bundled_harness_" + datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ"))
    ).resolve()
    report = run(args.candidate, out, args.config)
    checks = verify(out)
    dump(out / "test_result.json", checks)
    print("DELIVERED NETWORK CHECKS:", checks["delivered_network_checks"])
    print("MAPPED CABLES:", report["mapped_cables"], "/", report["required_cables"])
    print("STATIC CORRIDOR GEOMETRY:", report["corridor_static_geometry_passed"])
    print(
        "PHYSICAL HARNESS: pending connector fan-out, branch transitions and retention"
    )
    print("REPORT:", out / "index.html")
    if args.open:
        subprocess.run(["xdg-open", str(out / "index.html")], check=False)
    return (
        0
        if report["corridor_static_geometry_passed"]
        and report["candidate_connectivity_complete"]
        else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
