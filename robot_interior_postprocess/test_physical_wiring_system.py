"""Exercise physical wiring preparation on the saved cactus assembly plus routing regressions."""

import argparse
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path

from physical_wiring.catalog import read
from physical_wiring.pipeline import run
from route_physical_wiring import route
from shell_channels.assembly import snapshot
from shell_channels.pipeline import dump

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--assembly",
        type=Path,
        default=ROOT / "outputs/shell_channels_v2_cactus_verified/carrier_assembly",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    out = args.output or ROOT / "outputs" / (
        "physical_wiring_cactus_" + datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    )
    source = args.assembly.resolve()
    original = snapshot(source)
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            str(ROOT / "tests/test_physical_wiring.py"),
            str(ROOT / "tests/test_electrical_review.py"),
            str(ROOT / "tests/test_shell_channels.py"),
            str(ROOT / "tests/test_shell_channels_v2.py"),
        ],
        check=True,
    )
    result = run(source, out)
    assert result["source_unchanged"] and result["selected_scenario"] is None
    assert (
        result["controller_pad_count"] == 32
        and result["independent_ground_pad_count"] == 3
    )
    findings = read(out / "legacy_compatibility_audit.json")["findings"]
    assert len(findings) == 2 * len(read(out / "motor_candidates.json"))
    for bus, report in result["scenarios"].items():
        assert (
            report["errors"] == 0 and report["pending"] > 0 and not report["can_route"]
        )
        assert not (out / bus / "connections.json").exists()
        forbidden_output = out / ("blocked_routing_" + bus)
        try:
            route(source, out / bus, forbidden_output)
        except ValueError as error:
            assert "incomplete" in str(error)
        else:
            raise AssertionError(
                "Unregistered physical wiring was allowed into routing"
            )
        assert not forbidden_output.exists()
    assert snapshot(source) == original
    dump(
        out / "test_result.json",
        {
            "software_test_status": "PASS",
            "physical_harness_status": "pending_hardware_selection_and_registration",
            "source_unchanged": True,
            "incomplete_inputs_blocked_before_routing": True,
            "scope": "Saved assembly -> candidate plans -> physical input gate, plus existing static routing regression tests. Does not rerun upstream robot generation or certify hardware motion.",
        },
    )
    print(
        "SOFTWARE TEST: PASS; real hardware selection, assembly and motion validation remain pending"
    )
    print("REPORT:", out.resolve() / "index.html")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
