"""Opt-in wall-channel routing after carrier assembly; legacy files remain intact."""

import argparse
from pathlib import Path

from shell_channels.pipeline import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembly", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument(
        "--connections",
        type=Path,
        help="Explicit physical cable endpoints; required when the assembly has no registered routing input",
    )
    parser.add_argument(
        "--motor-terminals",
        type=Path,
        help="Explicit motor-body ownership and terminal positions in assembly millimetres",
    )
    args = parser.parse_args()
    result = run(
        args.assembly, args.output, args.config, args.connections, args.motor_terminals
    )
    print(
        f"CHANNEL RESULT: {result['accepted']}/{result['requested']} static branches; whole robot partial"
    )
    print("VIEWER:", args.output.resolve() / "index.html")
    return 2  # A prototype must not claim complete manufacturing certification.


if __name__ == "__main__":
    raise SystemExit(main())
