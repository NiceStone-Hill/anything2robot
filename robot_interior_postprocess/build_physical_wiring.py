"""Build candidate physical wiring plans without changing the source assembly."""

import argparse
from pathlib import Path

from physical_wiring.pipeline import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembly", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--registration", type=Path)
    args = parser.parse_args()
    result = run(args.assembly, args.output, args.registration)
    print("RESULT:", result["status"])
    print("REPORT:", args.output.resolve() / "index.html")
    return 0 if any(s["can_route"] for s in result["scenarios"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
