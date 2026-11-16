"""Route only registered physical cable exits against the matching assembly."""

import argparse
from pathlib import Path

from physical_wiring.pipeline import load_routing_contract
from shell_channels.pipeline import run, settings_from


def route(assembly, physical_plan, output, config=None):
    connections, bend = load_routing_contract(physical_plan, assembly)
    if settings_from(config)["bend_radius_mm"] < bend:
        raise ValueError(
            f"Groove bend_radius_mm must be at least the registered cable requirement: {bend} mm"
        )
    return run(assembly, output, config, connections)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembly", required=True, type=Path)
    parser.add_argument("--physical-plan", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    result = route(args.assembly, args.physical_plan, args.output, args.config)
    print(
        f"STATIC ROUTING: {result['accepted']}/{result['requested']}; electrical and motion verification pending"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
