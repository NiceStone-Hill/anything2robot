"""Independent carrier-based assembly design from Anything2robot outputs."""

import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--geometry-cache",
        type=Path,
        help="Optional exact-signature-matching hollow geometry only; never reuse placement or wiring",
    )
    parser.add_argument(
        "--proposal",
        type=Path,
        help="Explicit replay of a restricted VLM split ranking; provenance stays replay",
    )
    parser.add_argument(
        "--layout-schema-proposal",
        type=Path,
        help="Replay an explicit carrier restrict schema; otherwise use layout_constraints.mode from config",
    )
    parser.add_argument(
        "--no-process-trace",
        action="store_true",
        help="Disable read-only generation snapshots and process.html playback",
    )
    parser.add_argument(
        "--vlm",
        action="store_true",
        help="Use the existing restricted-schema VLM/environment to rank architectures",
    )
    args = parser.parse_args()
    if args.proposal and args.vlm:
        parser.error("Choose live VLM or explicit saved proposal")
    from carrier_layout.pipeline import run_pipeline

    return run_pipeline(
        args.config,
        args.output,
        args.geometry_cache,
        args.proposal,
        args.vlm,
        record_process=not args.no_process_trace,
        schema_proposal=args.layout_schema_proposal,
    )


if __name__ == "__main__":
    sys.exit(main())
