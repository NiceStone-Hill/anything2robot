"""Build an offline PCB-mount example using the official Pololu hole interface.

The rectangular enclosure is synthetic; the PCB/holes come from the local
catalog. The saved proposal is explicitly synthetic and does not call a model.
"""

import argparse
import json
from argparse import Namespace
from pathlib import Path

import numpy as np
import trimesh
from partition_shells import run


def build(output, template):
    root = Path(__file__).resolve().parent
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new empty output directory")
    source = output / "source"
    (source / "meshes").mkdir(parents=True)
    shell = trimesh.boolean.difference(
        [trimesh.creation.box([100] * 3), trimesh.creation.box([90] * 3)],
        engine="manifold",
    )
    shell.export(source / "meshes/room_hollow_assembly_mm.stl")
    trimesh.creation.box([24, 27, 15]).export(source / "meshes/regulator_proxy_mm.stl")
    files = {
        "manifest.json": {
            "config": {
                "placement_part": "room",
                "motor_mapping": {},
                "voxel_tool_source": str(root / "unused_for_replay"),
                "wire_radius_mm": 1,
            }
        },
        "mechanical_scene.json": {
            "parts": ["room"],
            "motors": [],
            "units": "millimeter",
            "frame": "synthetic test world",
        },
        "placement.json": {
            "units": "millimeter",
            "placements": [{"instance_id": "regulator", "component_id": "regulator"}],
        },
    }
    for name, data in files.items():
        (source / name).write_text(json.dumps(data, indent=2))
    configured = json.loads(
        (root / "configs/cactus_interface_supports.json").read_text()
    )
    profile = next(
        r["mounting"]["fastener"]
        for r in configured["targets"]
        if r["target_id"] == "REGULATOR_1"
    )
    binding = np.eye(4)
    binding[:3, 3] = [-8.89, -10.16, 0]
    plan = {
        "schema_version": 1,
        "units": "mm",
        "targets": [
            {
                "target_id": "regulator",
                "owner_link": "room",
                "templates": [template],
                "mounting": {
                    "definition": str(root / "hardware_catalog/pololu_d24v50f5.json"),
                    "world_from_interface": binding.tolist(),
                    "fastener": profile,
                    "registration": {
                        "status": "explicit_candidate",
                        "note": "Official Pololu PCB centred in synthetic enclosure. No measured robot registration.",
                    },
                },
            }
        ],
    }
    config = output / "support_plan.json"
    config.write_text(json.dumps(plan, indent=2))
    proposal = output / "synthetic_proposal.json"
    proposal.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "unknowns": ["Synthetic enclosure; physical fit and strength pending"],
                "parts": [
                    {
                        "part_id": "room",
                        "ranked_candidate_ids": [
                            "room:n2:f0.65",
                            "room:n2:f0.50",
                            "room:n2:f0.35",
                        ],
                        "rationale": "Synthetic offline mounting demonstration; this is not a model response",
                        "assembly_notes": ["Insert PCB along its explicit normal"],
                        "unknowns": [],
                    }
                ],
            },
            indent=2,
        )
    )
    return run(
        Namespace(
            layout=source,
            output=output / "result",
            support_plan=config,
            gap_mm=0.4,
            grip_radius_mm=2.0,
            llm_env=output / "unused.env",
            proposal=proposal,
            prepare_only=False,
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--template",
        choices=["hole_standoffs", "ribbed_shelf", "removable_carrier"],
        default="ribbed_shelf",
    )
    args = parser.parse_args()
    raise SystemExit(build(args.output, args.template))
