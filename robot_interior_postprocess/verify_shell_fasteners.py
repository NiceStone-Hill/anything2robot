"""Check exported printable STL, corresponding holes, and outside loading sweeps.

Usage: python verify_shell_fasteners.py --output outputs/<assembly>
Returns nonzero on missing coverage, inconsistent shared patterns, or obstructed
screw/driver geometry. This is not a physical fit or structural load test.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from hardware import cylinder_at, transform_solid
from shell_partition.supports import intersection_volume, solid, union


def verify(output):
    output = Path(output)
    report = json.loads((output / "shell_connections.json").read_text())
    manifest = json.loads((output / "manufacturing_manifest.json").read_text())
    meshes = {
        r["file"]: trimesh.load(output / r["file"], force="mesh")
        for r in manifest["printed_parts"]
    }
    if not meshes or not all(m.is_volume for m in meshes.values()):
        raise ValueError("Printed STL must be closed positive-volume solids")
    shell_files = {
        r["file"] for r in manifest["printed_parts"] if r["kind"].startswith("shell")
    }
    coverage = report["piece_coverage"]
    if set(coverage) != shell_files or not all(len(c) >= 3 for c in coverage.values()):
        raise ValueError("Every exported shell needs at least three connections")
    material = union([solid(m) for m in meshes.values()])
    screws = []
    for link in report["links"].values():
        transform = np.array(link["world_from_cut_frame"])
        for r in link["connections"]:
            point, seat = r["local_cut_xy_mm"], r["head_seat_local_z_mm"]
            spec = r["fastener_spec"]
            screws.append(
                (
                    r["id"],
                    transform_solid(
                        cylinder_at(
                            point,
                            r["screw_diameter_mm"] / 2,
                            seat - r["screw_length_mm"],
                            seat,
                        )
                        + cylinder_at(
                            point,
                            spec["head_diameter_mm"] / 2,
                            seat,
                            seat + spec["head_height_mm"],
                        ),
                        transform,
                    ),
                )
            )
    maximum = 0.0
    groups = {}
    for name, link in report["links"].items():
        transform = np.array(link["world_from_cut_frame"])
        inverse = np.linalg.inv(transform)
        if pattern := link.get("shared_pattern"):
            groups.setdefault(pattern["id"], []).append((name, link))
        for r in link["connections"]:
            point, seat, a = (
                r["local_cut_xy_mm"],
                r["head_seat_local_z_mm"],
                r["access"],
            )
            spec = r["fastener_spec"]
            mouth = (inverse @ [*a["mouth_world_mm"], 1])[:3]
            if not np.allclose(mouth[:2], point, atol=1e-4) or not np.isclose(
                mouth[2] - seat, a["mouth_to_seat_mm"]
            ):
                raise ValueError("Access mouth must be on the actual screw axis")
            load_z = (inverse @ [*a["loading_start_world_mm"], 1])[2]
            head_height = spec["head_height_mm"]
            screw_sweep = cylinder_at(
                point, r["screw_diameter_mm"] / 2, seat - r["screw_length_mm"], load_z
            )
            screw_sweep += cylinder_at(
                point, spec["head_diameter_mm"] / 2, seat, load_z + head_height
            )
            tool_sweep = cylinder_at(
                point,
                a["max_driver_shaft_diameter_mm"] / 2,
                seat + head_height,
                load_z + head_height + a["driver_shaft_length_mm"],
            )
            tool_sweep += cylinder_at(
                point,
                a["max_driver_handle_diameter_mm"] / 2,
                seat + head_height + a["driver_shaft_length_mm"],
                load_z
                + head_height
                + a["driver_shaft_length_mm"]
                + a["max_driver_handle_length_mm"],
            )
            # Check full port radius through the exported cover, not just a
            # center ray or the narrower nominal screw head.
            port = cylinder_at(
                point, a["port_diameter_mm"] / 2 - 0.02, seat + 0.01, load_z
            )
            sweep = transform_solid(screw_sweep + tool_sweep + port, transform)
            foreign_screws = union([s for key, s in screws if key != r["id"]])
            intrusion = intersection_volume(sweep, material + foreign_screws)
            maximum = max(maximum, intrusion)
            if intrusion > 0.02:
                raise ValueError(
                    f"Exported loading/tool tunnel blocked: {r['id']} ({intrusion:.4f} mm3)"
                )
    for members in groups.values():
        reference = members[0][1]["connections"]
        for _, link in members:
            connections = link["connections"]
            if len(connections) != len(reference):
                raise ValueError("Shared patterns have different hole counts")
            for a, b in zip(reference, connections):
                if (
                    not np.allclose(
                        a["pattern_normalized_xy"],
                        b["pattern_normalized_xy"],
                        atol=1e-7,
                    )
                    or a["fastener_spec"] != b["fastener_spec"]
                ):
                    raise ValueError(
                        "Shared pattern position or fastener specification differs"
                    )
                pattern = link["shared_pattern"]
                actual = (
                    np.array(b["local_cut_xy_mm"]) - pattern["local_outline_center_mm"]
                ) / pattern["local_outline_size_mm"]
                expected = (
                    np.array(pattern["normalized_matrix_from_reference"])
                    @ b["pattern_normalized_xy"]
                )
                if not np.allclose(actual, expected, atol=1e-6):
                    raise ValueError(
                        "Shared pattern metadata does not match exported hole axes"
                    )
    return {
        "status": "passed",
        "printed_solids": len(meshes),
        "shell_pieces_connected": len(coverage),
        "shell_screws": len(screws),
        "shared_pattern_groups": [[n for n, _ in m] for m in groups.values()],
        "maximum_loading_and_tool_intrusion_mm3": maximum,
        "scope": "exported geometry only; hardware fit, hand grip, torque and load testing pending",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.output)
    (args.output / "shell_fastener_verification.json").write_text(
        json.dumps(result, indent=2)
    )
    print(json.dumps(result, indent=2))
