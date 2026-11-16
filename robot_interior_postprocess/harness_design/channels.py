"""Replace spatial candidates with verified groove candidates when possible.

All other wires and proposed module envelopes participate in collision checks.
A rejected groove leaves its complete spatial route and the shell unchanged.
"""

import numpy as np
from shell_channels.geometry import Surface, union
from shell_channels.pipeline import dump, route_one, settings_from

from .geometry import wire_solid as sweep


def channelize(bundle, routes, obstacles, config, out):
    settings = settings_from()
    settings.update(
        bend_radius_mm=config["min_bend_radius_mm"],
        wire_gap_mm=config["clearance_mm"],
        entry_candidates=3,
        max_entry_pairs=4,
    )
    current = {r["id"]: dict(r) for r in routes}
    surfaces = {}
    modified = {}
    records = []
    modules = [value for key, value in obstacles.items() if key.startswith("proposed:")]
    reserved = {
        r["id"]: sweep(r["points_mm"], r["diameter_mm"] / 2 + config["clearance_mm"])
        for r in routes
    }
    for route in routes:
        key = route["id"]
        owner = route["source"]["owner_link"]
        if owner not in bundle["surfaces"]:
            records.append(
                {
                    "id": key,
                    "method": "spatial_fallback",
                    "reason": "no_supported_wall_surface",
                }
            )
            continue
        descriptor = dict(bundle["surfaces"][owner])
        other_wires = [value for k, value in reserved.items() if k != key]
        extras = modules + other_wires
        descriptor["closed_obstacles"] = [*descriptor["closed_obstacles"], *extras]
        print("HARNESS: wall-channel option", key, flush=True)
        if owner not in surfaces:
            surfaces[owner] = Surface(
                descriptor["shell"],
                descriptor["base"],
                descriptor["normal"],
                settings,
                descriptor["obstacles"],
                bundle["space"],
            )
        surface = surfaces[owner]
        surface.obstacles = union(
            o.transform(surface.local[:3]) for o in [*descriptor["obstacles"], *extras]
        )
        points = np.asarray(route["points_mm"], float)
        wire = {
            "id": key,
            "net": key,
            "diameter_mm": route["diameter_mm"],
            "evidence": "Explicit design assumptions; real hardware unconfirmed",
            "baseline_points_mm": route["points_mm"],
        }
        for side, index, neighbor in [("source", 0, 1), ("target", -1, -2)]:
            direction = points[neighbor] - points[index]
            direction /= np.linalg.norm(direction)
            wire[side] = {
                **route[side],
                "port_id": key + ":" + side,
                "normal": direction.tolist(),
            }
        temp = out / "meshes/channel_candidate.stl"
        try:
            changed, _, result = route_one(surface, descriptor, wire, [], temp)
            destination = out / descriptor["file"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            temp.replace(destination)
            surface.shell = changed
            world = changed.transform(surface.world[:3])
            modified[descriptor["file"]] = world
            # The existing router checks final shell material, other wiring,
            # installation sweep and exported shell reload before returning.
            r = {
                **route,
                "points_mm": result["points_mm"],
                "length_mm": result["geometric_length_mm"],
                "routing_method": "wall_channel",
                "channel_evidence": result,
                "status": "candidate_channel_geometry_passed_real_interfaces_pending",
            }
            from hardware import as_mesh64

            mesh = as_mesh64(sweep(r["points_mm"], r["diameter_mm"] / 2))
            filename = "meshes/channel_cable_" + str(len(records)) + ".stl"
            mesh.export(out / filename, file_type="stl_ascii")
            r["file"] = filename
            # V2 does not certify spatial bend radius across every surface seam.
            r["checks"] = {
                **route["checks"],
                "surface_channel_checked": True,
                "surface_seam_bend_validation": "pending",
            }
            current[key] = r
            reserved[key] = sweep(
                r["points_mm"], r["diameter_mm"] / 2 + config["clearance_mm"]
            )
            records.append(
                {
                    "id": key,
                    "method": "wall_channel",
                    "modified_shell": descriptor["file"],
                }
            )
        except ValueError as error:
            temp.unlink(missing_ok=True)
            records.append(
                {"id": key, "method": "spatial_fallback", "reason": str(error)}
            )
    bundle["parts"].update(modified)
    dump(
        out / "channel_options.json",
        {
            "options": records,
            "modified_shells": list(modified),
            "retention_and_physical_release": False,
            "old_source_unchanged": True,
        },
    )
    return [current[r["id"]] for r in routes], records
