"""Generic explicit-terminal routing, independent of the legacy Eagle adapter."""

from itertools import pairwise

import numpy as np
from common import dump, read
from hardware import number
from scipy.ndimage import binary_erosion
from shell_partition.io import fingerprint
from shell_partition.supports import rod, union


def blocked_route_cells(routes, shell_material, radius, origin, pitch):
    """Find finite-radius route grazing, including between grid corner vertices."""
    from routing import dense_path
    from shell_partition.supports import intersection_volume

    blocked = set()
    for route in routes:
        for a, b in pairwise(np.asarray(route.points_mm)):
            if np.linalg.norm(b - a) < 1e-6:
                continue
            if intersection_volume(rod(a, b, radius), shell_material) > 0.001:
                cells = np.rint((np.array([a, b]) - origin) / pitch).astype(int)
                blocked.update(dense_path([tuple(v) for v in cells]))
    return blocked


def explicit_route(
    spec, base, space, profiles, poses, designs, out, sources, voxel_source=None
):
    import sys

    if voxel_source:
        sys.path.insert(0, str(voxel_source))
    from voxel_tool.stages.routing.router import _a_star

    path = (base / spec["file"]).resolve()
    sources[str(path)] = fingerprint(path)
    data = read(path)
    if (
        set(data) != {"schema_version", "units", "nets"}
        or data["schema_version"] != 1
        or data["units"] != "mm"
    ):
        raise ValueError("Normalized wiring requires schema_version=1, units=mm, nets")
    radius = number(spec["wire_radius_mm"], "wire_radius_mm")
    free = space["usable"].copy()
    origin, pitch = np.asarray(space["origin"]), float(space["pitch"])

    def block(bounds):
        lo = np.maximum(0, np.floor((bounds[0] - origin) / pitch).astype(int))
        hi = np.minimum(
            free.shape, np.ceil((bounds[1] - origin) / pitch).astype(int) + 1
        )
        free[tuple(slice(int(a), int(b)) for a, b in zip(lo, hi))] = False

    for pose in poses:
        for bounds in pose["collision_boxes"]:
            block(bounds)
    for d in designs.values():
        for dock in d["docks"].values():
            from hardware import as_mesh

            block(as_mesh(dock).bounds)
    # Six-neighbour erosion gives a conservative grid guard for the declared
    # wire radius. Exact final tubes are additionally checked by the pipeline.
    free = binary_erosion(free, iterations=max(1, int(np.ceil(radius / pitch))))
    profiles_by_id = {p["spec"]["id"]: p for p in profiles}
    poses_by_id = {p["id"]: p for p in poses}
    endpoints = {}
    for key, profile in profiles_by_id.items():
        transform = poses_by_id[key]["transform"]
        for port_id, port in profile["spec"].get("ports", {}).items():
            position = (transform @ np.r_[port["position_mm"], 1])[:3]
            endpoints[key, port_id] = position
    for port in spec.get("fixed_ports", []):
        key = (port["instance_id"], port["port_id"])
        if key in endpoints or not port.get("evidence"):
            raise ValueError("Fixed ports require unique identity and evidence")
        position = np.asarray(port["world_position_mm"], float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("Invalid fixed port world position")
        endpoints[key] = position
    routes, failures, tubes, used_ids = [], [], [], set()
    for net in data["nets"]:
        if (
            set(net) != {"id", "terminals"}
            or net["id"] in used_ids
            or len(net["terminals"]) < 2
        ):
            raise ValueError("Unique nets with at least two terminals required")
        used_ids.add(net["id"])
        terminals = []
        for t in net["terminals"]:
            key = t["instance_id"], t["port_id"]
            if key not in endpoints:
                failures.append(
                    {
                        "net": net["id"],
                        "terminal": list(key),
                        "reason": "missing_explicit_port",
                    }
                )
                terminals = []
                break
            position = endpoints[key]
            cell = np.rint((position - origin) / pitch).astype(int)
            if np.any(cell < 0) or np.any(cell >= free.shape) or not free[tuple(cell)]:
                failures.append(
                    {
                        "net": net["id"],
                        "terminal": list(key),
                        "reason": "explicit_port_not_in_certified_routing_air",
                    }
                )
                terminals = []
                break
            terminals.append(tuple(int(v) for v in cell))
        if not terminals:
            continue
        # Star tree with deterministic terminal identities; no nearest-air fallback.
        for index, goal in enumerate(terminals[1:], 1):
            cells = _a_star(
                free, (terminals[0],), (goal,), np.zeros(free.shape, np.uint16), 1.0
            )
            if cells is None:
                failures.append(
                    {
                        "net": net["id"],
                        "branch": index,
                        "reason": "no_obstacle_free_route",
                    }
                )
                continue
            points = origin + np.asarray(cells) * pitch
            tubes.extend(
                rod(a, b, radius)
                for a, b in pairwise(points)
                if np.linalg.norm(b - a) > 1e-6
            )
            routes.append(
                {"net": net["id"], "branch": index, "points_mm": points.tolist()}
            )
    result = {
        "status": "passed" if not failures else "failed",
        "branches": len(routes),
        "unrouted": failures,
        "radius_mm": radius,
        "scope": "Explicit supplied terminals; static guarded voxel paths; connector insertion, joint harness and electrical performance pending",
    }
    dump(out / "routing.json", {**result, "routes": routes})
    return {**result, "wire_solid": union(tubes)}
