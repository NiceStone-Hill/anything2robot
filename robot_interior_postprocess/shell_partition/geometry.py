"""Geometry-only split search. No provider, workflow or robot-specific imports."""

from __future__ import annotations

import numpy as np
import trimesh


def overlap(a, b):
    if np.any(a.bounds[1] <= b.bounds[0]) or np.any(b.bounds[1] <= a.bounds[0]):
        return 0.0
    result = trimesh.boolean.intersection([a, b], engine="manifold")
    if result.is_empty:
        return 0.0
    # Tangent intended contacts can return a zero-volume sheet. Only volume is
    # needed; avoid the undefined centre-of-mass division for such intersections.
    return abs(
        float(
            trimesh.triangles.mass_properties(
                result.triangles,
                center_mass=np.zeros(3),
                skip_inertia=True,
            ).volume
        )
    )


def translated(mesh, delta):
    return mesh.copy().apply_translation(delta)


def convex_sweep(mesh, direction, distance):
    """Exact straight sweep for convex bodies; conservative for all other bodies."""
    delta = np.asarray(direction, dtype=float) * distance
    return trimesh.convex.convex_hull(np.vstack([mesh.vertices, mesh.vertices + delta]))


def target_sweep(target, direction, distance):
    """Sweep registered board and keepouts separately to retain board-bottom steps.

    Each source part still uses a conservative hull; no hole inference or sampled
    collision is used. Legacy targets retain their original convex-proxy method.
    """
    parts = target.get("sweep_parts")
    if not parts:
        return convex_sweep(target["mesh"], direction, distance)
    key = (*np.asarray(direction).tolist(), float(distance))
    cache = target.setdefault("sweep_cache", {})
    if key not in cache:
        values = [convex_sweep(m, direction, distance) for m in parts]
        if len(cache) >= 4:
            del cache[next(iter(cache))]
        cache[key] = trimesh.boolean.union(values, engine="manifold")
    return cache[key]


def candidates(part_id, shell, motors):
    """Axis/PCA directions, independent of gravity and of the tallest component."""
    normals = list(np.eye(3))
    _, axes = np.linalg.eigh(np.cov(shell.vertices.T))
    for normal in axes.T:
        if not any(abs(normal @ known) > 0.98 for known in normals):
            normals.append(normal)
    result = []
    for axis_id, normal in enumerate(normals):
        normal = np.asarray(normal).copy()
        if normal[np.argmax(abs(normal))] < 0:
            normal *= -1
        projections = shell.vertices @ normal
        lo, hi = float(projections.min()), float(projections.max())
        for fraction in (0.35, 0.5, 0.65):
            offset = lo + fraction * (hi - lo)
            result.append(
                {
                    "id": f"{part_id}:n{axis_id}:f{fraction:.2f}",
                    "normal": normal.tolist(),
                    "offset_mm": offset,
                    "fraction": fraction,
                    "motor_axis_alignment": {
                        m["id"]: abs(float(normal @ np.asarray(m["axis"])))
                        for m in motors
                    },
                }
            )
    return result


def split_shell(shell, candidate, gap_mm):
    if not np.isfinite(gap_mm) or gap_mm <= 0:
        raise ValueError("seam gap must be finite and positive")
    normal = np.asarray(candidate["normal"], dtype=float)
    if (
        normal.shape != (3,)
        or not np.isfinite(normal).all()
        or not np.isclose(np.linalg.norm(normal), 1)
    ):
        raise ValueError("split normal must be a finite unit vector")
    offset = float(candidate["offset_mm"])
    if not np.isfinite(offset):
        raise ValueError("split offset must be finite")
    center = shell.bounds.mean(axis=0)
    point = center + normal * (offset - center @ normal)
    span = float(
        np.linalg.norm(shell.extents) * 3 + abs(offset - center @ normal) * 2 + 10
    )
    rotation = trimesh.geometry.align_vectors([0, 0, 1], normal)
    pieces = []
    for sign in (-1, 1):
        transform = rotation.copy()
        transform[:3, 3] = point + sign * normal * (span / 2 + gap_mm / 2)
        clip = trimesh.creation.box([span, span, span], transform=transform)
        piece = trimesh.boolean.intersection([shell, clip], engine="manifold")
        if piece.is_empty or not piece.is_volume:
            raise ValueError("split creates an empty or invalid solid")
        # A sealed internal cavity is a separate negatively oriented boundary,
        # not a disconnected material fragment. Count positive solid regions.
        boundaries = piece.split(only_watertight=False)
        if sum(boundary.volume > 1e-6 for boundary in boundaries) != 1:
            raise ValueError("split creates disconnected shell fragments")
        if piece.volume < shell.volume * 0.02:
            raise ValueError("split leaves a fragment below 2% of source material")
        pieces.append(piece)
    removed = float(shell.volume - sum(p.volume for p in pieces))
    if removed < -0.01 or removed > shell.volume * 0.08:
        raise ValueError("seam removes more than 8% of shell material")
    return pieces, removed


def access_probe(target, direction, radius, distance):
    """Assumed radial finger/tool corridor, NOT a measured fastener location."""
    normal = np.asarray(direction)
    center = target.bounds.mean(axis=0)
    extent = float(np.max((target.vertices - center) @ normal))
    start = center + normal * extent
    rotation = trimesh.geometry.align_vectors([0, 0, 1], normal)
    rotation[:3, 3] = start + normal * distance / 2
    return trimesh.creation.cylinder(
        radius=radius, height=distance, sections=24, transform=rotation
    )


def evaluate(shell, candidate, targets, gap_mm=0.4, probe_radius_mm=12.0):
    pieces, removed = split_shell(shell, candidate, gap_mm)
    return evaluate_pieces(shell, pieces, candidate, targets, removed, probe_radius_mm)


def evaluate_pieces(shell, pieces, candidate, targets, removed, probe_radius_mm=12.0):
    """Evaluate final pieces, including fused supports, against the saved targets."""
    normal = np.asarray(candidate["normal"])
    # Enough travel to exit the whole local subassembly, not a fixed lift distance.
    bounds = np.array([shell.bounds, *[t["mesh"].bounds for t in targets]])
    distance = float(np.linalg.norm(bounds[:, 1].max(0) - bounds[:, 0].min(0)) * 2 + 20)
    options = []
    for panel_index, sign in ((0, -1), (1, 1)):
        direction = normal * sign
        panel, held = pieces[panel_index], pieces[1 - panel_index]
        checks = []
        for target in targets:
            body = target["mesh"]
            # Moving panel +d vs fixed convex hardware is equivalent to fixed
            # panel vs hardware swept -d. Unlike discrete lifts this covers the
            # complete translation, including the installed starting pose.
            panel_collision = overlap(panel, target_sweep(target, -direction, distance))
            insertion_collision = overlap(
                held, target_sweep(target, direction, distance)
            )
            baseline = overlap(shell, body)
            corridor = access_probe(body, direction, probe_radius_mm, distance)
            probe_collision = overlap(held, corridor)
            checks.append(
                {
                    "target_id": target["id"],
                    "kind": target["kind"],
                    "baseline_shell_overlap_mm3": baseline,
                    "panel_translation_overlap_mm3": panel_collision,
                    "body_insertion_overlap_mm3": insertion_collision,
                    "assumed_grip_corridor_overlap_mm3": probe_collision,
                    "panel_clear": panel_collision < 0.01,
                    "body_path_clear": insertion_collision < 0.01,
                    "assumed_grip_clear": probe_collision < 0.01,
                    "task_status": (
                        "input_proxy_overlap_unresolved"
                        if baseline >= 0.01
                        else "local_proxy_access_clear"
                        if max(panel_collision, insertion_collision, probe_collision)
                        < 0.01
                        else "blocked_by_shell"
                    ),
                    "scope": "Convex proxy vs this link only; other links/hardware, wiring and actual fasteners not certified",
                }
            )
        # Hard access success is body + grip + closure; a valid mesh alone is
        # retained as partial evidence if no candidate meets all access checks.
        clear = sum(
            c["panel_clear"] and c["body_path_clear"] and c["assumed_grip_clear"]
            for c in checks
        )
        options.append(
            {
                "panel_index": panel_index,
                "direction": direction.tolist(),
                "travel_mm": distance,
                "targets": checks,
                "clear_tasks": clear,
                "total_tasks": len(checks),
                "all_local_proxy_tasks_clear": clear == len(checks),
            }
        )
    chosen = max(
        options,
        key=lambda o: (o["clear_tasks"], sum(c["panel_clear"] for c in o["targets"])),
    )
    return pieces, {
        "candidate_id": candidate["id"],
        "plane": candidate,
        "status": "local_proxy_access_clear"
        if chosen["all_local_proxy_tasks_clear"]
        else "partial_access",
        "seam_removed_mm3": removed,
        "source_material_mm3": float(shell.volume),
        "piece_volumes_mm3": [float(p.volume) for p in pieces],
        "closed_connected_pieces": True,
        "piece_separation": "Continuous separating-plane proof for the two unfastened shell pieces",
        "proxy_path_method": "Continuous convex translation swept volume; no discrete lift sampling",
        "selected_opening": chosen,
        "opening_options": options,
    }
