"""Read-only diagnostics for a cut rejected by the manufacturing validator.

Recompute a half-space clip for inspection only. A diagnostic is never
returned to layout, routing, fastening or manufacturing export.
"""

import numpy as np
from hardware import as_mesh
from shell_partition.supports import solid


def cut_diagnostic(shell, plane, gap_mm, max_display_regions=64):
    normal = np.asarray(plane["normal"], float)
    material = solid(shell)
    sides, statistics = [], []
    for sign in (-1, 1):
        clipped = material.trim_by_plane(
            (sign * normal).tolist(), sign * plane["offset_mm"] + gap_mm / 2
        )
        regions = sorted(
            (part for part in clipped.decompose() if part.volume() > 1e-6),
            key=lambda part: -part.volume(),
        )
        statistics.append(
            {
                "side": "owner" if sign < 0 else "cover",
                "material_regions": len(regions),
                "displayed_regions": min(len(regions), max_display_regions),
                "volume_mm3": float(clipped.volume()),
            }
        )
        sides.append([as_mesh(part) for part in regions[:max_display_regions]])
    return sides, {
        "method": "diagnostic half-space CSG at the attempted plane; not a validated shell split",
        "sides": statistics,
        "total_material_regions": sum(s["material_regions"] for s in statistics),
        "expected_material_regions": 2,
        "omitted_regions": sum(
            s["material_regions"] - s["displayed_regions"] for s in statistics
        ),
        "manufacturing_approved": False,
    }


def candidate_info(link, index, opening, plane, shell, source_id, rank_source):
    normal = np.asarray(plane["normal"])
    axis = int(np.argmax(abs(normal)))
    lateral = [i for i in range(3) if i != axis]
    visible_axis = max(lateral, key=lambda i: shell.extents[i])
    view_axis = next(i for i in lateral if i != visible_axis)
    return {
        "key": plane["id"],
        "link": link,
        "rank": index + 1,
        "source_candidate_id": source_id,
        "plane": plane,
        "fraction": opening["fraction"],
        "axis": "XYZ"[axis],
        "coordinate_mm": float(plane["offset_mm"] / normal[axis]),
        "source_bounds_mm": shell.bounds.tolist(),
        "side_view_axis": view_axis,
        "profile_axes": [visible_axis, axis],
        "rank_source": rank_source,
        "status": "attempting",
    }
