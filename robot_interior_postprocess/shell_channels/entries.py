"""Search several wall entries while retaining exact terminal positions/normals."""

from __future__ import annotations

import math
from collections import Counter

import numpy as np

from .geometry import (
    capsule,
    check_self_clearance,
    rounded,
    simplify_curve,
    sweep,
    union,
)


class EntrySearchError(ValueError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def entry_candidates(
    surface, field, anchor, stub, preferred, diameter, closed, previous
):
    settings = surface.settings
    radius = diameter / 2 + settings["clearance_mm"]
    obstacle = closed + union(previous)
    fixed = sweep([anchor, stub], diameter / 2)
    if (fixed ^ (surface.shell + obstacle)).volume() > 0.001:
        raise EntrySearchError(
            "Fixed port lead intersects installed material or a reserved cable",
            {"stage": "fixed_port_lead"},
        )
    offsets = [np.asarray(preferred[:2]) - stub[:2], np.zeros(2)]
    spacing = max(settings["grid_mm"] * 2, settings["bend_radius_mm"] * 2)
    for distance in np.arange(
        spacing, settings["entry_search_radius_mm"] + 1e-6, spacing
    ):
        offsets.extend(
            distance * np.array([math.cos(a), math.sin(a)])
            for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)
        )
    candidates, seen, rejected = [], set(), Counter()
    occupied_rings = set()
    for offset in offsets:
        ring = round(np.linalg.norm(offset) / spacing)
        if ring in occupied_rings:
            continue
        uv = stub[:2] + offset
        key = tuple(np.round(uv, 6))
        if key in seen:
            continue
        seen.add(key)
        node = tuple(np.rint((uv - field.lo) / settings["grid_mm"]).astype(int))
        if not field.segment_clear(uv, field.uv_of(node)):
            rejected["unsafe_wall_entry"] += 1
            continue
        point = surface.point(uv, radius, settings["depth_mm"])[0]
        # A direct diagonal is cheapest. A lateral leg above the entry can
        # escape a board/battery footprint without shifting the terminal.
        above = np.array([*uv, stub[2]])
        options = [[anchor, stub, point], [anchor, stub, above, point]]
        chosen = None
        for raw in options:
            try:
                lead = rounded(raw, settings["bend_radius_mm"], settings["sample_mm"])
                # Keep the first tangent point exactly: a global chord
                # simplification could tilt the departure direction by its tolerance.
                directions = np.diff(lead, axis=0)
                directions /= np.linalg.norm(directions, axis=1)[:, None]
                normal = (stub - anchor) / np.linalg.norm(stub - anchor)
                turns = np.flatnonzero(
                    np.linalg.norm(directions - normal, axis=1) > 1e-7
                )
                tangent = int(turns[0]) if len(turns) else len(lead) - 1
                tail = simplify_curve(
                    lead[tangent:], min(0.03, settings["clearance_mm"] / 4)
                )
                lead = np.vstack((anchor, tail)) if tangent > 0 else tail
                check_self_clearance(lead, diameter)
                cable = sweep(lead, diameter / 2)
                entry_cut = capsule(point, point + [0, 0, surface.travel], radius)
                if (cable ^ ((surface.shell - entry_cut) + obstacle)).volume() > 0.001:
                    rejected["lead_collision"] += 1
                    continue
                chosen = lead
                break
            except ValueError:
                rejected["lead_bend_or_self_contact"] += 1
        if chosen is not None:
            occupied_rings.add(ring)
            candidates.append(
                {
                    "point": point,
                    "lead": chosen,
                    "length": float(
                        np.linalg.norm(np.diff(chosen, axis=0), axis=1).sum()
                    ),
                    "offset_mm": float(np.linalg.norm(offset)),
                }
            )
            if len(candidates) >= settings["entry_candidates"]:
                break
    diagnostics = {
        "tested_wall_entries": len(seen),
        "valid_candidates": len(candidates),
        "rejected": dict(rejected),
    }
    if not candidates:
        raise EntrySearchError(
            "No safe surface entry with a collision-free, bend-safe port lead",
            diagnostics,
        )
    return candidates, diagnostics
