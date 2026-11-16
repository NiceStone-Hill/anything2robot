"""A short pipeline presentation derived from real, unchanged trace events."""

from copy import deepcopy

import numpy as np

TITLES = {
    "mechanical_input": "Import the robot and fixed servo geometry",
    "hollow_geometry": "Prepare hollow shells and usable interior space",
    "carrier_profiles": "Build independent component carriers",
    "architecture_ranking": "Rank access-opening candidates",
    "layout_constraints": "Generate and validate the VLM restrict schema",
    "layout_candidates": "Compare layout seeds under the restrict schema",
    "candidate_rejected": "Screen opening candidates",
    "opening_candidate": "Open the electronics-bearing shell",
    "eps_seed": "Place component-and-carrier bundles",
    "eps_move": "Optimize the carrier layout with EPS",
    "candidate_supports": "Generate supports and check assembly access",
    "selected_layout": "Select the layout and split the remaining shells",
    "routing": "Create access ducts and route wires",
    "post_duct_split": "Rebuild supports and check every shell piece",
    "shell_fastening": "Add shell screws and tool-access tunnels",
    "final_validation": "Export parts and report validation results",
    "failed": "Report the processing failure",
}


def snapshot_delta(previous, event, assets):
    """Compare displayed scenes, including every move skipped by the overview."""
    old = {i["id"]: i for i in previous["instances"]} if previous else {}
    new = {i["id"]: i for i in event["instances"]}
    same_frame = (
        previous is not None
        and previous["coordinate_frame"] == event["coordinate_frame"]
    )
    common = new.keys() & old.keys()
    changed = sorted(k for k in common if old[k]["asset"] != new[k]["asset"])
    details = []
    for key in changed:
        before = assets[old[key]["asset"]]["source_volume_mm3"]
        after = assets[new[key]["asset"]]["source_volume_mm3"]
        details.append(
            {
                "id": key,
                "before_volume_mm3": before,
                "after_volume_mm3": after,
                "volume_delta_mm3": after - before,
            }
        )
    return {
        "added": sorted(new.keys() - old.keys()),
        "removed": sorted(old.keys() - new.keys()),
        "moved": sorted(
            k
            for k in common
            if same_frame
            and not np.allclose(old[k]["transform"], new[k]["transform"], atol=1e-7)
        ),
        "geometry_changed": changed,
        "geometry_details": details,
        "coordinate_frame_changed": previous is not None and not same_frame,
    }


def pipeline_presentation(trace):
    """At most 18 chapters, each using one actual recorded scene.

    Repeated per-link stages share a chapter; its scene is the last recorded
    example of that stage. Final selection shows the complete robot assembly.
    No new geometry, intermediate poses or solver results are invented.
    """
    events = trace["events"]
    groups = {}
    candidates = trace.get("candidates", {})
    selected = {k for k, c in candidates.items() if c.get("selected")}

    def key(event):
        return (event.get("inspection") or {}).get("key")

    layout_keys = {key(e) for e in events if e["stage"] == "eps_seed"}
    layout_keys.discard(None)
    if selected:
        layout_keys &= selected
    openings = [
        i
        for i, e in enumerate(events)
        if e["stage"] == "opening_candidate" and key(e) in layout_keys
    ]
    first_layout = min(openings, default=len(events))
    early_rejected = [
        i
        for i, e in enumerate(events)
        if e["stage"] == "candidate_rejected" and i < first_layout
    ]
    rejected_keys = {key(events[i]) for i in early_rejected}
    rejected_keys.discard(None)
    cut_members = [
        i
        for i, e in enumerate(events)
        if e["stage"] in {"opening_attempt", "opening_candidate"}
        and key(e) in layout_keys
    ]
    rejected_members = sorted(
        set(
            early_rejected
            + [
                i
                for i, e in enumerate(events)
                if e["stage"] == "opening_attempt" and key(e) in rejected_keys
            ]
        )
    )
    if openings:
        groups["opening_candidate"] = (openings[-1], cut_members)
    if early_rejected:
        groups["candidate_rejected"] = (early_rejected[-1], rejected_members)
    for stage in TITLES:
        if stage in {"opening_candidate", "candidate_rejected"}:
            continue
        members = [
            i
            for i, e in enumerate(events)
            if e["stage"] == stage
            and (
                stage
                not in {
                    "eps_seed",
                    "eps_move",
                    "candidate_supports",
                    "layout_candidates",
                }
                or not selected
                or key(e) in layout_keys
            )
        ]
        if members:
            groups[stage] = (members[-1], members)
    assigned = {i for _, members in groups.values() for i in members}
    other = [i for i in range(len(events)) if i not in assigned]
    if "selected_layout" in groups:
        representative, members = groups["selected_layout"]
        preceding = [i for i in other if i < representative]
        groups["selected_layout"] = (representative, sorted(members + preceding))
        other = [i for i in other if i not in preceding]
    # During an unfinished run, keep pending candidate activity in one chapter.
    if other:
        groups["other"] = (other[-1], other)
    chapters = []
    for stage, (representative, members) in sorted(
        groups.items(), key=lambda g: g[1][0]
    ):
        event = deepcopy(events[representative])
        event["index"] = len(chapters)
        event["source_event_indices"] = sorted(members)
        event["representative_event_index"] = representative
        event["title"] = TITLES.get(stage, "Review additional recorded processing")
        event["metrics"] = {
            "overview": {
                "source_event_indices": sorted(members),
                "representative_event_index": representative,
                "note": "One real recorded scene represents this chapter; full events remain available.",
            },
            **event["metrics"],
        }
        if stage == "candidate_rejected":
            rejected = [
                events[i] for i in members if events[i]["stage"] == "candidate_rejected"
            ]
            screening = []
            for rejected_event in rejected:
                info = rejected_event.get("inspection") or {}
                diagnostic = info.get("diagnostic", {})
                screening.append(
                    {
                        "link": info.get("link"),
                        "candidate": info.get("rank"),
                        "plane": info.get("plane"),
                        "reason": info.get("reason")
                        or rejected_event["metrics"].get("reason")
                        or rejected_event["changes"][0],
                        "expected_regions": diagnostic.get("expected_material_regions"),
                        "actual_regions": diagnostic.get("total_material_regions"),
                    }
                )
            fragmented = all(
                "disconnected shell fragments" in row["reason"] for row in screening
            )
            event["title"] = (
                "Reject cuts that break the shell into fragments"
                if fragmented
                else "Reject unusable opening candidates"
            )
            event["metrics"]["opening_screening"] = screening
            event["changes"] = [
                "Check shell-cut geometry before placing any electronics or running EPS.",
                "Each intended shell side must remain connected; extra detached material makes a candidate unusable."
                if fragmented
                else "Reject candidates that fail local checks; do not relax their geometry constraints.",
                f"Local checks rejected {len(rejected)} opening candidates; repeated attempts are condensed here.",
                *[
                    f"{(e.get('inspection') or {}).get('link', 'Link')} / "
                    f"candidate {(e.get('inspection') or {}).get('rank', '?')}: "
                    f"{(e.get('inspection') or {}).get('reason') or e['metrics'].get('reason') or e['changes'][0]}"
                    for e in rejected
                ],
                "The scene shows the last rejected candidate in this chapter, as diagnostic geometry only.",
            ]
        elif stage == "opening_candidate":
            event["changes"] = [
                "Open the shell that carries the electronics so complete carrier bundles can be inserted.",
                "This is still a candidate. EPS placement and assembly-path checks follow before final selection.",
            ]
        elif stage == "eps_move":
            event["changes"] = [
                f"Condense {len(members)} accepted EPS improvements into one step; show the last recorded optimized layout.",
                "EPS moves complete component-and-carrier bundles while checking allowed poses and clearances.",
                "The geometry changes below compare against the previous displayed scene, including skipped improvements.",
            ]
        elif stage == "selected_layout":
            event["changes"] += [
                "Links without electronic carriers still receive removable shell pieces and later fasteners.",
                "Repeated per-link cuts and remaining rejected attempts are included in this selection summary.",
            ]
        if (
            len(
                {
                    events[i].get("inspection", {}).get("link")
                    for i in members
                    if events[i].get("inspection")
                }
            )
            > 1
            and stage != "selected_layout"
        ):
            event["changes"].append(
                "This chapter includes multiple links; the scene shows the last recorded example. The selected-layout step shows their combined result."
            )
        event["delta"] = snapshot_delta(
            chapters[-1] if chapters else None, event, trace["assets"]
        )
        chapters.append(event)
    return {
        "schema_version": 1,
        "title": "Pipeline overview",
        "events": chapters,
        "note": "Condensed real snapshots. Detailed candidate trials and EPS iterations remain in the full trace.",
    }
