"""Check delivered membership, cross sections and end-to-end segment continuity."""

from pathlib import Path

import numpy as np
from physical_wiring.catalog import read
from physical_wiring.registration import cable_groups
from shell_channels.assembly import snapshot


def verify(out):
    out = Path(out)
    report = read(out / "report.json")
    network = read(out / "network.json")
    config = read(out / "assumptions.json")
    candidate = Path(report["previous_candidate"])
    source = Path(report["assembly_source"])
    groups = cable_groups(read(candidate / "logical_plan.json"))
    old = {r["id"]: r for r in read(candidate / "routes.json")["routes"]}
    records = {r["id"]: r for r in network["cables"]}
    assert len(records) == len(network["cables"]) and set(records) == set(groups), (
        "Missing or duplicate cables"
    )
    segments = {s["id"]: s for s in network["segments"]}
    assert len(segments) == len(network["segments"]), "Duplicate segment IDs"
    used = {key: set() for key in segments}
    for key, record in records.items():
        assert record["conductors"] == groups[key], "Electrical connection changed"
        points = []
        for step in record["segments"]:
            segment = segments[step["segment_id"]]
            assert key in segment["cables"], "Missing segment membership"
            used[segment["id"]].add(key)
            path = np.asarray(segment["points_mm"])
            if step["reverse"]:
                path = path[::-1]
            if len(points):
                assert np.allclose(points[-1], path[0]), "Disconnected branch"
            points.extend(path)
        assert len(points) >= 2, "Empty cable path"
        for device, point in (
            (record["source_device"], points[0]),
            (record["target_device"], points[-1]),
        ):
            assert np.allclose(point, network["device_exits"][device]["position_mm"]), (
                "Device exit moved"
            )
    for key, segment in segments.items():
        assert used[key] == set(segment["cables"]), "Unused or missing cable membership"
        circles = segment["packing_circles_xy_radius_mm"]
        order = segment["packing_cable_order"]
        assert (
            len(order) == len(circles) == len(used[key]) and set(order) == used[key]
        ), "Packing inventory differs"
        for i, (x, y, r) in enumerate(circles):
            assert np.isclose(2 * r, old[order[i]]["diameter_mm"]), (
                "Cable diameter changed"
            )
            assert (
                np.hypot(x, y) + r + config["sleeve_wall_mm"]
                <= segment["diameter_mm"] / 2 + 1e-7
            ), "Packing overflows sleeve"
            for a, b, s in circles[:i]:
                assert (
                    np.hypot(x - a, y - b) >= r + s + config["cable_gap_mm"] - 1e-7
                ), "Cables overlap within cross section"
        assert (out / segment["file"]).is_file(), "Missing envelope mesh"
    assert not report["physical_routing_complete"] and not report["print_ready"], (
        "Assumptions cannot release hardware"
    )
    assert read(out / "source_snapshot.json") == {
        "assembly": snapshot(source),
        "candidate": snapshot(candidate),
    }, "Input modified"
    if (out / "appearance_report.json").is_file():
        from physical_wiring.catalog import sha256

        appearance = read(out / "appearance_report.json")
        for path, digest in appearance["source_files"].items():
            assert sha256(path) == digest, "Visual CAD changed"
        for record in appearance["records"]:
            if record["status"] in {
                "source_CAD_visual_reference",
                "candidate_motor_source_CAD",
            }:
                assert record["scale"] == 1, "CAD was scaled to fit an envelope"
        if (
            read(source / "assembly_config_snapshot.json")
            .get("electrical", {})
            .get("wiring_context", {})
            .get("controller_profile")
            == "raspberry_pi_pico2"
        ):
            controller = next(
                r
                for r in appearance["records"]
                if r["id"]
                == read(source / "assembly_config_snapshot.json")["electrical"][
                    "wiring_context"
                ]["controller_instance"]
            )
            assert "Pico 2" in controller["model"], (
                "Pico replaced by wrong controller display"
            )
    for name in ("index.html", "preview.png"):
        assert (out / name).is_file(), "Missing report artifact"
    if report.get("pipeline") == "shared_harness_corridor_v2":
        from physical_wiring.pipeline import assembly_context

        from .details import motor_coverage

        audit = motor_coverage(
            read(candidate / "logical_plan.json"),
            network["cables"],
            network["device_exits"],
            assembly_context(source)["motor_candidates"],
            read(source / "mechanical_scene.json"),
        )
        assert read(out / "motor_coverage.json") == audit, (
            "Motor coverage differs from delivered wiring"
        )
        assert audit["logical_coverage_complete"], "Motor wiring omitted"
        assert len({c["color"] for c in network["cables"]}) == len(records), (
            "Cable colours are not distinct"
        )
        assert (out / "smoothing_report.json").is_file(), "Missing smoothing checks"
    return {
        "delivered_network_checks": "PASS",
        "mapped_cables": len(records),
        "segments": len(segments),
        "corridor_static_geometry_passed": report["corridor_static_geometry_passed"],
        "physical_routing_complete": False,
    }
