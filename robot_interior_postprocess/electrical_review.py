"""Review explicit module wiring before mechanical routing, without robot names.

This is a limited voltage/protocol review, not ERC, power simulation or release
certification. Logical terminals are never converted into physical coordinates.
"""

import argparse
import html
import math
from pathlib import Path

from common import dump, read


def voltage_range(value):
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            for v in value
        )
        or value[0] < 0
        or value[1] < value[0]
    ):
        raise ValueError("Voltage range requires two finite ordered nonnegative values")
    return value


def review(plan):
    if plan.get("schema_version") != 1:
        raise ValueError("Electrical plan requires schema_version=1")
    errors, pending, terminals, assigned = [], [], {}, {}
    instances = set()
    kinds = {
        "power_in",
        "power_out",
        "passive",
        "ground",
        "signal_in",
        "signal_out",
        "bus",
    }
    for component in plan["components"]:
        identity = component["id"]
        if identity in instances:
            raise ValueError("Duplicate component ID")
        instances.add(identity)
        pending.extend(
            {"instance_id": identity, "reason": p} for p in component.get("pending", [])
        )
        if not component.get("evidence"):
            pending.append(
                {"instance_id": identity, "reason": "missing_source_evidence"}
            )
        for pin, spec in component["ports"].items():
            if spec["kind"] not in kinds:
                raise ValueError("Unknown electrical terminal kind")
            if "voltage_range_v" in spec:
                voltage_range(spec["voltage_range_v"])
            terminals[identity, pin] = spec
    net_ids = set()
    for net in plan["nets"]:
        name = net["id"]
        if name in net_ids or len(net["terminals"]) < 2:
            raise ValueError("Unique nets with at least two terminals required")
        net_ids.add(name)
        ports, seen = [], set()
        span = (
            voltage_range(net["voltage_range_v"]) if "voltage_range_v" in net else None
        )
        for terminal in net["terminals"]:
            key = terminal["instance_id"], terminal["port_id"]
            if key not in terminals:
                errors.append(
                    {"net": name, "terminal": list(key), "reason": "unknown_terminal"}
                )
                continue
            if key in seen or key in assigned:
                errors.append(
                    {"net": name, "terminal": list(key), "reason": "terminal_reused"}
                )
            seen.add(key)
            assigned[key] = name
            spec = terminals[key]
            ports.append(spec)
            bounds = spec.get("voltage_range_v")
            if bounds and span and (span[0] < bounds[0] or span[1] > bounds[1]):
                errors.append(
                    {
                        "net": name,
                        "terminal": list(key),
                        "reason": "voltage_out_of_range",
                    }
                )
            if bounds and span is None:
                pending.append({"net": name, "reason": "net_voltage_not_specified"})
            if spec.get("protocol") and net.get("protocol") != spec["protocol"]:
                errors.append(
                    {"net": name, "terminal": list(key), "reason": "protocol_mismatch"}
                )
        types = [p["kind"] for p in ports]
        if "ground" in types and (
            any(t in types for t in ("power_in", "power_out"))
            or (span is not None and span[1] > 0)
        ):
            errors.append({"net": name, "reason": "power_ground_short"})
        if sum(t == "power_out" for t in types) > 1:
            errors.append({"net": name, "reason": "multiple_unisolated_power_sources"})
        if sum(t == "signal_out" for t in types) > 1:
            errors.append({"net": name, "reason": "multiple_signal_drivers"})
        if any(t.startswith("signal_") for t in types) and not any(
            t in {"signal_out", "bus"} for t in types
        ):
            pending.append({"net": name, "reason": "no_signal_driver"})
    for key, spec in terminals.items():
        if spec.get("required", True) and key not in assigned:
            pending.append(
                {"terminal": list(key), "reason": "unconnected_required_terminal"}
            )
    pending.extend({"reason": p} for p in plan.get("pending", []))
    return {
        "status": "failed"
        if errors
        else "pending"
        if pending
        else "limited_checks_passed",
        "complete_electrical_design": False,
        "scope": "Explicit logical voltage bounds, terminal ownership, source contention and protocol labels only; no physical routing, current, firmware, protection or PCB verification",
        "component_count": len(instances),
        "net_count": len(net_ids),
        "errors": errors,
        "pending": pending,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--html", type=Path, help="Optional readable logical wiring review"
    )
    args = parser.parse_args()
    plan = read(args.plan)
    report = review(plan)
    dump(args.output, report)
    if args.html:

        def esc(value):
            return html.escape(str(value))

        rows = "".join(
            "<tr><td>"
            + esc(n["id"])
            + "</td><td>"
            + " ↔ ".join(
                esc(t["instance_id"] + "." + t["port_id"]) for t in n["terminals"]
            )
            + "</td><td>"
            + esc(n.get("voltage_range_v", n.get("protocol", "pending")))
            + "</td></tr>"
            for n in plan["nets"]
        )
        modules = "".join(
            "<tr><td>" + esc(c["id"]) + "</td><td>" + esc(c["model"]) + "</td></tr>"
            for c in plan["components"]
        )
        issues = "".join(
            "<li>" + esc(p) + "</li>" for p in report["errors"] + report["pending"]
        )
        args.html.parent.mkdir(parents=True, exist_ok=True)
        args.html.write_text(
            '<!doctype html><meta charset="utf-8"><title>Logical wiring review</title>'
            "<style>body{font:16px system-ui;max-width:1200px;margin:40px auto;padding:24px;"
            "color:#192c40}table{border-collapse:collapse;width:100%;margin:24px 0}"
            "td,th{border:1px solid #ccd7df;padding:10px;text-align:left}"
            "li{margin:12px 0}aside{background:#fff3d4;padding:20px}</style>"
            "<h1>Logical wiring review</h1><aside>Status: "
            + esc(report["status"])
            + ". Logical connections only. Not a PCB design or validated physical harness. "
            "See pending items before selecting parts or energizing.</aside>"
            "<h2>Modules</h2><table><tr><th>Instance</th><th>Candidate</th></tr>"
            + modules
            + "</table><h2>Connections</h2><table><tr><th>Net</th>"
            "<th>Logical terminals</th><th>Voltage range (V) / protocol</th></tr>"
            + rows
            + "</table><h2>Errors and pending evidence</h2><ul>"
            + issues
            + "</ul>"
        )
    print(
        f"Electrical review: {report['status']}; {len(report['errors'])} errors, "
        f"{len(report['pending'])} pending items"
    )
    return 1 if report["errors"] else 2 if report["pending"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
