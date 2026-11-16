"""Check electrical continuity across declared physical distribution parts."""

from collections import defaultdict

from electrical_review import review as base_review


def review(plan):
    report = base_review(plan)
    ports = {
        (c["id"], p): spec for c in plan["components"] for p, spec in c["ports"].items()
    }
    parents = {key: key for key in ports}

    def root(key):
        while parents[key] != key:
            parents[key] = parents[parents[key]]
            key = parents[key]
        return key

    def join(keys):
        if any(key not in parents for key in keys):
            raise ValueError("Conductive group references an unknown terminal")
        for key in keys[1:]:
            parents[root(key)] = root(keys[0])

    for component in plan["components"]:
        for group in component.get("conductive_groups", []):
            if len(group) < 2 or len(set(group)) != len(group):
                raise ValueError("Conductive groups require distinct terminals")
            join([(component["id"], pin) for pin in group])
    for net in plan["nets"]:
        keys = [(t["instance_id"], t["port_id"]) for t in net["terminals"]]
        if all(key in ports for key in keys):
            join(keys)
    connected = defaultdict(list)
    for key in ports:
        connected[root(key)].append(key)
    for keys in connected.values():
        specs = [ports[key] for key in keys]
        roles = {p["conductor_role"] for p in specs if p.get("conductor_role")}
        kinds = [p["kind"] for p in specs]
        protocols = {p["protocol"] for p in specs if p.get("protocol")}
        reasons = []
        if len(roles) > 1:
            reasons.append("conductive_path_crosses_bus_polarities")
        if len(protocols) > 1:
            reasons.append("conductive_path_protocol_mismatch")
        if "ground" in kinds and any(
            k in kinds for k in ("power_in", "power_out", "bus", "signal_out")
        ):
            reasons.append("conductive_path_ground_short")
        if kinds.count("power_out") > 1:
            reasons.append("conductive_path_multiple_power_sources")
        report["errors"].extend(
            {"reason": reason, "terminals": [list(k) for k in keys]}
            for reason in reasons
        )
    if report["errors"]:
        report["status"] = "failed"
    report["scope"] += (
        "; includes continuity and conductor polarity through declared conductive groups"
    )
    return report
