"""Read-only assembly -> evidence catalogue -> candidate circuits -> route gate."""

from __future__ import annotations

import html
from collections import Counter
from pathlib import Path

import numpy as np
from shell_channels.assembly import safe_path, snapshot
from shell_channels.pipeline import dump

from .catalog import build_catalog, read, rigid_matrix, sha256
from .registration import compile_connections, template
from .topology import scenario


def assembly_context(source):
    source = Path(source).resolve()
    manifest = read(source / "manifest.json")
    if (
        manifest.get("pipeline") != "carrier_assembly_v1"
        or manifest.get("stage") != "completed_with_explicit_blockers"
    ):
        raise ValueError("Expected completed carrier assembly bundle")
    config = read(source / "assembly_config_snapshot.json")
    scene = read(source / "mechanical_scene.json")
    summary = read(source / "routing_summary.json")
    directory = summary.get("artifacts_directory")
    declared = config.get("electrical", {}).get("wiring_context")
    motors = {m["id"]: m for m in scene["motors"]}
    instances, adjacent, candidates = [], {}, {}
    if declared:
        pairs = list(declared["motor_instances"].items())
        legacy_routes = {
            "routes": [],
            "status": "no_legacy_wiring_for_declared_controller",
        }
    else:
        if not directory:
            raise ValueError(
                "An explicit electrical instance-to-motor mapping is required"
            )
        mapping = read(safe_path(source, f"{directory}/circuit_mapping.json"))
        audit = read(safe_path(source, f"{directory}/circuit_audit.json"))
        pairs = [
            (mapping["instances"][ref], motor_id)
            for ref, motor_id in audit["motor_mapping"].items()
        ]
        legacy_routes = read(safe_path(source, f"{directory}/routing.json"))
    if len({m for _, m in pairs}) != len(pairs) or len({i for i, _ in pairs}) != len(
        pairs
    ):
        raise ValueError("Duplicate electrical/mechanical motor mapping")
    for identity, motor_id in pairs:
        if motor_id not in motors:
            raise ValueError("Unresolved mechanical/electrical motor identity")
        instances.append(identity)
        m = motors[motor_id]
        adjacent[identity] = [m["parent"], m["link"]]
        candidates[identity] = {
            "motor_id": motor_id,
            "configured_model": m.get("hardware_assignment", {}).get("model_id"),
            "selection_status": "unconfirmed",
            "adjacent_links": adjacent[identity],
        }
    poses = {
        p["id"]: rigid_matrix(p["world_from_interface"]).tolist()
        for p in read(source / "carrier_placement.json")["placements"]
    }
    owners = {
        s["target_id"]: s["owner_link"]
        for s in read(source / "support_ownership.json")["supports"]
    }
    board, regulator = [], []
    for spec in config["components"]:
        definition = Path(spec.get("mounting", {}).get("definition", "")).stem
        if definition in {"arduino_uno_th_rev3e", "arduino_uno_th_rev3e_footprints"}:
            board.append(spec["id"])
        if definition == "pololu_d24v50f5":
            regulator.append(spec["id"])
    if declared:
        board = [declared["controller_instance"]]
        regulator = [declared["regulator_instance"]]
        if not set(board + regulator) <= set(poses):
            raise ValueError("Declared controller/regulator missing from assembly")
    if len(board) != 1 or len(regulator) != 1:
        raise ValueError(
            "This catalogue adapter needs one explicitly identified Uno Rev3e and D24V50F5; do not infer another board from its display mesh"
        )
    return {
        "links": scene["parts"],
        "owners": owners,
        "poses": poses,
        "placed_instances": sorted(set(poses) | set(instances)),
        "motor_instances": instances,
        "motor_adjacent_links": adjacent,
        "motor_candidates": candidates,
        "controller": board[0],
        "regulator": regulator[0],
        "legacy_routing": legacy_routes,
        "controller_profile": declared["controller_profile"]
        if declared
        else "arduino_uno_rev3e",
    }


def legacy_audit(context, catalog):
    profile = catalog["profiles"]["mg4005v2_can"]
    findings = []
    for route in context["legacy_routing"]["routes"]:
        for side in ("source", "target"):
            instance = route[f"{side}_instance"]
            if instance not in context["motor_instances"]:
                continue
            pin = route[f"{side}_pin"]
            if pin == "SIG":
                findings.append(
                    {
                        "branch": route["branch_id"],
                        "instance_id": instance,
                        "reason": "concept_servo_signal_is_not_a_registered_bus_terminal",
                        "conditional_on": "Selecting a CAN or RS485 MG4005V2 variant",
                        "action": "Replace provisional SIG with correct bus interface and connector pin mapping",
                    }
                )
            if pin == "PWR":
                findings.append(
                    {
                        "branch": route["branch_id"],
                        "instance_id": instance,
                        "reason": "legacy_5v_servo_rail_below_candidate_motor_input",
                        "legacy_supply_evidence": "Existing legacy_eagle adapter replaces simulator servo supply with REGULATOR 5 V, through S1",
                        "legacy_rail_nominal_v": 5,
                        "candidate_driver_input_range_v": profile[
                            "driver_input_range_v"
                        ],
                        "conditional_on": "Exact installed motor is MG4005-i10v2 / DF40V2 as in catalogue page 51",
                        "action": "Define a separate compatible motor supply; do not energize this candidate from the 5 V logic regulator",
                    }
                )
    return {
        "status": "not_validated_for_physical_hardware",
        "selected_motor": None,
        "findings": findings,
        "old_geometry_passes_are_not_electrical_certificates": True,
    }


def html_report(out, catalog, context, reports, audit):
    esc = lambda s: html.escape(str(s))
    rows = "".join(
        f"<tr><td>{esc(k)}</td><td>{esc(v['configured_model'])}</td><td>尚未确定</td></tr>"
        for k, v in context["motor_candidates"].items()
    )
    labels = {
        "hardware_scenario_not_selected": "尚未选定硬件及通信版本",
        "motor_supply_voltage_range_missing": "电机电源实际电压范围待确认",
        "component_model_revision_unconfirmed": "零件型号、版本或来源待确认",
        "component_not_in_assembly": "零件尚未加入装配",
        "mounting_side_or_evidence_missing": "固定位置或安装依据待补充",
        "physical_pin_or_cable_exit_missing": "真实针脚或插头出线点待登记",
        "wire_or_bundle_spec_missing": "线径、弯曲半径或线材依据待补充",
        "concept_servo_signal_is_not_a_registered_bus_terminal": "旧 SIG 舵机信号不能直接代替总线针脚",
        "legacy_5v_servo_rail_below_candidate_motor_input": "旧 5 V 电机电源不满足该候选驱动的输入范围",
    }
    blocks = ""
    for bus, data in reports.items():
        counts = Counter(x["reason"] for x in data["pending"])
        items = "".join(
            f"<li>{esc(labels.get(k, k))}：{v}</li>" for k, v in sorted(counts.items())
        )
        plan = read(out / bus / "logical_plan.json")
        connections = "".join(
            "<tr><td>"
            + esc(c["id"])
            + "</td><td>"
            + esc(c["source"]["instance_id"] + " / " + c["source"]["port_id"])
            + "</td><td>"
            + esc(c["target"]["instance_id"] + " / " + c["target"]["port_id"])
            + "</td></tr>"
            for c in plan["cables"]
        )
        detail = (
            "<details><summary>展开逻辑接线表（针脚仍需实物确认）</summary>"
            "<p>差分对在表中显示两根导体，在外部线束数量中合计为一束；总线主干内部走向尚未设计。</p>"
            "<table><tr><th>连接</th><th>起点端子</th><th>终点端子</th></tr>"
            + connections
            + "</table></details>"
        )
        blocks += (
            f"<section><h2>{bus} 候选方案</h2><p>外部线缆／线束连接：{data['required_cables']}；"
            f"端点与线材参数已解析：{data['resolved_cables']}。"
            f"允许送入布线：{'是（仅静态几何）' if data['can_route'] else '否，仍有缺项'}。</p>"
            f'<p><a href="{bus}/logical_plan.json">接线方案</a> · '
            f'<a href="{bus}/registration_template.json">接口登记表</a> · '
            f'<a href="{bus}/carrier_requirements.json">新增零件与载板需求</a> · '
            f'<a href="{bus}/readiness.json">逐项检查结果</a></p><ul>{items}</ul>{detail}</section>'
        )
    findings = "".join(
        "<li>"
        + esc(labels.get(x["reason"], x["reason"]))
        + "："
        + esc(x["instance_id"])
        + "</li>"
        for x in audit["findings"]
    )
    sources = "".join(
        '<li><a href="'
        + esc(v["url"])
        + '">'
        + esc(k)
        + "</a> — "
        + esc(v["scope"])
        + "</li>"
        for k, v in catalog["sources"].items()
    )
    (out / "index.html").write_text(
        """<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>实际硬件接线检查</title>
<style>body{font:16px/1.65 system-ui;max-width:1060px;margin:36px auto;padding:0 24px;background:#f5f7fa;color:#203247}h1{font-size:28px}section,aside{background:white;border:1px solid #d9e2ec;border-radius:12px;padding:20px;margin:18px 0}aside{border-left:5px solid #c4831a}table{border-collapse:collapse;width:100%}td,th{padding:9px;text-align:left;border-bottom:1px solid #d9e2ec}a{color:#1464a0}</style>
<h1>实际硬件接线检查</h1><aside>电机型号／版本尚未确定；以下是候选设计，不是可以照接通电的最终线束。原三线舵机布线结果保留，不能作为新候选电机的电气验证。</aside>
<h2>已有资料与缺口</h2><p>从官方 Uno PCB 提取 32 个独立排针焊盘，包含 3 个独立 GND 位置；焊盘 XY 不等于插头外部出线点。电机已有候选参数及安装图，物理针脚顺序、插头型号、固定侧仍须确认。</p><table><tr><th>电路实例</th><th>原配置候选</th><th>实际选择</th></tr>"""
        + rows
        + "</table>"
        + blocks
        + "<section><h2>旧接法与候选型号的差异</h2><p>若采用资料中的 MG4005V2 驱动，其输入范围为 7.4–32 V；原 5 V 舵机电源不能直接沿用。CAN 候选需要控制器及收发器；RS485 候选需要收发器。两种方案都需要实际供电、分线和接口零件。</p><ul>"
        + findings
        + "</ul></section><section><h2>来源</h2><ul>"
        + sources
        + "</ul></section></html>"
    )

    if context.get("controller_profile") == "raspberry_pi_pico2":
        page = (out / "index.html").read_text()
        page = page.replace(
            "从官方 Uno PCB 提取 32 个独立排针焊盘，包含 3 个独立 GND 位置；焊盘 XY 不等于插头外部出线点。",
            "主控已迁移为无排针 Raspberry Pi Pico 2。按官方针脚号记录 VSYS、3V3、GND 与候选 GPIO 接线；真实焊线出口和插头坐标仍待确认。",
        )
        (out / "index.html").write_text(page)


def run(source, output, registration_path=None):
    source, out = Path(source).resolve(), Path(output).resolve()
    if source == out or out.is_relative_to(source) or source.is_relative_to(out):
        raise ValueError("Source and output must be separate non-nested directories")
    if out.exists() and any(out.iterdir()):
        raise ValueError(
            "Use a new/empty output directory; existing results are never overwritten"
        )
    original = snapshot(source)
    context, catalog = assembly_context(source), build_catalog()
    registration = read(registration_path) if registration_path else None
    if registration and registration.get("scenario") not in {"CAN", "RS485"}:
        raise ValueError("Registration scenario must be CAN or RS485")
    out.mkdir(parents=True, exist_ok=True)
    dump(out / "source_snapshot.json", original)
    dump(out / "interface_catalog.json", catalog)
    dump(out / "motor_candidates.json", context["motor_candidates"])
    # Source-authored pad XY and assembly transform: z=0 is a coordinate datum,
    # NOT a guessed physical terminal height or certified cable escape direction.
    pose = np.asarray(context["poses"][context["controller"]])
    pads = catalog["profiles"][context["controller_profile"]]["physical_pads"]
    registered_pads = {
        name: {
            **p,
            "assembly_pcb_datum_mm": (pose @ [*p["pad_xy_mm"], 0, 1])[:3].tolist()
            if p.get("pad_xy_mm") is not None
            else None,
            "cable_exit_status": "pending_connector_height_and_mating_plug",
        }
        for name, p in pads.items()
    }
    dump(
        out / "controller_pad_references.json",
        {"instance_id": context["controller"], "pads": registered_pads},
    )
    audit = (
        legacy_audit(context, catalog)
        if context["controller_profile"] == "arduino_uno_rev3e"
        else {"findings": [], "status": "not_applicable_pico_has_no_legacy_uno_wiring"}
    )
    dump(out / "legacy_compatibility_audit.json", audit)
    reports = {}
    for bus in ("CAN", "RS485"):
        folder = out / bus
        folder.mkdir()
        plan = scenario(
            bus,
            context["motor_instances"],
            context["controller"],
            context["regulator"],
            catalog,
            controller_profile=context["controller_profile"],
        )
        example = template(plan, context, catalog)
        supplied = (
            registration
            if registration and registration["scenario"] == bus
            else example
        )
        result = compile_connections(plan, supplied, context)
        dump(folder / "logical_plan.json", plan)
        dump(folder / "electrical_review.json", plan["review"])
        dump(folder / "registration_template.json", example)
        dump(folder / "registration_used.json", supplied)
        dump(
            folder / "readiness.json",
            {k: v for k, v in result.items() if k != "connections"},
        )
        needs = [
            {
                "instance_id": c["id"],
                "requirement": c["model"],
                "needs_new_placement": c["id"] not in context["placed_instances"],
                "selected_model": supplied.get("components", {})
                .get(c["id"], {})
                .get("model"),
                "dimensions_mm": None,
                "mounting_holes": None,
                "ports": list(c["ports"]),
                "status": "select_part_and_register_geometry_before_carrier_generation",
            }
            for c in plan["components"]
            if c["id"] not in context["placed_instances"]
        ]
        dump(
            folder / "carrier_requirements.json",
            {"schema_version": 1, "units": "mm", "components": needs},
        )
        if result["can_route"]:
            dump(folder / "connections.json", result["connections"])
        contract = {
            "schema_version": 1,
            "pipeline": "physical_wiring_v1",
            "scenario": bus,
            "source": str(source),
            "can_route": result["can_route"],
            "connections_sha256": sha256(folder / "connections.json")
            if result["can_route"]
            else None,
            "readiness_sha256": sha256(folder / "readiness.json"),
            "source_snapshot_sha256": sha256(out / "source_snapshot.json"),
            "scope": "Validated data contract for static geometry; not electrical/manufacturing release",
        }
        dump(folder / "routing_contract.json", contract)
        reports[bus] = result
    unchanged = snapshot(source) == original
    if not unchanged:
        raise RuntimeError("Read-only source assembly changed")
    html_report(out, catalog, context, reports, audit)
    summary = {
        "pipeline": "physical_wiring_v1",
        "status": "pending_hardware_selection_and_registration",
        "source": str(source),
        "source_unchanged": True,
        "complete_assemblable": False,
        "selected_scenario": registration["scenario"]
        if registration and registration.get("selection_confirmed") is True
        else None,
        "controller_pad_count": len(pads),
        "independent_ground_pad_count": sum(
            p["board_signal"] == "GND" for p in pads.values()
        ),
        "scenarios": {
            bus: {
                "can_route": r["can_route"],
                "required_cables": r["required_cables"],
                "resolved_cables": r["resolved_cables"],
                "errors": len(r["errors"]),
                "pending": len(r["pending"]),
            }
            for bus, r in reports.items()
        },
    }
    if any(r["can_route"] for r in reports.values()):
        summary["status"] = "static_routing_inputs_ready_electrical_release_pending"
    dump(out / "manifest.json", summary)
    return summary


def load_routing_contract(folder, source):
    folder, source = Path(folder).resolve(), Path(source).resolve()
    contract = read(folder / "routing_contract.json")
    if (
        contract.get("schema_version") != 1
        or contract.get("pipeline") != "physical_wiring_v1"
        or contract.get("can_route") is not True
    ):
        raise ValueError(
            "Physical wiring input is incomplete; see readiness.json before routing"
        )
    if Path(contract["source"]).resolve() != source:
        raise ValueError("Physical wiring contract is for another assembly")
    if (
        sha256(folder.parent / "source_snapshot.json")
        != contract["source_snapshot_sha256"]
    ):
        raise ValueError("Assembly fingerprint record changed")
    if snapshot(source) != read(folder.parent / "source_snapshot.json"):
        raise ValueError("Assembly changed since physical interface registration")
    if sha256(folder / "readiness.json") != contract["readiness_sha256"]:
        raise ValueError("Physical readiness report changed")
    readiness = read(folder / "readiness.json")
    if not readiness["can_route"] or readiness["errors"] or readiness["pending"]:
        raise ValueError("Physical input has unresolved blockers")
    if sha256(folder / "connections.json") != contract["connections_sha256"]:
        raise ValueError("Registered physical connections changed")
    from shell_channels.assembly import validate_connections

    data = validate_connections(read(folder / "connections.json"))
    if len(data["cables"]) != readiness["required_cables"]:
        raise ValueError("Not all required physical cables are represented")
    return folder / "connections.json", max(
        c["min_bend_radius_mm"] for c in data["cables"]
    )
