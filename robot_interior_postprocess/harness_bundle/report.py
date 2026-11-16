"""Offline coloured wiring viewer with visible motors and unresolved connectors."""

import html
import json
import os
from pathlib import Path

import numpy as np
import trimesh
from hardware import as_mesh64
from physical_wiring.catalog import read
from PIL import Image, ImageDraw
from shell_partition.io import display_mesh

from .details import CATEGORIES


def preview(out, data):
    image = Image.new("RGB", (1400, 840), "#10222c")
    draw = ImageDraw.Draw(image)
    draw.text(
        (24, 18),
        "SMOOTH SHARED HARNESS | motor circuits shown; real connector tails remain unrouted",
        fill="#e4eef2",
    )
    cable_by_id = {c["id"]: c for c in data["cables"]}
    motor_cables = {key for m in data["motors"] for key in m["expected_cables"]}
    for col, (axes, title) in enumerate(
        [((0, 2), "ALL CIRCUITS / FRONT"), ((1, 2), "MOTOR CIRCUITS ONLY / SIDE")]
    ):
        allpts = np.concatenate([np.asarray(s["vertices"]) for s in data["shells"]])[
            :, axes
        ]
        lo, hi = allpts.min(axis=0), allpts.max(axis=0)
        scale = min(620 / (hi[0] - lo[0]), 650 / (hi[1] - lo[1]))
        center = (lo + hi) / 2

        def project(p, axes=axes, center=center, scale=scale, col=col):
            values = (np.asarray(p)[:, axes] - center) * scale
            return np.column_stack([350 + col * 700 + values[:, 0], 420 - values[:, 1]])

        def line(points, color, width):
            draw.line(
                [tuple(p) for p in points], fill=color, width=width, joint="curve"
            )

        draw.text((24 + col * 700, 52), title, fill="#bed5de")
        for shell in data["shells"]:
            points = project(shell["vertices"])
            edges = set()
            for a, b, c in shell["faces"]:
                edges.update(tuple(sorted(e)) for e in ((a, b), (b, c), (c, a)))
            for a, b in edges:
                line(points[[a, b]], "#243b46", 1)
        triangles = []
        depth_axis = next(i for i in range(3) if i not in axes)
        for model in [*data["actual_models"], *data["motors"]]:
            mesh = model["mesh"]
            vertices = np.asarray(mesh["vertices"])
            faces = np.asarray(mesh["faces"])
            projected = project(vertices)
            normals = np.cross(
                vertices[faces[:, 1]] - vertices[faces[:, 0]],
                vertices[faces[:, 2]] - vertices[faces[:, 0]],
            )
            norms = np.linalg.norm(normals, axis=1)
            norms[norms < 1e-12] = 1
            light = 0.55 + 0.45 * np.abs(normals[:, depth_axis] / norms)
            colors = np.asarray(mesh.get("face_colors", [[110, 135, 150]] * len(faces)))
            colors = np.clip(colors * light[:, None], 0, 255).astype(int)
            depths = vertices[faces, depth_axis].mean(axis=1)
            for i, face in enumerate(faces):
                triangles.append((depths[i], projected[face], tuple(colors[i])))
        for _, points, color in sorted(triangles, key=lambda t: t[0]):
            draw.polygon([tuple(p) for p in points], fill=color)
        for segment in data["segments"]:
            points = project(segment["points_mm"])
            width = max(3, segment["diameter_mm"] * scale)
            line(points, "#34505b", round(width))
            members = [
                key for key in segment["cables"] if col == 0 or key in motor_cables
            ]
            if not members:
                continue
            categories = list(
                dict.fromkeys(cable_by_id[key]["category"] for key in members)
            )
            for i, kind in enumerate(categories):
                line(
                    points,
                    CATEGORIES[kind]["color"],
                    max(1, round(width * (len(categories) - i) / len(categories))),
                )
        label_boxes = []
        for motor in data["motors"]:
            if motor["aggregation_exit_mm"] is None:
                continue
            p, q = project(
                [motor["aggregation_exit_mm"], motor["envelope_reference_mm"]]
            )
            length = np.linalg.norm(q - p)
            for distance in np.arange(0, length, 10):
                line(
                    [
                        p + (q - p) * distance / max(length, 1),
                        p + (q - p) * min(distance + 5, length) / max(length, 1),
                    ],
                    "#f7edbd",
                    2,
                )
            x, y = project([motor["mechanical_envelope"]["center_mm"]])[0]
            tx, ty = x + 6, y - 8
            for _ in range(len(label_boxes) + 1):
                hit = next(
                    (
                        r
                        for r in label_boxes
                        if tx < r[0] + 210
                        and tx + 210 > r[0]
                        and ty < r[1] + 32
                        and ty + 32 > r[1]
                    ),
                    None,
                )
                if hit is None:
                    break
                ty = hit[1] + 36
            label_boxes.append((tx, ty))
            line([(x, y), (tx, ty)], "#819ba8", 1)
            draw.text((tx, ty), motor["motor_id"], fill="#eef4f6")
            draw.text(
                (tx, ty + 16),
                f"tail pending ({motor['unresolved_gap_distance_mm']:.1f} mm gap)",
                fill="#f7dc96",
            )
            draw.ellipse((p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3), fill="white")
    for i, (key, entry) in enumerate(CATEGORIES.items()):
        draw.rectangle((24 + i * 230, 788, 38 + i * 230, 802), fill=entry["color"])
        draw.text((46 + i * 230, 788), key.upper(), fill="#cfdee5")
    draw.text(
        (24, 817),
        "Colour stripes identify members only, not internal wire positions. Dashed gaps are NOT routed cable geometry.",
        fill="#a6c0ca",
    )
    image.save(out / "preview.png")


def write(out, bundle, proposals):
    out = Path(out)
    report = read(out / "report.json")
    network = read(out / "network.json")
    audit = read(out / "motor_coverage.json")
    shells = []
    for key, value in bundle["parts"].items():
        mesh = display_mesh(as_mesh64(value))
        shells.append(
            {
                "name": key,
                "vertices": mesh.vertices.tolist(),
                "faces": mesh.faces.tolist(),
            }
        )
    from .appearance import load

    actual_models, motor_models, appearance = load(
        report["assembly_source"], proposals, audit["motors"]
    )
    from shell_channels.pipeline import dump

    dump(out / "appearance_report.json", appearance)
    motors = []
    for record in audit["motors"]:
        m = record["mechanical_envelope"]
        mesh = trimesh.creation.cylinder(
            radius=m["radius_mm"],
            segment=[m["axial_start_mm"], m["axial_end_mm"]],
            sections=20,
        )
        motors.append(
            {
                **record,
                "mesh": motor_models.get(
                    record["device"],
                    {"vertices": mesh.vertices.tolist(), "faces": mesh.faces.tolist()},
                ),
            }
        )
    data = {
        "shells": shells,
        "modules": proposals,
        "actual_models": actual_models,
        "missing_models": [
            r
            for r in appearance["records"]
            if r["status"] == "unselected_component_no_real_CAD"
        ],
        "segments": network["segments"],
        "ports": network["device_exits"],
        "cables": network["cables"],
        "motors": motors,
        "categories": CATEGORIES,
    }
    preview(out, data)
    esc = lambda v: html.escape(str(v))
    old = esc(os.path.relpath(Path(report["previous_candidate"]) / "index.html", out))
    summary = f'<p class="stats">{report["mapped_cables"]}/{report["required_cables"]} 条候选线路 · 电机线路 {audit["mapped_motor_cables"]}/{audit["expected_motor_cables"]} · {report["larger_bend_segments"]} 段增大圆角</p>'
    summary += f'<nav><a href="{old}">原逐线方案</a><a href="network.json">线路与共享段</a><a href="motor_coverage.json">电机接线核对</a><a href="smoothing_report.json">平滑检查</a><a href="report.json">全部检查结果</a><a href="preview.png">前视／侧视图</a></nav>'
    static = "通过" if report["corridor_static_geometry_passed"] else "存在未通过项"
    summary += f'<div class="note">通道静态检查：{static}。电机供电、回线和通信均按原电路逐项核对。线路目前连接至集中出口，尚未接到真实电机插头；电机型号／接口未确认，虚线仅标出这一缺口。分叉展开、固定件和运动验证仍待完成，不能据此直接制造通电。</div>'
    summary += f'<p>真实来源 CAD：{appearance["source_CAD_count"]} 个部件；未确定型号／缺少 CAD：{appearance["missing_CAD_count"]} 个。<a href="appearance_report.json">查看模型来源与缺项</a>。电机模型是当前配置的候选型号，外观更新不代表硬件确认。</p>'
    missing_labels = [
        r["id"]
        for r in appearance["records"]
        if r["status"] in {"unselected_component_no_real_CAD", "missing_source_CAD"}
    ]
    if missing_labels:
        summary += (
            "<p>等待选型／模型："
            + esc("、".join(missing_labels))
            + "。可勾选“未选型部件预留框”查看它们的位置。</p>"
        )
    plan = read(Path(report["previous_candidate"]) / "logical_plan.json")
    if plan.get("controller_profile") == "raspberry_pi_pico2":
        summary += (
            "<p><strong>主控：Raspberry Pi Pico 2（无排针）</strong>；当前为 "
            + esc(plan["protocol"])
            + " 候选。供电：稳压 5 V → 防倒灌保护 → VSYS / 脚39；地线：脚38。</p>"
        )
        summary += (
            "<p>候选信号："
            + esc(
                "；".join(
                    name + "=" + item["gpio"] + " / 脚" + str(item["physical_pin"])
                    for name, item in plan["controller_signal_assignment"].items()
                )
            )
            + "。接口模块须支持 3.3 V 逻辑，电流预算和固件待验证。</p>"
        )
    if report["local_bend_pending_segments"]:
        summary += (
            "<p>局部弯折仍待优化："
            + esc("、".join(report["local_bend_pending_segments"]))
            + "；静态不碰撞不代表弯曲半径合格。</p>"
        )
    motor_rows = ""
    for m in motors:
        circuits = "<br>".join(
            esc(CATEGORIES[c["category"]]["label"] + "：" + c["id"])
            for c in m["circuits"]
        )
        state = (
            "齐全"
            if m["logical_coverage_complete"]
            else "缺少 " + ", ".join(m["missing_cables"])
        )
        distance = (
            "未知"
            if m["unresolved_gap_distance_mm"] is None
            else f"{m['unresolved_gap_distance_mm']:.1f} mm"
        )
        motor_rows += f'<tr><td><button data-motor="{esc(m["device"])}">{esc(m["motor_id"])}</button><br>{esc(m["device"])}</td><td>{circuits}</td><td>{esc(state)}</td><td>未接通；包络参考间隔 {distance}</td></tr>'
    motor_table = (
        "<table><tr><th>电机（点击查看）</th><th>候选线路</th><th>逻辑覆盖</th><th>真实接头</th></tr>"
        + motor_rows
        + "</table>"
    )
    segment_table = "<table><tr><th>段号</th><th>线数</th><th>直径 mm</th><th>圆角半径 mm</th><th>平滑结果</th></tr>"
    for s in network["segments"]:
        status = {
            "larger_collision_checked_fillets": "增大圆角，碰撞复查通过",
            "already_straight": "原本为直线",
            "kept_previous_checked_geometry": "受空间限制，保留原路径",
        }[s["smoothing"]["status"]]
        segment_table += f"<tr><td>{esc(s['id'])}</td><td>{len(s['cables'])}</td><td>{s['diameter_mm']:.1f}</td><td>{s['assumed_bend_radius_mm']:.1f}</td><td>{status}</td></tr>"
    segment_table += "</table>"
    page = (Path(__file__).with_name("viewer.html")).read_text()
    for key, value in {
        "SUMMARY": summary,
        "MOTOR_TABLE": motor_table,
        "SEGMENT_TABLE": segment_table,
        "DATA": json.dumps(data, ensure_ascii=True).replace("<", "\\u003c"),
    }.items():
        page = page.replace("__" + key + "__", value)
    (out / "index.html").write_text(page)
