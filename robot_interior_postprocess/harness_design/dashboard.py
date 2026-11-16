"""Readable dossier for the delivered geometry; no success hidden behind unit tests."""

import html
import os
from pathlib import Path

from physical_wiring.catalog import read


def write(out):
    out = Path(out)
    report = read(out / "routing_report.json")
    routes = read(out / "routes.json")["routes"]
    source = Path(report["source"])
    esc = lambda value: html.escape(str(value))
    rows = "".join(
        "<tr><td>"
        + esc(r["id"])
        + "</td><td>"
        + esc(r["source"]["instance_id"])
        + " → "
        + esc(r["target"]["instance_id"])
        + "</td><td>"
        + str(len(r["conductors"]))
        + "</td><td>"
        + (
            "内壁槽道"
            if r["routing_method"] == "wall_channel"
            else "空间走线；固定件待设计"
        )
        + "</td><td>"
        + f"{r['length_mm']:.1f}"
        + "</td></tr>"
        for r in routes
    )
    failures = (
        "".join(
            "<li>" + esc(f["id"]) + "：" + esc(f["reason"]) + "</li>"
            for f in report["unrouted"]
        )
        or "<li>外部线束候选没有缺失路径。</li>"
    )
    channels = sum(r["routing_method"] == "wall_channel" for r in routes)
    assembly = esc(os.path.relpath(source / "process.html", out))
    conductor_count = sum(len(r["conductors"]) for r in routes)
    page = """<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>机器人完整布线候选</title>
<style>body{font:16px/1.6 system-ui;margin:24px;color:#203040}iframe{width:100%;height:740px;border:0}table{border-collapse:collapse;width:100%}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:left}aside{background:#fff1d5;padding:18px}a{color:#1468a0}nav{display:flex;gap:16px;flex-wrap:wrap}.table{overflow-x:auto}</style><h1>机器人完整布线候选</h1>"""
    page += f"<p>{report['scenario']} 实验方案：<strong>{len(routes)}/{report['required_cables']} 条外部线束有候选路径</strong>，对应 {conductor_count} 条逻辑导体连接；其中 {channels} 条采用内壁槽道，{len(routes) - channels} 条保留空间走线。</p>"
    page += "<aside>候选路径覆盖不等于真实硬件布线完成。电机与接口型号未确定，图中接头出口及新增模块尺寸属于明确的设计假设。真实针脚至这些出口的连接、总线线束内部主干、新模块载板、盖条／线夹和运动验证仍未完成。不能直接据此制作并通电。</aside>"
    page += f'<nav><a href="{assembly}">重新生成装配的过程</a><a href="../physical_wiring/index.html">候选电路与真实接口缺项</a><a href="routing_report.json">完整检查结果</a><a href="wire_schedule.csv">线缆表</a><a href="channel_options.json">开槽与回退原因</a></nav>'
    page += (
        '<h2>三维布线</h2><p>拖动旋转；右侧可逐个显示或隐藏零件、线束。展开滑杆仅用于观察，不代表机器人运动。</p><iframe src="assembly_view.html" title="全部线束的三维候选"></iframe><h2>全部外部线束</h2><p>长度为候选路径几何长度，不含未确定的端子、尾线和装配余量。</p><div class="table"><table><tr><th>线束标识</th><th>起点 → 终点</th><th>导体连接数</th><th>走线方式</th><th>几何长度 mm</th></tr>'
        + rows
        + "</table></div><h2>未布通项</h2><ul>"
        + failures
        + "</ul>"
    )
    page += '<h2>设计输入与待制造内容</h2><nav><a href="design_assumptions.json">全部设计假设</a><a href="component_proposals.json">模块空间与位置候选</a><a href="terminal_proposals.json">接头出口候选</a><a href="retention_plan.json">线槽／线夹固定需求</a><a href="routes.json">全部路径坐标</a></nav></html>'
    (out / "index.html").write_text(page)
