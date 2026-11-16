"""Human-readable shell fastener procurement and outside-loading instructions."""

import csv
from collections import Counter
from html import escape


def export_access_guide(out, closures):
    records = [
        (name, joint)
        for name, c in closures.items()
        for joint in c["report"]["connections"]
    ]
    fields = [
        "id",
        "link",
        "designation",
        "quantity",
        "port_diameter_mm",
        "mouth_to_seat_mm",
        "mouth_world_mm",
        "insertion_direction_world",
        "matching_pattern_position",
    ]
    with (out / "shell_fasteners.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for name, r in records:
            a = r["access"]
            writer.writerow(
                dict(
                    zip(
                        fields,
                        [
                            r["id"],
                            name,
                            r["fastener_spec"]["designation"],
                            1,
                            a["port_diameter_mm"],
                            round(a["mouth_to_seat_mm"], 3),
                            a["mouth_world_mm"],
                            a["insertion_direction_world"],
                            r.get("shared_pattern_position", ""),
                        ],
                    )
                )
            )
    quantities = Counter(r["fastener_spec"]["designation"] for _, r in records)
    with (out / "shell_fasteners_summary.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["designation", "quantity", "connection_ids"])
        for designation, count in sorted(quantities.items()):
            writer.writerow(
                [
                    designation,
                    count,
                    "; ".join(
                        r["id"]
                        for _, r in records
                        if r["fastener_spec"]["designation"] == designation
                    ),
                ]
            )
    sections = []
    for name, r in records:
        a, spec = r["access"], r["fastener_spec"]
        number = r["id"].rsplit("_", 1)[-1]
        xyz = ", ".join(f"{v:.2f}" for v in a["mouth_world_mm"])
        direction = ", ".join(f"{v:g}" for v in a["insertion_direction_world"])
        sections.append(f'''<article id="{escape(r["id"])}"><h3>{escape(name)} / Screw {number}</h3>
<p><b>{escape(spec["designation"])}</b>, quantity 1; under-head length {r["screw_length_mm"]:g} mm; 2.5 mm hex drive.</p>
<p>Through loading port diameter {a["port_diameter_mm"]:g} mm; mouth-to-seat depth {a["mouth_to_seat_mm"]:.2f} mm.
World loading-mouth coordinates ({xyz}) mm; insertion direction ({direction}).</p>
<ol><li>Install threaded inserts while the shell is open. Install carriers, connect the harness, then close the cover along the locating spigots.</li>
<li>Hold the screw on a straight 2.5 mm hex driver with a magnetic or holding tip. Feed it through the marked external port along the axis until its head reaches the bearing seat.</li>
<li>Keep the tool coaxial and tighten into the insert. Maximum driver shaft diameter: {a["max_driver_shaft_diameter_mm"]:g} mm; effective straight shaft length: {a["driver_shaft_length_mm"]:g} mm.
The handle envelope is checked at a maximum diameter of {a["max_driver_handle_diameter_mm"]:g} mm and length of {a["max_driver_handle_length_mm"]:g} mm. Actual grip, torque and fit require a physical trial.</li></ol></article>''')
    html = """<!doctype html><html lang="en"><meta charset="utf-8"><title>Shell fastener specifications and loading paths</title>
<style>body{max-width:1000px;margin:24px auto;padding:16px;font:16px system-ui;color:#203140}article{border-top:1px solid #bbc;padding:12px 0}p,li{line-height:1.7}a{color:#16708c}svg{width:100%;max-height:250px;background:#f4f6f8}</style>
<h1>Shell fastener specifications and loading paths</h1><p><a href="index.html">3D viewer</a> · <a href="shell_fasteners.csv">Per-hole list</a> · <a href="shell_fasteners_summary.csv">Specifications and quantities</a> · <a href="shell_fastener_map.png">Hole map</a></p>
<p>Select a link in the 3D viewer and click Align to screw entrance. Cyan markers show outside loading directions. Similar outlines use jointly checked hole patterns; different-sized shells are not interchangeable parts.</p>
<svg viewBox="0 0 900 210" role="img" aria-label="Axial section schematic; not to scale"><defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0L8 4L0 8Z" fill="#168ca0"/></marker></defs>
<path d="M240 45H520V85H240ZM240 125H520V165H240" fill="#efc695"/>
<path d="M530 45H650V93H530ZM530 117H650V165H530" fill="#85bdcd"/>
<path d="M430 87H445V123H430ZM445 101H610V109H445" fill="#988147"/>
<path d="M100 105H405" stroke="#168ca0" stroke-width="4" marker-end="url(#arrow)"/>
<text x="55" y="65">Outside: screw and driver</text><text x="245" y="25">Loading tunnel</text>
<text x="355" y="195">Head seat</text><text x="510" y="25">Stop / locator</text><text x="590" y="195">Blind threaded insert</text>
<text x="680" y="105">Owner shell</text></svg>
<p>The schematic explains loading order. Use the per-hole coordinates, tunnel depths and screw lengths below. Prototype specification: ISO 4762 M3 socket head cap screws, 0.5 mm pitch, fully threaded steel, property class 12.9. Overall structural strength remains unverified.</p>"""
    (out / "shell_fastener_assembly.html").write_text(
        html + "".join(sections) + "</html>"
    )
