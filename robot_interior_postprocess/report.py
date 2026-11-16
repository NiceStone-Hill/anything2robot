import json

import numpy as np
import trimesh
from common import dump
from solid_query import SolidQuery
from voxel_tool.presentation.layout_canvas import export_layout_canvas


def joint_payload(scene):
    return json.dumps(
        [
            {
                "name": m["link"],
                "center": np.asarray(m["center_mm"]).tolist(),
                "axis": np.asarray(m["axis"]).tolist(),
            }
            for m in scene["motors"]
        ],
        ensure_ascii=True,
    ).replace("<", "\\u003c")


def component_mesh(p, schema, out=None):
    spec = next(c for c in schema["components"] if c["id"] == p.component_id)
    reference = spec["visualization"].get("mesh")
    mesh = (
        trimesh.load(out / reference, force="mesh")
        if reference and out is not None
        else trimesh.creation.box(extents=spec["visualization"]["proxy_size_mm"])
    )
    transform = np.eye(4)
    if p.rotation_matrix_local_to_shell is not None:
        transform[:3, :3] = p.rotation_matrix_local_to_shell
    transform[:3, 3] = p.center_mm
    mesh.apply_transform(transform)
    return mesh


def export_report(config, scene, hollows, grid, placement, regions, routing, out):
    from power_report import export_power_report

    export_power_report(config, placement, routing, out)
    schema = json.loads((out / "components.json").read_text())
    visual = trimesh.Scene()
    for name, mesh in hollows.items():
        mesh.visual.face_colors = [175, 190, 205, 65]
        visual.add_geometry(mesh, node_name="shell_" + name)
    body = trimesh.util.concatenate(list(hollows.values()))
    body.export(out / "meshes/assembled_hollow_shell_mm.stl")
    joint_meshes = []
    for motor in scene["motors"]:
        axis = np.asarray(motor["axis"])
        center = np.asarray(motor["center_mm"])
        marker = trimesh.util.concatenate(
            [
                trimesh.creation.icosphere(subdivisions=1, radius=4).apply_translation(
                    center
                ),
                trimesh.creation.cylinder(
                    radius=1.5,
                    segment=np.vstack([center - axis * 30, center + axis * 30]),
                ),
            ]
        )
        path = out / "meshes" / f"joint_{motor['link']}.stl"
        marker.export(path)
        joint_meshes.append(path)
        marker.visual.face_colors = [20, 220, 255, 255]
        visual.add_geometry(marker, node_name="joint_" + motor["link"])
    component_checks = []
    for p in placement.placements:
        mesh = component_mesh(p, schema, out)
        mesh.export(out / "meshes" / f"{p.instance_id}_proxy_mm.stl")
        mesh.visual.face_colors = np.array([*p.color, 1]) * 255
        visual.add_geometry(mesh, node_name=p.instance_id)
        if p.component_id not in config["motor_mapping"]:
            overlaps = {}
            for name, shell in hollows.items():
                if np.any(mesh.bounds[1] < shell.bounds[0]) or np.any(
                    mesh.bounds[0] > shell.bounds[1]
                ):
                    continue
                intersection = trimesh.boolean.intersection(
                    [mesh, shell], engine="manifold"
                )
                volume = 0.0 if intersection.is_empty else float(intersection.volume)
                if not np.isfinite(volume):
                    raise ValueError("Non-finite component-shell intersection volume")
                overlaps[name] = float(max(0, volume))
            component_checks.append(
                {
                    "component": p.instance_id,
                    "shell_overlap_mm3": overlaps,
                    "clear": all(v < 1e-3 for v in overlaps.values()),
                }
            )
    # Check the actual exported solid geometry at dense route samples too, beyond grid labels.
    route_checks = []
    queries = {name: SolidQuery(mesh) for name, mesh in hollows.items()}
    for route in routing.routes:
        samples = []
        for a, b in zip(route.points_mm[:-1], route.points_mm[1:]):
            a = np.array(a)
            b = np.array(b)
            n = max(2, int(np.ceil(np.linalg.norm(b - a) / (grid.pitch / 3))) + 1)
            samples.extend(np.linspace(a, b, n))
        samples = np.asarray(samples)
        hits = np.zeros(len(samples), bool)
        for name, shell in hollows.items():
            eligible = np.where(
                np.all(samples >= shell.bounds[0], axis=1)
                & np.all(samples <= shell.bounds[1], axis=1)
            )[0]
            for start in range(0, len(eligible), 64):
                ids = eligible[start : start + 64]
                hits[ids] |= queries[name].contains(samples[ids])
        route_checks.append(
            {
                "branch": route.branch_id,
                "samples": len(samples),
                "shell_interior_hits": int(hits.sum()),
            }
        )
        for a, b in zip(route.points_mm[:-1], route.points_mm[1:]):
            if np.linalg.norm(np.asarray(a) - b) < 1e-6:
                continue
            tube = trimesh.creation.cylinder(
                radius=config["wire_radius_mm"], sections=8, segment=[a, b]
            )
            tube.visual.face_colors = np.array([*route.color, 1]) * 255
            visual.add_geometry(
                tube, node_name=route.branch_id + "_" + str(len(visual.geometry))
            )
    visual.export(out / "assembly.glb")
    from urdf_export import export_urdf

    export_urdf(config, scene, placement, out)
    validation = {
        "component_checks": component_checks,
        "route_solid_samples": route_checks,
        "all_components_clear": all(c["clear"] for c in component_checks),
        "all_routes_clear_of_shell": all(
            r["shell_interior_hits"] == 0 for r in route_checks
        ),
        "scope": "Static proxy geometry, sampled centerlines and Boolean component-shell intersections; not manufacturing or motion certification",
    }
    dump(out / "geometry_validation.json", validation)
    # Only the display meshes are simplified. Validation and manufacturing files
    # retain their full-resolution geometry.
    preview_parts = []
    for mesh in hollows.values():
        compact = SolidQuery(mesh).solid.simplify(0.8).to_mesh()
        preview_parts.append(
            trimesh.Trimesh(
                compact.vert_properties[:, :3], compact.tri_verts, process=False
            )
        )
    preview = trimesh.util.concatenate(preview_parts)
    preview.export(out / "meshes/assembled_preview_only_mm.stl")
    export_layout_canvas(
        grid,
        placement,
        routing,
        out / "layout_canvas.html",
        component_schema_path=out / "components.json",
        shell_mesh_path=out / "meshes/assembled_preview_only_mm.stl",
        reference_mesh_paths=tuple(joint_meshes),
        show_shell=True,
        show_interior=False,
        show_openings=False,
        show_axes=True,
        max_mesh_faces=60000,
        title=config.get("display_name", "Robot layout"),
    )
    joint_overlay = """  if (controls.referenceMeshes.checked) {
    const joints = __JOINTS__;
    ctx.font = "bold 13px system-ui";
    for (const joint of joints) {
      const a=joint.center.map((v,i)=>v-joint.axis[i]*35);
      const b=joint.center.map((v,i)=>v+joint.axis[i]*35);
      line(project(a),project(b),"#008da8",3);
      dots([joint.center],"#006c86",5);
      const p=project(b); ctx.fillStyle="#005c75";
      ctx.fillText("Joint: "+joint.name,p[0]+5,p[1]-5);
    }
  }
""".replace("__JOINTS__", joint_payload(scene))
    viewer = out / "layout_canvas.html"
    viewer.write_text(
        viewer.read_text()
        .replace(
            "controls.candidates.checked = scene.show_candidates;",
            "controls.surfaceApertures.checked = false;\ncontrols.candidates.checked = scene.show_candidates;",
        )
        .replace(
            'id="surfaceApertures" type="checkbox" checked',
            'id="surfaceApertures" type="checkbox"',
        )
        .replace(
            "meshTriangles(item.mesh,item.color,0.16,faceStride)",
            "meshTriangles(item.mesh,[0.02,0.65,0.85],0.85,faceStride)",
        )
        .replace("Reference assembly meshes", "Joint markers")
        .replace(
            "  if (controls.axes.checked) {",
            joint_overlay + "  if (controls.axes.checked) {",
        )
        .replace("${meshCount} CAD meshes", "${meshCount} motor-envelope meshes")
    )
    from cad_models import ASSETS, assembly_preview

    if (ASSETS / "converted/models.json").is_file():
        assembly_preview(out, config)
    write_viewer_page(out, placement, routing, validation)
    return validation


def write_viewer_page(out, placement, routing, validation):
    """Keep the landing page compact; detailed checks remain in downloadable JSON."""
    status = (
        "Concept layout ready"
        if not placement.unplaced
        and not routing.unrouted
        and validation["all_components_clear"]
        and validation["all_routes_clear_of_shell"]
        else "Partial layout — see report"
    )
    page = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Robot layout</title>
<style>*{box-sizing:border-box}body{margin:0;font:13px system-ui;background:#f0f3f7;color:#192331;height:100dvh;display:flex;flex-direction:column}header{min-height:42px;padding:10px 16px;background:#15263d;color:white;display:flex;align-items:center;gap:18px;flex-wrap:wrap}header span{opacity:.75}nav{margin-left:auto;display:flex;gap:16px}a{color:#b8deff;text-decoration:none}a:hover{text-decoration:underline}iframe{border:0;width:100%;flex:1;min-height:0}</style>
<header><b>Robot layout</b><span>__STATUS__ · __COUNT__ routes</span><nav>
<a href="assembly.glb" download>GLB</a><a href="placement.json">Placement</a>
<a href="routing.json">Routing</a><a href="manifest.json">Report</a>
__CAD_LINKS__
</nav></header><iframe title="Interactive robot layout" src="layout_canvas.html"></iframe></html>"""
    for a, b in {
        "__STATUS__": status,
        "__COUNT__": str(len(routing.routes)),
        "__CAD_LINKS__": '<a href="cad_preview.html">CAD layout</a><a href="cad_models.html">Models</a>'
        if (out / "cad_preview.html").is_file()
        else "",
    }.items():
        page = page.replace(a, b)
    if (out / "cad_preview.html").is_file():
        page = page.replace('src="layout_canvas.html"', 'src="cad_layout_canvas.html"')
        page = page.replace('href="assembly.glb"', 'href="cad_assembly.glb"')
        page = page.replace('<a href="placement.json">', '<a href="layout_canvas.html">Proxy layout</a><a href="placement.json">')
    (out / "index.html").write_text(page)
