"""Convert downloaded CAD and display it without changing validated layout geometry."""

import argparse
import hashlib
import itertools
import json
import sys
import tempfile
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import trimesh
from common import dump, read

ASSETS = Path(__file__).parent / "assets/electronics"
MODELS = [
    (
        "arduino_uno_r3",
        "Arduino Uno",
        "arduino_uno_r3/arduino_uno_r3.FCStd",
        [0.1, 0.5, 0.7],
    ),
    (
        "battery_holder_8aa",
        "8 AA holder (candidate)",
        "battery_holder_8aa/battery_holder_8aa.step",
        [0.18, 0.20, 0.24],
    ),
    (
        "regulator_d24v50f5",
        "Pololu D24V50F5",
        "regulator_d24v50f5/d24v50f5.step",
        [0.15, 0.55, 0.25],
    ),
    (
        "slide_switch",
        "SPDT switch (candidate)",
        "slide_switch/slide_switch.step",
        [0.35, 0.40, 0.46],
    ),
    (
        "sg90_servo",
        "SG90 (simulator reference)",
        "sg90_servo/sg90_servo.stl",
        [0.15, 0.35, 0.85],
    ),
]


def cad_mesh(path, color):
    import gmsh

    gmsh.clear()
    gmsh.model.add(path.stem)
    gmsh.model.occ.importShapes(str(path))
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.35)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 1.6)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 18)
    gmsh.model.mesh.generate(2)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    order = np.argsort(tags)
    tags = tags[order]
    vertices = np.asarray(xyz).reshape(-1, 3)[order]
    types, _, node_tags = gmsh.model.mesh.getElements(2)
    faces = np.vstack(
        [
            np.searchsorted(tags, nodes).reshape(-1, 3)
            for kind, nodes in zip(types, node_tags)
            if kind == 2
        ]
    )
    mesh = trimesh.Trimesh(vertices, faces, process=True)
    mesh.visual.face_colors = np.r_[np.array(color) * 255, 255].astype(np.uint8)
    return mesh


def freecad_mesh(path):
    # FreeCAD saves placement in the BREP root location already. Applying the
    # Document.xml placement again would transform every board component twice.
    parts = []
    omitted = []
    with zipfile.ZipFile(path) as archive, tempfile.TemporaryDirectory() as folder:
        document = ET.fromstring(archive.read("Document.xml"))
        gui = ET.fromstring(archive.read("GuiDocument.xml"))
        providers = {
            p.get("name"): p for p in gui.findall("./ViewProviderData/ViewProvider")
        }
        for obj in document.findall("./ObjectData/Object"):
            shape = obj.find('./Properties/Property[@name="Shape"]/Part')
            if shape is None:
                continue
            name = obj.get("name")
            visible = providers[name].find(
                './Properties/Property[@name="Visibility"]/Bool'
            )
            if visible is not None and visible.get("value") == "false":
                continue
            target = Path(folder) / (Path(shape.get("file")).stem + ".brep")
            target.write_bytes(archive.read(shape.get("file")))
            color = (
                [0.10, 0.48, 0.65]
                if "pcb" in name
                else ([0.65, 0.67, 0.70] if "missblack" in name else [0.14, 0.16, 0.18])
            )
            try:
                parts.append(cad_mesh(target, color))
            except Exception as exc:  # noqa: BLE001 - record unsupported legacy BREP subobjects.
                omitted.append(
                    {"object": name, "shape": shape.get("file"), "error": repr(exc)}
                )
                print("  Unsupported source subobject:", name, flush=True)
    result = trimesh.util.concatenate(parts)
    result.metadata["omitted_source_objects"] = omitted
    return result


def convert_models():
    import gmsh

    destination = ASSETS / "converted"
    destination.mkdir(exist_ok=True)
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setNumber("General.NumThreads", 1)
    records = []
    try:
        for model_id, name, filename, color in MODELS:
            path = ASSETS / filename
            print("CAD conversion:", name, flush=True)
            if path.suffix.lower() == ".fcstd":
                mesh = freecad_mesh(path)
            elif path.suffix.lower() == ".stl":
                mesh = trimesh.load(path, force="mesh")
                mesh.visual.face_colors = np.r_[np.array(color) * 255, 255].astype(
                    np.uint8
                )
            else:
                mesh = cad_mesh(path, color)
            center = mesh.bounds.mean(axis=0)
            mesh.apply_translation(-center)
            mesh.export(destination / f"{model_id}.ply")
            mesh.export(destination / f"{model_id}.stl")
            trimesh.Scene(mesh).export(destination / f"{model_id}.glb")
            records.append(
                {
                    "id": model_id,
                    "name": name,
                    "source": filename,
                    "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "mesh": f"converted/{model_id}.ply",
                    "extents_mm": mesh.extents,
                    "faces": len(mesh.faces),
                    "source_center_mm": center,
                    "display_color": color,
                    "omitted_source_objects": mesh.metadata.get(
                        "omitted_source_objects", []
                    ),
                }
            )
            print(
                "  mm:",
                np.round(mesh.extents, 2),
                "triangles:",
                len(mesh.faces),
                flush=True,
            )
    finally:
        gmsh.finalize()
    dump(
        destination / "models.json",
        {
            "models": records,
            "converter": "gmsh " + gmsh.__version__,
            "note": "Source CAD geometry, centered without rescaling. Colors are illustrative. Not installed hardware certification.",
        },
    )


def make_display_meshes():
    """Preserve full conversions; use a smaller colored mesh for interactive views."""
    from scipy.spatial import cKDTree

    path = ASSETS / "converted/models.json"
    catalog = read(path)
    for record in catalog["models"]:
        source = record.get("full_mesh", record["mesh"])
        mesh = trimesh.load(ASSETS / source, force="mesh")
        if len(mesh.faces) > 40000:
            simplified = mesh.simplify_quadric_decimation(face_count=40000)
            nearest = cKDTree(mesh.triangles_center).query(simplified.triangles_center)[
                1
            ]
            simplified.visual.face_colors = mesh.visual.face_colors[nearest]
        else:
            simplified = mesh
        target = ASSETS / "converted" / (record["id"] + "_preview.ply")
        simplified.export(target)
        record.update(
            full_mesh=source,
            mesh=str(target.relative_to(ASSETS)),
            preview_faces=len(simplified.faces),
            preview_extents_mm=simplified.extents,
        )
    dump(path, catalog)


def color_viewer(path, models, focus_models=True):
    """Retain mesh colors and provide a focus selector for small components."""
    text = path.read_text()
    prefix = "const scene = "
    start = text.index(prefix) + len(prefix)
    end = text.index(";\n", start)
    data = json.loads(text[start:end])
    for component in data["components"]:
        mesh = models.get(component["id"])
        if mesh is not None and "mesh" in component:
            colors = mesh.visual.face_colors[:, :3] / 255
            if len(colors) == len(component["mesh"]["faces"]):
                component["mesh"]["colors"] = colors.tolist()
    text = text[:start] + json.dumps(data, separators=(",", ":")) + text[end:]
    text = text.replace(
        "const shaded=color.map(", "const shaded=(mesh.colors?.[i] || color).map("
    )
    text = text.replace("<style>", "<style>\n[hidden]{display:none!important}")
    text = text.replace(
        "ctx.fillStyle=triangle.color; ctx.fill();",
        "ctx.fillStyle=triangle.color; ctx.fill(); if(alpha>0.9){ctx.strokeStyle=triangle.color; ctx.lineWidth=0.5; ctx.stroke();}",
    )
    text = text.replace(
        "${meshCount} CAD meshes", str(len(models)) + " downloaded CAD models"
    )
    text = text.replace(
        "Interactive layout view · Canvas 2D renderer",
        "Downloaded CAD · mm · drag to rotate",
    )
    text = text.replace("const extent = Math.max", "let extent = Math.max")
    text = text.replace(
        '<label><input id="shell"',
        '<select id="cadFocus" style="width:100%;padding:8px;margin:8px 0"><option value="all">All models</option></select><label><input id="shell"',
    )
    extra = """
const allCadParts=scene.components.slice(), focus=document.getElementById('cadFocus');
for(const part of allCadParts){const option=document.createElement('option');option.value=part.id;option.textContent=part.name;focus.appendChild(option);}
focus.addEventListener('change',()=>{
 scene.components=focus.value==='all'?allCadParts:allCadParts.filter(p=>p.id===focus.value);
 const pts=scene.components.flatMap(p=>p.mesh?.vertices || [p.center]);
 const low=[0,1,2].map(i=>pts.reduce((v,p)=>Math.min(v,p[i]),Infinity));
 const high=[0,1,2].map(i=>pts.reduce((v,p)=>Math.max(v,p[i]),-Infinity));
 for(let i=0;i<3;i++)center[i]=(low[i]+high[i])/2;
 extent=Math.max(...high.map((v,i)=>v-low[i]),1)*1.2;zoom=1;panX=0;panY=0;draw();
});
focus.dispatchEvent(new Event('change'));
"""
    if focus_models:
        extra += "\nfor(const id of ['shell','referenceMeshes','interior','exterior','openings','surfaceApertures','routes','endpoints','candidates','proxies']){controls[id].closest('label').hidden=true;}\n"
    else:
        text = text.replace(
            '<select id="cadFocus" style="width:100%;padding:8px;margin:8px 0"><option value="all">All models</option></select>',
            "",
        )
        extra = """
controls.surfaceApertures.checked=false;
const powerButton=document.createElement('button');powerButton.textContent='Locate power';
document.querySelector('aside').prepend(powerButton);
const powerParts=scene.components.filter(p=>p.id==='BATTERY_1'||p.id==='REGULATOR_1');
const fullComponents=scene.components.slice(), fullRoutes=scene.routes.slice();
const originalCenter=center.slice(), originalExtent=extent;let powerOnly=false;
powerButton.addEventListener('click',()=>{
 powerOnly=!powerOnly;powerButton.textContent=powerOnly?'Show full layout':'Locate power';
 scene.components=powerOnly?powerParts:fullComponents;
 scene.routes=powerOnly?fullRoutes.filter(r=>['BATTERY_RAW','P1+','P1-','SERVO2_PWR'].includes(r.name)):fullRoutes;
 if(!powerOnly){for(let i=0;i<3;i++)center[i]=originalCenter[i];extent=originalExtent;zoom=1;panX=0;panY=0;controls.shell.checked=true;draw();return;}
 controls.shell.checked=false; controls.names.checked=true; controls.proxies.checked=true;
 controls.referenceMeshes.checked=false;
 const points=powerParts.flatMap(p=>p.mesh?.vertices||[p.center]);
 const lo=[0,1,2].map(i=>points.reduce((v,p)=>Math.min(v,p[i]),Infinity));
 const hi=[0,1,2].map(i=>points.reduce((v,p)=>Math.max(v,p[i]),-Infinity));
 for(let i=0;i<3;i++)center[i]=(lo[i]+hi[i])/2;
 extent=Math.max(...hi.map((v,i)=>v-lo[i]),1)*1.6;zoom=1;panX=0;panY=0;draw();
});draw();
if(new URLSearchParams(location.search).has('power'))powerButton.click();
"""
    text = text.replace("</script>", extra + "\n</script>")
    # Do not drop triangles as a display optimization: small pins would disappear.
    text = text.replace("Math.ceil(sceneFaceCount/5000)", "Math.ceil(0/5000)")
    path.write_text(text)


def gallery(output):
    from voxel_tool.domain.voxel import VoxelGrid
    from voxel_tool.presentation.layout_canvas import export_layout_canvas
    from voxel_tool.stages.placement.planner import Placement, PlacementResult
    from voxel_tool.stages.routing.router import RoutingResult

    catalog = read(ASSETS / "converted/models.json")["models"]
    catalog = [r for r in catalog if r["id"] != "sg90_servo"]
    from motor_cad import ASSETS as MOTOR_ASSETS

    if (MOTOR_ASSETS / "models.json").exists():
        catalog.extend({"id": r["model_id"], "name": r["model_id"],
                        "mesh": str((MOTOR_ASSETS / r["mesh"]).resolve()),
                        "display_color": [.35, .38, .43]}
                       for r in read(MOTOR_ASSETS / "models.json")["models"])
    specs, placements, meshes = [], [], {}
    positions = [[(i % 3) * 150, (i // 3) * 130, 0] for i in range(len(catalog))]
    for record, position in zip(catalog, positions):
        model_id = record["id"]
        mesh = trimesh.load(ASSETS / record["mesh"], force="mesh")
        meshes[model_id] = mesh
        specs.append(
            {
                "id": model_id,
                "visualization": {"mesh": str((ASSETS / record["mesh"]).resolve())},
            }
        )
        placements.append(
            Placement(
                model_id,
                model_id,
                record["name"],
                tuple(position),
                tuple(mesh.extents),
                (0, 0, 0),
                (1, 1, 1),
                tuple(record["display_color"]),
                ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
            )
        )
    schema = output.with_suffix(".components.json")
    dump(schema, {"components": specs})
    grid = VoxelGrid(
        np.zeros((2, 2, 2), bool),
        np.zeros((2, 2, 2), np.uint8),
        220,
        np.array([-40, -40, -20]),
    )
    export_layout_canvas(
        grid,
        PlacementResult(tuple(placements), (), 0),
        RoutingResult((), (), 0, 0),
        output,
        component_schema_path=schema,
        show_shell=False,
        show_interior=False,
        max_mesh_faces=1000000,
        title="Electronic CAD models",
    )
    color_viewer(output, meshes)


def fitted_rotation(extents, envelope):
    """Choose axis registration only. Never shrink a real model to fit a proxy."""
    candidates = []
    for permutation in itertools.permutations(range(3)):
        rotation = np.eye(3)[list(permutation)]
        if np.linalg.det(rotation) < 0:
            rotation[2] *= -1
        size = np.abs(rotation) @ extents
        candidates.append((float(np.linalg.norm(np.log(size / envelope))), rotation))
    return min(candidates, key=lambda x: x[0])[1]


def assembly_preview(out, config):
    from voxel_tool.presentation.layout_canvas import export_layout_canvas
    from voxel_tool.stages.placement.planner import load_placement_result
    from voxel_tool.stages.routing.router import load_routing_result
    from voxel_tool.storage.voxel import load_voxel_grid

    catalog = {m["id"]: m for m in read(ASSETS / "converted/models.json")["models"]}
    schema = read(out / "components.json")
    mapping = config.get(
        "cad_model_mapping",
        {
            "U1": "arduino_uno_r3",
            "BATTERY": "battery_holder_8aa",
            "REGULATOR": "regulator_d24v50f5",
            "S1": "slide_switch",
        },
    )
    meshes, registrations = {}, []
    (out / "cad_meshes").mkdir(exist_ok=True)
    for spec in schema["components"]:
        model_id = mapping.get(spec["id"])
        if model_id is None:
            continue
        record = catalog[model_id]
        mesh = trimesh.load(ASSETS / record["mesh"], force="mesh")
        envelope = np.array(spec["visualization"]["proxy_size_mm"])
        rotation = fitted_rotation(mesh.extents, envelope)
        mesh.apply_transform(
            np.block(
                [[rotation, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]
            )
        )
        path = out / "cad_meshes" / (spec["id"] + ".ply")
        mesh.export(path)
        spec["visualization"]["mesh"] = str(path.relative_to(out))
        meshes[spec["id"] + "_1"] = mesh
        registrations.append(
            {
                "component": spec["id"],
                "model": model_id,
                "size_mm": mesh.extents,
                "proxy_size_mm": envelope,
                "exceeds_proxy": bool(np.any(mesh.extents > envelope + 0.01)),
                "axis_registration": rotation,
                "scale": 1,
            }
        )
    from motor_cad import add_motor_previews

    motor_meshes, motor_records = add_motor_previews(
        schema, read(out / "mechanical_scene.json"), config, out
    )
    meshes.update(motor_meshes)
    registrations.extend(motor_records)
    dump(out / "cad_components.json", schema)
    dump(
        out / "cad_registration.json",
        {
            "registration": registrations,
            "validation": "Visual reference only. Placement, routing, connector locations and collisions have NOT been revalidated for these CAD candidates.",
        },
    )
    canvas = out / "cad_layout_canvas.html"
    export_layout_canvas(
        load_voxel_grid(out / "planning_domain.npz"),
        load_placement_result(out / "placement.json"),
        load_routing_result(out / "routing.json"),
        canvas,
        component_schema_path=out / "cad_components.json",
        shell_mesh_path=out / "meshes/assembled_preview_only_mm.stl",
        reference_mesh_paths=tuple((out / "meshes").glob("joint_*.stl")),
        show_shell=True,
        show_interior=False,
        show_openings=False,
        max_mesh_faces=1000000,
        title="CAD reference assembly",
    )
    color_viewer(canvas, meshes, focus_models=False)
    cad_scene = trimesh.Scene()
    for p in load_placement_result(out / "placement.json").placements:
        if p.instance_id in meshes:
            mesh = meshes[p.instance_id].copy()
            mesh.apply_translation(p.center_mm)
            cad_scene.add_geometry(mesh, node_name=p.instance_id)
    shell = trimesh.load(out / "meshes/assembled_preview_only_mm.stl", force="mesh")
    shell.visual.face_colors = [175, 190, 205, 65]
    cad_scene.add_geometry(shell, node_name="shell")
    cad_scene.export(out / "cad_assembly.glb")
    page = """<!doctype html><meta charset="utf-8"><title>CAD reference assembly</title>
<style>body{margin:0;font:13px system-ui;height:100vh;display:flex;flex-direction:column}header{padding:12px 16px;background:#15263d;color:#fff}a{color:#b8deff;margin-left:18px}iframe{border:0;width:100%;flex:1}</style>
<header>CAD reference · geometry not revalidated<a href="power_view.html">Power</a><a href="cad_models.html">Models</a><a href="index.html">Validated proxy layout</a><a href="cad_registration.json">Details</a></header><iframe src="cad_layout_canvas.html"></iframe>"""
    (out / "cad_preview.html").write_text(page)
    (out / "power_view.html").write_text(
        page.replace("CAD reference assembly", "Battery and regulator").replace(
            'src="cad_layout_canvas.html"', 'src="cad_layout_canvas.html?power=1"'
        )
    )
    gallery(out / "cad_models.html")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).parent / "outputs/layout_test_verified",
    )
    parser.add_argument("--convert", action="store_true")
    args = parser.parse_args()
    config = read(Path(__file__).parent / "configs/cactus.json")
    sys.path.insert(0, config["voxel_tool_source"])
    if args.convert or not (ASSETS / "converted/models.json").exists():
        convert_models()
    make_display_meshes()
    gallery(ASSETS / "viewer.html")
    out = args.output.resolve()
    assembly_preview(out, read(out / "manifest.json")["config"])
    print("MODELS:", ASSETS / "viewer.html")
    print("ASSEMBLY:", out / "cad_preview.html")


if __name__ == "__main__":
    main()
