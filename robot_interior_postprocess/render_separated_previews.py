"""Export two isolated views while retaining the solved assembly coordinates."""

import argparse
import copy
import json
from pathlib import Path

import numpy as np


def export_views(out):
    manifest = json.loads((out / "manifest.json").read_text())
    template = (out / "cad_layout_canvas.html").read_text()
    prefix = "const scene = "
    start = template.index(prefix) + len(prefix)
    end = template.index(";\n", start)
    source = json.loads(template[start:end])
    fixed = {name + "_1" for name in manifest["config"]["motor_mapping"]}
    for filename, mechanical in [
        ("shell_actuators", True),
        ("other_components", False),
    ]:
        scene = copy.deepcopy(source)
        scene["components"] = [
            c for c in scene["components"] if (c["id"] in fixed) == mechanical
        ]
        scene["reference_meshes"] = []
        scene["routes"] = []
        scene["shell"] = []
        scene["interior"] = []
        scene["exterior"] = []
        scene["show_shell"] = mechanical
        if not mechanical:
            scene["shell_mesh"] = None
        points = [np.asarray(c["mesh"]["vertices"]) for c in scene["components"]]
        if mechanical:
            points.append(np.asarray(scene["shell_mesh"]["vertices"]))
        points = np.vstack(points)
        low, high = points.min(axis=0), points.max(axis=0)
        padding = (high - low).max() * 0.12
        scene["bounds"] = [(low - padding).tolist(), (high + padding).tolist()]
        scene["title"] = (
            "Shell + fixed actuators"
            if mechanical
            else "Electronic components — solved positions"
        )
        text = (
            template[:start] + json.dumps(scene, separators=(",", ":")) + template[end:]
        )
        css = """<style>
aside,#hint{display:none!important}#app{grid-template-columns:1fr!important;grid-template-rows:1fr!important;height:100vh!important}
#preview-title{position:absolute;top:24px;left:30px;z-index:2;font:600 24px system-ui;color:#20334b;pointer-events:none}
#preview-subtitle{position:absolute;top:62px;left:30px;z-index:2;font:15px system-ui;color:#627085;pointer-events:none}
</style>"""
        subtitle = (
            "Assigned motor CAD where available; millimetres; visual registration only"
            if mechanical
            else "Battery / controller / regulator / switch; original layout retained"
        )
        text = text.replace("</head>", css + "</head>")
        text = text.replace(
            '<div id="app">',
            '<div id="preview-title">'
            + scene["title"]
            + '</div><div id="preview-subtitle">'
            + subtitle
            + '</div><div id="app">',
        )
        controls = """
controls.shell.checked=__SHELL__;controls.referenceMeshes.checked=false;
for(const id of ['routes','endpoints','proxies','interior','exterior','openings','surfaceApertures','axes','candidates'])controls[id].checked=false;
controls.names.checked=__NAMES__;yaw=__YAW__;pitch=-Math.PI/2;zoom=1.12;draw();
""".replace("__SHELL__", str(mechanical).lower()).replace(
            "__NAMES__", str(not mechanical).lower()
        )
        controls = controls.replace("__YAW__", "0" if mechanical else "Math.PI/4")
        if not mechanical:
            controls += """
controls.names.checked=false;
const geometryDraw=draw;
draw=function(){
 geometryDraw();
 const labels={BATTERY_1:'Battery + 8 AA holder',U1_1:'Arduino Uno R3',REGULATOR_1:'5 V regulator',S1_1:'Slide switch'};
 const items=scene.components.map(p=>({p,xy:project(p.center)})).sort((a,b)=>a.xy[1]-b.xy[1]);
 ctx.font='18px system-ui';
 items.forEach((item,i)=>{
  const x=width-300,y=height*.27+i*height*.13;
  ctx.strokeStyle='#708399';ctx.lineWidth=1.5;
  ctx.beginPath();ctx.moveTo(item.xy[0],item.xy[1]);ctx.lineTo(x-15,y);ctx.lineTo(x,y);ctx.stroke();
  ctx.beginPath();ctx.arc(item.xy[0],item.xy[1],4,0,Math.PI*2);ctx.fillStyle='#708399';ctx.fill();
  ctx.fillStyle='#ffffffee';ctx.fillRect(x-5,y-22,285,33);
  ctx.fillStyle='#243850';ctx.fillText(labels[item.p.id]||item.p.name,x+5,y);
 });
};draw();
"""
        text = text.replace("</script>", controls + "\n</script>")
        (out / (filename + ".html")).write_text(text)
        print(out / (filename + ".html"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    export_views(parser.parse_args().output.resolve())
