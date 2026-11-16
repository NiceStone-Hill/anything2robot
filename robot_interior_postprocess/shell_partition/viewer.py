"""Small offline viewer, independent of the upstream presentation layer."""

import json

from .io import display_mesh, render_views


def export_viewer(out, pieces, targets, reports, supports=()):
    expanded = []
    for owner, meshes in pieces.items():
        opening = reports[owner]["selected_opening"]
        for side, mesh in enumerate(meshes):
            sign = 1 if side == opening["panel_index"] else -1
            expanded.append(
                mesh.copy().apply_translation(
                    [v * sign * 30 for v in opening["direction"]]
                )
            )
    if expanded:
        shown_supports = []
        for support in supports:
            if support.get("display_role") == "tools":
                continue
            owner = support["owners"][0]
            opening = reports[owner]["selected_opening"]
            sign = 1 if support["owner_piece_index"] == opening["panel_index"] else -1
            shown_supports.append(
                {
                    **support,
                    "mesh": support["mesh"]
                    .copy()
                    .apply_translation([v * sign * 30 for v in opening["direction"]]),
                }
            )
        render_views(
            expanded,
            [*targets, *shown_supports],
            out / "preview.png",
            "Partition review; exploded for display only; assembly validation pending",
        )
    entries = []
    colors = ["#85bdcd", "#efb774", "#a8c78f", "#b6a8d8"]
    for i, (owner, meshes) in enumerate(pieces.items()):
        opening = reports[owner]["selected_opening"]
        for side, mesh in enumerate(meshes):
            shown = display_mesh(mesh)
            sign = 1 if side == opening["panel_index"] else -1
            entries.append(
                {
                    "name": f"{owner} / {side + 1}",
                    "owner": owner,
                    "vertices": shown.vertices.tolist(),
                    "faces": shown.faces.tolist(),
                    "color": colors[i % len(colors)] if side == 0 else "#efc695",
                    "explode": [v * sign for v in opening["direction"]],
                }
            )
    for target in targets:
        source = target.get("display_mesh", target["mesh"])
        shown = (
            source.copy()
            if source.metadata.get("display_reference")
            else display_mesh(source)
        )
        entries.append(
            {
                "name": target["id"],
                "owner": "hardware",
                "owners": target["owners"],
                "vertices": shown.vertices.tolist(),
                "faces": shown.faces.tolist(),
                "color": "#d77465" if target["kind"] == "motor" else "#748fde",
                "face_colors": shown.visual.face_colors[:, :3].tolist()
                if source.metadata.get("display_reference") else None,
                "explode": [0, 0, 0],
            }
        )
    for support in supports:
        owner = support["owners"][0]
        opening = reports[owner]["selected_opening"]
        sign = 1 if support["owner_piece_index"] == opening["panel_index"] else -1
        shown = display_mesh(support["mesh"])
        entries.append(
            {
                "name": support["id"],
                "owner": owner,
                "is_support": True,
                "vertices": shown.vertices.tolist(),
                "faces": shown.faces.tolist(),
                "color": {
                    "detachable": "#64c6cf",
                    "fastener_solid": "#d5b978",
                    "tools": "#de8bbf",
                    "shell_connection": "#78c4d0",
                }.get(support.get("display_role"), "#70d68a"),
                "display_role": support.get("display_role"),
                "explode": [v * sign for v in opening["direction"]],
            }
        )
    data = json.dumps(
        {"meshes": entries, "reports": reports}, ensure_ascii=True
    ).replace("<", "\\u003c")
    template = """<!doctype html><html lang="en"><meta charset="utf-8">
<title>Robot shell partition review</title>
<style>*{box-sizing:border-box}body{margin:0;background:#14202b;color:#edf3f7;font:14px system-ui}header{padding:14px 20px;background:#203140;display:flex;gap:18px;align-items:center;flex-wrap:wrap}a{color:#8fd7ed}main{display:grid;grid-template-columns:minmax(0,1fr) 360px;height:calc(100vh - 100px)}canvas{width:100%;height:100%;touch-action:none}aside{padding:18px;overflow:auto;background:#1d2c39}label{display:block;padding:4px}select{padding:6px}p{line-height:1.55}.pending{color:#edbc7e}small{color:#bcc9d2}@media(max-width:800px){main{grid-template-columns:1fr;height:auto}canvas{height:60vh}aside{max-height:40vh}}</style>
<header><b>Robot shells · Assembly review</b><select id="part"><option value="all">All links</option></select>
<span>Explode <input id="explode" type="range" min="0" max="100" value="35"></span>
<span><input id="hardware" type="checkbox" checked>Show hardware</span>
<span><input id="supports" type="checkbox" checked>Support overlays</span>
<span><input id="fasteners" type="checkbox" checked>Show fasteners</span>
<a href="partition_report.json">Validation report</a><a href="proposal_raw.json">VLM proposal</a></header>
<div style="padding:8px 20px" class="pending">Prototype geometry: physical assembly, fastener fit, strength and harness validation remain pending. Exploded views are for inspection.</div>
<main><canvas id="view"></canvas><aside><div id="summary"></div><div id="toggles"></div></aside></main>
<script>const data=__DATA__;const meshes=data.meshes;
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');let yaw=.65,pitch=-.30,zoom=1,down=null;
const part=document.getElementById('part'),explode=document.getElementById('explode'),hardware=document.getElementById('hardware'),supportToggle=document.getElementById('supports'),fastenerToggle=document.getElementById('fasteners');
for(const owner of Object.keys(data.reports)){const o=document.createElement('option');o.value=owner;o.textContent=owner;part.append(o)}
for(const m of meshes){m.show=m.display_role!=='tools';const l=document.createElement('label'),c=document.createElement('input');c.type='checkbox';c.checked=m.show;c.onchange=()=>{m.show=c.checked;draw()};l.append(c,document.createTextNode(m.name));document.getElementById('toggles').append(l)}
function project(v,center){const [x,y,z]=v.map((q,i)=>q-center[i]);const a=Math.cos(yaw)*x-Math.sin(yaw)*y,b=Math.sin(yaw)*x+Math.cos(yaw)*y;return [a,Math.cos(pitch)*b-Math.sin(pitch)*z,Math.sin(pitch)*b+Math.cos(pitch)*z]}
function draw(){const visible=meshes.filter(m=>m.show&&(m.display_role!=='fastener_solid'||fastenerToggle.checked)&&(!m.is_support||supportToggle.checked)&&(m.owner!=='hardware'||hardware.checked)&&(part.value==='all'||m.owner===part.value||(m.owners||[]).includes(part.value)));canvas.width=canvas.clientWidth*devicePixelRatio;canvas.height=canvas.clientHeight*devicePixelRatio;ctx.clearRect(0,0,canvas.width,canvas.height);if(!visible.length)return;
const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];for(const m of visible)for(const v of m.vertices)for(let i=0;i<3;i++){lo[i]=Math.min(lo[i],v[i]);hi[i]=Math.max(hi[i],v[i])}const center=lo.map((v,i)=>(v+hi[i])/2),span=Math.hypot(...lo.map((v,i)=>hi[i]-v));const scale=Math.min(canvas.width,canvas.height)/Math.max(span,1)*.8*zoom,tris=[];
for(const m of visible){const points=m.vertices.map(v=>project(v.map((q,i)=>q+m.explode[i]*Number(explode.value)),center));for(const f of m.faces){const t=f.map(i=>points[i]);tris.push({t,color:m.color,depth:t.reduce((a,v)=>a+v[2],0)})}}tris.sort((a,b)=>a.depth-b.depth);for(const {t,color} of tris){ctx.beginPath();t.forEach((v,i)=>ctx[i?'lineTo':'moveTo'](canvas.width/2+v[0]*scale,canvas.height/2-v[1]*scale));ctx.closePath();ctx.fillStyle=color;ctx.fill();ctx.strokeStyle='#27374655';ctx.lineWidth=.3;ctx.stroke()}}
function summary(){const el=document.getElementById('summary');el.replaceChildren();for(const [owner,r] of Object.entries(data.reports)){if(part.value!=='all'&&owner!==part.value)continue;const h=document.createElement('h3');h.textContent=owner;el.append(h);const p=document.createElement('p');p.textContent=r.status+' · '+r.selected_opening.clear_tasks+'/'+r.selected_opening.total_tasks+' local envelope tasks passed';el.append(p);if(r.support_design){const s=document.createElement('p');s.textContent='Supports '+r.support_design.generated_count+'/'+r.support_design.required_count+'; mounted on shell piece '+(r.support_design.installation_piece_index+1)+'; physical fit and strength pending';el.append(s)}const reason=document.createElement('p');reason.textContent=r.strategy.rationale;el.append(reason);for(const t of r.selected_opening.targets){const q=document.createElement('p');q.textContent=t.target_id+': insertion '+t.body_path_clear+' / tool envelope '+t.assumed_grip_clear+' / cover '+t.panel_clear;el.append(q)}}}
part.onchange=()=>{summary();draw()};explode.oninput=draw;hardware.onchange=draw;supportToggle.onchange=draw;fastenerToggle.onchange=draw;canvas.onpointerdown=e=>{down=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointerup=()=>down=null;canvas.onpointermove=e=>{if(down){yaw+=(e.clientX-down[0])*.008;pitch+=(e.clientY-down[1])*.008;down=[e.clientX,e.clientY];draw()}};canvas.onwheel=e=>{e.preventDefault();zoom*=e.deltaY<0?1.1:.9;draw()};window.onresize=draw;summary();draw();</script></html>"""
    if any("support_design" in r for r in reports.values()):
        template = template.replace(
            "</a></header>",
            '</a><a href="support_ownership.json">Support ownership</a></header>',
        )
    template = template.replace(
        "for(const f of m.faces){const t=f.map(i=>points[i]);tris.push({t,color:m.color,depth:t.reduce((a,v)=>a+v[2],0)})}",
        "for(const [fi,f] of m.faces.entries()){const t=f.map(i=>points[i]);tris.push({t,color:m.face_colors?'rgb('+m.face_colors[fi].join(',')+')':m.color,depth:t.reduce((a,v)=>a+v[2],0)})}",
    )
    if any("shell_connection" in r for r in reports.values()):
        template = template.replace(
            "const reason=document.createElement('p');",
            """if(r.shell_connection){const c=document.createElement('p');c.textContent='Shell fasteners '+r.shell_connection.connection_count+' connections; cyan marks the external loading ports leading to head seats. Physical fit and strength pending.';el.append(c);
if(r.shell_connection.shared_pattern){const sp=document.createElement('p');sp.textContent='Shared pattern: '+r.shell_connection.shared_pattern.members.join(' / ');el.append(sp)}
for(const [i,j] of r.shell_connection.connections.entries()){if(!j.access)continue;const p=document.createElement('p');p.textContent=(i+1)+': '+j.fastener_spec.designation+'; loading port diameter '+j.access.port_diameter_mm+' mm; head seat depth '+j.access.mouth_to_seat_mm.toFixed(1)+' mm';el.append(p);const b=document.createElement('button');b.textContent='Align to screw '+(i+1)+' entrance';b.onclick=()=>{part.value=owner;explode.value=0;fastenerToggle.checked=false;supportToggle.checked=false;hardware.checked=false;const n=j.screw_axis_world;yaw=Math.atan2(n[0],n[1]);pitch=Math.atan2(Math.hypot(n[0],n[1]),n[2]);summary();draw()};el.append(b)}}const reason=document.createElement('p');""",
        )
        # Axis arrows use actual unsimplified report coordinates and move with
        # the cover. They do not depend on STL display decimation preserving holes.
        template = template.replace(
            "function summary(){",
            """const drawMeshes=draw;draw=function(){drawMeshes();const all=Object.values(data.reports).filter(r=>r.shell_connection);if(!all.length)return;
const visible=meshes.filter(m=>m.show&&(m.display_role!=='fastener_solid'||fastenerToggle.checked)&&(!m.is_support||supportToggle.checked)&&(m.owner!=='hardware'||hardware.checked)&&(part.value==='all'||m.owner===part.value||(m.owners||[]).includes(part.value)));if(!visible.length)return;
const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];for(const m of visible)for(const v of m.vertices)for(let i=0;i<3;i++){lo[i]=Math.min(lo[i],v[i]);hi[i]=Math.max(hi[i],v[i])}const center=lo.map((v,i)=>(v+hi[i])/2),span=Math.hypot(...lo.map((v,i)=>hi[i]-v)),scale=Math.min(canvas.width,canvas.height)/Math.max(span,1)*.8*zoom;
for(const [owner,r] of Object.entries(data.reports)){if(!r.shell_connection||(part.value!=='all'&&part.value!==owner))continue;for(const [i,j] of r.shell_connection.connections.entries()){if(!j.access)continue;const n=j.screw_axis_world,m=j.access.mouth_world_mm.map((q,k)=>q+n[k]*Number(explode.value)),start=project(m.map((q,k)=>q+18*n[k]),center),end=project(m,center);const xy=q=>[canvas.width/2+q[0]*scale,canvas.height/2-q[1]*scale],a=xy(start),b=xy(end);ctx.strokeStyle='#02b9ca';ctx.fillStyle='#02b9ca';ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();ctx.beginPath();ctx.arc(...b,5*devicePixelRatio,0,Math.PI*2);ctx.stroke();ctx.font=(12*devicePixelRatio)+'px system-ui';ctx.fillText(owner+' #'+(i+1),b[0]+7*devicePixelRatio,b[1]-7*devicePixelRatio)}}};
function summary(){""",
        )
    (out / "index.html").write_text(template.replace("__DATA__", data))
