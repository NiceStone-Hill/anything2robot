"""Correct the concept battery's CAD display frame in separate, labelled previews.

Preserve every solver output and validation artifact. No placement or geometry
check is changed. Flat battery CAD is centred in XY and has its bottom at Z=0,
the same interface convention as the planning tray.
"""
import copy
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2]
out=ROOT/'robot_interior_postprocess/outputs/custom_dog_assembly'
base=ROOT/'robot_interior_postprocess/configs'
spec_path=base/'custom_dog_carrier_assembly_head_battery.json'
config=json.loads(spec_path.read_text());battery=next(p for p in config['components'] if p['component_id']=='BATTERY')
source=(base/battery['display']['file']).resolve()
mesh=trimesh.load(source,force='mesh',process=False)
registration=np.eye(4);registration[:3,3]=[-mesh.bounds.mean(0)[0],-mesh.bounds.mean(0)[1],-mesh.bounds[0,2]]
mesh.apply_transform(registration)
size=np.array(battery['envelope']['size_mm'])
assert np.all(mesh.bounds[0] >= np.r_[-size[:2]/2,0]-1e-6)
assert np.all(mesh.bounds[1] <= np.r_[size[:2]/2,size[2]]+1e-6)
registered=copy.deepcopy(config)
registered['components'][0]['display']['interface_from_mesh']=registration.tolist()
registered['components'][0]['display']['registration']='Conceptual flat battery: centred in XY and bottom at local Z=0. Visual reference only; no measured retention or electrical compatibility certification.'
(base/'custom_dog_carrier_assembly_head_battery_registered.json').write_text(json.dumps(registered,indent=2))
trace=json.loads((out/'process_trace.json').read_text())
old_keys={item['asset'] for e in trace['events'] for item in e.get('instances',[]) if item['id']=='BATTERY_1:component'}
if not old_keys:raise ValueError('No recorded battery appearance to register')
vertices=np.asarray(mesh.vertices,dtype='<f8');faces=np.asarray(mesh.faces,dtype='<i8')
new_key=hashlib.sha256(vertices.tobytes()+faces.tobytes()).hexdigest()
new_asset={'vertices':np.round(vertices,8).tolist(),'faces':faces.tolist(),
           'source_geometry_sha256':new_key,'source_faces':len(faces),'source_volume_mm3':float(mesh.volume),
           'source_bounds_mm':mesh.bounds.tolist(),'preview_simplified':False,
           'face_colors':mesh.visual.face_colors[:,:3].tolist()}
def replace(v):
    if isinstance(v,str): return new_key if v in old_keys else v
    if isinstance(v,list): return [replace(x) for x in v]
    if isinstance(v,dict): return {k:replace(x) for k,x in v.items()}
    return v
trace=replace(trace)
for key in old_keys:trace['assets'].pop(key,None)
trace['assets'][new_key]=new_asset
note='Battery CAD appearance frame corrected in this separate preview; raw solver output, planning envelopes, poses and validations are unchanged.'
trace['preview_note']+=' '+note
trace['display_registration']={'instance':'BATTERY_1','note':note,'world_pose_changed':False,'interface_from_mesh':registration.tolist()}
module=ROOT/'robot_interior_postprocess/carrier_layout'
template=(module/'process_viewer.html').read_text()
template=template.replace('__PROCESS_EVIDENCE__',(module/'process_evidence.js').read_text()+'\n'+(module/'process_schema_guides.js').read_text())
template=template.replace('__PROCESS_RENDERER__',(module/'process_renderer.js').read_text())
(out/'process_registered.html').write_text(template.replace('__PROCESS_DATA__',json.dumps(trace,ensure_ascii=False,separators=(',',':')).replace('<','\\u003c')))
report={'status':'display_envelope_check_passed','source':str(source),'interface_from_mesh':registration.tolist(),
        'cad_bounds_in_interface_mm':mesh.bounds.tolist(),'planning_size_mm':size.tolist(),'note':note}
index=out/'index.html';placement=out/'carrier_placement.json'
if index.exists() and placement.exists():
    poses=json.loads(placement.read_text())['placements'];pose=next(p for p in poses if p['id']=='BATTERY_1')
    world=mesh.copy().apply_transform(np.array(pose['world_from_interface']))
    html=index.read_text();prefix='const data=';suffix=';const meshes=data.meshes;'
    a=html.index(prefix)+len(prefix);b=html.index(suffix,a);data=json.loads(html[a:b])
    entry=next(m for m in data['meshes'] if m['name']=='BATTERY_1')
    entry['vertices']=world.vertices.tolist();entry['faces']=world.faces.tolist();entry['face_colors']=mesh.visual.face_colors[:,:3].tolist()
    data['display_registration']=report
    html=html[:a]+json.dumps(data,ensure_ascii=True).replace('<','\\u003c')+html[b:]
    (out/'index_registered.html').write_text(html)
    report['registered_viewer']='index_registered.html'
(out/'display_registration_report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
