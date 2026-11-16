"""Geometric diagnosis of cardinal split candidates on the new robot."""
from pathlib import Path
import json
import sys
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'robot_interior_postprocess'))
from shell_partition.geometry import split_shell
out=ROOT/'robot_interior_postprocess/outputs/custom_dog_assembly_network_failed'
scene=json.loads((out/'mechanical_scene.json').read_text())
rows=[]; openings={}
for name in scene['parts']:
    shell=trimesh.load(out/'meshes'/f'{name}_hollow_assembly_mm.stl',force='mesh')
    feasible=[]
    for axis in [1,0,2]:
        normal=np.eye(3)[axis]
        projections=shell.vertices@normal;lo,hi=projections.min(),projections.max()
        for fraction in [.5,.35,.65,.2,.8]:
            candidate={'normal':normal.tolist(),'offset_mm':float(lo+fraction*(hi-lo))}
            try:
                pieces,removed=split_shell(shell,candidate,.4)
                status='connected_split';detail={'piece_volumes_mm3':[float(p.volume) for p in pieces]}
                feasible.append({'normal':normal.tolist(),'fraction':fraction})
            except ValueError as e:
                status='rejected';detail={'reason':str(e)}
            row={'part':name,'axis':axis,'fraction':fraction,'status':status,**detail};rows.append(row)
            print(row,flush=True)
    openings[name]=feasible
    if not feasible:print('NO CONNECTED SPLIT:',name,flush=True)
(ROOT/'data/custom_dog/opening_probe.json').write_text(json.dumps(rows,indent=2))
base=ROOT/'robot_interior_postprocess/configs'
for config_name in ['custom_dog_carrier_assembly.json','custom_dog_carrier_assembly_local.json']:
    p=base/config_name;config=json.loads(p.read_text());config['openings']=openings;config['search']['max_architectures']=min(64,max(len(v) for v in openings.values())*2)
    p.write_text(json.dumps(config,indent=2))
print('Updated explicit robot opening candidates; all geometry-only filters, no model inference.')
