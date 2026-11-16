"""Static review figure from the recorded post-duct candidate, not a finished robot."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
ROOT=Path(__file__).resolve().parents[2]
out=ROOT/'robot_interior_postprocess/outputs/custom_dog_assembly'
trace=json.loads((out/'process_trace.json').read_text())
event=next(e for e in reversed(trace['events']) if e['stage']=='post_duct_split')
report=json.loads((out/'display_registration_report.json').read_text())
# The separately documented CAD display correction is applied before the unchanged world pose.
raw_config=json.loads((out/'assembly_config_snapshot.json').read_text())
old=np.array(raw_config['components'][0]['display']['interface_from_mesh'])
new=np.array(report['interface_from_mesh']);delta=new@np.linalg.inv(old)
items=[]
colors={'shell':[.75,.57,.39,.035],'motor':[.24,.31,.52,.9],'component':[.24,.60,.43,.85],
        'carrier':[.18,.68,.75,.85],'support':[.31,.53,.66,.8],'wire':[.88,.46,.15,1]}
for item in event['instances']:
    asset=trace['assets'][item['asset']];v=np.array(asset['vertices']);f=np.array(asset['faces'])
    if item['id']=='BATTERY_1:component':v=v@delta[:3,:3].T+delta[:3,3]
    t=np.array(item['transform']);v=v@t[:3,:3].T+t[:3,3]
    items.append((item['id'],item['role'],v,f))
fig,axes=plt.subplots(1,2,figsize=(11,8),facecolor='white')
for ax,angle,title in zip(axes,[0,35],['Front view','Oblique view']):
    a=np.deg2rad(angle);x=np.array([np.cos(a),np.sin(a),0]);depth=np.array([-np.sin(a),np.cos(a),0]);up=np.array([0,0,1])
    faces2=[];depths=[];rgba=[]
    for name,role,v,f in items:
        xy=np.column_stack((v@x,v@up));tri=xy[f];d=(v@depth)[f].mean(1)
        color=np.tile(colors.get(role,[.5,.5,.5,.8]),(len(f),1))
        if name=='BATTERY_1:component':color[:,:3]=[.32,.44,.32]
        if role!='shell':
            normals=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
            normal_len=np.linalg.norm(normals,axis=1);normals/=np.maximum(normal_len[:,None],1e-9)
            shade=.65+.35*np.abs(normals@np.array([.2,-.5,.84]));color[:,:3]*=shade[:,None]
        faces2.extend(tri);depths.extend(d);rgba.extend(color)
    order=np.argsort(-np.array(depths))
    ax.add_collection(PolyCollection(np.array(faces2)[order],facecolors=np.array(rgba)[order],edgecolors='none',rasterized=True))
    allv=np.concatenate([v for _,_,v,_ in items]);proj=np.column_stack((allv@x,allv@up))
    ax.set_xlim(proj[:,0].min()-30,proj[:,0].max()+30);ax.set_ylim(-20,800);ax.set_aspect('equal')
    ax.set_title(title,fontsize=12);ax.set_xlabel('mm');ax.set_ylabel('mm');ax.grid(alpha=.12)
fig.suptitle('Dog robot: tested assembly candidate',fontsize=17)
fig.text(.5,.92,'5 motors / 4 carriers / routing 6 of 20 / final shell fastening FAILED',ha='center',fontsize=10,color='#9b433b')
fig.text(.5,.025,'Ghosted shells show the selected candidate. Colours: motors blue, carriers cyan, electronics green, partial wires orange.\nLocal carrier paths passed; this is not a complete printable assembly.',ha='center',fontsize=9)
fig.tight_layout(rect=[0,.065,1,.91]);path=ROOT/'data/custom_dog/test_result.png';fig.savefig(path,dpi=180);print(path)
