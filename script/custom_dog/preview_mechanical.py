"""Export actual original-pipeline parts and optimized motor envelopes for inspection."""
from pathlib import Path
import json
import sys
import numpy as np
import trimesh
import plotly.graph_objects as go
ROOT=Path(__file__).resolve().parents[2]
source=ROOT/'result/custom_dog_test/dog_20261101-204826/result_round1'
colors=['#d6b78d','#efd9b4','#6fa6ca','#83c0b4','#bb90c9','#deaa7d']
fig=go.Figure()
for path,color in zip(sorted((source/'urdf').glob('*.stl')),colors):
    mesh=trimesh.load(path,force='mesh');mesh.apply_scale(1000)
    v=mesh.vertices; f=mesh.faces
    fig.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],name=path.stem,color=color,opacity=.45,showlegend=True))
for item in json.loads((ROOT/'data/custom_dog/motor_selection.json').read_text()):
    center=np.array(item['center_mm']);axis=np.array(item['axis']);half=axis*item['envelope_length_mm']/2
    mesh=trimesh.creation.cylinder(radius=item['radius_mm'],segment=[center-half,center+half],sections=32)
    v=mesh.vertices;f=mesh.faces
    fig.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],name=item['link']+' '+item['model'],color='#2f4567',opacity=1,showlegend=True))
    point=np.array(item['annotated_joint_mm'])
    fig.add_trace(go.Scatter3d(x=[point[0],center[0]],y=[point[1],center[1]],z=[point[2],center[2]],mode='lines+markers',line=dict(color='#944de0',width=6),marker=dict(size=4),name=item['link']+' annotation -> motor',showlegend=True))
fig.update_layout(title='Dog: actual mechanical result, 5 optimized motors (mm)',scene=dict(aspectmode='data',camera=dict(eye=dict(x=0,y=-2,z=.3),up=dict(x=0,y=0,z=1))),margin=dict(l=0,r=0,b=0,t=45),legend=dict(itemsizing='constant'))
fig.write_html(ROOT/'data/custom_dog/mechanical_preview.html',include_plotlyjs=True)
print(ROOT/'data/custom_dog/mechanical_preview.html')
