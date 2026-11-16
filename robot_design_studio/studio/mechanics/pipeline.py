"""Fresh-input mechanical generation. No historical mesh outputs are consumed."""
import hashlib
import io
import json
import math
import platform
import time
import zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
import scipy
from ..project import atomic
from ..contracts import now
from ..catalog import Catalog
from .solid import (box, cylinder, union, cut, intersect, overlap, convex_sweep, rotate,
                    regions, normalize, inset_convex, link_labels)
from .modules import motor_module, shell_with_lid, fixed_carrier, moving_bridge, electronics_box
from .planning import propose

APP=Path(__file__).resolve().parents[2]


def generate(project, source_path, directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);started=time.monotonic();events=[]
    def progress(stage,detail):
        events.append({'at':now(),'type':stage,'detail':detail})
        atomic(directory/'progress.json',events)
    config=project['mechanical_config'];source=project['source_shapes'][0]
    progress('import','读取本项目原始网格，校验封闭实体和坐标')
    raw=Path(source_path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=source['sha256']:raise ValueError('外观文件摘要不匹配')
    mesh=tm.load(io.BytesIO(raw),file_type=source['format'],force='mesh',process=True)
    shape,transform=normalize(mesh,source['unit'],config['length_m']*1000)
    plan=propose(shape,config,project,directory,progress)
    # Candidate motor placement shifts the WHOLE joint and module, preserving shaft alignment.
    progress('layout','按完整模块占用筛选三处轴位置；固定侧和输出侧保持同轴')
    candidates=[];nominal=shape.bounds[0,0]+shape.extents[0]*plan['joint_fraction']
    angle=math.radians(plan['amplitude_deg']);gap=4+max(abs(shape.bounds[:,1]))*math.tan(angle)
    for x in [shape.bounds[0,0]+8,nominal-6,nominal,nominal+6,shape.bounds[1,0]-4]:
        module=union(*motor_module().values()).apply_translation([x,0,0])
        outside=max(0.,module.volume-overlap(module,shape))
        room=min(x-shape.bounds[0,0],shape.bounds[1,0]-x)-gap
        candidates.append({'axis_x_mm':x,'outside_mm3':outside,'housing_room_mm':room,'feasible':bool(outside<1 and room>=78),'score':outside+max(0,78-room)*100+abs(x-nominal)*.1})
    chosen=min(candidates,key=lambda c:c['score'])
    if chosen['outside_mm3']>1:raise ValueError('完整电机模块无法放入外观')
    axis=chosen['axis_x_mm'];shape.apply_translation([-axis,0,0]);transform[0,3]-=axis
    if min(-shape.bounds[0,0],shape.bounds[1,0])-gap<78:
        raise ValueError('关节两侧空间不足：当前完整模块需要每侧至少 78 mm；请增大长度或减小幅度')
    progress('decompose','按 A2R 最近连杆分配并实体切分，依据扫掠空间扩大关节缝')
    labels=link_labels(shape.vertices,0,shape.bounds[0,0],shape.bounds[1,0])
    inner=inset_convex(shape,2.4)
    before_f,_,_,_=shell_with_lid(shape,inner,'fixed',4)
    before_m,_,_,_=shell_with_lid(shape,inner,'moving',4)
    interference_before=[{'angle_deg':d,'intersection_mm3':overlap(before_f,rotate(before_m,math.radians(d)))} for d in [-plan['amplitude_deg'],0,plan['amplitude_deg']]]
    fixed,cap_f,holes_f,skin_f=shell_with_lid(shape,inner,'fixed',gap)
    moving,cap_m,holes_m,skin_m=shell_with_lid(shape,inner,'moving',gap)
    module=motor_module();carrier=fixed_carrier(module['case'],shape,gap);bridge=moving_bridge(shape,gap)
    tray,tray_lid,dock,dock_points=electronics_box(shape,gap)
    contacts={'fixed_shell_to_carrier_mm3':overlap(fixed,carrier), 'moving_shell_to_bridge_mm3':overlap(moving,bridge), 'fixed_shell_to_dock_mm3':overlap(fixed,dock)}
    progress('connections','生成承力桥、安装座、螺孔、螺母窝、可拆外盖和电子盒')
    fixed=union(fixed,carrier,dock);moving=union(moving,bridge)
    # Drill AFTER joining the carrier; otherwise the rib fills the case's nut holes.
    motor_holes=[cylinder(x,y,-26,14,1.2) for x in [-23.5,5.5] for y in [-8.5,8.5]]
    motor_nuts=[cylinder(x,y,-25.1,-22.4,2.3,6) for x in [-23.5,5.5] for y in [-8.5,8.5]]
    fixed=cut(fixed,*motor_holes,*motor_nuts)
    parts=[('fixed_shell','固定壳体与电机座','fixed','convex_shell',fixed,True),
           ('moving_shell','运动壳体与舵盘连接桥','moving','convex_shell',moving,True),
           ('fixed_lid','固定侧可拆外盖','fixed','convex_shell',cap_f,True),
           ('moving_lid','运动侧可拆外盖','moving','convex_shell',cap_m,True),
           ('motor_retainer','电机可拆压盖','fixed','sg90_fixture',module['retainer'],True),
           ('electronics_tray','可拆电子盒','fixed','electronics_tray',tray,True),
           ('electronics_lid','电子盒盖','fixed','electronics_tray',tray_lid,True),
           ('motor_envelope','SG90 假设外形 · 非打印件','fixed','sg90_fixture',module['motor_envelope'],False),
           ('stock_horn_envelope','配套舵盘假设外形 · 非打印件','moving','stock_horn_candidate',module['stock_horn_envelope'],False)]
    checks=[]
    def check(name,passed,**details):checks.append({'name':name,'passed':bool(passed),**details})
    assets=[];preview=[];scene=tm.Scene();palette=['#779e84','#deae6e','#a1b6a8','#eacda5','#8c98b7','#708fa7','#a3bbc9','#546578','#be826b']
    for idx,(identity,name,owner,template,m,printable) in enumerate(parts):
        valid=m.is_volume and regions(m)==1
        check('solid_'+identity,valid,watertight=bool(m.is_watertight),positive_regions=regions(m),volume_mm3=float(m.volume))
        path=directory/(identity+'.stl');m.export(path)
        vertices=(m.vertices/1000).tolist();faces=m.faces.tolist()
        assets.append({'id':identity,'name':name,'owner_body_id':owner,'template_id':template,'template_revision':'1',
                       'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'export_length_unit':'mm',
                       'transform_m':np.eye(4).tolist(),'bounds_m':(m.bounds/1000).tolist(),'volume_m3':float(m.volume)*1e-9,
                       'material':'pla_candidate' if printable else 'hardware_unknown','printable_candidate':printable})
        preview.append({'id':identity,'name':name,'owner':owner,'color':palette[idx],'vertices':vertices,'faces':faces,'hardware':not printable})
        display=m.copy();display.apply_scale(.001);display.visual.face_colors=tm.visual.color.hex_to_rgba(palette[idx]);scene.add_geometry(display,node_name=identity,geom_name=identity)
    for name,vol in contacts.items():check(name,vol>1,contact_volume_mm3=vol)
    progress('checks','检查实体连通、模块间碰撞、抽取通道、工具空间和逐角度扫掠')
    # Final assembly, not intermediate envelopes; no collision exemptions for touching parts.
    collisions=[]
    for i,a in enumerate(parts):
        for b in parts[i+1:]:
            vol=overlap(a[4],b[4])
            if vol>.05:collisions.append({'a':a[0],'b':b[0],'volume_mm3':vol})
    check('assembly_collisions',not collisions,collisions=collisions,tolerance_mm3=.05)
    fixed_group=union(*[p[4] for p in parts if p[2]=='fixed'])
    moving_group=union(*[p[4] for p in parts if p[2]=='moving'])
    sweeps=[]
    for degrees in np.linspace(-plan['amplitude_deg'],plan['amplitude_deg'],17):
        volume=overlap(fixed_group,rotate(moving_group,math.radians(float(degrees))))
        sweeps.append({'angle_deg':float(degrees),'intersection_mm3':volume})
    check('joint_sweep',all(s['intersection_mm3']<.05 for s in sweeps),samples=sweeps,continuous_guarantee=False)
    # Remove caps/retainer before extracting hardware; conservative convex swept volume.
    for identity,target,obstacles,removed in [
        ('motor_insertion',module['motor_envelope'],fixed,['fixed_lid','motor_retainer','stock_horn_envelope']),
        ('electronics_removal',union(tray,tray_lid),union(fixed,module['motor_envelope'],module['retainer']),['fixed_lid']),
        ('fixed_lid_removal',cap_f,union(fixed,tray,tray_lid,module['motor_envelope'],module['retainer']),[]),
        ('moving_lid_removal',cap_m,moving,[])]:
        vol=overlap(convex_sweep(target,shape.extents[2]+40),obstacles)
        check(identity,vol<.05,conservative_sweep_intersection_mm3=vol,removed_first=removed,direction=[0,0,1])
    # Tool corridor from fastener head outward, checked with lids removed where necessary.
    tools=[]
    for x,y,z in holes_f+holes_m:
        probe=cylinder(x,y,shape.bounds[1,2]+.5,shape.bounds[1,2]+40,3)
        tools.append(overlap(probe,union(fixed_group,moving_group)))
    check('external_lid_tools',max(tools,default=0)<.05,intersection_mm3=tools,assumed_tool_radius_mm=3)
    horn_probe=cylinder(0,0,20.1,80,3)
    check('horn_screw_access',overlap(horn_probe,union(fixed,moving))<.05,intersection_mm3=overlap(horn_probe,union(fixed,moving)),removed_first=['fixed_lid','moving_lid'])
    check('retainer_fastener_bores',all(overlap(h,fixed)<.05 for h in motor_holes),intersection_mm3=[overlap(h,fixed) for h in motor_holes])
    seats=[]
    for base,lid,points in [(fixed,cap_f,holes_f),(moving,cap_m,holes_m)]:
        for x,y,z in points:
            lower=box([x+2.3,y-.2,z-.05],[x+2.7,y+.2,z-.01])
            upper=box([x+2.3,y-.2,z+.01],[x+2.7,y+.2,z+.05])
            seats.append({'point_m':[x/1000,y/1000,z/1000],'base_material_mm3':overlap(lower,base),'lid_material_mm3':overlap(upper,lid)})
    check('lid_fastener_seats',all(v['base_material_mm3']>.003 and v['lid_material_mm3']>.003 for v in seats),seats=seats)
    # Local-frame meshes share an identity rest transform; rotations apply once around joint origin.
    bodies=[{'id':b,'name':'固定侧' if b=='fixed' else '运动侧','transform_m':np.eye(4).tolist(),'asset_ids':[a['id'] for a in assets if a['owner_body_id']==b]} for b in ['fixed','moving']]
    joint={'id':'joint_1','type':'revolute','parent_body_id':'fixed','child_body_id':'moving','axis':[0,0,1],
           'origin_m':[0,0,0],'limits_rad':[-angle,angle],'motor_instance_id':config['motor_instance_id'],'torque_required_nm':None,'load_verified':False}
    assembly=[
        {'step':1,'action':'核实 SG90 本体、舵盘与孔位；试配前保持断电；所有几何为工程试件','parts':['motor_envelope','stock_horn_envelope']},
        {'step':2,'action':'外盖打开时装入 M3 候选螺母，检查壳体和承力肋','parts':['fixed_shell','moving_shell']},
        {'step':3,'action':'沿 +Z 反向放入电机，走侧面线槽；安装四个 M2 候选紧固件和电机压盖','parts':['fixed_shell','motor_envelope','motor_retainer']},
        {'step':4,'action':'按厂家方式归零；用原装舵盘及中心螺钉连接输出桥，槽孔螺钉防转；不使用打印花键','parts':['stock_horn_envelope','moving_shell']},
        {'step':5,'action':'核实板卡和绝缘后，以可调扎带槽固定板卡；装电子盒盖，四个 M3 候选紧固件固定盒体','parts':['electronics_tray','electronics_lid','fixed_shell']},
        {'step':6,'action':'整理电机线并保留余量；手动验证幅度和工具可达后装两侧外盖，各四个 M3 候选紧固件','parts':['fixed_lid','moving_lid']}
    ]
    templates=json.loads((APP/'catalog/mechanical_templates.json').read_text())
    mounts=[{'id':'motor_mount','owner_body_id':'fixed','template_ids':['sg90_fixture'],'asset_id':'fixed_shell','motor_instance_id':config['motor_instance_id'],'contact_volume_m3':contacts['fixed_shell_to_carrier_mm3']*1e-9},
            {'id':'output_bridge','owner_body_id':'moving','template_ids':['horn_bridge','stock_horn_candidate'],'asset_id':'moving_shell','contact_volume_m3':contacts['moving_shell_to_bridge_mm3']*1e-9},
            {'id':'electronics_dock','owner_body_id':'fixed','template_ids':['electronics_tray'],'asset_id':'fixed_shell','contact_volume_m3':contacts['fixed_shell_to_dock_mm3']*1e-9}]
    bom=[{'template_id':'fastener_m3_candidate','quantity':12,'purpose':'两侧外盖 8、电子盒固定 4','length_m':None},
         {'template_id':'fastener_m2_candidate','quantity':10,'purpose':'电机压盖 4、电子盒盖 4、舵盘槽 2','length_m':None},
         {'template_id':'stock_horn_candidate','quantity':1,'purpose':'原装舵盘与中心螺丝，配套性待实测'},
         {'template_id':'electronics_retention_candidate','quantity':2,'purpose':'板卡绝缘固定，尺寸待选'}]
    valid=all(c['passed'] for c in checks)
    result={'schema_version':'mechanics-0.2','project_id':project['id'],'design_revision':project['revision'],
      'generator_revision':'0.2.0','mechanical_catalog_sha256':hashlib.sha256((APP/'catalog/mechanical_templates.json').read_bytes()).hexdigest(),
      'generator_sources_sha256':{str(f.relative_to(APP)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((APP/'studio/mechanics').glob('*.py'))},
      'source_sha256':source['sha256'],'source_to_joint_mm':transform.tolist(),'plan':plan,'layout_candidates':candidates,
      'selected_layout':chosen,'decomposition':{'method':'A2R nearest finite link / exact single-plane split; motion-derived seam removal','assigned_vertex_counts':np.bincount(labels,minlength=2).tolist(),'gap_m':gap*2/1000,'wall_m':.0024,'before_clearance_sweep':interference_before,'source_removed_fraction':float(1-(intersect(shape,box([-500,-500,-500],[-gap,500,500])).volume+intersect(shape,box([gap,-500,-500],[500,500,500])).volume)/shape.volume)},
      'units':{'length':'m','angle':'rad','mass':'kg'},'rigid_bodies':bodies,'joints':[joint],'mounts':mounts,'geometry_assets':assets,
      'assembly_dependencies':assembly,'hardware_requests':bom,'templates':templates,'checks':checks,'geometry_valid':valid,
      'manufacturing_ready':False,'status':'engineering_fixture' if valid else 'geometry_rejected',
      'pending':templates['pending']+['电子盒预留包络未等同板卡已装入；端口配准、线束三维路由和动态线长待实现','材料质量、惯量、负载、连续碰撞、支撑稳定性和实物装配未验证'],
      'elapsed_s':round(time.monotonic()-started,3),'dependencies':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,'trimesh':tm.__version__}}
    atomic(directory/'preview.json',{'parts':preview,'joint':joint})
    scene.export(directory/'assembly.glb')
    atomic(directory/'manifest.json',result)
    atomic(directory/'checks.json',checks)
    atomic(directory/'assembly.json',assembly)
    atomic(directory/'mechanical_templates.json',templates)
    atomic(directory/'catalog.json',Catalog().document)
    # Copy original bytes, not reconstructed source, so future regeneration is reproducible.
    (directory/('source.'+source['format'])).write_bytes(raw)
    atomic(directory/'input_project.json',project)
    progress('export','导出每件实体、装配场景、来源、检查和工程试件包')
    files=[a['file'] for a in assets]+['assembly.glb','preview.json','manifest.json','checks.json','assembly.json','mechanical_templates.json','catalog.json','input_project.json','source.'+source['format'],'top.png','front.png','side.png','views.json','progress.json']
    with zipfile.ZipFile(directory/'mechanical-review.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for name in files:archive.write(directory/name,name)
    result['artifacts']=files+['mechanical-review.zip']
    atomic(directory/'result.json',result)
    return result
