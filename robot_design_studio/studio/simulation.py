"""Export consistent SI URDF/MJCF from final layout without a simulator runtime.

This is a fixed-base, no-contact, assumed-density model. It deliberately cannot
release hardware, replace CAD collision tests, or claim measured motor limits.
"""
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
from .project import atomic
from .mechanics.electronics_layout import read_mesh
from .mechanics import component_visuals
from .mechanics.graph_assembly import stl_deliverable


def vec(values):return ' '.join(format(float(x),'.12g') for x in values)


def generate(project,layout_directory,directory):
    start=time.monotonic();source=Path(layout_directory);directory=Path(directory)
    manifest=json.loads((source/'result.json').read_text())
    if not manifest.get('layout_valid') or manifest['project_id']!=project['id'] or manifest['design_revision']!=project['revision']:
        raise ValueError('导出需要当前项目已通过的内部装配')
    config=project['simulation_config'];graph=project['articulation']['graph'];root=graph['root_body_id']
    incoming={j['child']:j for j in graph['joints']}
    origins={b['id']:np.array(incoming[b['id']]['position_mm'])/1000 if b['id'] in incoming else np.zeros(3) for b in graph['bodies']}
    meshes=[];mass_report=[];inertials={};mesh_export_checks=[]
    visual_assets={a['replaces_envelope_id']:a for a in manifest.get('visual_assets',[])}
    for asset in manifest['geometry_assets']:
        mesh=read_mesh(source,asset).apply_scale(.001)
        density=config['printed_density_kg_m3'] if asset['printable_candidate'] else config['hardware_density_kg_m3']
        mesh.density=density;owner=asset['owner_body_id'];mass=float(mesh.mass)
        local=mesh.copy().apply_translation(-origins[owner]);filename='sim-'+asset['id']+'.stl'
        local,storage=stl_deliverable(local,millimeters_per_unit=1000.)
        if not storage['passed']:raise ValueError('米制 STL 导出检查未通过：'+asset['id'])
        if storage.get('stl_encoding')=='ascii':
            # MuJoCo STL requires binary. OBJ preserves these positions and is
            # supported by both MJCF and URDF mesh resources.
            filename='sim-'+asset['id']+'.obj'
            local.export(directory/filename,file_type='obj',digits=17,include_color=False)
        else:local.export(directory/filename)
        reopened=tm.load(directory/filename,force='mesh')
        mesh_export_checks.append({'name':'meter_stl_roundtrip','asset_id':asset['id'],'passed':bool(reopened.is_volume and reopened.is_watertight),'storage_precision':storage})
        visual=visual_assets.get(asset['id'])
        if visual:
            reference=component_visuals.load(source,visual).apply_scale(.001).apply_translation(-origins[owner])
            filename='sim-'+visual['id']+'.obj'
            (directory/filename).write_text(reference.export(file_type='obj',include_color=True))
            reopened=tm.load(directory/filename,force='mesh')
            mesh_export_checks.append({'name':'reference_cad_visual_resource','asset_id':visual['id'],'passed':bool(len(reopened.faces)>0 and np.isfinite(reopened.vertices).all()),'physical_variant_verified':False})
        meshes.append({'asset':asset,'mesh':mesh,'filename':filename})
        mass_report.append({'asset_id':asset['id'],'owner_body_id':owner,'mass_kg':mass,'volume_m3':float(mesh.volume),
            'density_kg_m3':density,'evidence':'engineering_assumption','actual_mass_kg':None,
            'source_asset_sha256':asset['sha256']})
    for body in graph['bodies']:
        body_id=body['id'];pieces=[m['mesh'] for m in meshes if m['asset']['owner_body_id']==body_id]
        mass=sum(m.mass for m in pieces);center=sum(m.mass*m.center_mass for m in pieces)/mass
        inertia=np.zeros((3,3))
        for m in pieces:
            d=m.center_mass-center;inertia+=m.moment_inertia+m.mass*((d@d)*np.eye(3)-np.outer(d,d))
        if mass<=0 or np.min(np.linalg.eigvalsh(inertia))<=0:raise ValueError('刚体质量或惯量非正')
        inertials[body_id]={'mass_kg':float(mass),'center_m':center.tolist(),'local_center_m':(center-origins[body_id]).tolist(),
            'inertia_kg_m2':inertia.tolist(),'evidence':'engineering_assumption'}
    model=ET.Element('mujoco',model='robot_design_candidate')
    ET.SubElement(model,'compiler',angle='radian',inertiafromgeom='false')
    ET.SubElement(model,'option',timestep='.002',gravity='0 0 -9.80665',integrator='implicitfast')
    model.append(ET.Comment('Fixed base. Contact deliberately disabled: display meshes are concave and are not validated convex collision proxies.'))
    asset_node=ET.SubElement(model,'asset');world=ET.SubElement(model,'worldbody');actuators=ET.SubElement(model,'actuator')
    for m in meshes:ET.SubElement(asset_node,'mesh',name=m['asset']['id'],file=m['filename'])
    body_nodes={};pending=list(graph['bodies'])
    while pending:
        available=[b for b in pending if b['id']==root or incoming[b['id']]['parent'] in body_nodes]
        if not available:raise ValueError('运动学图无法导出')
        for b in available:
            identity=b['id'];joint=incoming.get(identity);parent=joint['parent'] if joint else None
            node=ET.SubElement(world if parent is None else body_nodes[parent],'body',name=identity,
                pos=vec(origins[identity]-(origins[parent] if parent else 0)))
            body_nodes[identity]=node;inertia=inertials[identity];a=np.array(inertia['inertia_kg_m2'])
            ET.SubElement(node,'inertial',pos=vec(inertia['local_center_m']),mass=str(inertia['mass_kg']),
                fullinertia=vec([a[0,0],a[1,1],a[2,2],a[0,1],a[0,2],a[1,2]]))
            if joint:
                limits=np.deg2rad(joint['limits_deg']);ET.SubElement(node,'joint',name=joint['id'],type='hinge',axis=vec(joint['axis']),pos='0 0 0',limited='true',range=vec(limits),damping='.05')
                ET.SubElement(actuators,'position',name='drive_'+joint['id'],joint=joint['id'],kp='20',kv='1',
                    ctrllimited='true',ctrlrange=vec(limits),forcelimited='true',forcerange=vec([-config['assumed_torque_nm'],config['assumed_torque_nm']]))
            for m in meshes:
                if m['asset']['owner_body_id']==identity:ET.SubElement(node,'geom',name='visual_'+m['asset']['id'],type='mesh',mesh=m['asset']['id'],contype='0',conaffinity='0',group='1',mass='0')
            pending.remove(b)
    ET.indent(model);ET.ElementTree(model).write(directory/'robot.xml',encoding='utf-8',xml_declaration=True)
    urdf=ET.Element('robot',name='robot_design_candidate');urdf.append(ET.Comment('Assumed masses. No validated collision geometry. Fixed-base dynamics candidate only.'))
    ET.SubElement(urdf,'link',name='world')
    fixed=ET.SubElement(urdf,'joint',name='studio_fixed_base',type='fixed');ET.SubElement(fixed,'parent',link='world');ET.SubElement(fixed,'child',link=root)
    for b in graph['bodies']:
        identity=b['id'];link=ET.SubElement(urdf,'link',name=identity);record=inertials[identity];a=np.array(record['inertia_kg_m2'])
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',xyz=vec(record['local_center_m']),rpy='0 0 0');ET.SubElement(inertial,'mass',value=str(record['mass_kg']))
        ET.SubElement(inertial,'inertia',**{k:str(a[i,j]) for k,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]})
        for m in meshes:
            if m['asset']['owner_body_id']==identity:ET.SubElement(ET.SubElement(ET.SubElement(link,'visual'),'geometry'),'mesh',filename=m['filename'])
        if identity in incoming:
            j=incoming[identity];joint=ET.SubElement(urdf,'joint',name=j['id'],type='revolute');ET.SubElement(joint,'parent',link=j['parent']);ET.SubElement(joint,'child',link=identity)
            ET.SubElement(joint,'origin',xyz=vec(origins[identity]-origins[j['parent']]),rpy='0 0 0');ET.SubElement(joint,'axis',xyz=vec(j['axis']))
            ET.SubElement(joint,'limit',lower=str(math.radians(j['limits_deg'][0])),upper=str(math.radians(j['limits_deg'][1])),effort=str(config['assumed_torque_nm']),velocity='2')
    ET.indent(urdf);ET.ElementTree(urdf).write(directory/'robot.urdf',encoding='utf-8',xml_declaration=True)
    # Static delivery checks only; no physics engine or trajectories.
    checks=mesh_export_checks+check_package(directory,graph,origins)
    mapping=[{'joint_id':j['id'],'motor_instance_id':next(b['motor_instance_id'] for b in project['motor_bindings'] if b['joint_id']==j['id']),
        'mjcf_actuator_name':'drive_'+j['id'],'actual_zero_rad':None,'actual_direction':None,'hardware_ready':False} for j in graph['joints']]
    result={'project_id':project['id'],'design_revision':project['revision'],'pipeline':'simulator_export_0.2',
        'export_valid':all(c['passed'] for c in checks),'simulation_performed':False,
        'manufacturing_ready':False,'physical_verified':False,'layout_run_id':project['layout_result']['run_id'],
        'checks':checks,'visual_assets':list(visual_assets.values()),'mass_report':mass_report,'body_inertials':inertials,'joint_mapping':mapping,
        'total_assumed_mass_kg':sum(x['mass_kg'] for x in mass_report),'parameters':config,
        'scope':'URDF / MJCF 与米制局部网格导出；仅检查文件、资源与坐标定义，不运行仿真，不生成控制程序',
        'pending':['质量与惯量由显式假设密度计算，实物质量未知',
            '固定底座；接触禁用，URDF 不含碰撞代理；需要接触仿真时应在目标工具中配置',
            '力矩、阻尼、速度及执行器参数为导出占位假设，不代表电机能力'],
        'elapsed_s':round(time.monotonic()-start,3),'generator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (directory/'README.txt').write_text('URDF / MJCF 模型包。解压后保持网格和 robot.urdf / robot.xml 位于同一目录，再在目标仿真软件中打开。\n单位：m、kg、rad；固定底座，质量与惯量为工程假设。未生成碰撞代理，接触禁用。\n只执行静态文件/资源/坐标检查；未运行仿真，未生成控制程序。\n',encoding='utf-8')
    atomic(directory/'manifest.json',result);atomic(directory/'input-project.json',project)
    result['artifacts']=sorted(p.name for p in directory.iterdir() if p.is_file() and p.suffix!='.zip' and p.name not in {'run.json','input.json','worker.log','result.json'})
    result['artifact_sha256']={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in result['artifacts']}
    with zipfile.ZipFile(directory/'simulation-model.zip','w',zipfile.ZIP_DEFLATED) as package:
        for filename in result['artifacts']:package.write(directory/filename,filename)
    result['artifacts'].append('simulation-model.zip');atomic(directory/'result.json',result);return result


def check_package(directory,graph,origins):
    """Check the exported contract without executing a simulation or model loader."""
    directory=Path(directory)
    mjcf=ET.parse(directory/'robot.xml').getroot();urdf=ET.parse(directory/'robot.urdf').getroot()
    resources=[n.attrib['file'] for n in mjcf.findall('./asset/mesh')]+[n.attrib['filename'] for n in urdf.findall('.//visual/geometry/mesh')]
    resource_ok=bool(resources) and all(Path(name).name==name and (directory/name).is_file() for name in resources)
    body_ids={b['id'] for b in graph['bodies']};joint_ids={j['id'] for j in graph['joints']}
    tree_ok=({n.attrib['name'] for n in mjcf.findall('.//body')}==body_ids
        and {n.attrib['name'] for n in urdf.findall('./link')}==body_ids|{'world'}
        and {n.attrib['name'] for n in mjcf.findall('.//joint')}==joint_ids
        and {n.attrib['name'] for n in urdf.findall('./joint') if n.attrib['type']=='revolute'}==joint_ids)
    coordinate_ok=True
    for j in graph['joints']:
        uj=next(n for n in urdf.findall('./joint') if n.attrib['name']==j['id'])
        mb=next(n for n in mjcf.findall('.//body') if n.attrib['name']==j['child'])
        mj=mb.find('./joint');expected=origins[j['child']]-origins[j['parent']]
        coordinate_ok=coordinate_ok and all(np.allclose(np.fromstring(text,sep=' '),value,atol=1e-10) for text,value in [
            (uj.find('./origin').attrib['xyz'],expected),(mb.attrib['pos'],expected),
            (uj.find('./axis').attrib['xyz'],j['axis']),(mj.attrib['axis'],j['axis'])])
        coordinate_ok=coordinate_ok and uj.find('./parent').attrib['link']==j['parent'] and uj.find('./child').attrib['link']==j['child']
    return [{'name':'xml_documents','passed':mjcf.tag=='mujoco' and urdf.tag=='robot'},
        {'name':'local_mesh_resources','passed':resource_ok,'mesh_count':len(resources)//2},
        {'name':'body_joint_definitions','passed':tree_ok},
        {'name':'joint_coordinate_definitions','passed':bool(coordinate_ok)}]
