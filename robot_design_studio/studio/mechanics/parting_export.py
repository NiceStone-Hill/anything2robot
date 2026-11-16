"""Export a shell-parting stage without claiming an assembled robot is ready."""
import hashlib
import time
import zipfile
import numpy as np
import trimesh as tm
from ..project import atomic
from .solid import regions
from .articulation import PALETTE


def export(project,directory,source_path,pieces,lids,panels,modules,analysis,started):
    from .graph_assembly import stl_deliverable
    graph=project['articulation']['graph'];assets=[];preview=[];checks=[];scene=tm.Scene()
    for index,body in enumerate(graph['bodies']):
        owner=body['id'];color=PALETTE[index%len(PALETTE)]
        for suffix,mesh in [('_shell',pieces[owner]),('_lid',lids[owner])]:
            if mesh.is_empty:continue
            identity=owner+suffix;mesh,report=stl_deliverable(mesh)
            valid=report['passed'] and mesh.is_volume and regions(mesh)==1
            file=identity+'.stl'
            mesh.export(directory/file,file_type='stl_ascii' if report.get('stl_encoding')=='ascii' else 'stl')
            reopened=tm.load(directory/file,force='mesh')
            valid=bool(valid and reopened.is_volume and reopened.is_watertight)
            checks.append({'name':identity+'_shell_export','passed':valid,'export_report':report})
            name=body['name']+(' · 主壳' if suffix=='_shell' else ' · 可拆壳盖')
            assets.append({'id':identity,'name':name,'owner_body_id':owner,'file':file,
                'template_id':'shell_parting','template_revision':'1',
                'sha256':hashlib.sha256((directory/file).read_bytes()).hexdigest(),
                'export_length_unit':'mm','bounds_m':(mesh.bounds/1000).tolist(),
                'volume_m3':float(mesh.volume)*1e-9,'printable_candidate':True,
                'scope':'shell_parting_only_not_complete_robot'})
            shown=mesh.simplify_quadric_decimation(face_count=3500) if len(mesh.faces)>3500 else mesh
            record={'id':identity,'name':name,'owner':owner,'vertices':(shown.vertices/1000).tolist(),
                    'faces':shown.faces.tolist(),'color':color,'hardware':False}
            if suffix=='_lid':
                panel=panels[owner]
                record['opening_offset_m']=(np.asarray(panel['normal'])*panel['extraction_distance_mm']/1000).tolist()
            preview.append(record)
            shown=shown.copy().apply_scale(.001);shown.visual.face_colors=tm.visual.color.hex_to_rgba(color)
            scene.add_geometry(shown,node_name=identity)
    module_interfaces=[]
    for p in modules:
        mesh=p['mesh'];shown=mesh.simplify_quadric_decimation(face_count=2500) if len(mesh.faces)>2500 else mesh
        reference='reference-'+p['id']+'.npz'
        np.savez_compressed(directory/reference,vertices=mesh.vertices,faces=mesh.faces)
        module_interfaces.append({'id':p['id'],'joint_id':p['joint_id'],'owner_body_id':p['owner'],
            'role':p['role'],'template_id':p['template'],'template_revision':p['template_revision'],
            'geometry_file':reference,'length_unit':'mm','manufacturing_part':False,
            'requires_shell_attachment':p['role'] in ('fixed_mount','output_mount'),
            'sha256':hashlib.sha256((directory/reference).read_bytes()).hexdigest()})
        preview.append({'id':p['id'],'name':p['name']+' · 位置参考，尚未连接外壳','owner':p['owner'],
            'vertices':(shown.vertices/1000).tolist(),'faces':shown.faces.tolist(),'color':'#d5a14d',
            'hardware':True,'scope':'position_reference_only'})
    for owner,panel in panels.items():
        panel.update(owner_body_id=owner,cover_asset_id=owner+'_lid')
        checks.append({'name':owner+'_empty_shell_opening','passed':bool(panel['continuous_extraction']),
                       'scope':'empty_shell_parts_without_installed_hardware'})
    analysis['lid_planes']=list(panels.values())
    result={'project_id':project['id'],'design_revision':project['revision'],
        'status':'shell_partition_ready','shell_partition_valid':all(c['passed'] for c in checks),
        'geometry_valid':False,'manufacturing_ready':False,'pipeline':'shell_partition_1',
        'source_sha256':analysis['source_sha256'],'analysis':analysis,'checks':checks,'warnings':[],
        'geometry_assets':assets,'mounts':[],'joints':[],'rigid_bodies':graph['bodies'],
        'module_interfaces':module_interfaces,
        'mounting_preferences':[{'owner_body_id':owner,'preferred_board_normal':panel['normal'],
                                'shell_assignment_pending':True} for owner,panel in panels.items()],
        'assembly_dependencies':[{'action':'拆开壳件，沿切面法向放入螺母；合拢两件后从外侧拧入螺丝。安装内部硬件后须重新检查通道。'}],
        'hardware_requests':[{'catalog_id':'fastener_candidate','specification':panel['fasteners'][0].get('candidate_thread','M2')+' 螺钉与螺母套件',
            'quantity':len(panel['fasteners']),'purpose':owner+' 壳体闭合','owner_body_id':owner,
            'length_m':None,'required_underhead_lengths_m':[f['required_underhead_length_mm']/1000 for f in panel['fasteners']],
            'physical_fit_verified':False} for owner,panel in panels.items() if panel['fasteners']],
        'plan':{'reason':'壳体切分阶段：可拆空壳已生成，舵机安装座、电子支架及接线尚未完成；不是完整机器人打印包'},
        'pending':['安装座连接与分件归属','安装硬件后的装配与拆卸检查','Pico 2、电源与排线','实物配合'],
        'elapsed_s':round(time.monotonic()-started,3)}
    atomic(directory/'preview.json',{'parts':preview,'graph':graph,'joint':{'limits_rad':[-1,1]}})
    scene.export(directory/'assembly.glb')
    atomic(directory/'manifest.json',result);atomic(directory/'input-project.json',project)
    (directory/'source.raw').write_bytes(source_path.read_bytes())
    (directory/'README.txt').write_text('SHELL PARTING STAGE ONLY\nUnits: mm.\nThese shells do not yet contain completed actuator supports or electronics mounts.\nDo not treat this package as a print-ready assembled robot.\n',encoding='utf-8')
    result['artifacts']=sorted(p.name for p in directory.iterdir() if p.is_file() and p.suffix!='.zip'
                              and p.name not in {'run.json','input.json','worker.log','result.json'})
    with zipfile.ZipFile(directory/'shell-partition-review.zip','w',zipfile.ZIP_DEFLATED) as package:
        for name in result['artifacts']:package.write(directory/name,name)
    result['artifacts'].append('shell-partition-review.zip');atomic(directory/'result.json',result)
    return result
