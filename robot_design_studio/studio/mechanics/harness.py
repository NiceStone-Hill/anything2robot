"""Bounded same-body engineering routes with explicit, unmeasured terminal assumptions."""
import json,heapq,math,time,csv,hashlib,zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
from scipy.ndimage import binary_dilation
from ..project import atomic
from ..contracts import now
from .electronics_layout import read_mesh,overlap
from .graph_assembly import poses,sample_angles

def tube(a,b,radius):
    delta=np.asarray(b)-a;length=float(np.linalg.norm(delta))
    if length<1e-8:return tm.Trimesh()
    matrix=tm.geometry.align_vectors([0,0,1],delta/length);matrix[:3,3]=(np.asarray(a)+b)/2
    return tm.creation.cylinder(radius,length,sections=12,transform=matrix)

def astar(free,start,end):
    start=tuple(start);end=tuple(end);heap=[(0.,0.,start)];cost={start:0.};previous={};shape=free.shape
    while heap:
        _,g,node=heapq.heappop(heap)
        if g!=cost[node]:continue
        if node==end:
            path=[node]
            while node in previous:node=previous[node];path.append(node)
            return path[::-1]
        if len(cost)>150000:break
        for axis in range(3):
            for sign in [-1,1]:
                nxt=list(node);nxt[axis]+=sign;nxt=tuple(nxt)
                if any(v<0 or v>=shape[i] for i,v in enumerate(nxt)) or not free[nxt]:continue
                ng=g+1
                if ng<cost.get(nxt,float('inf')):
                    cost[nxt]=ng;previous[nxt]=node;heapq.heappush(heap,(ng+sum(abs(nxt[i]-end[i]) for i in range(3)),ng,nxt))
    raise ValueError('当前格距与内部障碍下找不到线束通道')

def generate_individual(project,layout_directory,directory):
    started=time.monotonic();directory=Path(directory);source=Path(layout_directory)
    layout=json.loads((source/'result.json').read_text());graph=project['articulation']['graph'];root=graph['root_body_id']
    if not layout['layout_valid']:raise ValueError('内部布局尚未通过')
    events=[]
    def progress(message):events.append({'at':now(),'detail':message});atomic(directory/'progress.json',events)
    parts=[{'record':a,'mesh':read_mesh(source,a)} for a in layout['geometry_assets']]
    placements={v['instance_id']:v for v in layout['component_placements']};instances={i['id']:i for i in project['instances']}
    endpoints={}
    for net in project['circuit']['nets']:
        for ep in net['endpoints']:endpoints[(ep['instance_id'],ep['port_id'])]=ep
    owner={};meshes={}
    for identity,_ in endpoints:
        if identity in meshes:continue
        if identity in placements:
            owner[identity]=placements[identity]['owner_body_id'];asset=placements[identity]['envelope_asset_id']
            meshes[identity]=next(p['mesh'] for p in parts if p['record']['id']==asset)
        else:
            binding=next((b for b in project['motor_bindings'] if b['motor_instance_id']==identity),None)
            if not binding:raise ValueError('连接端点缺少实体包络：'+identity)
            joint=next(j for j in graph['joints'] if j['id']==binding['joint_id']);owner[identity]=joint['parent'] if 'parent' in joint else joint['parent_body_id']
            candidates=[p for p in parts if p['record']['owner_body_id']==owner[identity] and not p['record']['printable_candidate'] and binding['joint_id'] in p['record']['id']]
            if not candidates:raise ValueError('电机本体缺少固定侧资产')
            meshes[identity]=max(candidates,key=lambda p:p['mesh'].volume)['mesh']
    if any(v!=root for v in owner.values()):raise ValueError('当前排线求解只开放同一固定刚体；跨关节柔性线束仍须单独生成，不能按静态直线冒充')
    distributors=[i['id'] for i in project['instances'] if i['catalog_id']=='splitter_candidate' and i['id'] in placements]
    if any(len(n['endpoints'])>2 for n in project['circuit']['nets']) and not distributors:raise ValueError('多端网络缺少实体分线器')
    for identity in distributors:
        owner[identity]=placements[identity]['owner_body_id'];meshes[identity]=next(p['mesh'] for p in parts if p['record']['id']==placements[identity]['envelope_asset_id'])
    bounds=np.array([m.bounds for m in meshes.values()]);pitch=project['harness_config']['grid_mm'];radius=project['harness_config']['wire_diameter_mm']/2
    lo=np.floor((bounds[:,0].min(0)-20)/pitch)*pitch;hi=np.ceil((bounds[:,1].max(0)+20)/pitch)*pitch
    axes=[np.arange(lo[i],hi[i]+pitch/2,pitch) for i in range(3)];shape=tuple(len(a) for a in axes)
    if np.prod(shape)>250000:raise ValueError('排线局部网格超过250000点预算')
    points=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    mechanical=source.parent/project['layout_config']['mechanical_run_id'];manifest=json.loads((mechanical/'result.json').read_text())
    domain=read_mesh(mechanical,next(a for a in manifest['layout_domains'] if a['owner_body_id']==root))
    progress('建立真实壳体内部的排线网格；端口位置明确为试算假设')
    def contains(mesh,pts):
        result=np.zeros(len(pts),bool);inside=np.flatnonzero(np.all((pts>=mesh.bounds[0])&(pts<=mesh.bounds[1]),axis=1))
        for start in range(0,len(inside),512):
            selected=inside[start:start+512];result[selected]=mesh.contains(pts[selected])
        return result
    free=contains(domain,points)
    for item in parts:
        free &= ~contains(item['mesh'],points)
    free=free.reshape(shape)
    # One-cell clearance is conservative; exact cylindrical segment checks follow.
    free &= ~binary_dilation(~free,iterations=1)
    valid=np.argwhere(free)
    if not len(valid):raise ValueError('内部没有满足线束网格间隙的空间')
    xyz=lo+valid*pitch
    terminals={};reserved=set()
    def terminal(identity,port):
        key=(identity,port)
        if key in terminals:return terminals[key]
        mesh=meshes[identity];bounds=mesh.bounds;center=bounds.mean(0)
        distance=np.linalg.norm(np.maximum(np.maximum(bounds[0]-xyz,xyz-bounds[1]),0),axis=1)
        score=distance+np.linalg.norm(xyz-center,axis=1)*.01
        for idx in np.argsort(score):
            node=tuple(valid[idx])
            if free[node] and node not in reserved and distance[idx]<=16:
                reserved.add(node);terminals[key]=node;return node
        raise ValueError('部件附近找不到试算接线端口空间：'+identity)
    topology=[]
    for net in project['circuit']['nets']:
        eps=[(e['instance_id'],e['port_id']) for e in net['endpoints']]
        if len(eps)>2:
            for index,ep in enumerate(eps):topology.append((net,ep,(distributors[0],net['id']+'_junction_'+str(index))))
        else:topology.append((net,*eps))
    wire_instances=[i['id'] for i in project['instances'] if i['catalog_id']=='wire_candidate']
    if len(wire_instances)<len(topology):raise ValueError('所选连接线数量不足：需要 '+str(len(topology)))
    def endpoint(identity,port):
        if identity in distributors:
            return {'instance_id':identity,'port_id':None,'junction_id':port,'evidence':'engineering_assumption_unregistered_junction'}
        return {'instance_id':identity,'port_id':port}
    routes=[];wire_meshes=[];checks=[]
    obstacles=[item['mesh'] for item in parts]
    def segment_ok(a,b):
        m=tube(a,b,radius)
        return not any(overlap(m,other)>.01 for other in obstacles)
    for index,(net,a,b) in enumerate(topology):
        progress('排线 '+str(index+1)+' / '+str(len(topology))+'：'+net['id'])
        nodes=astar(free,terminal(*a),terminal(*b));path=[lo+np.array(n)*pitch for n in nodes]
        # Visibility shortcut with exact cylindrical collision tests, not just centres.
        smooth=[path[0]];cursor=0
        while cursor<len(path)-1:
            nxt=len(path)-1
            while nxt>cursor+1 and not segment_ok(path[cursor],path[nxt]):nxt-=1
            if not segment_ok(path[cursor],path[nxt]):raise ValueError('网格路径的实际线径发生碰撞：'+net['id'])
            smooth.append(path[nxt]);cursor=nxt
        pieces=[tube(a,b,radius) for a,b in zip(smooth,smooth[1:])]
        mesh=tm.util.concatenate(pieces);wire_meshes.append(mesh)
        length=sum(np.linalg.norm(b-a) for a,b in zip(smooth,smooth[1:]))
        obstacles.append(mesh)
        blocked=np.zeros(len(points),bool)
        for start,end in zip(smooth,smooth[1:]):
            delta=end-start;t=np.clip(((points-start)@delta)/(delta@delta),0,1)
            blocked|=np.linalg.norm(points-(start+t[:,None]*delta),axis=1)<radius*2+.25
        free.reshape(-1)[blocked]=False
        routes.append({'id':'wire_'+str(index+1),'wire_instance_id':wire_instances[index],'net_id':net['id'],'kind':net['kind'],'owner_body_id':root,
            'from':endpoint(*a),'to':endpoint(*b),'points_m':(np.array(smooth)/1000).tolist(),
            'centerline_length_m':float(length/1000),'suggested_cut_length_m':float((length+30)/1000),'diameter_m':radius*2/1000,
            'terminal_evidence':'engineering_assumption_not_measured','connector_verified':False,'bend_radius_verified':False})
    for angles in sample_angles(graph):
        moved=poses(graph,angles)
        bad=[]
        for item in parts:
            body=item['record']['owner_body_id']
            if body==root:continue
            obstacle=item['mesh'].copy().apply_transform(moved[body])
            for index,mesh in enumerate(wire_meshes):
                if overlap(mesh,obstacle)>.01:bad.append(index+1)
        checks.append({'name':'moving_parts_clearance','angles_rad':angles,'passed':not bad,'colliding_wires':bad})
    preview=json.loads((source/'preview.json').read_text());scene=tm.Scene()
    for item in parts:scene.add_geometry(item['mesh'].copy().apply_scale(.001),node_name=item['record']['id'])
    colors={'ground':'#454545','power':'#d97339','pwm':'#b45bb9','i2c_sda':'#29a77a','i2c_scl':'#397bc3'}
    for route,mesh in zip(routes,wire_meshes):
        color=colors.get(route['kind'],'#e6b34c');m=mesh.copy().apply_scale(.001);m.visual.face_colors=tm.visual.color.hex_to_rgba(color);scene.add_geometry(m,node_name=route['id'])
        preview['parts'].append({'id':route['id'],'name':route['net_id']+' · 试算线束','owner':root,'vertices':m.vertices.tolist(),'faces':m.faces.tolist(),'color':color,'hardware':True})
    scene.export(directory/'assembly-with-wires.glb');atomic(directory/'preview.json',preview)
    result={'structure_verified':layout.get('structure_verified',True),'inherited_structure_checks':layout.get('inherited_structure_checks',[]),'project_id':project['id'],'design_revision':project['revision'],'pipeline':'same-body-harness-0.1','routing_valid':all(c['passed'] for c in checks),
        'manufacturing_ready':False,'layout_run_id':project['layout_result']['run_id'],'routes':routes,'checks':checks,'terminal_assumptions':[{**endpoint(*k),'position_m':((lo+np.array(v)*pitch)/1000).tolist(),'evidence':'engineering_assumption','physical_position_m':None} for k,v in terminals.items()],
        'unresolved_electrical_requests':project['circuit'].get('missing_parts',[]),'scope':'同一固定刚体内的实体避障试算；分支落到已布局分线器，电机固定侧接线，无跨关节活动线。',
        'pending':['端口与连接器位置为显式试算假设，必须实测后重算','线材电流、线径、弯曲半径、固定线夹及实际插拔未验证','路径折点尚未生成制造级圆角；不得直接作为裁线装配放行依据'],
        'elapsed_s':round(time.monotonic()-started,3),'artifacts':['preview.json','assembly-with-wires.glb','harness.json','wire-list.csv','harness-review.zip']}
    atomic(directory/'harness.json',result)
    with (directory/'wire-list.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['wire','net','length_m','cut_length_assumption_m','from_instance','from_port','to_instance','to_port'])
        for r in routes:writer.writerow([r['id'],r['net_id'],r['centerline_length_m'],r['suggested_cut_length_m'],r['from']['instance_id'],r['from']['port_id'],r['to']['instance_id'],r['to']['port_id']])
    with zipfile.ZipFile(directory/'harness-review.zip','w',zipfile.ZIP_DEFLATED) as z:
        for file in result['artifacts']:
            if not file.endswith('.zip'):z.write(directory/file,file)
    result['artifact_sha256']={file:hashlib.sha256((directory/file).read_bytes()).hexdigest() for file in result['artifacts']}
    atomic(directory/'result.json',result);return result


def generate(project,layout_directory,directory):
    if project['harness_config'].get('routing_mode','bundled')=='individual':return generate_individual(project,layout_directory,directory)
    from .bundled_harness import generate as bundled
    return bundled(project,layout_directory,directory)
