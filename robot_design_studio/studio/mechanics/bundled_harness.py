"""Shared trunk/branch corridors adapted from the project's verified legacy router.

Cable membership and cross-section packing are retained. Aggregation exits and
branch fan-out remain explicit assumptions, exactly as in the legacy corridor.
"""
import csv,hashlib,json,time,zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
from scipy.ndimage import distance_transform_edt,label
from . import bundle_network as network,component_visuals
from .electronics_layout import read_mesh,overlap
from .graph_assembly import poses,sample_angles,stl_deliverable
from .solid import union
from .harness import tube
from ..contracts import now
from ..project import atomic

COLORS={'ground':'#454545','power':'#d97339','pwm':'#b45bb9','i2c_sda':'#29a77a','i2c_scl':'#397bc3'}

def corridor(points,radius):
    pieces=[tube(a,b,radius) for a,b in zip(points,points[1:]) if np.linalg.norm(b-a)>1e-6]
    pieces += [tm.creation.icosphere(subdivisions=2,radius=radius).apply_translation(p) for p in points]
    return union(*pieces)

def contains(mesh,points):
    result=np.zeros(len(points),bool);inside=np.flatnonzero(np.all((points>=mesh.bounds[0])&(points<=mesh.bounds[1]),axis=1))
    for start in range(0,len(inside),512):
        selected=inside[start:start+512];result[selected]=mesh.contains(points[selected])
    return result

def generate(project,layout_directory,directory):
    started=time.monotonic();source=Path(layout_directory);directory=Path(directory);layout=json.loads((source/'result.json').read_text())
    if not layout['layout_valid'] or layout['project_id']!=project['id'] or layout['design_revision']!=project['revision']:raise ValueError('Complete the current project version internal layout first.')
    graph=project['articulation']['graph'];root=graph['root_body_id'];events=[]
    def progress(detail):events.append({'at':now(),'detail':detail});atomic(directory/'progress.json',events)
    parts=[{'record':a,'mesh':read_mesh(source,a)} for a in layout['geometry_assets']]
    placements={v['instance_id']:v for v in layout['component_placements']}
    by_asset={p['record']['id']:p['mesh'] for p in parts};meshes={};owners={}
    endpoints={(ep['instance_id'],ep['port_id']) for net in project['circuit']['nets'] for ep in net['endpoints']}
    for identity,_ in sorted(endpoints):
        if identity in placements:
            owners[identity]=placements[identity]['owner_body_id'];meshes[identity]=by_asset[placements[identity]['envelope_asset_id']]
        else:
            binding=next((b for b in project['motor_bindings'] if b['motor_instance_id']==identity),None)
            if binding is None:raise ValueError('No physical envelope for electrical endpoint: '+identity)
            joint=next(j for j in graph['joints'] if j['id']==binding['joint_id']);owners[identity]=joint['parent']
            candidates=[p['mesh'] for p in parts if p['record']['owner_body_id']==owners[identity] and not p['record']['printable_candidate'] and binding['joint_id'] in p['record']['id']]
            if not candidates:raise ValueError('Motor envelope is missing.')
            meshes[identity]=max(candidates,key=lambda m:m.volume)
    distributors=[i['id'] for i in project['instances'] if i['catalog_id']=='splitter_candidate' and i['id'] in placements]
    for identity in distributors:owners[identity]=placements[identity]['owner_body_id'];meshes[identity]=by_asset[placements[identity]['envelope_asset_id']]
    if any(owner!=root for owner in owners.values()):raise ValueError('Cross-joint flexible harness routing is not registered; use one fixed body.')
    topology=[]
    for net in project['circuit']['nets']:
        ends=[dict(ep) for ep in net['endpoints']]
        if len(ends)>2:
            if not distributors:raise ValueError('Select and place a distribution component for multi-endpoint nets.')
            for index,end in enumerate(ends):topology.append((net,end,{'instance_id':distributors[0],'port_id':None,'junction_id':net['id']+'_'+str(index),'junction_net_id':net['id'],'evidence':'assumed_distribution_contact'}))
        elif len(ends)==2:topology.append((net,*ends))
        else:raise ValueError('A wire net needs at least two endpoints.')
    wires=[i['id'] for i in project['instances'] if i['catalog_id']=='wire_candidate']
    if len(wires)<len(topology):raise ValueError('Not enough wire instances: '+str(len(topology)))
    pitch=project['harness_config']['grid_mm'];diameter=project['harness_config']['wire_diameter_mm'];gap=.2;sleeve=.5;clearance_mm=.5
    maximum_radius,_=network.pack((diameter,)*len(topology),gap,sleeve)
    bounds=np.asarray([m.bounds for m in meshes.values()]);lo=np.floor((bounds[:,0].min(0)-20)/pitch)*pitch;hi=np.ceil((bounds[:,1].max(0)+20)/pitch)*pitch
    axes=[np.arange(lo[i],hi[i]+pitch/2,pitch) for i in range(3)];shape=tuple(len(v) for v in axes)
    if np.prod(shape)>250000:raise ValueError('Routing grid exceeds 250000 cells. Increase grid size.')
    points=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    mechanical=source.parent/project['layout_config']['mechanical_run_id'];manifest=json.loads((mechanical/'result.json').read_text())
    domain=read_mesh(mechanical,next(a for a in manifest['layout_domains'] if a['owner_body_id']==root))
    progress('Build a shared corridor from the current shell, mounts and component envelopes.')
    occupied=~contains(domain,points)
    for item in parts:occupied |= contains(item['mesh'],points)
    raw_free=(~occupied).reshape(shape);clearance=distance_transform_edt(raw_free)*pitch
    free=raw_free&(clearance>maximum_radius+clearance_mm+pitch*.87)
    components,count=label(free)
    if not count:raise ValueError('No space for the packed cable bundle at full wire diameter.')
    sizes=np.bincount(components.ravel());sizes[0]=0;free=components==sizes.argmax()
    cells=np.argwhere(free);xyz=lo+cells*pitch
    if len(cells)<2:raise ValueError('No connected shared routing space.')
    center=np.mean([m.bounds.mean(0) for m in meshes.values()],axis=0)
    start=int(np.argmin(np.linalg.norm(xyz-center,axis=1)-.4*clearance[tuple(cells.T)]))
    cells,ids,distances,_=network.rooted_grid_tree(free,clearance,cells[start]);xyz=lo+cells*pitch
    terminals={};chosen=[]
    obstacles=[p['mesh'] for p in parts]
    def collision(mesh):return any(overlap(mesh,other)>.01 for other in obstacles)
    device_ids=sorted({end['instance_id'] for _,a,b in topology for end in [a,b]})
    for identity in device_ids:
        bounds=meshes[identity].bounds;distance=np.linalg.norm(np.maximum(np.maximum(bounds[0]-xyz,xyz-bounds[1]),0),axis=1)
        score=distance+.02*np.linalg.norm(xyz-bounds.mean(0),axis=1)+.01*distances
        found=None
        for index in np.argsort(score):
            if not np.isfinite(distances[index]) or distance[index]>45:continue
            if chosen and np.min(np.linalg.norm(np.asarray(chosen)-xyz[index],axis=1))<max(10,2*maximum_radius+1):continue
            sphere=tm.creation.icosphere(subdivisions=2,radius=maximum_radius+clearance_mm).apply_translation(xyz[index])
            if collision(sphere):continue
            found=int(index);break
        if found is None:raise ValueError('No aggregation exit near '+identity+' within 45 mm. No component was moved or shrunk.')
        chosen.append(xyz[found]);terminals[identity]={'node':found,'position_m':(xyz[found]/1000).tolist(),'distance_to_envelope_mm':float(distance[found]),
            'instance_id':identity,'port_id':None,'evidence':'assumed_device_aggregation_exit','physical_position_m':None,'connector_fanout_verified':False}
    progress('Grow one shared tree; preserve each wire identity and calculate packed segment diameters.')
    parent=network.growing_tree(free,clearance,{p['node'] for p in terminals.values()},terminals[device_ids[0]]['node'])
    paths={};routes=[]
    for index,(net,a,b) in enumerate(topology):
        key='wire_'+str(index+1);path=network.tree_path(terminals[a['instance_id']]['node'],terminals[b['instance_id']]['node'],parent)
        if len(path)<2:raise ValueError('Device exits collapsed to one node.')
        paths[key]=path
        routes.append({'id':key,'wire_instance_id':wires[index],'net_id':net['id'],'kind':net['kind'],'owner_body_id':root,'from':a,'to':b,
            'diameter_m':diameter/1000,'terminal_evidence':'assumed_device_aggregation_exit','connector_verified':False,'bend_radius_verified':False})
    segments=network.shared_segments(paths,{p['node'] for p in terminals.values()});solids={};checks=[]
    for index,segment in enumerate(segments):
        segment['id']='bundle_'+str(index+1);radius,packing=network.pack((diameter,)*len(segment['cables']),gap,sleeve)
        path=xyz[segment['nodes']];smooth=[path[0]];cursor=0
        while cursor<len(path)-1:
            nxt=len(path)-1
            while nxt>cursor+1 and collision(corridor([path[cursor],path[nxt]],radius+clearance_mm)):nxt-=1
            if collision(corridor([path[cursor],path[nxt]],radius+clearance_mm)):raise ValueError('Packed bundle clearance failed: '+segment['id'])
            smooth.append(path[nxt]);cursor=nxt
        smooth=np.asarray(smooth);mesh=corridor(smooth,radius);mesh,storage=stl_deliverable(mesh)
        if not storage['passed']:raise ValueError('Bundle solid export failed: '+segment['id'])
        file=segment['id']+'.stl';mesh.export(directory/file,file_type='stl_ascii' if storage.get('stl_encoding')=='ascii' else 'stl');reopened=tm.load(directory/file,force='mesh');solids[segment['id']]=reopened
        segment.update(points_m=(smooth/1000).tolist(),length_m=float(np.linalg.norm(np.diff(smooth,axis=0),axis=1).sum()/1000),diameter_m=2*radius/1000,
            packing_circles_xy_radius_mm=packing,packing_wire_order=segment['cables'],kind='shared_trunk' if len(segment['cables'])>1 else 'single_wire_branch',
            file=file,sha256=hashlib.sha256((directory/file).read_bytes()).hexdigest(),branch_transition_verified=False,visual_role='packed_bundle_corridor')
        checks.append({'name':'bundle_solid_and_clearance','segment_id':segment['id'],'passed':bool(reopened.is_volume and not collision(reopened)),'wire_count':len(segment['cables'])})
    for route in routes:
        path=paths[route['id']];items=network.cable_segment_path(segments,route['id'],path[0],path[-1]);restored=[];ordered=[]
        for item in items:
            segment=next(s for s in segments if s['id']==item['segment_id']);values=segment['points_m'][::-1] if item['reverse'] else segment['points_m']
            if restored and not np.allclose(restored[-1],values[0],atol=1e-10):raise ValueError('Bundle route is not continuous.')
            restored+=values if not restored else values[1:];ordered.append(item)
        pts=np.asarray(restored);length=float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum())
        route.update(points_m=restored,segments=ordered,centerline_length_m=length,suggested_cut_length_m=length+.03,length_evidence='shared_corridor_centerline_with_assumed_cut_allowance')
        checks.append({'name':'wire_membership_and_continuity','wire_id':route['id'],'passed':bool(len(restored)>=2 and ordered)})
    nonincident=[]
    for i,a in enumerate(segments):
        for b in segments[:i]:
            shared={a['nodes'][0],a['nodes'][-1]}&{b['nodes'][0],b['nodes'][-1]}
            if not shared and overlap(solids[a['id']],solids[b['id']])>.01:nonincident.append([a['id'],b['id']])
    checks.append({'name':'nonincident_bundle_clearance','passed':not nonincident,'collisions':nonincident})
    for angles in sample_angles(graph):
        moved=poses(graph,angles);bad=[]
        for item in parts:
            owner=item['record']['owner_body_id']
            if owner==root:continue
            obstacle=item['mesh'].copy().apply_transform(moved[owner])
            for key,mesh in solids.items():
                if overlap(mesh,obstacle)>.01:bad.append(key)
        checks.append({'name':'moving_parts_clearance','angles_rad':angles,'passed':not bad,'colliding_segments':bad})
    from .harness_connections import add_connections,guide_mesh
    progress('Map reference pads using CAD poses; check terminal leads and draw separately coloured wires.')
    endpoint_records=add_connections(project,layout,parts,meshes,terminals,segments,routes,raw_free&(clearance>diameter/2+.15+pitch*.87),lo,pitch)
    preview=json.loads((source/'preview.json').read_text());scene=component_visuals.scene(source,parts,layout.get('visual_assets',[]))
    for segment in segments:
        mesh=solids[segment['id']].copy().apply_scale(.001);color='#555a65' if len(segment['cables'])>1 else COLORS.get(next(r['kind'] for r in routes if r['id']==segment['cables'][0]),'#d2a052')
        preview['parts'].append({'id':segment['id'],'name':segment['kind']+' · '+str(len(segment['cables']))+' wires','owner':root,'vertices':mesh.vertices.tolist(),'faces':mesh.faces.tolist(),'color':color,'hardware':True,'role':'bundle_corridor'})
    for route in routes:
        groups=[('lane',route['guide_points_m'],False)]+[(side,route['fanout_points_m'][side],not route['endpoints'][side]['fanout_clearance_checked']) for side in ['from','to']]
        for suffix,points,dashed in groups:
            mesh=guide_mesh(points,diameter/2,dashed).apply_scale(.001)
            mesh.visual.face_colors=tm.visual.color.hex_to_rgba(route['color']);scene.add_geometry(mesh,node_name=route['id']+'_'+suffix)
            preview['parts'].append({'id':route['id']+'_'+suffix,'name':route['id']+' '+suffix,'owner':root,'vertices':mesh.vertices.tolist(),'faces':mesh.faces.tolist(),
                'color':route['color'],'hardware':True,'role':'wire_guide','wire_id':route['id'],'dashed':dashed})
        for side,ep in route['endpoints'].items():
            mesh=tm.creation.icosphere(subdivisions=1,radius=diameter*.65/1000).apply_translation(ep['position_m'])
            mesh.visual.face_colors=tm.visual.color.hex_to_rgba(route['color']);scene.add_geometry(mesh,node_name=route['id']+'_'+side+'_port')
            preview['parts'].append({'id':route['id']+'_'+side+'_port','name':ep['label'],'owner':root,'vertices':mesh.vertices.tolist(),'faces':mesh.faces.tolist(),
                'color':route['color'],'hardware':True,'role':'wire_endpoint','wire_id':route['id'],'instance_id':ep['instance_id']})
    preview['default_hidden_roles']=['bundle_corridor']
    scene.export(directory/'assembly-with-wires.glb');atomic(directory/'preview.json',preview)
    original=Path(__file__).resolve().parents[3]/'robot_interior_postprocess/harness_bundle/network.py'
    result={'structure_verified':layout.get('structure_verified',True),'inherited_structure_checks':layout.get('inherited_structure_checks',[]),'project_id':project['id'],'design_revision':project['revision'],'pipeline':'shared-bundle-harness-0.3-pin-guides','routing_mode':'bundled','routing_valid':all(c['passed'] for c in checks),
        'manufacturing_ready':False,'physical_verified':False,'layout_run_id':project['layout_result']['run_id'],'routes':routes,'bundle_segments':segments,'checks':checks,
        'terminal_assumptions':list(terminals.values()),'endpoint_registration':endpoint_records,
        'fanout_clearance_checked':all(r['fanout_clearance_checked'] for r in routes),'full_connection_verified':False,
        'fanout_checks':[{'wire_id':r['id'],'side':side,'instance_id':ep['instance_id'],'port_label':ep['label'],'passed':ep['fanout_clearance_checked'],'failure':ep.get('fanout_failure')} for r in routes for side,ep in r['endpoints'].items()],
        'visualization':{'wire_diameter_mm':diameter,'color_meaning':'Wire identity only; not physical insulation or polarity.','bundle_corridors_hidden_by_default':True,'lane_transitions_verified':False},
        'carrier_attachments':[{'instance_id':p['instance_id'],'carrier_asset_id':p['carrier_asset_id'],'support_asset_id':p.get('support_asset_id'),'lid_mounted':p.get('lid_mounted'),'owner_body_id':p['owner_body_id']} for p in layout['component_placements']],
        'shared_segments':sum(len(s['cables'])>1 for s in segments),'wire_count':len(routes),'device_count':len(terminals),
        'packing':{'wire_diameter_mm':diameter,'gap_mm':gap,'sleeve_wall_mm':sleeve,'clearance_mm':clearance_mm,'maximum_reserved_radius_mm':maximum_radius,'evidence':'engineering_assumption'},
        'legacy_algorithm':{'source':'robot_interior_postprocess/harness_bundle/network.py','sha256':hashlib.sha256(original.read_bytes()).hexdigest() if original.exists() else None,'adapter_sha256':hashlib.sha256(Path(network.__file__).read_bytes()).hexdigest()},
        'visual_assets':layout.get('visual_assets',[]),'unresolved_electrical_requests':project['circuit'].get('missing_parts',[]),
        'scope':'Checked shared corridors on one fixed body. Individually coloured assembly guides reach reference pads or explicitly unregistered face placeholders. routing_valid refers to corridors only.',
        'pending':['Reference pad positions are not measured connectors. MPU6050 label order is inferred from its source footprint; power, servo and distributor contacts remain placeholders.','Dashed leads failed fan-out clearance. Solid leads passed a static obstacle check; connector insertion, wire-to-wire transitions, bends and assembly access remain unverified.','Display colours identify wires only; verify port labels and polarity. Wire diameter, sleeve, cut lengths, bend radii, ties and strain relief require hardware selection.'],
        'elapsed_s':round(time.monotonic()-started,3)}
    atomic(directory/'harness.json',result)
    with (directory/'wire-list.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['wire','display_color','net','shared_corridor_length_m','corridor_cut_allowance_only_m','from_instance','from_port','from_pin','from_position_evidence','to_instance','to_port','to_pin','to_position_evidence','fanout_clearance_checked','physical_connection_verified','bundle_segments'])
        for route in routes:
            a,b=route['endpoints']['from'],route['endpoints']['to']
            writer.writerow([route['id'],route['color'],route['net_id'],route['centerline_length_m'],route['suggested_cut_length_m'],a['instance_id'],a['label'],a['physical_pin'],a['evidence'],b['instance_id'],b['label'],b['physical_pin'],b['evidence'],route['fanout_clearance_checked'],False,' '.join(i['segment_id'] for i in route['segments'])])
    model_files=component_visuals.copy_artifacts(source,directory,layout.get('visual_assets',[]))
    result['artifacts']=model_files+['preview.json','assembly-with-wires.glb','harness.json','wire-list.csv']+[s['file'] for s in segments]
    with zipfile.ZipFile(directory/'harness-review.zip','w',zipfile.ZIP_DEFLATED) as package:
        for file in result['artifacts']:package.write(directory/file,file)
    result['artifacts'].append('harness-review.zip');result['artifact_sha256']={file:hashlib.sha256((directory/file).read_bytes()).hexdigest() for file in result['artifacts']}
    atomic(directory/'result.json',result);return result
