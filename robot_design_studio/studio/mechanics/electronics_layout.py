"""Internal envelopes, removable strap carriers, shell anchors and extraction checks.

No PCB hole pattern or connector position is invented. Dimensions are explicit
engineering assumptions and all resulting solids remain unreleased candidates.
"""
import copy
import hashlib
import itertools
import json
import math
import time
import zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
from ..contracts import now
from ..project import atomic
from .solid import box, cylinder, union, cut, regions, convex_sweep, manifold_solid, intersect, dilate
from .graph_assembly import poses, sample_angles, stl_deliverable

VERSION = 'electronics-carrier-0.5-soft-extraction'


def carrier_depths(bounds, height, side):
    """Sample the cavity, not the split seam, for a tray's support face.

    An offset split can leave most of the cavity in the removable shell.
    Actual attachment surfaces are resolved by ray casting later.
    """
    low, high = np.asarray(bounds)[:, 2]
    depth = max(5., height + 2.) + .5
    start = low + 2. + (depth if side == -1 else 0.)
    stop = high - 2. - (depth if side == 1 else 0.)
    if stop < start:
        return []
    return np.linspace(start, stop, max(2, int(np.ceil((stop-start)/4.))+1)).tolist()


def read_mesh(directory, record):
    filename=record['file']
    if Path(filename).name!=filename:raise ValueError('结构资产路径无效')
    path=directory/filename
    if hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:raise ValueError('结构资产已改变')
    mesh=tm.load(path,force='mesh')
    if not mesh.is_volume:raise ValueError('布局输入实体不闭合')
    return mesh


def overlap(a,b):
    """Cache immutable kernels in Trimesh's geometry-invalidated cache during search."""
    if a.is_empty or b.is_empty or np.any(a.bounds[1]<=b.bounds[0]) or np.any(b.bounds[1]<=a.bounds[0]):return 0.
    def kernel(mesh):
        key='layout_boolean_kernel'
        value=mesh._cache[key]
        if value is None:
            value=manifold_solid(mesh);mesh._cache[key]=value
        return value
    return abs(float((kernel(a)^kernel(b)).volume()))


def collision(mesh, obstacles, tolerance=.1):
    return any(overlap(mesh,other)>tolerance for other in obstacles)


def carrier(width, depth, height):
    """Board bottom z=2; tray bottom z=0. Strap slots avoid board envelope."""
    tray=box([-width/2-8,-depth/2-5,0],[width/2+8,depth/2+5,2])
    stops=[box([-width/2-1,-depth/2-5,1],[width/2+1,-depth/2-3,5]),
           box([-width/2-1,depth/2+3,1],[width/2+1,depth/2+5,5])]
    holes=[cylinder(x,0,-1,6,1.2) for x in [-width/2-5,width/2+5]]
    slots=[box([x-1.5,y-1,-1],[x+1.5,y+1,3])
           for x in [-width/4,width/4] for y in [-depth/2-2,depth/2+2]]
    tray=cut(union(tray,*stops),*holes,*slots)
    envelope=box([-width/2,-depth/2,2],[width/2,depth/2,2+height])
    return tray,envelope


def upward_volume(bounds, top, margin=0):
    lo,hi=np.array(bounds)
    return box(lo+[-margin,-margin,.02],[hi[0]+margin,hi[1]+margin,top])


def generate(project, mechanical_directory, directory, _strategy=0, _started=None, _finalize=False):
    started=time.monotonic() if _started is None else _started;directory=Path(directory);source=Path(mechanical_directory)
    original=json.loads((source/'result.json').read_text());checks=[];events=[]
    def check(name,passed,**detail):checks.append({'name':name,'passed':bool(passed),**detail})
    def progress(detail):
        events.append({'at':now(),'type':'layout','detail':detail});atomic(directory/'progress.json',events)
    from ..layout_readiness import draft_layout_ready
    draft=bool(project['layout_config'].get('allow_unverified_structure') and not original['geometry_valid'])
    if original['project_id']!=project['id'] or (not original['geometry_valid'] and not (draft and draft_layout_ready(original))):raise ValueError('布局必须使用本项目有效结构或明确选择的空间布局草案')
    if not original.get('layout_domains'):raise ValueError('此历史结构没有内部布局空间记录；请重新生成一次结构，历史产物会保留')
    graph=project['articulation']['graph'];domains={r['owner_body_id']:read_mesh(source,r) for r in original['layout_domains']}
    parts=[{'record':dict(a),'mesh':read_mesh(source,a)} for a in original['geometry_assets']]
    baseline_hashes={a['id']:a['sha256'] for a in original['geometry_assets']}
    instances={i['id']:i for i in project['instances']}
    placements=[];unplaced=[];hardware=[];searches=[];added=[];removable=[];extraction_warnings=[]
    # Larger footprints first; ordering does not alter the selected hardware set.
    profiles=sorted(copy.deepcopy(project['layout_config']['components']),key=lambda c:-np.prod(c['dimensions_m'][:2]))
    # Automatic ownership is a search proposal, never a manual world position.
    # Exact collision, attachment and removal checks below remain authoritative.
    loads={owner:0. for owner in domains}
    available={p['owner_body_id'] for p in original.get('analysis',{}).get('lid_planes',[])}
    for profile in profiles:
        if profile['owner_body_id']!='auto':continue
        size=np.asarray(profile['dimensions_m'])*1000
        candidates=[owner for owner in domains if owner in available and
                    np.all(np.sort(domains[owner].extents)>np.sort(size)+6.)]
        if not candidates:raise ValueError('没有壳件能容纳元件包络和安装余量：'+profile['instance_id'])
        # First explore sharing the largest cavity: a greedy volume load score
        # otherwise sends the second board into a smaller, unusable body on
        # every retry. Then retain load-balanced and alternate-body proposals.
        if _strategy<4:
            candidates.sort(key=lambda owner:-domains[owner].volume)
            owner=candidates[0]
        else:
            candidates.sort(key=lambda owner:(loads[owner]+np.prod(size))/max(domains[owner].volume,1.))
            owner=candidates[((_strategy-4)//4)%len(candidates)]
        profile['owner_body_id']=owner;loads[owner]+=np.prod(size)

    alternatives=list(itertools.islice(itertools.permutations(profiles[1:]),22))
    if _strategy>=2:profiles=profiles[:1]+list(alternatives[(_strategy-2)%len(alternatives)])
    for profile_index,profile in enumerate(profiles):
        identity=profile['instance_id'];owner=profile['owner_body_id'];dim=np.array(profile['dimensions_m'])*1000
        orientation=np.eye(4)
        if _strategy>=3:
            # Stand the longest board dimension into the body, in a strapped cradle.
            order=[1,2,0] if _strategy<6 or _strategy>=8 else [2,0,1]
            orientation[:3,:3]=np.eye(3)[order,:]
            dim=dim[order]
        progress('为 '+instances[identity]['label']+' 搜索内部位置与固定座')
        domain=domains[owner].copy();shell=next(p for p in parts if p['record']['id']==owner+'_shell')
        lid=next((p for p in parts if p['record']['id']==owner+'_lid'),None)
        if lid is None:
            unplaced.append({'instance_id':identity,'owner_body_id':owner,'reason':'此刚体保留完整外壳，没有电子元件检修入口；指定元件后需重新生成结构规划开口'})
            continue
        # Solve in the hatch frame: +Z is its outward normal, not world up.
        panel=next((v for v in original.get('analysis',{}).get('lid_planes',[]) if v['owner_body_id']==owner),{})
        frame=np.asarray(panel.get('transform_mm',np.eye(4)),dtype=float)
        inverse=np.linalg.inv(frame)
        domain.apply_transform(inverse)
        for item in parts+added:item['mesh'].apply_transform(inverse)
        def restore_frame():
            for item in parts+added:item['mesh'].apply_transform(frame)
        top=float(max(domain.bounds[1,2],lid['mesh'].bounds[1,2])+30);candidates=[];attempts=0;chosen=None;reasons={}
        radii=np.asarray(panel.get('radii_mm',[40,30]));target=np.zeros(2)
        if _strategy and profile_index:target=np.array([[1,0],[-1,0],[0,-1],[0,1]][(profile_index-1)%4])*radii*.27
        if _strategy>=4:
            domain_bias=np.clip(domain.center_mass[:2],-radii*.32,radii*.32)
            if profile_index==0:target=domain_bias+np.array([[0,0],[.15,0],[0,.15],[-.15,0]][(_strategy-4)%4])*radii
            elif _strategy>=8:target+=domain_bias*.5

        # Lid mounting first; floor mounting remains a fallback. Board orientation
        # and supporting surface are explicit in each placement.
        for angle in [0,math.pi/2]:
            w,d=(dim[:2] if angle==0 else dim[:2][::-1]);half=np.array([w/2+8,d/2+5])
            lower=lid['mesh'].bounds[0,:2]+half;upper=lid['mesh'].bounds[1,:2]-half
            if np.any(lower>upper):continue
            for x in np.linspace(lower[0],upper[0],11 if _strategy>=3 else 7):
                for y in np.linspace(lower[1],upper[1],11 if _strategy>=3 else 7):
                    if panel.get('radii_mm'):
                        opening=np.array(panel['radii_mm'])-4.5
                        corners=np.array([[x+sx*half[0],y+sy*half[1]] for sx in [-1,1] for sy in [-1,1]])
                        if np.any(np.sum((corners/opening)**2,axis=1)>1):continue
                    for side in [-1,1]:
                        for z in carrier_depths(domain.bounds, dim[2], side):
                            level=domain.bounds[1,2]-z if side==-1 else z-domain.bounds[0,2]
                            candidates.append((abs(x-target[0])+abs(y-target[1])+abs(level-(48 if max(dim[:2])>=40 else 12))*.5+(0 if side==-1 else 1000),angle,w,d,x,y,z,side))
        from . import pcb_mounts
        templates={};mount_templates={}
        for angle in [0,math.pi/2]:
            w,d=dim[:2] if angle==0 else dim[:2][::-1]
            registered=pcb_mounts.attach(instances[identity]['catalog_id'],np.asarray(profile['dimensions_m'])*1000,orientation,angle,dim[2],w,d)
            if registered:
                templates[angle]=registered[:2];mount_templates[angle]=registered[2]
            else:templates[angle]=carrier(w,d,dim[2])
        ordered=sorted(candidates)
        corners=[]
        for _,angle,w,d,x,y,z,side in ordered:
            zpair=sorted([z,z+side*(max(5,dim[2]+2)+.5)])
            corners.extend([[xx,yy,zz] for xx in [x-w/2-8,x+w/2+8] for yy in [y-d/2-5,y+d/2+5] for zz in zpair])
        corner_inside=[]
        for start in range(0,len(corners),512):corner_inside.extend(domain.contains(np.asarray(corners[start:start+512])))
        contained=np.asarray(corner_inside,dtype=bool).reshape((-1,8)).all(axis=1) if corners else []
        for candidate_index,(_,angle,w,d,x,y,z,side) in enumerate(ordered):
            attempts+=1
            if attempts%40==0:progress(instances[identity]["label"]+" 布局候选 "+str(attempts)+" / "+str(len(candidates))+"，拒绝原因 "+str(reasons))
            tray,board=(m.copy() for m in templates[angle]);translation=[x,y,z]
            if side==-1:
                flip=tm.transformations.rotation_matrix(math.pi,[1,0,0]);tray.apply_transform(flip);board.apply_transform(flip)
            tray.apply_translation(translation);board.apply_translation(translation)
            support=lid if side==-1 else shell
            zpair=sorted([z,z+side*(max(5,dim[2]+2)+.5)])
            occupied=box([x-w/2-8,y-d/2-5,zpair[0]],[x+w/2+8,y+d/2+5,zpair[1]])
            # Closed body domain contains the whole envelope, including concave walls.
            if not contained[candidate_index]:
                reasons['outside_domain']=reasons.get('outside_domain',0)+1;continue
            obstacles=[p['mesh'] for p in parts]+[p['mesh'] for p in added]
            if collision(occupied,obstacles):
                reasons['occupied']=reasons.get('occupied',0)+1;continue
            # Perpendicular posts attach to actual shell surface, not a bounding box.
            origins=np.array([[x-w/2-5,y,z-side*.02],[x+w/2+5,y,z-side*.02]])
            hits,rays,_=support['mesh'].ray.intersects_location(origins,np.tile([0,0,-side],(2,1)))
            feet=[]
            for n in range(2):
                found=hits[rays==n]
                if not len(found):break
                hit=found[np.argmin(abs(found[:,2]-z))];height=abs(z-hit[2])
                if not 5<=height<=65:break
                feet.append(hit)
            if len(feet)!=2:
                reasons['no_support']=reasons.get('no_support',0)+1;continue
            posts=[cylinder(f[0],f[1],min(f[2]-side,z),max(f[2]-side,z),3.5) for f in feet]
            # Relieve only peripheral interference, retaining a connected post
            # and the central screw-bearing core. This is derived support geometry.
            relieved=[]
            for post,foot in zip(posts,feet):
                obstacles=[item['mesh'] for item in parts if item is not support]+[item['mesh'] for item in added]
                clips=[intersect(post,other) for other in obstacles if overlap(post,other)>.1]
                candidate=post
                if clips:
                    candidate=cut(post,*[dilate(clip,.4) for clip in clips if clip.is_volume])
                    core=cylinder(foot[0],foot[1],post.bounds[0,2],post.bounds[1,2],1.8)
                    if not candidate.is_volume or regions(candidate)!=1 or candidate.volume<post.volume*.7 or core.volume-overlap(core,candidate)>.1:
                        candidate=post
                relieved.append(candidate)
            posts=relieved
            if any(collision(post,[p['mesh'] for p in parts if p is not support]+[p['mesh'] for p in added]) for post in posts):
                if not reasons.get('support_collision'):
                    atomic(directory/('support-debug-'+identity+'.json'),{'xyz':translation,'feet':[f.tolist() for f in feet],'hits':[{'asset':item['record']['id'],'volume':overlap(post,item['mesh'])} for post in posts for item in parts+added if item is not support and overlap(post,item['mesh'])>.1]})
                    for n,post in enumerate(posts):post.export(directory/('support-debug-'+identity+'-'+str(n)+'.stl'))
                reasons['support_collision']=reasons.get('support_collision',0)+1;continue
            if any(float(post.volume)-overlap(post,domain)>.1 for post in posts):continue
            if any(overlap(post,support['mesh'])<1 for post in posts):continue
            # Existing motor and permanent shell must allow a complete vertical lift.
            sweep=upward_volume(occupied.bounds,top)
            permanent=[p['mesh'] for p in parts if p is not lid]
            if collision(sweep,permanent):
                if not reasons.get('extraction_blocked'):atomic(directory/('extraction-debug-'+identity+'.json'),{'bounds':occupied.bounds.tolist(),'hits':[{'asset':it['record']['id'],'volume':overlap(sweep,it['mesh'])} for it in parts if it is not lid and overlap(sweep,it['mesh'])>.1]})
                # Straight extraction is advisory; installed collision remains hard.
                pass
            # True Boolean containment final check after the cheap vertex filter.
            if float(occupied.volume)-overlap(occupied,domain)>.1:continue
            bores=[cylinder(f[0],f[1],min(z-side*5,z+side*.1),max(z-side*5,z+side*.1),1.4) for f in feet]
            modified=cut(union(support['mesh'],*posts),*bores)
            if not modified.is_volume or regions(modified)!=1:continue
            # Full tool cylinders up to the opened lid; same two screw centres.
            toolend=top if side==1 else domain.bounds[0,2]-30
            tools=[cylinder(f[0],f[1],min(z+side*2,toolend),max(z+side*2,toolend),3) for f in feet]
            if side==1 and any(collision(tool,[p['mesh'] for p in parts if p is not lid and p is not shell]) for tool in tools):continue
            chosen=(tray,board,modified,occupied,sweep,feet,tools,angle,[x,y,z],side,support);break
        searches.append({'instance_id':identity,'owner_body_id':owner,'oriented_dimensions_mm':dim.tolist(),'cavity_bounds_mm':domain.bounds.tolist(),'attempts':attempts,'candidates':len(candidates),'rejected_reasons':reasons})
        if chosen is None:
            restore_frame()
            unplaced.append({'instance_id':identity,'reason':'搜索预算内找不到同时满足空间、固定柱和沿检修盖方向拆卸通道的布局；没有放大外观或删除部件'})
            continue
        tray,board,modified,occupied,sweep,feet,tools,angle,xyz,side,support=chosen;support['mesh']=modified
        blocked=[p['record']['id'] for p in parts if p is not lid and overlap(sweep,p['mesh'])>.1]
        if blocked:extraction_warnings.append({'instance_id':identity,'obstacles':blocked})
        # Save local bench removal geometry, then return every asset to world space.
        bench_sweep=sweep if side==1 else box([*occupied.bounds[0,:2],domain.bounds[0,2]-30],[*occupied.bounds[1,:2],occupied.bounds[1,2]-.02])
        bench_sweep.apply_transform(frame)
        for tool in tools:tool.apply_transform(frame)
        restore_frame()
        tray.apply_transform(frame);board.apply_transform(frame)
        world_feet=tm.transform_points(np.asarray(feet),frame)
        prefix='electronics_'+identity
        for suffix,name,mesh,printable in [('carrier','孔位配准螺钉载板' if angle in mount_templates else '可拆绑带托盘',tray,True),('envelope','待核实部件包络',board,False)]:
            added.append({'record':{'id':prefix+'_'+suffix,'name':instances[identity]['label']+' · '+name,'owner_body_id':owner,
                'template_id':VERSION,'template_revision':'1','printable_candidate':printable,'material':'unverified'},'mesh':mesh})
        transform=tm.transformations.rotation_matrix(angle,[0,0,1])@orientation
        if side==-1:transform=tm.transformations.rotation_matrix(math.pi,[1,0,0])@transform
        transform[:3,3]=np.array(xyz)/1000+[0,0,side*(2+dim[2]/2)/1000]
        frame_m=frame.copy();frame_m[:3,3]/=1000
        transform=frame_m@transform
        placements.append({'instance_id':identity,'catalog_id':instances[identity]['catalog_id'],'owner_body_id':owner,
            'transform_m':transform.tolist(),'envelope_dimensions_m':profile['dimensions_m'],'evidence':profile['evidence'],
            'note':profile['note'],'physical_dimensions_m':None,'physical_fit_verified':False,
            'carrier_asset_id':prefix+'_carrier','envelope_asset_id':prefix+'_envelope',
            'mount_positions_m':[(f/1000).tolist() for f in world_feet],'removal_direction':(frame[:3,:3]@np.array([0,0,side])).tolist(),'support_asset_id':support['record']['id'],'lid_mounted':side==-1,'orientation_strategy':'upright_cradle' if _strategy>=3 else 'flat_tray'})
        if angle in mount_templates:
            placements[-1]['component_mount']=pcb_mounts.world_record(mount_templates[angle],frame,xyz,side)
            hardware.append({'catalog_id':'fastener_candidate','specification':mount_templates[angle]['fastener'],
                'quantity':len(mount_templates[angle]['holes']),'purpose':instances[identity]['label']+' registered PCB mounting screws and captive nuts',
                'physical_fit_verified':False})
        removable.append({'instance_id':identity,'owner':owner,'ids':{prefix+'_carrier',prefix+'_envelope'},'sweep':bench_sweep,'tools':tools,'lid_mounted':side==-1,'orientation_strategy':'upright_cradle' if _strategy>=3 else 'flat_tray'})
        if placements[-1].get('component_mount'):
            for suffix,solid in pcb_mounts.fasteners(placements[-1]['component_mount']):
                asset_id=prefix+'_'+suffix
                added.append({'record':{'id':asset_id,'name':instances[identity]['label']+' · '+suffix+' (candidate)',
                    'owner_body_id':owner,'template_id':VERSION,'template_revision':'1','printable_candidate':False,
                    'material':'unverified','role':'pcb_fastener','physical_fit_verified':False},'mesh':solid})
                removable[-1]['ids'].add(asset_id)
        hardware.append({'catalog_id':'fastener_candidate','specification':'M2 螺钉与匹配嵌件（候选孔径，待实配）','quantity':2,'purpose':instances[identity]['label']+' 托盘固定','length_m':None,'physical_fit_verified':False})
        if angle not in mount_templates:
            hardware.append({'catalog_id':None,'specification':'绝缘绑带及隔离垫（宽度/压力/避让待实配）','quantity':2,'purpose':instances[identity]['label']+' 无 PCB 孔位假设的固定','physical_fit_verified':False})
    atomic(directory/('layout-search-'+str(_strategy)+'.json'),{'strategy':_strategy,'searches':searches,'placed':[p['instance_id'] for p in placements],'unplaced':unplaced})
    if unplaced and not _finalize and _strategy<11 and time.monotonic()-started<140:
        progress('重新安排托盘顺序与位置，保留全部已选部件；搜索策略 '+str(_strategy+2))
        return generate(project,mechanical_directory,directory,_strategy+1,started)
    if unplaced and not _finalize:
        # Export the most complete proposal rather than losing it when a later
        # bounded-search strategy places fewer boards.
        previous=[json.loads(p.read_text()) for p in directory.glob('layout-search-*.json')]
        best=max(previous,key=lambda r:len(r.get('placed',[])),default={})
        if len(best.get('placed',[]))>len(placements):
            return generate(project,mechanical_directory,directory,best['strategy'],started,True)
    check('all_selected_components_placed',not unplaced,unplaced=unplaced)
    final=parts+added
    from .cover_fasteners import finish as finish_cover_fasteners
    fastener_analysis=copy.deepcopy(original.get('analysis',{}))
    cover_records=finish_cover_fasteners(final,fastener_analysis,overlap,domains=domains,assembly_steps=original.get('assembly_dependencies',[]))
    # Relocation replaces meshes transactionally; previews must use the same
    # accepted geometry as the STL exports and all subsequent checks.
    original_count=len(parts);parts=final[:original_count];added=final[original_count:]
    check('cover_screw_access',bool(cover_records) and all(r['passed'] for r in cover_records),fasteners=cover_records)
    atomic(directory/'cover-fasteners.json',{'connections':cover_records,'physical_fit_verified':False})
    progress('检查托盘取出顺序、工具空间、装配实体与关节运动')
    # Lift each lid together with its boards before bench disassembly.
    lid_checks=[]
    for owner in {item['owner'] for item in removable}:
        lid_ids={owner+'_lid'}
        for item in removable:
            if item['owner']==owner and item['lid_mounted']:lid_ids|=item['ids']
        group=[p for p in final if p['record']['id'] in lid_ids]
        panel=next((p for p in original.get('analysis',{}).get('lid_planes',[]) if p['owner_body_id']==owner),{})
        direction=np.array(panel.get('normal',[0,0,1]),dtype=float)
        distance=max(float((p['mesh'].vertices@direction).max()) for p in final)-min(float((p['mesh'].vertices@direction).min()) for p in group)+10
        obstacles=[p['mesh'] for p in final if p['record']['id'] not in lid_ids]
        bad=[];offsets=sorted(set([.25,.5,1,2,5,*np.linspace(0,distance,13).tolist()]))
        for item in group:
            blocked=[]
            for offset in offsets:
                if collision(item['mesh'].copy().apply_translation(direction*offset),obstacles):blocked.append(offset)
            if blocked:bad.append({'asset_id':item['record']['id'],'offsets_mm':blocked})
        lid_checks.append({'owner_body_id':owner,'lift_distance_mm':distance,'direction':direction.tolist(),'blocked_assets':bad,'samples_mm':offsets})
    check('populated_lid_extraction',not any(c['blocked_assets'] for c in lid_checks),lids=lid_checks,
          scope='检修盖及内部组件沿登记开口法向离散取出检查，密集覆盖初始开盖；不是连续路径保证，线路拔出尚未实现',continuous_guarantee=False,blocking=False)
    remaining=list(removable);removed=set();order=[]
    while remaining:
        accepted=None
        for item in remaining:
            excluded=removed|item['ids']
            if item['lid_mounted']:
                # Once off the robot, only this lid and its remaining carriers obstruct access.
                present={item['owner']+'_lid'}
                for other_item in removable:
                    if other_item['owner']==item['owner'] and other_item['lid_mounted']:present|=other_item['ids']
                other=[p['mesh'] for p in final if p['record']['id'] in present-excluded]
            else:
                excluded|={b+'_lid' for b in domains}
                for other_item in removable:
                    if other_item['lid_mounted']:excluded|=other_item['ids']
                other=[p['mesh'] for p in final if p['record']['id'] not in excluded]
            if not any(collision(tool,other) for tool in item['tools']):
                if collision(item['sweep'],other):extraction_warnings.append({'instance_id':item['instance_id'],'stage':'bench_removal'})
                accepted=item;break
        if accepted is None:break
        remaining.remove(accepted);removed|=accepted['ids'];order.append(accepted['instance_id'])
    check('component_tool_access',not remaining,removal_order=order,blocked=[r['instance_id'] for r in remaining],
          scope='上螺丝工具空间为硬检查；直线取出扫掠单列为软检查；不含拔插连接器和绑带拆卸')
    check('component_straight_extraction',not extraction_warnings,blocking=False,warnings=extraction_warnings,scope='直线取出受阻不淘汰布局；需要人工确认转动或倾斜取出的路径')
    # Added material and components must not obstruct original sampled robot motion.
    collisions=[]
    for index,a in enumerate(final):
        for b in final[index+1:]:
            volume=overlap(a['mesh'],b['mesh'])
            if volume>.1:collisions.append({'a':a['record']['id'],'b':b['record']['id'],'volume_mm3':volume})
    check('assembly_collisions',not collisions,collisions=collisions)
    bodies=[b['id'] for b in graph['bodies']];groups={b:union(*[p['mesh'] for p in final if p['record']['owner_body_id']==b]) for b in bodies}
    sweeps=[]
    for angles in sample_angles(graph):
        matrices=poses(graph,angles);moved={b:m.copy().apply_transform(matrices[b]) for b,m in groups.items()};pairs=[]
        for index,a in enumerate(bodies):
            for b in bodies[index+1:]:
                volume=overlap(moved[a],moved[b])
                if volume>.1:pairs.append({'a':a,'b':b,'volume_mm3':volume})
        sweeps.append({'angles_rad':angles,'collisions':pairs})
    check('joint_sweep',not any(s['collisions'] for s in sweeps),samples=sweeps,continuous_guarantee=False)
    # Moving electronics cannot repair a cover screw blocked by another
    # original shell. Preserve that hard failure without repeating packing.
    added_ids={p['record']['id'] for p in added}
    cover_placement_conflict=any(added_ids.intersection(r.get('tool_blocked_assets',[])+r.get('shaft_blocked_assets',[])) for r in cover_records if not r['passed'])
    retry_checks=[c for c in checks if c.get('blocking',True) and (c['name']!='cover_screw_access' or cover_placement_conflict)]
    if not _finalize and not all(c['passed'] for c in retry_checks) and _strategy<11 and time.monotonic()-started<140:
        atomic(directory/('layout-search-'+str(_strategy)+'.json'),{'strategy':_strategy,'searches':searches,'checks':checks,'placed':[p['instance_id'] for p in placements]})
        return generate(project,mechanical_directory,directory,_strategy+1,started)
    assets=[];preview=[];scene=tm.Scene()
    for p in final:
        record=p['record'];mesh,repair=stl_deliverable(p['mesh']);check('stl_'+record['id'],repair['passed'] and mesh.is_volume and (not record['printable_candidate'] or regions(mesh)==1),repair=repair)
        filename=record['id']+'.stl';mesh.export(directory/filename,file_type='stl_ascii' if repair.get('stl_encoding')=='ascii' else 'stl')
        a={**record,'file':filename,'sha256':hashlib.sha256((directory/filename).read_bytes()).hexdigest(),
           'export_length_unit':'mm','bounds_m':(mesh.bounds/1000).tolist(),'volume_m3':float(mesh.volume)*1e-9,'transform_m':np.eye(4).tolist()}
        assets.append(a);color='#677581' if record.get('role')=='pcb_fastener' else '#eab453' if record['id'].startswith('electronics_') else '#8da5a0'
        display=mesh.simplify_quadric_decimation(face_count=3000) if len(mesh.faces)>3000 else mesh
        preview.append({'id':a['id'],'name':a['name'],'owner':a['owner_body_id'],'vertices':(display.vertices/1000).tolist(),'faces':display.faces.tolist(),'color':color,'hardware':not a['printable_candidate']})
        display=mesh.copy().apply_scale(.001);display.visual.face_colors=tm.visual.color.hex_to_rgba(color);scene.add_geometry(display,node_name=a['id'])
    from . import component_visuals
    exported_cover_checks=[]
    for screw in cover_records:
        pose=tm.geometry.align_vectors([0,0,1],screw['axis_world']);pose[:3,3]=screw['head_seat_world_mm']
        shaft=cylinder(0,0,-screw['candidate_underhead_length_mm'],0,(screw['shaft_clearance_diameter_mm']-.4)/2).apply_transform(pose)
        head=cylinder(0,0,.01,60,(screw['head_entry_diameter_mm']-.2)/2).apply_transform(pose)
        cap=read_mesh(directory,next(a for a in assets if a['id']==screw['cover_asset_id']))
        base=read_mesh(directory,next(a for a in assets if a['id']==screw['mate_asset_id']))
        obstruction=overlap(shaft,cap)+overlap(shaft,base)+overlap(head,cap)
        exported_cover_checks.append({'id':screw['id'],'obstruction_mm3':obstruction,'passed':obstruction<.02})
    check('cover_screw_exports',bool(exported_cover_checks) and all(r['passed'] for r in exported_cover_checks),fasteners=exported_cover_checks)
    visual_assets,visual_reports=component_visuals.generate(placements,directory)
    # Independent transforms: CAD uses the saved component pose; carrier holes
    # were constructed in the tray frame before its flip, placement and hatch pose.
    registrations=[]
    for placement in placements:
        mount=placement.get('component_mount')
        if not mount:continue
        visual=next((v for v in visual_assets if v['instance_id']==placement['instance_id']),None)
        carrier_mesh=read_mesh(directory,next(a for a in assets if a['id']==placement['carrier_asset_id']))
        matrix=np.asarray(mount['source_to_world_mm']);rows=[]
        for hole in mount['holes']:
            native=hole['source_center_mm'];expected=tm.transform_points([native],np.asarray(visual['source_to_world_mm']))[0] if visual else np.full(3,np.inf)
            shaft=cylinder(native[0],native[1],mount['pcb_bottom_mm']-5.99,mount['pcb_bottom_mm']+.01,mount['fastener']['shaft_diameter_mm']/2-.05).apply_transform(matrix)
            ring=cut(cylinder(native[0],native[1],mount['pcb_bottom_mm']-.2,mount['pcb_bottom_mm'],mount['fastener']['nut_circumradius_mm']+.3),
                     cylinder(native[0],native[1],mount['pcb_bottom_mm']-.3,mount['pcb_bottom_mm']+.1,mount['fastener']['clearance_diameter_mm']/2+.1)).apply_transform(matrix)
            error=float(np.linalg.norm(expected-hole['center_world_mm']));blocked=overlap(shaft,carrier_mesh);bearing=overlap(ring,carrier_mesh)/ring.volume
            rows.append({'hole_id':hole['id'],'axis_alignment_error_mm':error,'shaft_obstruction_mm3':blocked,'bearing_coverage':bearing,
                'passed':bool(error<1e-5 and blocked<.02 and bearing>.95)})
        registrations.append({'instance_id':placement['instance_id'],'reference_variant':mount['reference_variant'],'holes':rows,'physical_fit_verified':False})
    check('registered_pcb_mounts',all(h['passed'] for m in registrations for h in m['holes']),components=registrations)
    atomic(directory/'pcb-mounts.json',{'components':registrations,'placements':[p for p in placements if p.get('component_mount')]})
    replaced={a['replaces_envelope_id'] for a in visual_assets}
    preview=[p for p in preview if p['id'] not in replaced]+component_visuals.preview_parts(directory,visual_assets)
    scene=component_visuals.scene(directory,parts+added,visual_assets)
    result={'project_id':project['id'],'design_revision':project['revision'],'pipeline':VERSION,'status':'layout_draft_structure_unverified' if draft else 'layout_candidate_checked',
        'structure_verified':bool(original['geometry_valid']),'inherited_structure_checks':[c for c in original['checks'] if not c['passed']],
        'layout_valid':all(c['passed'] for c in checks if c.get('blocking',True)),'manufacturing_ready':False,'assembly_access_verified':original.get('assembly_access_verified',True) and not any(c['blocked_assets'] for c in lid_checks) and not extraction_warnings,
        'assembly_policy':project.get('assembly_policy','strict'),'geometry_assets':assets,
        'fastener_search':fastener_analysis.get('fastener_search',{}),'cover_assembly_plan':fastener_analysis.get('cover_assembly_plan'),'component_placements':placements,'visual_assets':visual_assets,'visual_registration':visual_reports,'checks':checks,'searches':searches,'hardware_requests':hardware,
        'rigid_bodies':original['rigid_bodies'],'joints':original['joints'],
        'mechanical_run_id':project['layout_config']['mechanical_run_id'],'source_asset_hashes':baseline_hashes,
        'assembly_dependencies':((fastener_analysis.get('cover_assembly_plan') or {}).get('steps',[]))+[{'action':('人工拆卸审阅模式：直线取出可能受阻，转动/倾斜路径尚未验证；先确认壳件和组件能实际取出再制造。' if project.get('assembly_policy')=='manual_extraction_review' else '先断开线束并将检修盖及其内部电子组件整体沿登记的开盖法向移出；上盖托盘在外部从盖内侧拆卸，壳体托盘沿登记取出方向拆出。安装按逆序，嵌件先配好。')}]+
            [{'action':instances[i]['label']+('：先断开接线并卸下两颗载板固定螺钉，将载板移到台面；沿已登记PCB孔轴卸下PCB螺钉及螺母，按逆序安装。实物规格与接头待核实。' if next(p for p in placements if p['instance_id']==i).get('component_mount') else '：解除连接器与绑带，移除两颗托盘螺钉，按登记拆出方向取出；PCB接触与拔线未物理校核') } for i in order],
        'pending':(['直线取出通道受阻，已作为软规则保留布局；转动或倾斜拆卸需人工确认。'] if extraction_warnings else [])+(['结构尚未通过完整制造检查；本结果只供布局迭代，继承的结构失败项见 inherited_structure_checks。'] if draft else [])+['全部电子外形仍是显式工程假设；没有实测板卡、焊接、连接器和端口坐标',
                   '绑带避让元件、夹紧力、绝缘、嵌件公差、螺钉长度与上盖紧固件工具路径仍待核实',
                   '线束、供电散热、质量载荷、连续运动、动力学与固件尚未验证'],
        'circuit_available':bool(project.get('circuit')),'elapsed_s':round(time.monotonic()-started,3),
        'packing_strategy':_strategy,'generator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    from .print_package import generate as generate_print_package
    result['prototype_print_package']=generate_print_package(result,directory)
    atomic(directory/'input-project.json',project);atomic(directory/'manifest.json',result)
    atomic(directory/'preview.json',{'parts':preview,'graph':graph,'joint':{'limits_rad':[-1,1]}});scene.export(directory/'assembly.glb')
    result['artifacts']=sorted(p.name for p in directory.iterdir() if p.is_file() and p.suffix!='.zip' and p.name not in {'run.json','input.json','worker.log','result.json'})
    with zipfile.ZipFile(directory/'layout-review.zip','w',zipfile.ZIP_DEFLATED) as package:
        for filename in result['artifacts']:package.write(directory/filename,filename)
    result['artifacts'].append('layout-review.zip')
    if result.get('prototype_print_package'):result['artifacts'].append('prototype-print.zip')
    atomic(directory/'result.json',result)
    return result
