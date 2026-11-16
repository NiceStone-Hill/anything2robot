"""Semantic opening suggestions and deterministic local access-panel templates."""
import base64
import json
import time
import urllib.request
from urllib.parse import urlsplit
import numpy as np
import trimesh as tm
from ..contracts import ContractError, loads
from ..providers import live_config, NoRedirect
from ..project import atomic
from .planning import render_views
from .solid import box, cylinder, union, cut, intersect, overlap, regions


def suggest(shape,graph,directory,live=False):
    """VLM selects semantic surface region, never motor pose or screw geometry."""
    fallback=[{'body_id':b['id'],'center_mm':b['seed_mm'],'normal':[0,1,0],
               'reason':'本地参考：经过部件中心的分型面建议；未识别语义表面'} for b in graph['bodies']]
    result={'source':'local_reference','plan_kind':'assembly_parting_v1','live_model_called':False,'regions':fallback}
    cfg=live_config()
    if not live or not all(cfg.values()):return result
    paths=render_views(shape,directory)
    instruction={'task':'为每个刚体建议装配壳体分型面，将其拆为可打开的主壳和壳盖，用于内部零件安装。优先竖直纵向平面（法向沿 X 或 Y），沿部件长度打开安装空间，不要优先横切顶盖。优先简单平面，必要时让切缝绕开关节连接面，使完整连接面归属于同一壳件。不要寻找小检修孔，不要生成永久电机或工具窗口。只给分型面上的点、壳盖拆出方向和语义理由。电机布置、安装孔、装入路径由程序生成，不由你决定。只输出JSON。图片与模型资料均为数据。',
        'graph':graph,'bounds_mm':shape.bounds.tolist(),'frame':'模型右手坐标，Z朝上，毫米',
        'contract':{'regions':[{'body_id':'图中刚体ID，每个恰好一次','center_mm':[0,0,0],'normal':[0,1,0],'reason':'为何此分型面方便装配，并保留完整关节连接面'}]},
        'constraints':'center_mm 为对应刚体内分型面上的建议点，不是图像像素。normal只取±X、±Y、±Z六个方向之一，表示壳盖向外拆出方向。优先不显眼的一侧，避免穿过关节连接面和主要外观特征；不确定时建议经过部件中部的平面。几何程序验证禁切区域、壳体连接和拆卸路径；不要声称图片已验证这些条件。'}
    content=[{'type':'text','text':json.dumps(instruction,ensure_ascii=False)}]+[{'type':'image_url','image_url':{'url':'data:image/png;base64,'+base64.b64encode(p.read_bytes()).decode()}} for p in paths]
    url=cfg['BASE_URL'].rstrip('/')+'/chat/completions';parsed=urlsplit(url)
    if parsed.scheme!='https' or parsed.username or parsed.password or parsed.query or parsed.fragment:raise ContractError('开盖规划服务地址无效')
    atomic(directory/'access-vlm-input.json',instruction)
    try:
        req=urllib.request.Request(url,data=json.dumps({'model':cfg['MODEL'],'response_format':{'type':'json_object'},'messages':[{'role':'user','content':content}]}).encode(),headers={'Authorization':'Bearer '+cfg['API_KEY'],'Content-Type':'application/json'})
        with urllib.request.build_opener(NoRedirect()).open(req,timeout=45) as response:raw=response.read(100001)
        if len(raw)>100000:raise ValueError('开盖建议过大')
        answer=loads(loads(raw)['choices'][0]['message']['content']);validate_suggestions(answer,shape.bounds,graph)
        result={'source':'live_vlm_surface_regions','plan_kind':'assembly_parting_v1','live_model_called':True,'model':cfg['MODEL'],**answer}
    except Exception as exc:
        result.update(source='local_reference_after_vlm_failure',live_model_called=True,error=type(exc).__name__)
    atomic(directory/'access-suggestions.json',result)
    return result


def validate_suggestions(answer,bounds,graph):
    if not isinstance(answer,dict) or set(answer)!={'regions'} or not isinstance(answer['regions'],list):raise ValueError('开盖字段无效')
    ids=[r.get('body_id') for r in answer['regions']]
    if len(ids)!=len(set(ids)) or set(ids)!={b['id'] for b in graph['bodies']}:raise ValueError('开盖刚体ID无效')
    for r in answer['regions']:
        if set(r)!={'body_id','center_mm','normal','reason'}:raise ValueError('开盖区域字段无效')
        p=np.asarray(r['center_mm'],dtype=float);n=np.asarray(r['normal'],dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all() or np.any(p<bounds[0]) or np.any(p>bounds[1]):raise ValueError('开口定位超出模型')
        if n.shape!=(3,) or np.count_nonzero(n)!=1 or np.max(abs(n))!=1:raise ValueError('开口法向无效')
        if not isinstance(r['reason'],str) or not 1<=len(r['reason'])<=1500:raise ValueError('开口依据无效')


def access_frame(normal):
    return tm.geometry.align_vectors([0,0,1],normal)


def ellipse(rx,ry,z0,z1):
    m=cylinder(0,0,z0,z1,1,48);m.apply_scale([rx,ry,1]);return m


class PanelSearchError(ValueError):
    """Expected exhausted panel search, distinct from malformed geometry/errors."""


def access_requirement(owner, graph, modules, project):
    """Semantic service need; geometric access can require a split on any body.

    The graph generator also checks all assembly corridors on the intact shell,
    including output-only leaves. External-service metadata cannot authorize a
    permanent tool port. Future electronics require replanning.
    """
    owned = [m for m in modules if m['owner'] == owner]
    outgoing=[j for j in graph['joints'] if j['parent']==owner]
    if any(not any(m.get('joint_id')==j['id'] and m['role']=='motor' and m.get('external_service') for m in owned) for j in outgoing):
        return {'required': True, 'reason': '下一级关节未声明外部装配路径，保留独立维护入口'}
    if any(m['role'] not in ('rotor', 'output_mount') and not m.get('external_service') for m in owned):
        return {'required': True, 'reason': '此刚体内部有电机或需要维护的模块'}
    assigned = list(project.get('component_placements', []))
    assigned += project.get('layout_config', {}).get('components', []) if project.get('layout_config') else []
    assigned += project.get('instances', [])
    if any(c.get('owner_body_id') == owner for c in assigned):
        return {'required': True, 'reason': '此刚体已指定内部电子元件'}
    if any(c.get('owner_body_id')==owner for c in project.get('service_opening_requirements',[])):
        return {'required': True, 'reason': '此刚体已指定内部检修需求'}
    if any(m.get('external_service') for m in owned):
        return {'required': False, 'method': 'external_module_access',
                'reason': '无电子检修需求；仍须在完整壳体上验证所有装配路径，受阻时生成可拆壳件，不允许永久工具孔'}
    if owner == graph['root_body_id']:
        return {'required': True, 'reason': '主体未登记外部装配路径，需要装配与检修入口'}
    incoming = [j['id'] for j in graph['joints'] if j['child'] == owner]
    return {'required': False, 'method': 'output_only_leaf', 'joint_ids': incoming,
            'reason': '末端输出连接也须验证螺丝和工具路径；受阻时拆开壳体，安装面与承力连接完整保留'}


def cover_seats_supported(cap,base,x,y,plane,thread="M2"):
    boss,bore,nut=(4.4,1.8,3.45) if thread=="M3" else (3.4,1.3,2.6)
    seat=plane+1.5
    washer=cut(cylinder(x,y,plane+.1,seat-.1,boss),cylinder(x,y,plane,seat,bore))
    mate=cut(cylinder(x,y,plane-2.8,plane-.1,boss),cylinder(x,y,plane-3,plane,nut))
    return overlap(washer,cap)>1 and overlap(mate,base)>1


def cover_tools_clear(fasteners,transform,plane,top,obstacles):
    """Use the downstream screw/tool probes before accepting a cover position."""
    inverse=np.linalg.inv(transform);seat=plane+1.5
    for fastener in fasteners:
        x,y,_=tm.transform_points([fastener['position_mm']],inverse)[0]
        head_r,shaft_r=(3.1,1.5) if fastener.get('candidate_thread')=='M3' else (2.1,1.)
        tool=cylinder(x,y,seat+.01,top+30,head_r).apply_transform(transform)
        shaft=cylinder(x,y,seat-6,top+20,shaft_r).apply_transform(transform)
        if any(overlap(tool,mesh)>.02 or overlap(shaft,mesh)>.02 for mesh in obstacles):return False
    return True


def local_panel(shell,outer,hint,hardware,protected,wall,gap=.5,insertion_meshes=None,clearance_obstacles=None,service_radii=None,service_depth_factor=1.7,extraction_obstacles=None):
    """Search bounded connected patches; leave remote limbs and ears untouched.

    Frame local Z is extraction normal. Tooling/fastener dimensions are explicit
    engineering assumptions, not manufacturer mounting-hole data.
    """
    clearance_obstacles=clearance_obstacles or []
    candidates=[];preferred=np.array(hint['normal'],dtype=float)
    normals=[preferred,*[v for v in ([0,0,1],[0,0,-1],[0,1,0],[0,-1,0],[1,0,0],[-1,0,0]) if not np.array_equal(v,preferred)]]
    tested=[];fallback=None
    for normal in normals:
        normal=np.asarray(normal,dtype=float)
        transform=access_frame(normal);inverse=np.linalg.inv(transform)
        s=shell.copy().apply_transform(inverse);o=outer.copy().apply_transform(inverse)
        hardware_local=[m.copy().apply_transform(inverse) for m in hardware]
        # Hardware cross section plus clearance sets a minimum usable opening.
        hw=np.vstack([m.vertices for m in hardware_local]) if hardware_local else np.zeros((1,3))
        span=np.ptp(hw,axis=0)[:2] if hardware_local else np.array([15.,15.])
        radii=np.array(service_radii,dtype=float) if service_radii is not None else np.maximum((span+8)/np.sqrt(2),[16,16])  # ellipse contains the whole rectangular section
        if np.any(2*radii>o.extents[:2]*.9):continue
        semantic=tm.transform_points([hint['center_mm']],inverse)[0]
        centres=[semantic[:2]]
        if hardware_local:centres.insert(0,(hw.min(0)+hw.max(0))[:2]/2)
        centres.extend([semantic[:2]+offset*radii*.3 for offset in [np.array([1,0]),np.array([-1,0]),np.array([0,1]),np.array([0,-1])]])
        # Semantic advice identifies a region, not the final hatch centre. Prefer
        # candidates with actual body volume behind them instead of accepting
        # the first connected cover over a narrow tail/leg edge.
        def chamber_score(xy):
            hits,_,_=o.ray.intersects_location([np.r_[xy,o.bounds[1,2]+1]],[[0,0,-1]])
            if not len(hits):return -1.
            top=float(hits[:,2].max())
            probe=ellipse(*np.maximum(radii-8,4),top-min(90,float(o.extents[2])*.6),top)
            probe.apply_translation([*xy,0])
            return overlap(probe,o)
        centres=sorted(centres,key=chamber_score,reverse=True)
        for xy in centres:
            start=np.r_[xy,o.bounds[1,2]+1]
            hits,_,_=o.ray.intersects_location([start],[[0,0,-1]],multiple_hits=True)
            if not len(hits):continue
            top=float(hits[:,2].max());plane=top-max(wall*service_depth_factor,5.)
            shifted=transform.copy();shifted[:3,3]=transform[:3,:3]@np.r_[xy,0]
            inv=np.linalg.inv(shifted);local_shell=shell.copy().apply_transform(inv);local_outer=outer.copy().apply_transform(inv)
            plug=ellipse(*radii,plane,top+max(wall*2,5))
            cutter=ellipse(*(radii+gap),plane,top+max(wall*2,5))
            cap=intersect(local_outer,plug)
            if hardware or clearance_obstacles:cap=cut(cap,*[m.copy().apply_transform(inv) for m in hardware+clearance_obstacles])
            base=cut(local_shell,cutter)
            # A conformal solid cover has a continuous backing instead of loose
            # skin islands. Reject excessive solid plugs on deep appendages.
            if cap.volume>np.pi*np.prod(radii)*wall*4:continue
            if cap.is_empty or base.is_empty:continue
            ncap=regions(cap);nbase=regions(base)
            report={'normal':np.array(normal).tolist(),'center_mm':tm.transform_points([[0,0,plane]],shifted)[0].tolist(),'radii_mm':radii.tolist(),'lid_regions':ncap,'shell_regions':nbase}
            tested.append(report)
            if ncap!=1 or nbase!=1:continue
            worldcutter=cutter.copy().apply_transform(shifted)
            if any(overlap(worldcutter,m)>.1 for m in protected):continue
            # Add an internal annular flange to the shell, not to remote appendages.
            ring=union(cut(ellipse(*(radii+4),plane-3,plane),ellipse(*(radii-4),plane-4,plane+1)),
                       cut(ellipse(*(radii+4),plane-.1,top+1),ellipse(*(radii+gap),plane-1,top+2)))
            ring=intersect(ring,local_outer)
            if hardware or clearance_obstacles:
                ring=cut(ring,*[m.copy().apply_transform(inv) for m in hardware+clearance_obstacles])
            if overlap(base,ring)<1:continue
            base=union(base,ring)
            # Four candidate screw locations on the oval flange. Require both seats.
            holes=[];records=[]
            for angle in [0,np.pi/2,np.pi,3*np.pi/2]:
                x,y=(radii-2)*[np.cos(angle),np.sin(angle)]
                bottom=cylinder(x,y,plane-3,plane,3.6)
                upper=intersect(intersect(cylinder(x,y,plane,top+1,3.6),local_outer),plug)
                if overlap(cap,upper)<1 or overlap(base,bottom)<1:continue
                hole=cylinder(x,y,plane-4,top+2,1.2)
                nut=cylinder(x,y,plane-3.1,plane-1,2.5,6)
                # All added seats must stay within the body and clear hardware.
                bottom=intersect(bottom,local_outer)
                proposed=union(bottom,upper)
                if any(overlap(proposed,m.copy().apply_transform(inv))>.1 for m in hardware+clearance_obstacles):continue
                candidate_base=cut(union(base,bottom),hole,nut);candidate_cap=cut(union(cap,upper),hole)
                # Match the final head-seat and nut-bearing checks before
                # accepting a screw on a tapered or locally thin cover edge.
                if not cover_seats_supported(candidate_cap,candidate_base,x,y,plane):continue
                base=candidate_base;cap=candidate_cap
                holes.append([x,y]);records.append({'position_mm':tm.transform_points([[x,y,plane]],shifted)[0].tolist(),'candidate_thread':'M2','required_underhead_length_mm':top-plane+4,'actual_fastener_length_mm':None,'physical_fit_verified':False})
            if len(records)<3 or regions(base)!=1 or regions(cap)!=1:continue
            base=base.apply_transform(shifted);cap=cap.apply_transform(shifted)
            if overlap(base,cap)>.1:continue
            # Reject lids that collide immediately or hit another body during opening.
            distance=float(np.ptp(outer.vertices@normal))+20
            opening_offsets=sorted(set([.25,.5,1.,3.,10.,*np.linspace(0,distance,12).tolist()]))
            obstacles=[base,*hardware,*(extraction_obstacles or [])]
            opening_blocked=any(overlap(cap.copy().apply_translation(normal*offset),obstacle)>.1 for offset in opening_offsets for obstacle in obstacles)
            report['lid_extraction_blocked']=opening_blocked
            if opening_blocked:continue
            report['cover_tools_blocked']=not cover_tools_clear(records,shifted,plane,top,[*hardware,*(extraction_obstacles or [])])
            if report['cover_tools_blocked']:continue
            blocked=False
            if insertion_meshes:
                distance=float(np.ptp(outer.vertices@normal))+20
                blocked=False
                for motor in insertion_meshes:
                    sweep=tm.convex.convex_hull(np.vstack([motor.vertices,motor.vertices+normal*distance]))
                    if overlap(base,sweep)>.1:blocked=True;break
            # Motor insertion is checked later on the final assembly, not inferred from opening size.
            candidate_result=(base,cap,{'method':'local_oval_access_panel','transform_mm':shifted.tolist(),'plane_z_mm':plane,'top_local_mm':top,'normal':np.array(normal).tolist(),
                'radii_mm':radii.tolist(),'source_hint':hint,'tested_candidates':tested,'fasteners':records,'clearance_mm':gap,'motor_path_blocked':blocked,'lid_extraction_samples_mm':opening_offsets})
            if blocked:
                if fallback is None:fallback=candidate_result
                continue
            return candidate_result
    if fallback is not None:return fallback
    raise PanelSearchError('没有找到同时连通、可固定且避开安装面的局部检修盖：'+str(tested[-3:]))


def reconnect_local(shell,outer,obstacles,radius=3,budget_s=30.):
    """Preserve all volume components with bounded, obstacle-clipped short ribs."""
    from scipy.spatial import cKDTree
    from .solid import volume_components
    from .graph_assembly import beam
    components=sorted(volume_components(shell),key=lambda m:float(m.volume),reverse=True)
    if len(components)<=1:return shell,{'passed':True,'paths':[]}
    current=components[0];paths=[];started=time.monotonic();attempts=0
    for fragment in components[1:]:
        distances,ids=cKDTree(current.vertices).query(fragment.vertices)
        order=np.argsort(distances)[::max(1,len(distances)//8)][:8]
        accepted=None
        for index in order:
            a=current.vertices[ids[index]];b=fragment.vertices[index]
            if np.linalg.norm(a-b)>35:continue
            for offset in [np.zeros(3),*[np.eye(3)[axis]*sign*radius*2 for axis in range(3) for sign in [-1,1]]]:
                if time.monotonic()-started>budget_s:
                    return shell,{'passed':False,'paths':paths,'reason':'局部连接搜索达到计算预算，尚未确认连接可行；保留全部材料供检查',
                                  'error_kind':'search_budget_exhausted','budget_s':budget_s,'attempts':attempts}
                attempts+=1
                middle=(a+b)/2+offset
                rib=union(beam(a,middle,radius),beam(middle,b,radius))
                # Remote motion samples cannot affect this bounded rib. Avoid
                # sending complete unrelated shells to every Boolean attempt.
                local_obstacles=[o for o in obstacles if not o.is_empty and np.all(o.bounds[1]>rib.bounds[0]) and np.all(o.bounds[0]<rib.bounds[1])]
                rib=intersect(cut(rib,*local_obstacles),outer)
                if rib.is_empty:continue
                candidate=union(current,fragment,rib)
                if regions(candidate)==1 and all(overlap(rib,o)<.05 for o in obstacles):
                    accepted=candidate;paths.append({'from_mm':a.tolist(),'via_mm':middle.tolist(),'to_mm':b.tolist(),'radius_mm':radius});break
            if accepted is not None:break
        if accepted is None:return shell,{'passed':False,'paths':paths,'reason':'在35 mm内找不到保留全部碎片且避开硬件的连接筋'}
        current=accepted
    return current,{'passed':regions(current)==1,'paths':paths,'method':'bounded_short_ribs_after_motion_trim'}


def separate_cutting_remnants(shell,protected,cutters,max_fraction=.005):
    """Explicit machining cleanup, not deletion of source model components.

    Only small positive remnants touching the actual opposing cutter envelope
    may be removed. Preserve interfaces. Archive every removed solid and limit
    total removal to 0.5% of this shell. Substantial fragments must be connected.
    """
    from .solid import volume_components, remove_planar_boolean_boundaries
    shell,planar_cleanup=remove_planar_boolean_boundaries(shell)
    components=sorted(volume_components(shell),key=lambda m:float(m.volume),reverse=True)
    if len(components)<=1:return shell,[],{'removed_mm3':0.,'fraction':0.,'planar_boundary_cleanup':planar_cleanup}
    total=float(shell.volume);keep=[components[0]];removed=[]
    from scipy.spatial import cKDTree
    cutter_surface=None
    for c in components[1:]:
        # A substantial fragment or protected interface must be kept regardless
        # of its distance to cutters. Do not query millions of motion triangles
        # merely to decide a predicate that already cannot be satisfied.
        if c.volume>total*.001 or any(overlap(c,p)>.001 for p in protected):
            keep.append(c);continue
        if cutter_surface is None:
            cutter_surface=tm.util.concatenate([m for m in cutters if not m.is_empty]) if any(not m.is_empty for m in cutters) else None
        near=cutter_surface is not None and float(tm.proximity.closest_point(cutter_surface,c.vertices)[1].min())<2.
        if near:removed.append(c)
        else:
            keep.append(c)
    volume=sum(float(m.volume) for m in removed)
    if volume>total*max_fraction:return shell,[],{'removed_mm3':0.,'fraction':0.,'reason':'切削残屑超过体积预算，保留并拒绝自动清理'}
    return union(*keep),removed,{'removed_mm3':volume,'fraction':volume/total,'maximum_fraction':max_fraction,
        'planar_boundary_cleanup':planar_cleanup,'component_volumes_mm3':[float(c.volume) for c in removed],'source_components_removed':False,
        'method':'bounded_recorded_cutting_remnants_near_motion_interface'}


def separate_voxel_artifacts(shell, source, protected, pitch, source_components):
    """Archive bounded disconnected voxel material wholly outside the real source.

    Never handles multiple source solids or intersecting source material. This is
    separate from cutting cleanup and cannot discard an original small component.
    """
    from .solid import volume_components, remove_planar_boolean_boundaries
    # Closed zero-volume planar tangencies contain no source material, even
    # when this owner has multiple legitimate source solids. Keep all solids.
    shell,planar_cleanup=remove_planar_boolean_boundaries(shell)
    components=sorted(volume_components(shell),key=lambda m:float(m.volume),reverse=True)
    report={'method':'bounded_outside_source_voxel_artifacts','planar_boundary_cleanup':planar_cleanup,'source_components_removed':False,
            'removed_mm3':0.,'maximum_fraction':.001,'source_components':source_components}
    if source_components!=1 or len(components)<2:return shell,[],report
    total=float(shell.volume);removed=[];keep=[components[0]];records=[]
    for fragment in components[1:]:
        volume=float(fragment.volume)
        # Large fragments and fragments containing real source material can
        # never qualify. Avoid an expensive nearest-surface query on them.
        # This preserves exactly the same cleanup predicate and all material.
        if volume>min(total*.001,pitch**3):keep.append(fragment);continue
        source_overlap=overlap(fragment,source)
        if source_overlap>1e-8:keep.append(fragment);continue
        if any(overlap(fragment,p)>1e-8 for p in protected):keep.append(fragment);continue
        distance=float(tm.proximity.closest_point(source,fragment.vertices)[1].max())
        # Strictly derived, sub-cell fragments only, fully outside the source.
        if distance<=pitch:
            removed.append(fragment);records.append({'volume_mm3':volume,'source_overlap_mm3':source_overlap,'max_vertex_surface_distance_mm':distance})
        else:keep.append(fragment)
    removed_volume=sum(float(m.volume) for m in removed)
    if removed_volume>total*.001:return shell,[],{**report,'reason':'超出体积预算，保留全部组件'}
    return union(*keep),removed,{**report,'removed_mm3':removed_volume,'fraction':removed_volume/total,'components':records}
