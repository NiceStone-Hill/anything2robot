"""Protected, reversible shell splits. All candidates are geometry-derived.

Assembly sweeps are constraints on the OPEN shell, never permanent cutters.
The stepped mask extrudes protected interfaces away from the removal direction,
keeping their complete backing on the structural half.
"""
import time
import numpy as np
import trimesh as tm
from .solid import box, cylinder, union, cut, intersect, overlap, regions, linear_sweep, manifold_solid, volume_components
from .shell_access import access_frame, PanelSearchError, cover_seats_supported, cover_tools_clear
from .parting_planes import plane_levels, longitudinal_normals


def open_paths_clear(base, corridors, tolerance=.1):
    return all(overlap(base, path) <= tolerance for path in corridors)


def translation_pose_blocked(cap, obstacles, span):
    """Fast rejection only; acceptance still needs a continuous sweep."""
    cap_solid=manifold_solid(cap)
    for obstacle in obstacles:
        if obstacle.is_empty:continue
        solid=None
        for offset in [0,.5,3.,10.,span*.25,span*.5,span]:
            bounds=cap.bounds+[0,0,offset]
            if np.any(bounds[1]<=obstacle.bounds[0]) or np.any(obstacle.bounds[1]<=bounds[0]):continue
            if solid is None:solid=manifold_solid(obstacle)
            if (cap_solid.translate([0,0,offset])^solid).volume()>.1:return True
    return False


def split_shell(shell, outer, hint, hardware, protected, wall, corridors,
                extraction_obstacles=(), gap=.4, budget_s=90., reserved_corridors=(), geometry_rejections=None,
                shell_parts_only=False, manual_extraction_review=False):
    """Return a main shell and a single removable shell with real screw seats.

    A bounded planar / protected-shadow search, not arbitrary unconstrained mesh
    segmentation. No feasible candidate is an explicit failure, not an open port.
    """
    started=time.monotonic();reports=[]
    geometry_rejections={} if geometry_rejections is None else geometry_rejections
    normals=longitudinal_normals(outer.vertices,hint['normal'])
    for normal in normals:
        transform=access_frame(normal);inv=np.linalg.inv(transform)
        s=shell.copy().apply_transform(inv);o=outer.copy().apply_transform(inv)
        fixed=[p.copy().apply_transform(inv) for p in protected]
        paths=[p.copy().apply_transform(inv) for p in corridors]
        hw=[p.copy().apply_transform(inv) for p in hardware]
        extraction_hw=[] if shell_parts_only else hw
        blockers=[p.copy().apply_transform(inv) for p in extraction_obstacles]
        reserved_paths=[p.copy().apply_transform(inv) for p in reserved_corridors]
        lo,hi=o.bounds;span=float(hi[2]-lo[2])+20
        # Whole installation seats and ribs remain on the main shell. The
        # shadow has no undercuts along +Z; unlike a spherical keep-out it can
        # produce a removable stepped edge around an intact joint end.
        # Conservative per-interface shadows keep all protected material.
        # Use convex hulls of swept vertices instead of a non-convex Minkowski
        # sum, whose cost grows sharply for rotated hardware and thin faces.
        def protected_shadow(p,padding):
            hull=p.convex_hull.vertices
            offsets=np.array([[x,y,z] for x in [-padding,padding] for y in [-padding,padding] for z in [-span-padding,padding]])
            return tm.convex.convex_hull((hull[:,None,:]+offsets[None,:,:]).reshape(-1,3))
        shadow=union(*[protected_shadow(p,.6) for p in fixed]) if fixed else tm.Trimesh()
        cap_shadow=union(*[protected_shadow(p,.6+gap) for p in fixed]) if fixed else shadow
        # These shadows are restored after every seam adjustment. If their
        # retained source material blocks a required path, no plane level can
        # make this mode feasible; reject once before expensive seam repairs.
        shadow_blocks=not open_paths_clear(intersect(s,shadow),paths) if fixed else False
        path_contacts=[]
        for path in paths:
            contact=intersect(s,path)
            # Contact bounds guide candidate planes only. Boolean tangencies
            # can contain distant zero-volume surfaces: they must not pull a
            # top opening down to a screw seat at the opposite end. Actual
            # clearance still checks each complete original path below.
            path_contacts.extend(p for p in contact.split(only_watertight=False,repair=False)
                                 if p.is_volume and p.volume>.01)
        # Local access lobes let a seam step behind its main plane only where
        # an assembly path actually touches the shell. The removed material is
        # kept in the cover, never discarded as a tool port.
        def access_lobes(padding):
            lobes=[]
            for contact in path_contacts:
                hull=contact.convex_hull.vertices
                offsets=np.array([[x,y,z] for x in [-padding,padding] for y in [-padding,padding] for z in [-padding,span+padding]])
                lobes.append(tm.convex.convex_hull((hull[:,None,:]+offsets[None,:,:]).reshape(-1,3)))
            return union(*lobes)
        lobes=access_lobes(.15) if path_contacts else tm.Trimesh()
        wider_lobes=access_lobes(.15+gap) if path_contacts else tm.Trimesh()
        point=hint.get('center_mm',outer.bounds.mean(axis=0)) if np.dot(normal,hint['normal'])>.999 else outer.bounds.mean(axis=0)
        semantic=tm.transform_points([point],inv)[0]
        levels=plane_levels(o.bounds,semantic[2],[p.bounds for p in fixed],wall,gap,
                            [float(p.bounds[0,2]) for p in path_contacts])
        routed_repair_attempted=False
        for plane in levels:
            if time.monotonic()-started>budget_s:
                error=PanelSearchError('可拆壳体搜索达到预算；未退回永久开孔。最近候选：'+str(reports[-4:]))
                error.candidates=reports
                raise error
            for mode in ['plane','protected','access'] if fixed else ['plane','access']:
                stepped=mode!='plane'
                record={'normal':normal.tolist(),'plane_mm':plane,'stepped':stepped,'mode':mode};reports.append(record)
                if stepped and shadow_blocks:
                    record['rejected']='retained_shadow_blocks_assembly';continue
                cache_key=(tuple(normal),plane,mode)
                if mode!='access' and cache_key in geometry_rejections:
                    record['rejected']=geometry_rejections[cache_key];record['reused_geometry_rejection']=True;continue
                try:
                    mask=box([lo[0]-20,lo[1]-20,plane],[hi[0]+20,hi[1]+20,hi[2]+20])
                    removal=box([lo[0]-20,lo[1]-20,plane-gap],[hi[0]+20,hi[1]+20,hi[2]+20])
                    active_shadow=shadow;active_cap_shadow=cap_shadow
                    if mode=='access':
                        mask=union(mask,lobes);removal=union(removal,wider_lobes)
                        # Hardware is an obstacle, not source material that
                        # must be retained behind its entire projected shadow.
                        # Exact pose and continuous sweep checks below enforce
                        # removal without overconstraining the parting surface.
                    if stepped:
                        removal=cut(removal,active_shadow)
                        # Material behind installed hardware stays with the main
                        # shell, so the cover cannot hook behind it during removal.
                        mask=cut(mask,active_cap_shadow)
                    elif any(overlap(removal,p)>.05 for p in fixed):
                        record['rejected']='protected_interface';continue
                    base=cut(s,removal);cap=intersect(s,mask)
                    # A plane through a protrusion may create loose shell islands.
                    # Route the seam around those islands instead of deleting them.
                    # Unprotected islands on the main side may move to the cover;
                    # disconnected cover details stay with the main through a
                    # retained strip of original material along the opening axis.
                    adjustments=[];seen_components=set()
                    for _ in range(6):
                        if time.monotonic()-started>budget_s:
                            error=PanelSearchError('可拆壳体搜索达到预算；未退回永久开孔。')
                            error.candidates=reports
                            raise error
                        if base.is_empty or cap.is_empty:break
                        main_parts=volume_components(base);cap_parts=volume_components(cap)
                        if len(main_parts)==1 and len(cap_parts)==1:break
                        signature=(tuple(sorted(round(float(p.volume),6) for p in main_parts)),
                                   tuple(sorted(round(float(p.volume),6) for p in cap_parts)))
                        if signature in seen_components:break
                        seen_components.add(signature)
                        changed=False
                        if len(main_parts)>1:
                            anchor=max(main_parts,key=lambda p:sum(overlap(p,f) for f in fixed) if fixed else p.volume)
                            for part in main_parts:
                                if part is anchor or any(overlap(part,f)>.01 for f in fixed):continue
                                mirrored=part.copy().apply_scale([1,1,-1])
                                take=protected_shadow(mirrored,.15).apply_scale([1,1,-1])
                                wide=protected_shadow(mirrored,.15+gap).apply_scale([1,1,-1])
                                mask=union(mask,take);removal=union(removal,wide);changed=True
                                adjustments.append({'side':'cover','volume_mm3':float(part.volume)})
                        if len(cap_parts)>1:
                            largest=max(cap_parts,key=lambda p:p.volume)
                            for part in cap_parts:
                                if part is largest:continue
                                keep=protected_shadow(part,.15);wide=protected_shadow(part,.15+gap)
                                removal=cut(removal,keep);mask=cut(mask,wide);changed=True
                                adjustments.append({'side':'main','volume_mm3':float(part.volume)})
                        if not changed:break
                        mask=cut(mask,active_cap_shadow);removal=cut(removal,active_shadow)
                        base=cut(s,removal);cap=intersect(s,mask)
                    record['component_seam_adjustments']=adjustments
                    if (not routed_repair_attempted and not base.is_empty and not cap.is_empty
                            and regions(base)>1 and open_paths_clear(base,paths)
                            and time.monotonic()-started<budget_s-5):
                        routed_repair_attempted=True
                        components=volume_components(base)
                        connected,repair=connected_interface_frame(s,components,[*paths,*reserved_paths],wall)
                        record['component_material_routing']=repair
                        if repair.get('added_bands',0):
                            bands=connected[len(components):]
                            padding=np.array([[x,y,z] for x in [-gap,gap] for y in [-gap,gap] for z in [-gap,gap]])
                            clearance=[tm.convex.convex_hull((b.vertices[:,None,:]+padding[None,:,:]).reshape(-1,3)) for b in bands]
                            # Retain only the connecting material here. A long
                            # directional shadow can cut the other shell into
                            # islands; the exact continuous extraction check
                            # below decides whether this local seam is usable.
                            removal=cut(removal,*bands)
                            mask=cut(mask,*clearance)
                            base=cut(s,removal);cap=intersect(s,mask)
                    if cap.is_empty or base.is_empty or regions(base)!=1 or regions(cap)!=1:
                        record['rejected']='disconnected_parts';continue
                    if any(overlap(base,p)<overlap(s,p)-.1 for p in fixed):
                        record['rejected']='protected_load_path';continue
                    if not open_paths_clear(base,paths):
                        record['rejected']='open_assembly_path';continue
                    if not manual_extraction_review and translation_pose_blocked(cap,[base,*extraction_hw,*blockers],span):
                        record['rejected']='cover_extraction_before_fastening';continue
                    # Keep original closed material except an explicitly recorded
                    # seam. Tool sweeps are never subtracted from the cover.
                    fasteners=[];holes=[]
                    # Search along the cut cross section, using actual material on
                    # both sides. No body-name or model-coordinate special cases.
                    base_solid=manifold_solid(base);cap_solid=manifold_solid(cap)
                    section=o.section(plane_origin=[0,0,plane],plane_normal=[0,0,1])
                    points=[]
                    if section is not None:
                        for loop in section.discrete:
                            xy=loop[:,:2];segments=np.diff(xy,axis=0)
                            lengths=np.linalg.norm(segments,axis=1);length=float(lengths.sum())
                            if length<6:continue
                            cumulative=np.r_[0,np.cumsum(lengths)]
                            sign=np.sign(np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])) or 1
                            for distance in np.linspace(0,length,min(64,max(8,int(length/4))),endpoint=False):
                                idx=min(len(lengths)-1,int(np.searchsorted(cumulative,distance,side='right')-1))
                                if lengths[idx]<1e-8:continue
                                tangent=segments[idx]/lengths[idx]
                                edge=xy[idx]+tangent*(distance-cumulative[idx])
                                inset=sign*np.array([-tangent[1],tangent[0]])*min(wall/2,2.)
                                points.append(tuple(edge+inset))
                    candidates=[]
                    for x,y in points:
                        lower=cylinder(x,y,plane-3,plane,4.6,16)
                        upper=cylinder(x,y,plane,plane+3.5,4.6,16)
                        if (base_solid^manifold_solid(lower)).volume()<2 or (cap_solid^manifold_solid(upper)).volume()<2:continue
                        candidates.append((x,y))
                    record['screw_candidates']=len(candidates);record['screw_rejections']={}
                    def reject_screw(reason):record['screw_rejections'][reason]=record['screw_rejections'].get(reason,0)+1
                    # Prefer spatially separated fasteners; clustered screws do not
                    # define a stable cover attachment.
                    while candidates and len(fasteners)<4:
                        if time.monotonic()-started>budget_s:break
                        xy=max(candidates,key=lambda p:min(np.linalg.norm(np.asarray(p)-h) for h in holes) if holes else np.linalg.norm(p))
                        candidates.remove(xy);x,y=xy
                        if holes and min(np.linalg.norm(np.asarray(xy)-h) for h in holes)<9:continue
                        lower=intersect(cylinder(x,y,plane-3,plane,4.6),o)
                        upper=intersect(cylinder(x,y,plane,plane+3.5,4.6),o)
                        seats=union(lower,upper)
                        if any(overlap(seats,p)>.02 for p in hw+paths+fixed+reserved_paths):reject_screw('seat_obstacle');continue
                        shaft=cylinder(x,y,plane-4,hi[2]+20,1.7)
                        head=cylinder(x,y,plane+1.5,hi[2]+20,3.2)
                        nut=cylinder(x,y,plane-3.1,plane+.1,3.35,6)
                        nut_path=cylinder(x,y,plane-3.1,hi[2]+20,3.35,6)
                        a=cut(union(base,lower),shaft,nut);b=cut(union(cap,upper),shaft,head)
                        if regions(a)!=1 or regions(b)!=1:reject_screw('disconnected_after_bore');continue
                        if overlap(a,b)>.02:reject_screw('seat_part_overlap');continue
                        if overlap(a,nut_path)>.02 or any(overlap(nut_path,p)>.02 for p in blockers):reject_screw('nut_entry');continue
                        if not cover_seats_supported(b,a,x,y,plane,thread="M3"):reject_screw('bearing');continue
                        if not open_paths_clear(a,paths):reject_screw('new_seat_blocks_assembly');continue
                        fastener={'position_mm':tm.transform_points([[x,y,plane]],transform)[0].tolist(),
                            'candidate_thread':'M3','required_underhead_length_mm':6.,'actual_fastener_length_mm':None,
                            'physical_fit_verified':False,'nut_preload':'cover_removed_along_opening_axis'}
                        from .fastener_search import packed
                        fastener['restore_cut_material']={'base':packed(cut(union(base,lower),a).apply_transform(transform)),
                            'cap':packed(cut(union(cap,upper),b).apply_transform(transform))}
                        if not cover_tools_clear([fastener],transform,plane,float(hi[2]),hardware+list(extraction_obstacles)):reject_screw('external_tool');continue
                        base,cap=a,b;holes.append(np.array(xy));fasteners.append(fastener)
                    if len(fasteners)<3 or regions(base)!=1 or regions(cap)!=1:
                        record['rejected']='cover_fastening';record['accepted_screws']=len(fasteners);continue
                    # Cheap exact-pose rejection before the continuous sweep.
                    # Sampling can reject, but can never certify an accepted path.
                    if not manual_extraction_review and translation_pose_blocked(cap,[base,*extraction_hw,*blockers],span):
                        record['rejected']='cover_extraction_pose';continue
                    # Accepted candidates require the full continuous sweep.
                    sweep=linear_sweep(cap,[0,0,1],span)
                    hits=[overlap(sweep,p) for p in [base,*extraction_hw,*blockers]]
                    if any(v>.1 for v in hits) and not manual_extraction_review:
                        record.update(rejected='cover_extraction',overlaps_mm3=hits);continue
                    if any(overlap(base,p)<overlap(s,p)-.1 for p in fixed):
                        record['rejected']='fastener_cut_protection';continue
                    added=cut(union(base,cap),s)
                    if any(overlap(added,p)>.02 for p in hw):
                        record['rejected']='added_hardware_collision';continue
                    seam_region=cut(removal,mask)
                    seam=intersect(s,seam_region)
                    authorized=union(seam_region,*[union(cylinder(x,y,plane-4,hi[2]+20,1.7),cylinder(x,y,plane+1.5,hi[2]+20,3.2),cylinder(x,y,plane-3.1,plane+.1,3.35,6)) for x,y in holes])
                    missing=cut(s,base,cap,authorized)
                    if missing.volume>.1:
                        record['rejected']='unregistered_closed_opening';record['unregistered_missing_mm3']=float(missing.volume);continue
                    record['passed']=True
                    return base.apply_transform(transform),cap.apply_transform(transform),{
                        'method':'protected_stepped_split' if stepped else 'offset_plane_split',
                        'transform_mm':transform.tolist(),'plane_z_mm':plane,'top_local_mm':float(hi[2]),
                        'normal':normal.tolist(),'fasteners':fasteners,'fastener_candidate_xy_mm':[list(p) for p in points],'clearance_mm':gap,'source_hint':hint,
                        'tested_candidates':reports,'continuous_extraction':not any(v>.1 for v in hits),'extraction_distance_mm':span,
                        'manual_extraction_review_required':any(v>.1 for v in hits),
                        'extraction_overlap_mm3':hits,'assembly_policy':'manual_extraction_review' if manual_extraction_review else 'strict',
                        'extraction_scope':'empty_shell_parts' if shell_parts_only else 'installed_hardware',
                        'closed_unregistered_missing_mm3':float(missing.volume),'seam_removed_mm3':float(seam.volume),
                        'permanent_assembly_ports':False,'protected_group_count':len(protected),
                        'open_path_count':len(corridors),'manufacturing_ready':False}
                except ValueError as exc:
                    if not any(word in str(exc) for word in ('实体','承力连接','表面')):raise
                    record.update(rejected='invalid_boolean_candidate',geometry_error=str(exc))
                    continue
                finally:
                    reason=record.get('rejected')
                    if mode!='access' and reason in {'protected_interface','protected_load_path'}:
                        geometry_rejections[cache_key]=reason
    error=PanelSearchError('未找到保护安装面、可拆卸且可闭合的壳体；未生成永久工具孔。最近候选：'+str(reports[-6:]))
    error.candidates=reports
    raise error


def connected_interface_frame(shell, protected, corridors, wall):
    """Retain source-shell bands between mounting groups; never invent ribs.

    Shortest paths run through actual shell material with assembly corridors
    excluded. The result consists solely of original shell material and is
    accepted only if all mounting groups form one connected protected frame.
    """
    if len(protected)<2:return protected,{'method':'single_interface','added_bands':0}
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    from .joint_support import beam
    combined=union(*protected)
    try:groups=volume_components(combined)
    except ValueError:return protected,{'method':'unresolved_interface_grouping','added_bands':0}
    if len(groups)<2:return protected,{'method':'already_connected_interfaces','added_bands':0}
    from scipy.ndimage import binary_dilation
    pitch=min(1.5,max(.8,wall*.4));radius=min(2.,max(1.2,wall*.6))
    if np.prod(np.ceil(shell.extents/pitch)+5)>2000000:
        return protected,{'method':'frame_grid_budget_exceeded','added_bands':0}
    voxel=shell.voxelized(pitch)
    occupied=binary_dilation(voxel.matrix,iterations=1)
    indices=np.argwhere(occupied);vertices=voxel.indices_to_points(indices)
    # The lattice is only a routing proposal. Keep actual material, not the
    # filled exterior volume, so internal cavities are never bridged by fiat.
    inside=shell.contains(vertices);indices=indices[inside];vertices=vertices[inside]
    blocked=np.zeros(len(vertices),dtype=bool)
    for corridor in corridors:
        if corridor.is_empty:continue
        nearby=np.flatnonzero(np.all(vertices>=corridor.bounds[0]-.8,axis=1)&np.all(vertices<=corridor.bounds[1]+.8,axis=1))
        if not len(nearby):continue
        points=vertices[nearby]
        blocked[nearby]|=(tm.proximity.closest_point(corridor,points)[1]<.7)|corridor.contains(points)
    indices=indices[~blocked];vertices=vertices[~blocked]
    if not len(vertices):return protected,{'method':'no_clear_frame_material','added_bands':0}
    tree=cKDTree(vertices);anchors=[]
    for group in groups:
        distance,index=tree.query(group.vertices)
        if distance.min()>radius*1.5:return protected,{'method':'no_clear_interface_anchor','added_bands':0}
        eligible=index[distance<=distance.min()+pitch*.25]
        anchors.append(int(eligible[np.argmin(np.linalg.norm(vertices[eligible]-group.centroid,axis=1))]))
    lookup=np.full(occupied.shape,-1,dtype=np.int32);lookup[tuple(indices.T)]=np.arange(len(indices))
    rows=[];cols=[];costs=[]
    for dx in [-1,0,1]:
        for dy in [-1,0,1]:
            for dz in [-1,0,1]:
                delta=np.array([dx,dy,dz])
                if (dx,dy,dz)<=(0,0,0):continue
                target=indices+delta
                valid=np.all(target>=0,axis=1)&np.all(target<occupied.shape,axis=1)
                source=np.flatnonzero(valid);destination=lookup[tuple(target[valid].T)]
                keep=destination>=0;source=source[keep];destination=destination[keep]
                rows.extend(source);cols.extend(destination);costs.extend([pitch*np.linalg.norm(delta)]*len(source))
    graph=coo_matrix((np.r_[costs,costs],(np.r_[rows,cols],np.r_[cols,rows])),shape=(len(vertices),len(vertices))).tocsr()
    distances,previous=dijkstra(graph,directed=False,indices=anchors[0],return_predecessors=True)
    if any(not np.isfinite(distances[a]) for a in anchors):return protected,{'method':'no_clear_material_connection','added_bands':0}
    paths=[]
    for anchor in anchors[1:]:
        path=[anchor]
        while path[-1]!=anchors[0]:
            parent=int(previous[path[-1]])
            if parent<0:return protected,{'method':'incomplete_surface_path','added_bands':0}
            path.append(parent)
        paths.append(path)
    for stride in [3,1]:
        bands=[]
        for path in paths:
            sampled=path[::stride]
            if sampled[-1]!=path[-1]:sampled.append(path[-1])
            for a,b in zip(sampled,sampled[1:]):
                if np.linalg.norm(vertices[a]-vertices[b])<1e-5:continue
                material=cut(intersect(shell,beam(vertices[a],vertices[b],radius)),*corridors)
                if material.is_empty or not material.is_volume or material.volume<.01:continue
                bands.append(material)
        if not bands:continue
        if any(overlap(b,p)>.1 for b in bands for p in corridors):continue
        frame=union(combined,*bands)
        if regions(frame)==1:
            return [*protected,*bands],{'method':'retained_source_material_frame','added_bands':len(bands),
                'frame_volume_mm3':float(frame.volume),'new_material_added_mm3':0.,'physical_strength_verified':False}
    return protected,{'method':'surface_frame_not_connected','added_bands':0}


def split_body(shell,outer,hint,hardware,protected,wall,corridors,extraction_obstacles=(),max_covers=6,
               connect_interfaces=True,total_budget_s=180.):
    """Bounded multi-cover fallback for opposing installation directions.

    Covers are grouped by actual obstructed shell contacts, not actuator or body
    names. Every accepted sub-split retains the earlier cover's fastening seats.
    """
    started=time.monotonic()
    if connect_interfaces:
        protected,frame_report=connected_interface_frame(shell,protected,corridors,wall)
    else:
        frame_report={'method':'mount_supports_generated_after_split','added_bands':0}
    groups={};center=np.asarray(outer.centroid)
    for path in corridors:
        contact=intersect(shell,path)
        if contact.is_empty or contact.volume<=.1:continue
        extensions=np.r_[np.maximum(path.bounds[1]-outer.bounds[1],0),np.maximum(outer.bounds[0]-path.bounds[0],0)]
        if extensions.max()>1:
            index=int(np.argmax(extensions));axis=index%3;sign=1 if index<3 else -1
        else:
            delta=np.asarray(contact.centroid)-center
            axis=int(np.argmax(abs(delta)/np.maximum(outer.extents,1)));sign=1 if delta[axis]>=0 else -1
        key=(axis,sign)
        groups.setdefault(key,[]).append(path)
    geometry_rejections={}
    try:
        base,cap,panel=split_shell(shell,outer,hint,hardware,protected,wall,corridors,extraction_obstacles,
            budget_s=8. if len(groups)>1 else 40.,geometry_rejections=geometry_rejections)
        panel['interface_frame']=frame_report
        return base,[(cap,panel)]
    except PanelSearchError as error:
        single_failure=str(error)
    if not groups:raise PanelSearchError(single_failure)
    base=shell;covers=[];reserved=list(protected)
    # Keep enough rim for end-cover screws before broad side covers remove
    # that rim. On roughly isotropic bodies, prefer the larger access group.
    longest=float(max(outer.extents))
    queue=sorted(groups.items(),key=lambda item:(
        0 if outer.extents[item[0][0]]>=longest*.9 else 1,
        -sum(overlap(shell,p) for p in item[1])))
    while queue:
        if time.monotonic()-started>=total_budget_s:
            raise PanelSearchError('装配分型达到总预算，需要调整分型建议或装配方向')
        (axis,sign),paths=queue.pop(0)
        paths=[p for p in paths if overlap(base,p)>.1]
        if not paths:continue
        if len(covers)>=max_covers:raise PanelSearchError('可拆壳件数量超过搜索上限；未生成永久工具孔')
        normal=np.eye(3)[axis]*sign
        local_hint={**hint,'normal':normal.tolist(),'reason':'根据实际装配通道与外壳接触区域搜索可拆壳件；'+hint.get('reason','')}
        adjacent=[p for (other_axis,_),group in groups.items() if other_axis!=axis for p in group if overlap(base,p)>.1]
        options=[paths,paths+adjacent] if adjacent else [paths]
        found=False;last_error=None
        for option in options:
            try:
                candidate,cap,panel=split_shell(base,outer,local_hint,hardware,reserved,wall,option,
                    [*extraction_obstacles,*[m for m,_ in covers]],budget_s=max(.1,min(40.,total_budget_s-(time.monotonic()-started))),reserved_corridors=corridors,geometry_rejections=geometry_rejections)
                base=candidate;geometry_rejections={};found=True;break
            except PanelSearchError as exc:last_error=exc
        if not found:
            if len(paths)>1:
                # A retained structural strip can separate two service areas
                # on the same side. Search independent removable panels.
                queue=[((axis,sign),[p]) for p in paths]+queue
                continue
            raise last_error
        panel['interface_frame']=frame_report
        panel['single_cover_search_failure']=single_failure
        covers.append((cap,panel))
        transform=np.asarray(panel['transform_mm']);inverse=np.linalg.inv(transform);plane=panel['plane_z_mm']
        for f in panel['fasteners']:
            x,y,_=tm.transform_points([f['position_mm']],inverse)[0]
            seat=cylinder(x,y,plane-3.2,plane+.1,4.).apply_transform(transform)
            material=intersect(base,seat)
            if not material.is_empty:reserved.append(material)
        # Seat ownership is protected explicitly; connectivity is checked on
        # each entire candidate base. Do not impose a particular additional
        # connecting strip here: its directional shadow can unnecessarily
        # obstruct a different cover's assembly path.
    if not open_paths_clear(base,corridors):raise PanelSearchError('多壳件拆开后仍有受阻装配路径；未生成永久工具孔')
    # Later seats must not block an earlier cover. Check actual finished parts.
    for i,(cap,panel) in enumerate(covers):
        inv=np.linalg.inv(np.asarray(panel['transform_mm']))
        local=cap.copy().apply_transform(inv)
        obstacles=[base,*hardware,*extraction_obstacles,*[m for j,(m,_) in enumerate(covers) if j!=i]]
        local_obstacles=[m.copy().apply_transform(inv) for m in obstacles]
        if translation_pose_blocked(local,local_obstacles,panel['extraction_distance_mm']):
            raise PanelSearchError('新增壳件固定座阻挡先前盖板拆卸；多壳件候选被拒绝')
        sweep=linear_sweep(local,[0,0,1],panel['extraction_distance_mm'])
        if any(overlap(sweep,m)>.1 for m in local_obstacles):
            raise PanelSearchError('多壳件的最终连续拆卸路径受阻')
    return base,covers
