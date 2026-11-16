"""Build new source-dependent shells and joint-specific complete modules.

Failed candidates are exported with their failed checks, never adopted as a
validated assembly. Geometry is generated from this input, not old artifacts.
"""
import hashlib
import io
import json
import math
import time
import zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
from scipy.ndimage import binary_dilation, distance_transform_edt
from scipy.spatial import cKDTree
from ..project import atomic
from ..contracts import now
from .articulation import load_source, link_partition, connected_partition, PALETTE
from .joint_modules import module, TEMPLATE_PATH
from .joint_placement import placement_metrics, room_fits, placement_score, registration_metrics, mounting_surface_checks
from .joint_support import beam, attachment
from .fastening import placed_access, check_access, cylinder_path, descendants, order_assembly, apply_shell_assembly_stages
from .solid import box, cylinder, union, cut, intersect, overlap, regions, manifold_solid, from_manifold, linear_sweep, placed_sweep
from .structural_paths import reconnect
from .voxel_surface import surface
from .assembly_split import split_shell, open_paths_clear, translation_pose_blocked
from .shell_access import suggest, reconnect_local, separate_cutting_remnants, access_requirement, PanelSearchError, separate_voxel_artifacts


def frame(joint, bodies):
    z=np.array(joint['axis'],dtype=float);x=np.array(bodies[joint['child']]['seed_mm'],dtype=float)-bodies[joint['parent']]['seed_mm']
    x=x-z*(x@z)
    if np.linalg.norm(x)<1e-8:
        x=np.eye(3)[np.argmin(abs(z))];x=x-z*(x@z)
    x/=np.linalg.norm(x);y=np.cross(z,x)
    transform=np.eye(4);transform[:3,:3]=np.column_stack([x,y,z]);transform[:3,3]=joint['position_mm']
    return transform


def poses(graph, angles):
    """World rest-frame meshes: parent motion times rotation around rest-world axis."""
    matrices={graph['root_body_id']:np.eye(4)};pending=list(graph['joints'])
    while pending:
        ready=[j for j in pending if j['parent'] in matrices]
        if not ready:raise ValueError('关节图无法求值')
        for j in ready:
            rotation=tm.transformations.rotation_matrix(angles.get(j['id'],0),j['axis'],j['position_mm'])
            matrices[j['child']]=matrices[j['parent']]@rotation;pending.remove(j)
    return matrices


def sample_angles(graph):
    samples=[{j['id']:0 for j in graph['joints']}]
    for j in graph['joints']:
        for degree in np.linspace(*j['limits_deg'],7):samples.append({j['id']:math.radians(float(degree))})
    samples.extend({j['id']:math.radians(j['limits_deg'][side]) for j in graph['joints']} for side in [0,1])
    return samples



def unique_relative_obstacles(owner, sampled_poses):
    """Visit every distinct obstacle pose once, preserving exact transforms.

    Samples for independent branches repeat the same relative poses. Within a
    phase the obstacle meshes are frozen, so repeating an identical subtraction
    cannot remove additional material. Do not round transforms or skip poses.
    """
    seen=set()
    for matrices in sampled_poses:
        inverse=np.linalg.inv(matrices[owner])
        for other in matrices:
            if owner==other:continue
            relative=inverse@matrices[other]
            key=(other,relative.tobytes())
            if key in seen:continue
            seen.add(key)
            yield other,relative



def region_source_component_count(region_bounds, component_bounds):
    """Conservative source ownership: disjoint bounds cannot share material.

    Overlapping bounds may overcount, which keeps the cleanup disabled. Never
    infer a single source from centroid distances or drop a real source solid.
    """
    bounds=np.asarray(region_bounds)
    return sum(bool(np.all(np.minimum(bounds[1],np.asarray(b)[1])>
                           np.maximum(bounds[0],np.asarray(b)[0]))) for b in component_bounds)


def interior_mask(mask,pitch,requested_wall):
    # Ceil to a complete cell: when pitch exceeds requested wall, comparing EDT
    # directly to requested_wall would classify every boundary cell as interior.
    effective_wall=math.ceil(requested_wall/pitch)*pitch
    distance=distance_transform_edt(np.pad(mask,1))[1:-1,1:-1,1:-1]*pitch
    return mask&(distance>effective_wall+1e-8),effective_wall


def stl_deliverable(mesh, millimeters_per_unit=1.):
    """Repair storage-level aliases within explicit geometric/volume budgets.

    Preserve the original indexed surface first. Distinct valid topology vertices
    may share a float32 position in STL; locally separate just those aliases
    inwards along their own surface normals, then independently reload and check.
    """
    scale=float(millimeters_per_unit)
    volume=float(mesh.volume);attempts=[];chosen=mesh;last_method='source'
    original_vertices=np.asarray(mesh.vertices).copy()
    vertex_tree=cKDTree(original_vertices)
    def volume_budget(displacement):
        base=max(.001/scale**3,abs(volume)*1e-6)
        # STL repair has a measured displacement budget of 0.001 mm.
        # Couple its volume allowance to area * displacement in either unit
        # system, capped at 100 ppm; closed topology is still mandatory.
        return max(base,min(mesh.area*displacement*2,abs(volume)*1e-4))
    sources=[('source',mesh,0)]
    if mesh.is_volume:
        _,inverse,counts=np.unique(np.round(np.asarray(mesh.vertices,dtype=np.float32).astype(float),8),axis=0,return_inverse=True,return_counts=True)
        aliases=np.flatnonzero(counts[inverse]>1)
        if len(aliases):
            normals=np.asarray(mesh.vertex_normals)[aliases].copy()
            for epsilon in [.00005,.0001,.00025,.0005]:
                candidate=mesh.copy();candidate.vertices[aliases]-=normals*(epsilon/scale)
                sources.append(('local_quantization_separation',candidate,len(aliases)))
    # Existing tiny-edge cleanup remains a fallback, within the same budgets.
    sources.append(('bounded_edge_simplification',None,0))
    for method,source,adjusted_vertices in sources:
        if source is None:
            source=from_manifold(manifold_solid(mesh).simplify(.0001/scale))
            # Many sequential cuts can leave both tiny edges and float32
            # aliases. Simplification alone cannot fix the aliases, and moving
            # aliases before simplification cannot resolve the tiny-edge fan.
            # Apply the existing bounded displacement repair to this reduced
            # surface too, always measuring against the original input below.
            if source.is_volume:
                _,inverse,counts=np.unique(np.round(np.asarray(source.vertices,dtype=np.float32).astype(float),8),axis=0,return_inverse=True,return_counts=True)
                aliases=np.flatnonzero(counts[inverse]>1)
                if len(aliases):
                    normals=np.asarray(source.vertex_normals)[aliases].copy()
                    for epsilon in [.00005,.0001,.00025,.0005]:
                        candidate=source.copy();candidate.vertices[aliases]-=normals*(epsilon/scale)
                        sources.append(('simplified_local_quantization_separation',candidate,len(aliases)))
        raw=source.export(file_type='stl')
        # A valid raw roundtrip needs no face cleanup (whose absolute threshold
        # otherwise deletes legitimate small faces in SI-unit meshes).
        reopened=tm.load(io.BytesIO(raw),file_type='stl',force='mesh')
        change=abs(float(reopened.volume)-volume)
        displacement=float(vertex_tree.query(reopened.vertices)[0].max()) if len(reopened.vertices) else float('inf')
        valid=bool(reopened.is_volume and reopened.is_watertight and change<volume_budget(displacement) and displacement<=.001/scale)
        attempts.append({'method':method,'adjusted_vertex_count':adjusted_vertices,'raw_roundtrip':True,
            'maximum_vertex_displacement_mm':displacement*scale,'maximum_allowed_displacement_mm':.001,
            'maximum_allowed_volume_change_mm3':volume_budget(displacement)*scale**3,'volume_change_mm3':change*scale**3,'roundtrip_valid':valid})
        if valid:return reopened,{'attempts':attempts,'passed':True,'method':method,'source_model_modified':False,'derived_mesh_repaired':method!='source'}
        for digits in [8,7,6,5,4,3]:
            candidate=tm.load(io.BytesIO(raw),file_type='stl',force='mesh',process=False)
            candidate.merge_vertices(digits_vertex=digits)
            candidate.update_faces(candidate.nondegenerate_faces());candidate.update_faces(candidate.unique_faces());candidate.remove_unreferenced_vertices()
            reopened=tm.load(io.BytesIO(candidate.export(file_type='stl')),file_type='stl',force='mesh')
            change=abs(float(reopened.volume)-volume)
            displacement=float(vertex_tree.query(reopened.vertices)[0].max()) if len(reopened.vertices) else float('inf')
            valid=bool(reopened.is_volume and reopened.is_watertight and change<volume_budget(displacement) and displacement<=.001/scale)
            attempts.append({'method':method,'adjusted_vertex_count':adjusted_vertices,'weld_tolerance_mm':10.**-digits*scale,
                'maximum_vertex_displacement_mm':displacement*scale,'maximum_allowed_displacement_mm':.001,
                'removed_degenerate_or_duplicate_faces':len(mesh.faces)-len(reopened.faces),
                'maximum_allowed_volume_change_mm3':volume_budget(displacement)*scale**3,'volume_change_mm3':change*scale**3,'roundtrip_valid':valid})
            chosen=reopened;last_method=method
            if valid:return chosen,{'attempts':attempts,'passed':True,'method':method,'source_model_modified':False,'derived_mesh_repaired':method!='source'}
    # Binary STL stores float32 positions. Preserve the unmodified float64
    # surface with standard ASCII STL when binary quantization destroys topology.
    raw=mesh.export(file_type='stl_ascii').encode()
    reopened=tm.load(io.BytesIO(raw),file_type='stl',force='mesh')
    change=abs(float(reopened.volume)-volume)
    displacement=float(vertex_tree.query(reopened.vertices)[0].max()) if len(reopened.vertices) else float('inf')
    valid=bool(reopened.is_volume and reopened.is_watertight and change<volume_budget(displacement) and displacement<=.001/scale)
    attempts.append({'method':'source_ascii_precision','maximum_vertex_displacement_mm':displacement*scale,'volume_change_mm3':change*scale**3,'roundtrip_valid':valid})
    if valid:return reopened,{'attempts':attempts,'passed':True,'method':'source_ascii_precision','stl_encoding':'ascii','source_model_modified':False,'derived_mesh_repaired':False}
    return chosen,{'attempts':attempts,'passed':False,'method':last_method,'source_model_modified':False,'derived_mesh_repaired':False}


def generate(project, source_path, directory):
    directory=Path(directory);events=[];started=time.monotonic();checks=[];warnings=[]
    generator_sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')}
    def progress(stage,detail):
        events.append({'at':now(),'type':stage,'detail':detail});atomic(directory/'progress.json',events)
    def check(name,passed,**detail):checks.append({'name':name,'passed':bool(passed),**detail})
    graph=project['articulation']['graph'];bodies={b['id']:b for b in graph['bodies']}
    binding={b['joint_id']:b for b in project['motor_bindings']}
    if set(binding)!={j['id'] for j in graph['joints']}:raise ValueError('每个标注关节都需要电机选择')
    progress('source','重新处理当前源模型；按照已标注关节和已选电机生成结构')
    shape,voxels,analysis=load_source(project,source_path)
    template_raw=TEMPLATE_PATH.read_bytes();library=json.loads(template_raw);design=library['design'];pitch=float(voxels.pitch[0])
    partition_report={};points=voxels.points;labels=connected_partition(voxels,graph,report=partition_report);indices=voxels.points_to_indices(points)
    analysis['partition_refinement']=partition_report
    labelgrid=np.full(voxels.shape,-1,dtype=np.int16);labelgrid[tuple(indices.T)]=labels
    def mesh_of(mask):
        mesh,report=surface(mask,pitch,voxels.transform[:3,3])
        analysis.setdefault('surface_extractions',[]).append(report)
        return mesh
    pieces={};lids={};skins={};outer_regions={};assembly=[];parts=[];cavities=[];all_modules=[];placement_candidates=[]
    for k,body in enumerate(graph['bodies']):
        progress('shells','为 '+body['name']+' 生成保留凹形的空心壳与可拆上盖')
        mask=labelgrid==k
        # One-cell semantic partition seam only; never radius times angle.
        # Hardware fitting below is sub-mm exact geometry, independent of this grid.
        other=(labelgrid>=0)&(labelgrid!=k)
        mask &= distance_transform_edt(~other)*pitch>pitch
        if np.count_nonzero(mask)<30:raise ValueError(body['name']+' 分缝后体积不足')
        outer=mesh_of(mask)
        inside,effective_wall=interior_mask(mask,pitch,design['shell_requested_wall'])
        analysis['shell_wall_grid_mm']=effective_wall
        analysis['partition_method']='connected_semantic_regions_with_internal_flood'
        analysis['boolean_precision']={'edge_simplification_mm':.0001,'maximum_relative_volume_change':1e-6}
        if np.count_nonzero(inside)<8:
            # A thin output link or externally serviced link need not contain
            # an artificial cavity. Internal electronics still require access.
            prospective=[]
            for joint in graph['joints']:
                template=library['templates'][binding[joint['id']]['catalog_id']]
                prospective.extend([{'owner':joint['parent'],'joint_id':joint['id'],'role':'motor',
                                     'external_service':template.get('mounting_style')=='open_cradle'},
                                    {'owner':joint['child'],'joint_id':joint['id'],'role':'output_mount'}])
            if access_requirement(body['id'],graph,prospective,project)['required']:
                raise ValueError(body['name']+' 过薄，无法形成内部器件所需空腔')
            shell=outer.copy()
            analysis.setdefault('solid_links',[]).append({'owner_body_id':body['id'],'reason':'薄小连接件无内部维护需求，保留实心'})
        else:shell=cut(outer,mesh_of(inside))
        skins[body['id']]=shell;outer_regions[body['id']]=outer
    component_bounds=[c['bounds_mm'] for c in analysis['components']]
    analysis['owner_source_component_counts']={owner:region_source_component_count(region.bounds,component_bounds) for owner,region in outer_regions.items()}
    for joint in graph['joints']:
        selected=binding[joint['id']];progress('joint_module','生成 '+joint['name']+' 的 '+selected['branch']+' 模块')
        built=module(selected['catalog_id'],library);base_frame=frame(joint,bodies)
        # The native output datum belongs at the annotated joint, not anywhere
        # along its infinite axis. Rectangular motors may roll, never drift.
        candidates=[]
        axial_step=float(built['cavity'].extents[2])*.4
        entry_hulls={step['role']:next(p[3] for p in built['parts'] if p[0]==step['role']).convex_hull for step in built['installation_steps']} if built['allow_exterior'] else {}
        previous_entries=[s for earlier in parts if earlier['joint']['parent']==joint['parent'] for s in earlier['entry_sweeps'].values()]
        previous_hardware=[p['mesh'] for p in all_modules if p['owner']==joint['parent']]
        parent_envelope=union(*[p[3] for p in built['parts'] if p[2]=='parent']) if built['allow_exterior'] else None
        def add_candidate(shift,roll,retainer_direction=None):
            local=tm.transformations.rotation_matrix(roll,[0,0,1]);local[2,3]=shift
            if built['allow_exterior']:local[:3,:3][abs(local[:3,:3])<1e-12]=0.
            transform=base_frame@local;registration=registration_metrics(transform,joint,built);candidate=built['cavity'].copy().apply_transform(transform)
            metrics=placement_metrics(candidate,shape,built['allow_exterior'])
            collision=sum(overlap(candidate,previous) for previous in cavities)
            previous_entry_overlap=sum(overlap(parent_envelope.copy().apply_transform(transform),s) for s in previous_entries) if built['allow_exterior'] else 0.
            support_distances=[float(cKDTree(skins[joint[side]].vertices).query(tm.transform_points([built[key]],transform)[0])[0]) for side,key in [('parent','fixed_anchor'),('child','moving_anchor')]]
            entry_intrusion=0.;entry_directions={};entry_hardware_overlap=0.
            for step in built['installation_steps'] if built['allow_exterior'] else []:
                travelling=entry_hulls[step['role']].copy().apply_transform(transform)
                options=[]
                for local_direction in ([retainer_direction] if step['role']=='retainer' and retainer_direction is not None else [step['direction'],*step.get('alternative_directions',[])]):
                    direction=transform[:3,:3]@np.array(local_direction)
                    distance=max(0.,float((outer_regions[joint['parent']].vertices@direction).max()-(travelling.vertices@direction).min()))+20
                    sweep=tm.convex.convex_hull(np.vstack([travelling.vertices,travelling.vertices+direction*distance]))
                    obstruction=sum(overlap(sweep,hardware) for hardware in previous_hardware)
                    options.append((obstruction,overlap(sweep,skins[joint['parent']]),local_direction))
                obstruction,intrusion,chosen_direction=min(options,key=lambda option:(option[0]>.1,option[0]*10+option[1]))
                entry_intrusion+=intrusion;entry_hardware_overlap+=obstruction;entry_directions[step['role']]=chosen_direction
            candidates.append({'axial_shift_mm':shift,'roll_rad':roll,**registration,**metrics,'other_module_overlap_mm3':collision,
                'previous_entry_overlap_mm3':previous_entry_overlap,'entry_other_module_overlap_mm3':entry_hardware_overlap,
                'support_distances_mm':support_distances,'entry_shell_cut_estimate_mm3':entry_intrusion,'installation_directions':entry_directions,
                'score':placement_score(metrics,built,support_distances,collision,shift)+entry_intrusion*2+(previous_entry_overlap+entry_hardware_overlap)*10,'transform_mm':transform.tolist()})
        def fits(candidate):
            return (candidate['output_registration_passed'] and room_fits(candidate,built,design) and candidate['other_module_overlap_mm3']<=.1
                    and candidate['previous_entry_overlap_mm3']<=.1 and candidate['entry_other_module_overlap_mm3']<=.1
                    and max(candidate['support_distances_mm'])<=design['max_bridge_span_mm'])
        shifts=[0] if built['allow_exterior'] else [0,axial_step,-axial_step]
        for shift in shifts:
            for roll in [0,math.pi/2,math.pi,3*math.pi/2]:add_candidate(shift,roll)
        # Keep the joint axis line and motor scale fixed. Expand only exhausted
        # searches, with explicit finite bounds; do not enlarge the robot or holes.
        if not built['allow_exterior'] and not any(fits(c) for c in candidates):
            for factor in ([0] if built['allow_exterior'] else [-1.5,1.5,-2,2,-.5,.5]):
                for roll in np.arange(8)*math.pi/4:add_candidate(float(factor*axial_step),float(roll))
                if any(fits(c) for c in candidates):break
        feasible=[c for c in candidates if fits(c)]
        chosen=min(feasible or candidates,key=lambda c:c['score']);transform=np.array(chosen['transform_mm'])
        chosen_entries={}
        if built['allow_exterior']:
            from .solid import dilate
            envelopes={step['role']:dilate(next(p[3] for p in built['parts'] if p[0]==step['role']),design['fit_clearance']) for step in built['installation_steps']}
            def support_proposals():
                yield from sorted(feasible,key=lambda c:c['score'])
                for candidate in sorted(feasible,key=lambda c:c['score']):
                    candidate['broaden_support_search']=True
                    yield candidate
                # Only expand roll and entry-direction combinations after the
                # cheap candidates exhaust actual support feasibility.
                start=len(candidates)
                for roll in np.arange(8)*math.pi/4:
                    for direction in [[0,0,1],[0,-1,0]]:
                        if any(abs(c['roll_rad']-roll)<1e-8 and c['installation_directions']['retainer']==direction for c in candidates):continue
                        add_candidate(0,float(roll),direction)
                yield from sorted([c for c in candidates[start:] if fits(c)],key=lambda c:c['score'])
                for candidate in sorted([c for c in candidates[start:] if fits(c)],key=lambda c:c['score']):
                    candidate['broaden_support_search']=True
                    yield candidate
            def record_support(proposal,reason=None):
                if reason is not None:proposal.update(support_paths_feasible=False,support_failure=reason)
                atomic(directory/'placement-search.json',{'joint_id':joint['id'],'candidates':candidates})
            for proposal in support_proposals():
                progress('joint_support',joint['name']+'：检查输出轴锁定后的安装面与装配路径，方向 '+format(math.degrees(proposal['roll_rad']),'.0f')+'°')
                trial=np.array(proposal['transform_mm'])
                world={p[0]:p[3].copy().apply_transform(trial) for p in built['parts']}
                trial_cavity=built['cavity'].copy().apply_transform(trial)
                obstacles=[m for role,m in world.items() if role not in ('fixed_mount','output_mount')]
                obstacles.append(built['fastener_bores'].copy().apply_transform(trial))
                trial_sites=placed_access({**built,'transform':trial,'joint':joint,'id':joint['id'],'graph':graph},outer_regions)
                child_tools=[tool['mesh'] for site in trial_sites if not site['detach_child_subtree'] for tool in site['tools']]
                parent_tools=[tool['mesh'] for site in trial_sites for tool in site['tools']]
                # Check the output bridge first: nearby surfaces alone do not
                # establish a route around the motor body and retaining ears.
                child_shell=cut(skins[joint['child']],trial_cavity,*cavities)
                child_anchor=tm.transform_points([built['moving_anchor']],trial)[0]
                child_support=attachment(child_shell,world['output_mount'],child_anchor,obstacles+child_tools,shape,design,True,proposal.get('broaden_support_search',False))
                if child_support is None:
                    record_support(proposal,'child_connection');continue
                rotations=[tm.transformations.rotation_matrix(math.radians(float(angle)),joint['axis'],joint['position_mm']) for angle in np.linspace(*joint['limits_deg'],7)]
                if any(overlap(child_support['core'].copy().apply_transform(rotation),world[role])>.01
                       for rotation in rotations for role in ('motor','retainer','fixed_mount')):
                    record_support(proposal,'child_motion');continue
                entries={}
                for step in built['installation_steps']:
                    hardware=world[step['role']];direction=trial[:3,:3]@np.array(proposal['installation_directions'][step['role']])
                    distance=max(0.,float((outer_regions[joint['parent']].vertices@direction).max()-(hardware.vertices@direction).min()))+20
                    # Sweep in the registered hardware frame. Rigidly transforming
                    # the result is equivalent but avoids pathological diagonal
                    # Minkowski operations on the detailed servo CAD.
                    entries[step['role']]=placed_sweep(envelopes[step['role']],trial,proposal['installation_directions'][step['role']],distance)
                parent_shell=cut(skins[joint['parent']],trial_cavity,*cavities)
                parent_anchor=tm.transform_points([built['fixed_anchor']],trial)[0]
                parent_support=attachment(parent_shell,world['fixed_mount'],parent_anchor,obstacles+parent_tools+list(entries.values())+previous_entries,shape,design,True,proposal.get('broaden_support_search',False))
                supported=parent_support is not None
                if not supported:proposal['support_failure']='parent_connection'
                if supported:
                    moving=union(world['output_mount'],world['rotor'],child_support['core'])
                    supported=not any(overlap(parent_support['core'],moving.copy().apply_transform(rotation))>.01 for rotation in rotations)
                    if not supported:proposal['support_failure']='parent_motion'
                proposal['support_paths_feasible']=supported
                record_support(proposal)
                if supported:
                    chosen=proposal;transform=trial;chosen_entries=entries;break
            else:
                atomic(directory/'placement-rejection.json',{'joint_id':joint['id'],'candidates':candidates})
                raise ValueError(joint['id']+': No feasible mounting surfaces found within the bounded search at the annotated joint. The output datum was not moved. Review the joint region, shell support and assembly openings.')
            built['installation_steps']=[{**step,'direction':chosen['installation_directions'][step['role']]} for step in built['installation_steps']]
            direction_text='从托架后侧滑入' if chosen['installation_directions']['retainer']==[0,-1,0] else '从上方装回'
            built['instructions'][1]=built['instructions'][1].replace('沿登记方向装回',direction_text)
        placement_candidates.append({'joint_id':joint['id'],'candidates':candidates,'selected':chosen,
            'mounting_style':built['mounting_style'],'axis_line_unchanged':True,'motor_scale':1,'expanded_search':len(candidates)>len(shifts)*4,'candidate_budget':16 if built['allow_exterior'] else len(shifts)*4+48})
        for name,title,side,mesh,printable in built['parts']:
            all_modules.append({'id':joint['id']+'_'+name,'name':joint['name']+' · '+title,'owner':joint[side],
                'mesh':mesh.copy().apply_transform(transform),'printable':printable,'template':selected['catalog_id'],
                'joint_id':joint['id'],'role':name,'template_revision':built['template_revision'],
                'mounting_style':built['mounting_style'],'external_service':built['allow_exterior']})
        cavity=built['cavity'].copy().apply_transform(transform);cavities.append(cavity)
        outside=max(0,float(cavity.volume)-overlap(cavity,shape))
        fraction=outside/float(cavity.volume)
        check(joint['id']+'_module_room',room_fits(chosen,built,design),outside_mm3=outside,cavity_volume_mm3=float(cavity.volume),
              outside_fraction=fraction,mounting_style=built['mounting_style'],
              maximum_outside_fraction=None if built['allow_exterior'] else design['module_max_outside_fraction'],
              sampled_protrusion_mm=chosen.get('sampled_protrusion_mm'),
              maximum_protrusion_mm=design['open_mount_max_protrusion_mm'] if built['allow_exterior'] else None,
              reason='开放托架允许外露；按突出距离、独立承力连接与装配路径校验' if built['allow_exterior'] else '内藏模块超出原外观不得超过 3%；不是实物制造验收')
        check(joint['id']+'_output_registration',chosen['output_registration_passed'],
              output_datum_error_mm=chosen['output_datum_error_mm'],axis_line_error_mm=chosen['axis_line_error_mm'],
              scope='方形电机输出基准锁定标注关节位置；本体偏心不能用沿轴移走整个电机补偿')
        check(joint['id']+'_placement',fits(chosen),support_distances_mm=chosen['support_distances_mm'],other_module_overlap_mm3=chosen['other_module_overlap_mm3'])
        if not built['allow_exterior'] and design['module_outside_warning_fraction']<fraction<=design['module_max_outside_fraction']:
            motor_mesh=next(p['mesh'] for p in all_modules if p['joint_id']==joint['id'] and p['role']=='motor')
            warnings.append({'code':'module_envelope_outside','joint_id':joint['id'],'outside_fraction':fraction,
                'outside_mm3':outside,'motor_body_outside_mm3':max(0.,float(motor_mesh.volume)-overlap(motor_mesh,shape)),
                'reason':joint['name']+'：完整安装包络有 '+format(fraction*100,'.2f')+'% 超出原外观；允许继续预览，但可能突出或需要局部加罩，须核实实物安装。'})
        assembly.extend({'id':joint['id']+'_step_'+str(i),'action':joint['name']+'：'+text} for i,text in enumerate(built['instructions']))
        built['transform']=transform;built['joint']=joint
        built['fixed_anchor']=tm.transform_points([built['fixed_anchor']],transform)[0]
        built['moving_anchor']=tm.transform_points([built['moving_anchor']],transform)[0]
        built['id']=joint['id'];built['graph']=graph;parts.append(built)
        built['entry_sweeps']=chosen_entries
        built['broaden_support_search']=chosen.get('broaden_support_search',False)
        built['fastener_bores']=built['fastener_bores'].copy().apply_transform(transform)
        built['interface_voids']=[{**v,'mesh':v['mesh'].copy().apply_transform(transform)} for v in built['interface_voids']]
        built['fastening_sites']=placed_access(built,outer_regions)
    # Build shells first; split them before constructing mount support ribs.
    lid_planes=[]
    for owner in skins:
        pieces[owner]=skins[owner].copy();lids[owner]=tm.Trimesh()
    progress('access_semantics','识别身体主体与可开盖区域；安装位置和路径由程序确定')
    reuse=project.get('surface_plan_reuse') or {}
    if ((reuse.get('plan') or {}).get('plan_kind')=='assembly_parting_v1'
            and reuse.get('source_sha256')==project['source_shapes'][0]['sha256'] and reuse.get('graph')==graph
            and reuse.get('articulation_config')==project.get('articulation_config')):
        from .shell_access import validate_suggestions
        validate_suggestions({'regions':reuse['plan']['regions']},shape.bounds,graph)
        access_suggestions={**reuse['plan'],'source':'reused_surface_plan','live_model_called':False,'original_source':reuse['plan'].get('source'),'original_run_id':reuse['source_run_id']}
        atomic(directory/'access-suggestions.json',access_suggestions)
    else:access_suggestions=suggest(shape,graph,directory,live=project.get('articulation_config',{}).get('mode')=='live')
    analysis['access_suggestions']=access_suggestions
    analysis['joint_clearance']={'global_radius_erosion':False,'partition_seam_per_side_mm':pitch,'method':'sampled_opposing_geometry','hardware_fit_clearance_mm':design['fit_clearance'],'hardware_offset_max_mm':design['fit_clearance']*1.259}
    # Drill space for every module into every shell, including lids.
    progress('source_modification','在原外观分区上去除模块空间，并生成固定侧与运动侧承力连接')
    for identity in pieces:
        for cavity in cavities:
            pieces[identity]=cut(pieces[identity],cavity);lids[identity]=cut(lids[identity],cavity)
    # Plan actual shell parts BEFORE generating mount-to-shell ribs. Mount
    # poses are fixed; their envelopes protect the seam, but no arbitrary
    # pre-existing rib network is imposed on the parting-line search.
    progress('shell_parting','先切分装配壳体，再连接位置已确定的舵机安装座')
    early_panels={}
    partition_only=project.get('assembly_stage')=='shell_partition'
    manual_extraction=project.get('assembly_policy')=='manual_extraction_review'
    for built in parts:apply_shell_assembly_stages(built['fastening_sites'])
    planned_corridors={}
    for owner in bodies:
        progress('shell_parting_body','切分 '+bodies[owner]['name']+' 的装配壳体')
        hint=next(h for h in access_suggestions['regions'] if h['body_id']==owner)
        mounts=[p['mesh'] for p in all_modules if p['owner']==owner and p['role'] in ('fixed_mount','output_mount')]
        hardware=[p['mesh'] for p in all_modules if p['owner']==owner and p['role'] not in ('fixed_mount','output_mount')]
        # Moving the split upstream also moves the existing bounded machining
        # cleanup upstream. Otherwise an already-registered cavity remnant
        # would be mistaken for a disconnected half of the new shell.
        count=analysis['owner_source_component_counts'][owner]
        if count==1:
            pieces[owner],remnants,report=separate_cutting_remnants(pieces[owner],mounts,cavities)
            analysis.setdefault('pre_split_cutting_cleanup',[]).append({'owner':owner,**report})
            for i,mesh in enumerate(remnants):mesh.export(directory/(owner+'-pre-split-remnant-'+str(i)+'.stl'))
        pieces[owner],artifacts,report=separate_voxel_artifacts(pieces[owner],shape,mounts,pitch,count)
        analysis.setdefault('pre_split_voxel_cleanup',[]).append({'owner':owner,**report})
        for i,mesh in enumerate(artifacts):mesh.export(directory/(owner+'-pre-split-artifact-'+str(i)+'.stl'))
        paths=[tool['mesh'] for built in parts for site in built['fastening_sites']
               if owner in site['open_cover_body_ids'] and owner not in site['detach_body_ids'] for tool in site['tools']]
        paths.extend(cut(sweep,*mounts) for built in parts for role,sweep in built['entry_sweeps'].items()
                     if any(p['joint_id']==built['id'] and p['role']==role and p['owner']==owner for p in all_modules))
        paths.extend(b['fastener_bores'] for b in parts if b['joint']['parent']==owner and not b['fastener_bores'].is_empty)
        planned_corridors[owner]=paths
        required=access_requirement(owner,graph,all_modules,project)['required'] or bool(mounts) or not open_paths_clear(pieces[owner],paths)
        if not required:continue
        # A mount not yet joined to the shell is an intact obstacle, not a
        # reason to retain a long strip of shell behind its whole projection.
        # Protect existing contact material; keep the complete mount stationary
        # during cover extraction, then connect it to the retained shell.
        protected_contacts=[intersect(pieces[owner],mount) for mount in mounts]
        protected_contacts=[m for m in protected_contacts if not m.is_empty and m.volume>.01]
        try:
            pieces[owner],lids[owner],panel=split_shell(pieces[owner],outer_regions[owner],hint,
                hardware+mounts,protected_contacts,analysis['shell_wall_grid_mm'],[] if partition_only else paths,
                budget_s=45.,shell_parts_only=partition_only,manual_extraction_review=manual_extraction)
        except PanelSearchError as exc:
            atomic(directory/'shell-split-failure.json',{'owner_body_id':owner,
                'stage':'before_mount_supports','reason':str(exc),
                'permanent_tool_ports':False,'candidates':getattr(exc,'candidates',[])})
            raise
        panel['planned_before_supports']=True
        early_panels[owner]=panel
    analysis['shell_parting_stage']='before_mount_supports'
    if partition_only:
        from .parting_export import export
        return export(project,directory,Path(source_path),pieces,lids,early_panels,all_modules,analysis,started)
    # Structural bridges terminate in actual source-derived shell material.
    attachments=[];exterior_domains={owner:[] for owner in bodies};structural_cores=[]
    for built in parts:
        j=built['joint']
        for side,anchor,role in [('parent',built['fixed_anchor'],'fixed_mount'),('child',built['moving_anchor'],'output_mount')]:
            owner=j[side];shell=pieces[owner]
            if shell.is_empty:raise ValueError(bodies[owner]['name']+' 去除模块空间后没有可连接壳体')
            obstacles=[p['mesh'] for p in all_modules if p['joint_id']==j['id'] and p['role'] not in ('fixed_mount','output_mount')]
            # The cover is a distinct printable part, not a valid rib target.
            obstacles.extend(planned_corridors[owner])
            if not lids[owner].is_empty:obstacles.append(lids[owner])
            if not built['fastener_bores'].is_empty:obstacles.append(built['fastener_bores'])
            obstacles.extend(tool['mesh'] for site in built['fastening_sites'] if side=='parent' or not site['detach_child_subtree'] for tool in site['tools'])
            if side=='parent':
                obstacles.extend(s for peer in parts if peer['joint']['parent']==owner for s in peer['entry_sweeps'].values())
            mount=next(p for p in all_modules if p['joint_id']==j['id'] and p['role']==role)
            obstacles.extend(v['mesh'] for peer in parts for v in peer['interface_voids'])
            from .cover_fasteners import reserved_channels
            if owner in early_panels:obstacles.extend(reserved_channels(early_panels[owner]))
            found=attachment(shell,mount['mesh'],anchor,obstacles,shape,design,built['allow_exterior'],built.get('broaden_support_search',False),relocate_anchor=True)
            if found is not None:anchor=found['anchor']
            link=found['mesh'] if found else tm.Trimesh()
            target=found['target'] if found else anchor.copy();length=found['length_mm'] if found else 0.
            check(j['id']+'_'+side+'_short_bridge',found is not None,length_mm=length,
                  maximum_length_mm=design['max_bridge_span_mm'],allow_exterior=built['allow_exterior'])
            contact_shell=overlap(shell,link) if found else 0.;contact_mount=overlap(mount['mesh'],link) if found else 0.
            contact_min=design['open_mount_min_contact_mm3'] if built['allow_exterior'] else .05
            check(j['id']+'_'+side+'_connection',contact_shell>contact_min and contact_mount>contact_min,
                  shell_contact_mm3=contact_shell,module_contact_mm3=contact_mount,minimum_contact_mm3=contact_min)
            # Fixed receiver / output plate become integral with corresponding shell.
            pieces[owner]=union(shell,mount['mesh'],*([link] if not link.is_empty else []));mount['merged']=True
            if built['allow_exterior'] and not link.is_empty:
                # Permit local reconnection around this external attachment,
                # without opening the entire world as a routing domain.
                exterior_domains[owner].append(box(link.bounds[0]-design['beam_radius'],link.bounds[1]+design['beam_radius']))
                structural_cores.append({'id':j['id']+'_'+side,'owner':owner,'mesh':found['core']})
            attachments.append({'joint_id':j['id'],'owner_body_id':owner,'from_m':(anchor/1000).tolist(),'to_m':(target/1000).tolist(),'length_mm':length,'method':('bounded_bent_rib' if found and found['via'] is not None else 'short_direct_rib'),'via_m':(found['via']/1000).tolist() if found and found['via'] is not None else None})
    # Union of a bridge must not refill a cartridge space or hardware seating
    # face. Drill again after joining, just as in the earlier single-joint solver.
    separate=[p['mesh'] for p in all_modules if not p.get('merged')]
    separate.extend(b['fastener_bores'] for b in parts if not b['fastener_bores'].is_empty)
    separate.extend(v['mesh'] for b in parts for v in b['interface_voids'])
    for owner in pieces:
        for obstacle in separate:
            pieces[owner]=cut(pieces[owner],obstacle);lids[owner]=cut(lids[owner],obstacle)
    for owner in lids:
        for shell in pieces.values():lids[owner]=cut(lids[owner],shell)
    progress('interference_removal','按关节树逐姿态去除外壳干涉，并检查不得破坏电机安装面')
    samples=sample_angles(graph)
    sampled_poses=[poses(graph,angles) for angles in samples]
    obstacles={owner:union(pieces[owner],lids[owner],*[p['mesh'] for p in all_modules if p['owner']==owner and not p.get('merged')]) for owner in bodies}
    core_protection={owner:union(*[c['mesh'] for c in structural_cores if c['owner']==owner]) for owner in bodies}
    removal=[]
    for owner in bodies:
        before=float(pieces[owner].volume)+float(lids[owner].volume)
        for other,relative in unique_relative_obstacles(owner,sampled_poses):
            cutter=obstacles[other].copy().apply_transform(relative)
            if not core_protection[owner].is_empty:cutter=cut(cutter,core_protection[owner])
            if overlap(pieces[owner],cutter)>.1:pieces[owner]=cut(pieces[owner],cutter)
            if overlap(lids[owner],cutter)>.1:lids[owner]=cut(lids[owner],cutter)
        after=float(pieces[owner].volume)+float(lids[owner].volume)
        removal.append({'owner_body_id':owner,'removed_mm3':before-after,'fraction':(before-after)/before if before else 1})
    check('interference_removal_limit',all(r['fraction']<=.25 for r in removal),changes=removal,maximum_shell_fraction=.25)
    # Protect actual motor interfaces, then reconnect source fragments around
    # sampled moving obstacles. Do not throw away small disconnected material.
    for mount in [p for p in all_modules if p.get('merged')]:
        pieces[mount['owner']]=union(pieces[mount['owner']],mount['mesh'])
    path_checks=[]
    # Re-check newly added ribs against the same motions. Preserve interfaces
    # while trimming rib ends; grid clearance alone is never a collision pass.
    progress('rib_clearance','对新增承力肋再做实体运动去干涉；安装面保持保护')
    protected={owner:union(core_protection[owner],*[p['mesh'] for p in all_modules if p.get('merged') and p['owner']==owner]) for owner in bodies}
    obstacles={owner:union(pieces[owner],lids[owner],*[p['mesh'] for p in all_modules if p['owner']==owner and not p.get('merged')]) for owner in bodies}
    rib_trims=[]
    for owner in bodies:
        before=float(pieces[owner].volume)
        for other,relative in unique_relative_obstacles(owner,sampled_poses):
            cutter=obstacles[other].copy().apply_transform(relative)
            if overlap(cutter,protected[owner])>.01:cutter=cut(cutter,protected[owner])
            if not cutter.is_empty and overlap(pieces[owner],cutter)>.1:pieces[owner]=cut(pieces[owner],cutter)
        rib_trims.append({'owner_body_id':owner,'removed_mm3':before-float(pieces[owner].volume)})
    for owner in lids:
        for shell in pieces.values():lids[owner]=cut(lids[owner],shell)
    # Reconnected rib endpoints must not refill hardware or cartridge space.
    for owner in pieces:
        pieces[owner]=cut(pieces[owner],*separate)
        lids[owner]=cut(lids[owner],*separate)
    analysis['rib_clearance_changes']=rib_trims
    for mount in [p for p in all_modules if p.get('merged')]:
        retained=overlap(pieces[mount['owner']],mount['mesh'])
        check(mount['id']+'_retained',retained>=float(mount['mesh'].volume)-.1,
              original_mm3=float(mount['mesh'].volume),retained_mm3=retained,reason='去干涉不能破坏完整电机安装面或输出连接盘')
    # Template-defined installation sequence before attaching the moving body.
    # Make its actual corridor in the shell, preserving the screw-bearing mount.
    installation_entries={};installation_corridors={owner:[] for owner in bodies}
    for built in parts:
        if not built['fastener_bores'].is_empty:
            installation_corridors[built['joint']['parent']].append(built['fastener_bores'])
        for step in built['installation_steps']:
            hardware=next(p for p in all_modules if p['joint_id']==built['id'] and p['role']==step['role'])
            owner=hardware['owner'];direction=built['transform'][:3,:3]@np.array(step['direction'])
            # Include the exterior starting point, not just the source span.
            distance=max(0.,float((outer_regions[owner].vertices@direction).max()-(hardware['mesh'].vertices@direction).min()))+20
            from .solid import dilate
            sweep=built['entry_sweeps'].get(step['role'])
            if sweep is None:
                envelope=dilate(hardware['mesh'],design['fit_clearance'])
                sweep=linear_sweep(envelope,direction,distance)
            fixed=[p['mesh'] for p in all_modules if p.get('merged') and p['owner']==owner]
            cutter=cut(sweep,*fixed)
            installation_corridors[owner].append(cutter)
            # Assembly clearance is checked with the removable shell absent.
            # Never cut this sweep into the closed shell.
            removed=0.
            check(hardware['id']+'_entry_cut_limit',removed/max(before,1)<.1,removed_mm3=removed,maximum_shell_fraction=.1)
            installation_entries[hardware['id']]={'normal':direction.tolist(),'distance_mm':distance,
                'method':'open_side_then_retaining_plate' if built['allow_exterior'] else 'joint_axis_before_moving_body_attachment',
                'removed_roles':step['removed_roles'],'removed_shell_mm3':removed}
    analysis['installation_entries']=installation_entries
    progress('fastener_access','为安装耳、舵盘及输出连接螺丝预留螺钉和工具通道')
    fastening_sites=[site for built in parts for site in built['fastening_sites']]
    apply_shell_assembly_stages(fastening_sites)
    for owner in pieces:
        tools=[tool['mesh'] for site in fastening_sites if owner in site['open_cover_body_ids'] and owner not in site['detach_body_ids'] for tool in site['tools']]
        protected=[p['mesh'] for p in all_modules if p.get('merged') and p['owner']==owner]
        protected.extend(core['mesh'] for core in structural_cores if core['owner']==owner)
        installation_corridors[owner].extend(tools)
        analysis.setdefault('fastener_openings',[]).append({'owner_body_id':owner,'removed_mm3':0.,
            'method':'removable_shell_access_only','tool_count':len(tools)})
    # Reuse the exact constraints that selected the split and support paths.
    installation_corridors=planned_corridors
    atomic(directory/'assembly-prechecks.json',{'checks':checks,'placement_candidates':placement_candidates,'mounts':attachments,'installation_entries':installation_entries})
    progress('local_access_panels','复核切分后的安装结构与壳盖；装配通道不作为永久孔洞')
    lid_fasteners=[];access_decisions=[];split_covers={}
    for item in all_modules:item['mesh'].export(directory/('diagnostic-module-'+item['id']+'.stl'))
    for owner in bodies:
        progress('access_body','验证 '+bodies[owner]['name']+' 已切分壳体及安装连接')
        hint=next(h for h in access_suggestions['regions'] if h['body_id']==owner)
        hardware=[p['mesh'] for p in all_modules if p['owner']==owner and not p.get('merged')]
        fixed=[p['mesh'] for p in all_modules if p['owner']==owner and p.get('merged')]
        others=[p['mesh'] for p in all_modules if not p.get('merged')]
        others.extend(pieces[b] for b in bodies if b!=owner)
        if not lids[owner].is_empty:others.append(lids[owner])
        # The cover has already been separated. Repair only the retained half,
        # preserving its open-state assembly corridors as routing obstacles.
        others.extend(installation_corridors[owner])
        pieces[owner]=cut(union(pieces[owner],*fixed),*separate)
        before_reconnect_hits=[overlap(pieces[owner],path) for path in installation_corridors[owner]]
        # A reconnecting rib must avoid complete opposing hardware throughout
        # the same sampled motion, not only stationary shells at the rest pose.
        opposing={b:union(pieces[b],lids[b],*[p['mesh'] for p in all_modules if p['owner']==b and not p.get('merged')]) for b in bodies if b!=owner}
        motion_cutters=[opposing[other].copy().apply_transform(relative) for other,relative in unique_relative_obstacles(owner,sampled_poses)]
        others.extend(motion_cutters)
        owner_source_count=analysis['owner_source_component_counts'][owner]
        np.savez_compressed(directory/(owner+'-before-access-cleanup.npz'),vertices=pieces[owner].vertices,faces=pieces[owner].faces)
        if owner_source_count==1:
            pieces[owner],remnants,cleanup=separate_cutting_remnants(pieces[owner],fixed,motion_cutters+cavities+separate)
        else:
            remnants=[];cleanup={'removed_mm3':0.,'fraction':0.,'reason':'多源实体保留全部组件，不自动清理残屑'}
        analysis.setdefault('cutting_remnants',[]).append({'owner':owner,**cleanup})
        for index,remnant in enumerate(remnants):remnant.export(directory/(owner+'-cutting-remnant-'+str(index)+'.stl'))
        pieces[owner],voxel_artifacts,artifact_report=separate_voxel_artifacts(pieces[owner],shape,fixed,pitch,owner_source_count)
        analysis.setdefault('voxel_artifacts',[]).append({'owner_body_id':owner,**artifact_report})
        for index,artifact in enumerate(voxel_artifacts):artifact.export(directory/(owner+'-voxel-artifact-'+str(index)+'.stl'))
        connection_domain=union(outer_regions[owner],*exterior_domains[owner]) if exterior_domains[owner] else outer_regions[owner]
        pieces[owner],reconnection=reconnect_local(pieces[owner],connection_domain,others,design['beam_radius'],budget_s=design.get('local_reconnect_budget_s',10.))
        if not reconnection['passed']:
            # New tool ports may disconnect a surface detail. Reuse the generic
            # material-domain A* solver instead of removing it or special-casing
            # an input model. Exact clipping/checks still decide acceptance.
            progress('access_reconnection','为 '+bodies[owner]['name']+' 搜索避开工具与运动障碍的体积内连接路径')
            original=pieces[owner]
            grid_attempts=[]
            for radius in dict.fromkeys([design['beam_radius'],design['open_mount_min_core_radius_mm']]):
                repaired,grid_report=reconnect(original,labelgrid==list(bodies).index(owner),voxels,others,radius)
                grid_report['candidate_radius_mm']=radius
                if grid_report['passed']:
                    added=intersect(cut(repaired,original),connection_domain)
                    local=[o for o in others if not o.is_empty and np.all(o.bounds[1]>added.bounds[0]) and np.all(added.bounds[1]>o.bounds[0])] if not added.is_empty else []
                    added=cut(added,*local)
                    candidate=union(original,added)
                    obstruction=sum(overlap(added,o) for o in others)
                    accepted=regions(candidate)==1 and obstruction<=.05
                    grid_report.update(passed=accepted,added_obstacle_overlap_mm3=obstruction,
                        connected_regions=regions(candidate),domain_clipped=True,obstacle_clipped=True,original_material_preserved=True)
                    if accepted:pieces[owner]=candidate
                grid_attempts.append(grid_report)
                if grid_report['passed']:break
            reconnection={'passed':grid_report['passed'],'method':'local_then_bounded_material_astar',
                          'local_search':reconnection,'grid_search':grid_report,'grid_attempts':grid_attempts,
                          'reason':grid_report.get('reason','连接路径须保持全部材料并避开工具及运动障碍')}
        path_checks.append({'owner_body_id':owner,**reconnection})
        check(owner+'_final_connections',reconnection['passed'],**{k:v for k,v in reconnection.items() if k!='passed'})
        pieces[owner].export(directory/(owner+'-before-access.stl'))
        np.savez_compressed(directory/(owner+'-before-access.npz'),vertices=pieces[owner].vertices,faces=pieces[owner].faces)
        requirement=access_requirement(owner,graph,all_modules,project)
        corridor_hits=[overlap(pieces[owner],path) for path in installation_corridors[owner]]
        atomic(directory/(owner+'-assembly-path-check.json'),{'before_reconnection_mm3':before_reconnect_hits,'after_reconnection_mm3':corridor_hits})
        blocked_paths=any(hit>.1 for hit in corridor_hits)
        requirement['required']=bool(requirement['required'] or blocked_paths or owner in early_panels)
        if blocked_paths:requirement['reason']='完整壳体阻挡装配操作，必须生成可拆壳件，禁止永久工具孔'
        decision={'owner_body_id':owner,**requirement}
        # Output-only bodies also need a split whenever their actual screw
        # or tool paths are blocked. A clear exterior path needs no new seam.
        if not requirement['required']:
            decision.update(method=requirement.get('method','intact_output_only_leaf'),panel_created=False)
            access_decisions.append(decision)
            check(owner+'_intact_leaf_shell',regions(pieces[owner])==1,reason=requirement['reason'])
            assembly.append({'id':owner+'_output_attachment','action':bodies[owner]['name']+'：通过已验证装配路径安装外露模块或输出连接件，不另切检修盖；内部新增电子元件时需重新规划入口。'})
            continue
        if project.get('service_opening_requirements'):
            analysis['electronics_service_access_verified']=False
        try:
            detached_for_closure=set(descendants(owner,graph))-{owner}
            if owner not in early_panels:
                raise PanelSearchError('后续安装新增了开壳需求，需要返回上游切分阶段')
            if not open_paths_clear(pieces[owner],installation_corridors[owner]):
                raise PanelSearchError('切分后新增安装结构阻挡装配路径，需要重选连接位置')
            panel=early_panels[owner]
            inv=np.linalg.inv(np.asarray(panel['transform_mm']))
            cap=lids[owner].copy().apply_transform(inv)
            blockers=[pieces[owner],*hardware,*[pieces[b] for b in bodies if b!=owner and b not in detached_for_closure],
                *[lids[b] for b in lids if b!=owner and b not in detached_for_closure],
                *[p['mesh'] for p in all_modules if p['owner']!=owner and p['owner'] not in detached_for_closure and not p.get('merged')]]
            local_blockers=[m.copy().apply_transform(inv) for m in blockers if not m.is_empty]
            if not manual_extraction and translation_pose_blocked(cap,local_blockers,panel['extraction_distance_mm']):
                raise PanelSearchError('安装结构生成后壳体拆卸路径受阻')
            sweep=linear_sweep(cap,[0,0,1],panel['extraction_distance_mm'])
            clear=not any(overlap(sweep,m)>.1 for m in local_blockers)
            panel['continuous_extraction']=clear
            panel['manual_extraction_review_required']=not clear
            if not clear and not manual_extraction:
                raise PanelSearchError('安装结构生成后壳体连续拆卸路径受阻')
            panel['extraction_rechecked_after_supports']=True
            covers=[(lids[owner],panel)]
        except PanelSearchError as exc:
            atomic(directory/'shell-split-failure.json',{'owner_body_id':owner,'reason':str(exc),
                'permanent_tool_ports':False,'candidates':getattr(exc,'candidates',[])})
            if requirement['required']:raise PanelSearchError(bodies[owner]['name']+'：'+str(exc)) from exc
            decision.update(method='intact_output_only_leaf',panel_created=False,search_failure=str(exc))
            access_decisions.append(decision)
            check(owner+'_intact_leaf_shell',regions(pieces[owner])==1,reason=requirement['reason'])
            assembly.append({'id':owner+'_output_attachment','action':bodies[owner]['name']+'：保持壳体完整，通过关节输出连接件安装；不切独立检修盖。输出连接螺丝与工具路径尚待核实；以后在此放电子元件需重新规划入口。'})
            continue
        split_covers[owner]=covers
        lids[owner]=union(*[m for m,_ in covers])
        panel=covers[0][1]
        decision.update(method='protected_multi_cover_split' if len(covers)>1 else panel['method'],panel_created=True,cover_count=len(covers));access_decisions.append(decision)
        assembly.append({'id':owner+'_lid','action':bodies[owner]['name']+'：可拆壳体保持打开，预装盖板螺母，完成此壳体内部安装后闭合并紧固，再连接下一级身体部件；实际配合待试装。'})
        for index,(_,panel) in enumerate(covers):
            panel['owner_body_id']=owner
            panel['detached_body_ids']=sorted(detached_for_closure)
            panel['assembly_stage']='before_child_body_joining'
            panel['cover_asset_id']=owner+('_lid' if index==0 else '_cover'+str(index+1)+'_lid')
            lid_planes.append(panel)
            check(panel['cover_asset_id']+'_closed_shell_no_assembly_ports',panel['closed_unregistered_missing_mm3']<=.1,
                  unregistered_missing_mm3=panel['closed_unregistered_missing_mm3'],permanent_assembly_ports=False)
            check(panel['cover_asset_id']+'_continuous_extraction',panel['continuous_extraction'],distance_mm=panel['extraction_distance_mm'],blocking=not manual_extraction)
            if not panel['continuous_extraction']:
                warnings.append({'code':'manual_extraction_review','owner_body_id':owner,'reason':'直线拆卸路径受阻；人工转动/倾斜路径未验证，需审阅装配。'})
            records=panel['fasteners']
            check(panel['cover_asset_id']+'_fasteners',len(records)>=3,fasteners=records)
            lid_fasteners.append({'owner_body_id':owner,'fasteners':records})
    check('structural_paths',all(r['passed'] for r in path_checks),bodies=path_checks)
    analysis['lid_planes']=lid_planes;analysis['lid_fasteners']=lid_fasteners;analysis['access_decisions']=access_decisions
    analysis['assembly_access_policy']={'permanent_tool_ports':False,'method':'split_before_mount_supports',
        'vlm_role':'semantic_surface_preference_only','geometry_decides_feasibility':True}
    # Independently verify final cover screw bearing and tool paths, including
    # neighboring hardware. The finish routine uses the recorded real axes.
    from .cover_fasteners import finish as finish_covers
    cover_parts=[{'record':{'id':owner+'_shell','owner_body_id':owner},'mesh':mesh} for owner,mesh in pieces.items()]
    cover_parts.extend({'record':{'id':panel['cover_asset_id'],'owner_body_id':panel['owner_body_id']},'mesh':mesh} for covers in split_covers.values() for mesh,panel in covers)
    cover_parts.extend({'record':{'id':p['id'],'owner_body_id':p['owner']},'mesh':p['mesh']} for p in all_modules if not p.get('merged'))
    from .fastener_search import packed
    analysis['support_core_records']=[{'id':c['id'],'owner_body_id':c['owner'],'mesh':packed(c['mesh'])} for c in structural_cores]
    cover_reports=finish_covers(cover_parts,analysis,overlap,domains=outer_regions)
    for part in cover_parts:
        for owner in bodies:
            if part['record']['id']==owner+'_shell':pieces[owner]=part['mesh']
            for index,(mesh,panel) in enumerate(split_covers.get(owner,[])):
                if part['record']['id']==panel['cover_asset_id']:split_covers[owner][index]=(part['mesh'],panel)
    for owner,covers in split_covers.items():lids[owner]=union(*[m for m,_ in covers])
    analysis['cover_fastener_access']=cover_reports
    for report in cover_reports:check(report['id']+'_closed_access',report['passed'],**{k:v for k,v in report.items() if k not in ('id','passed')})
    for built in parts:
        for index,void in enumerate(built['interface_voids']):
            owner=built['joint'][void['side']]
            obstruction=overlap(union(pieces[owner],lids[owner]),void['mesh'])
            check(built['id']+'_interface_void_'+str(index),obstruction<.1,
                  owner_body_id=owner,obstruction_mm3=obstruction,
                  scope='模板内有界螺孔与槽孔在闭壳后保持畅通；不是贯穿外壳的工具通道')
        if not built['fastener_bores'].is_empty:
            obstruction=overlap(pieces[built['joint']['parent']],built['fastener_bores'])
            check(built['id']+'_ear_screw_bores',obstruction<.1,obstruction_mm3=obstruction,
                  scope='安装耳贯穿螺孔不得被壳体或连接筋填回；螺丝长度和工具空间仍待实物核实')
    for core in structural_cores:
        retained=overlap(pieces[core['owner']],core['mesh']);original=float(core['mesh'].volume)
        check(core['id']+'_support_core_retained',retained>=original-.1,original_mm3=original,retained_mm3=retained,
              minimum_core_radius_mm=design['open_mount_min_core_radius_mm'],scope='连续承力芯不被后续开口或运动裁剪切断；不是承载强度验证')
    mating_reports=[]
    for built in parts:
        reports=mounting_surface_checks(built,pieces)
        mating_reports.extend(reports)
        for report in reports:
            check(report['id'],report['passed'],**{k:v for k,v in report.items() if k not in ('id','passed')})
    analysis['mounting_surfaces']=mating_reports
    progress('motor_installation','按电机外形与固定模板检查最终开口和直线装入路径')
    installation=[]
    for hardware in [p for p in all_modules if p['id'] in installation_entries]:
        owner=hardware['owner']
        entry=installation_entries[hardware['id']]
        normal=np.array(entry['normal']);span=entry['distance_mm']
        # The retainer is installed after the motor. Its path must clear the
        # motor; conversely the motor enters with the retainer removed.
        built=next(b for b in parts if b['id']==hardware['joint_id'])
        detached=set(descendants(built['joint']['child'],graph))
        present=[p['mesh'] for p in all_modules if p['owner'] not in detached and not p.get('merged') and p['id']!=hardware['id']
                 and not (p['joint_id']==hardware['joint_id'] and p['role'] in entry['removed_roles'])]
        blockers=union(*[mesh for body,mesh in pieces.items() if body not in detached],
            *[mesh for body,mesh in lids.items() if body!=owner and body not in detached],*present)
        path=[]
        for distance in np.linspace(span,0,13):
            moved=hardware['mesh'].copy().apply_translation(normal*distance)
            path.append({'outward_distance_mm':float(distance),'shell_overlap_mm3':overlap(moved,blockers)})
        built=next(b for b in parts if b['id']==hardware['joint_id'])
        local_mesh=next(p[3] for p in built['parts'] if p[0]==hardware['role'])
        local_direction=next(step['direction'] for step in built['installation_steps'] if step['role']==hardware['role'])
        swept=placed_sweep(local_mesh,built['transform'],local_direction,span)
        swept_overlap=overlap(swept,blockers)
        passed=all(p['shell_overlap_mm3']<=.1 for p in path) and swept_overlap<=.1
        check(hardware['id']+'_insertion',passed,samples=path,linear_sweep_overlap_mm3=swept_overlap,continuous_guarantee=True,
              scope='按模板装配顺序检查直线扫掠与最终壳体、同侧已装硬件；对侧运动部位及指定可拆件尚未安装，工具通道待核实')
        installation.append({'motor_asset_id':hardware['id'],'owner_body_id':owner,'role':hardware['role'],
            'method':entry['method'],'removed_roles':entry['removed_roles'],'normal':normal.tolist(),'passed':passed,'samples':path,'detached_body_ids':sorted(detached),'includes_ancestor_bodies':True})
    analysis['motor_installation']=installation
    # Parent shells close before a child is joined. Child/output fastening
    # must therefore work with the ancestor cover physically present.
    apply_shell_assembly_stages(fastening_sites)
    fastening_reports=check_access(fastening_sites,pieces,lids,all_modules)
    analysis['fastener_access']=fastening_reports
    preloads=[]
    for report in fastening_reports:
        for tool in report['tools']:
            if tool.get('assembly_stage')!='preload_nuts_before_body_joining':continue
            preloads.append({'id':report['id']+'_preload_nut','action':'在装入电机、连接其他刚体之前，从登记通道将候选防转螺母预装入固定座。',
                'owner_body_id':report['owner_body_id'],'fastener_access_id':report['id'],
                'detach_body_ids':tool['detach_body_ids'],'removed_module_roles':tool['removed_module_roles'],'passed':tool['passed']})
    for step in assembly:
        if '_step_' in step['id']:
            joint_id=step['id'].rsplit('_step_',1)[0]
            step['depends_on']=[p['id'] for p in preloads if p['fastener_access_id'].startswith(joint_id+'_')]
    assembly=preloads+assembly
    for owner,covers in split_covers.items():
        preload_id=owner+'_cover_nuts'
        assembly.insert(0,{'id':preload_id,'owner_body_id':owner,
            'action':bodies[owner]['name']+'：壳体打开且内部硬件尚未安装时，沿已验证入口预装盖板螺母。',
            'assembly_stage':'before_internal_hardware'})
        for built in parts:
            if owner in (built['joint']['parent'],built['joint']['child']):
                first=next(step for step in assembly if step['id']==built['id']+'_step_0')
                first.setdefault('depends_on',[]).append(preload_id)
    for report in fastening_reports:
        check(report['id']+'_fastener_access',report['passed'],**{k:v for k,v in report.items() if k!='passed'})
        assembly.append({'id':report['id']+'_fasten','action':report['purpose']+'：打开登记的可拆盖，沿工具通道装入并拧紧；安装顺序和拆离部位见紧固报告。',
                         'joint_id':report['joint_id'],'owner_body_id':report['owner_body_id'],
                         'open_cover_body_ids':report['open_cover_body_ids'],'detach_body_ids':report['detach_body_ids'],
                         'fastener_access_id':report['id'],'passed':report['passed'],
                         'assembly_stage':report.get('assembly_stage'),'deferred_joint_ids':report.get('deferred_joint_ids',[])})
    # Close each body after its own fastening, before joining its children.
    # The checked output paths include already-closed ancestor covers.
    closing=[step for step in assembly if step['id'].endswith('_lid')]
    assembly=[step for step in assembly if step not in closing]
    for step in closing:
        owner=step['id'][:-4]
        step['owner_body_id']=owner;step['assembly_stage']='before_child_body_joining'
        step['depends_on']=[r['id']+'_fasten' for r in fastening_reports if r['owner_body_id']==owner]
        for built in parts:
            if built['joint']['parent']==owner:
                join_id=built['id']+'_step_'+str(len(built['instructions'])-1)
                joining=next(s for s in assembly if s['id']==join_id)
                joining.setdefault('depends_on',[]).append(step['id'])
    assembly.extend(closing)
    assembly=order_assembly(assembly,fastening_reports,graph)
    from .fastener_search import sequence as sequence_covers
    cover_plan=sequence_covers(cover_reports,analysis['lid_planes'],assembly)
    analysis['cover_assembly_plan']=cover_plan
    if cover_plan is not None:
        assembly=cover_plan['steps']
        for report in cover_reports:
            existing=next(c for c in checks if c['name']==report['id']+'_closed_access')
            existing.update({k:v for k,v in report.items() if k!='id'})
    check('assembly_dependency_order',True,operation_count=len(assembly),
          scope='上级关节紧固完成后才安装下级电机；后装电机的装入路径包含已连接的上级刚体')
    final=[]
    for k,b in enumerate(graph['bodies']):
        final.append({'id':b['id']+'_shell','name':b['name']+' · 空心壳与连接座','owner':b['id'],'mesh':pieces[b['id']],'printable':True,'template':'voxel_shell','template_revision':library['revision']})
        for index,(mesh,panel) in enumerate(split_covers.get(b['id'],[])):
            final.append({'id':panel['cover_asset_id'],'name':b['name']+' · 可拆壳体 '+str(index+1),'owner':b['id'],'mesh':mesh,'printable':True,'template':'voxel_shell','template_revision':library['revision']})
    final.extend(p for p in all_modules if not p.get('merged'))
    assets=[];preview=[];scene=tm.Scene();export_repairs=[]
    for index,p in enumerate(final):
        m=p['mesh']
        if not m.is_empty:
            # STL loses vertex indices. Weld its exact float32 coordinates and
            # remove zero-height Boolean slivers before validating the deliverable.
            m,repair=stl_deliverable(m)
            check('export_deviation_'+p['id'],repair['passed'],attempts=repair['attempts'])
            export_repairs.append({'asset_id':p['id'],**repair})
            if repair['passed'] and repair.get('method')=='local_quantization_separation':
                warnings.append({'code':'stl_local_precision_repair','asset_id':p['id'],'reason':p['name']+'：已自动修复 STL 存储造成的局部顶点重合；调整量及体积偏差有记录，实物仍待核实。'})
            if repair['passed']:p['mesh']=m
        valid=not m.is_empty and m.is_volume
        connected=regions(m) if valid else 0
        check('solid_'+p['id'],valid and (not p['printable'] or connected==1),connected_regions=connected)
        if m.is_empty:continue
        file=p['id']+'.stl';m.export(directory/file,file_type='stl_ascii' if repair.get('stl_encoding')=='ascii' else 'stl')
        reopened=tm.load(directory/file,force='mesh')
        check('stl_roundtrip_'+p['id'],reopened.is_volume and reopened.is_watertight,
              watertight=bool(reopened.is_watertight),volume_mm3=float(reopened.volume))
        asset={'id':p['id'],'name':p['name'],'owner_body_id':p['owner'],'file':file,'sha256':hashlib.sha256((directory/file).read_bytes()).hexdigest(),
            'template_id':p['template'],'template_revision':p['template_revision'],'export_length_unit':'mm','transform_m':np.eye(4).tolist(),
            'bounds_m':(m.bounds/1000).tolist(),'volume_m3':float(m.volume)*1e-9,'printable_candidate':p['printable'],'material':'unverified'}
        assets.append(asset);color=PALETTE[index%len(PALETTE)]
        display_mesh=m.simplify_quadric_decimation(face_count=3500) if len(m.faces)>3500 else m
        preview.append({'id':p['id'],'name':p['name'],'owner':p['owner'],'vertices':(display_mesh.vertices/1000).tolist(),'faces':display_mesh.faces.tolist(),'color':color,'hardware':not p['printable']})
        panel=next((v for v in lid_planes if p['id']==v['cover_asset_id']),None)
        if panel:preview[-1]['opening_offset_m']=(np.asarray(panel['normal'])*panel['extraction_distance_mm']/1000).tolist()
        display=m.copy().apply_scale(.001);display.visual.face_colors=tm.visual.color.hex_to_rgba(color);scene.add_geometry(display,node_name=p['id'])
    analysis['export_repairs']=export_repairs
    # Show the exact integral mount surfaces through a translucent skin. These
    # duplicate only a subset of the printed shell for inspection, never a new part.
    for mount in [p for p in all_modules if p.get('merged')]:
        mesh=intersect(mount['mesh'],pieces[mount['owner']])
        if mesh.is_empty:continue
        preview.append({'id':'mount_interface_'+mount['id'],'name':mount['name']+' · integral shell interface',
            'owner':mount['owner'],'vertices':(mesh.vertices/1000).tolist(),'faces':mesh.faces.tolist(),
            'color':'#d5a14d','hardware':False,'role':'mount_interface',
            'scope':'integral portion of shell, diagnostic duplicate; not a separate printed part'})
    # Diagnostic meshes are preview-only; they are not printable parts or hardware.
    for site in fastening_reports:
        for i,tool in enumerate(site['tools']):
            mesh=cylinder_path(tool['point_mm'],tool['direction'],min(tool['length_mm'],14.),tool['radius_mm'],tool.get('sections',24))
            preview.append({'id':'fastener_tool_'+site['id']+'_'+str(i),'name':site['purpose']+' · tool direction',
                'owner':site['owner_body_id'],'vertices':(mesh.vertices/1000).tolist(),'faces':mesh.faces.tolist(),
                'color':'#21bfd1' if tool['passed'] else '#e04d46','hardware':False,
                'role':'fastener_tool_path','fastener_access_id':site['id'],
                'scope':'first 14 mm of checked path; diagnostic only'})
    progress('checks','检查装配交叠与各关节离散扫掠，保留失败候选')
    collisions=[]
    for i,a in enumerate(final):
        if a['mesh'].is_empty:continue
        for b in final[i+1:]:
            if b['mesh'].is_empty:continue
            volume=overlap(a['mesh'],b['mesh'])
            if volume>.1:collisions.append({'a':a['id'],'b':b['id'],'volume_mm3':volume})
    check('assembly_collisions',not collisions,collisions=collisions)
    sweeps=[]
    # At least independent limit/midpoint poses for every joint; combined poses too.
    samples=sample_angles(graph)
    groups={b:union(*[p['mesh'] for p in final if p['owner']==b and not p['mesh'].is_empty]) for b in bodies}
    for angles in samples:
        matrices=poses(graph,angles);pairs=[]
        moved={b:m.copy().apply_transform(matrices[b]) for b,m in groups.items()}
        for i,a in enumerate(bodies):
            for b in list(bodies)[i+1:]:
                volume=overlap(moved[a],moved[b])
                if volume>.1:pairs.append({'a':a,'b':b,'volume_mm3':volume})
        sweeps.append({'angles_rad':angles,'collisions':pairs})
    check('joint_sweep',not any(p['collisions'] for p in sweeps),samples=sweeps,continuous_guarantee=False)
    # These are real missing software checks, separate from physical unknowns.
    pending=['盖板螺钉实际规格、螺母配合及工具操作仍需实物试装','电子元件布局、线束及承载尚未计算','体素边界偏差须在制造前核查']
    for built in parts:pending.extend(built['template']['unknown'])
    joints=[{'id':j['id'],'name':j['name'],'parent_body_id':j['parent'],'child_body_id':j['child'],'origin_m':(np.array(j['position_mm'])/1000).tolist(),
        'axis':j['axis'],'limits_rad':np.deg2rad(j['limits_deg']).tolist(),'motor_instance_id':binding[j['id']]['motor_instance_id']} for j in graph['joints']]
    hardware_requests=[]
    for owner in lid_fasteners:
        hardware_requests.append({'catalog_id':'fastener_candidate','specification':owner['fasteners'][0].get('candidate_thread','M2')+' 螺钉与螺母套件',
            'quantity':len(owner['fasteners']),'purpose':bodies[owner['owner_body_id']]['name']+' 上盖固定',
            'owner_body_id':owner['owner_body_id'],'length_m':None,
            'required_underhead_lengths_m':[f['required_underhead_length_mm']/1000 for f in owner['fasteners']],
            'physical_fit_verified':False})
    for built in parts:
        joint=built['joint'];template=built['template']
        if template['branch']=='text2robot_rectangular':
            hardware_requests.extend([
                {'catalog_id':'fastener_candidate','specification':'候选 M1.6 螺丝、垫片、螺母（实物待核实）','quantity':2,'purpose':joint['name']+' 安装耳、压板与开放托架贯穿固定','joint_id':joint['id'],'length_m':None,'physical_fit_verified':False},
                {'catalog_id':None,'specification':'对应电机原装舵盘及中心螺钉','quantity':1,'purpose':joint['name']+' 轴输出连接',
                 'joint_id':joint['id'],'physical_fit_verified':False},
                {'catalog_id':'fastener_candidate','specification':'舵盘槽孔配套螺钉','quantity':2,'purpose':joint['name']+' 输出连接片',
                 'joint_id':joint['id'],'length_m':None,'physical_fit_verified':False}])
        else:
            for side in ['fixed','output']:
                hardware_requests.append({'catalog_id':'fastener_candidate','specification':'M3 螺钉','quantity':template[side+'_holes'],
                    'purpose':joint['name']+(' 固定盘' if side=='fixed' else ' 输出盘'),'joint_id':joint['id'],'length_m':None,
                    'drawing_thread_depth_m':template[side+'_thread_depth']/1000,'plate_thickness_m':template['plate_thickness']/1000,
                    'physical_fit_verified':False})
    result={'project_id':project['id'],'design_revision':project['revision'],'status':'candidate_manual_review' if any(w['code']=='manual_extraction_review' for w in warnings) else 'candidate_checked','geometry_valid':all(c['passed'] for c in checks if c.get('blocking',True)),
        'assembly_policy':project.get('assembly_policy','strict'),'assembly_access_verified':not any(w['code']=='manual_extraction_review' for w in warnings),
        'manufacturing_ready':False,'pipeline':'graph_assembly_0.13.0-split-before-supports','source_sha256':analysis['source_sha256'],'analysis':analysis,
        'annotation_run_id':project['articulation']['run_id'],'placement_candidates':placement_candidates,'plan':{'reason':'从源模型与真实硬件重算；安装面和承力连接保持完整，打开可拆壳体进行装入与紧固，闭合当前壳体后连接下级部件，不生成永久工具孔；失败候选明确拒绝'},
        'rigid_bodies':[{'id':b['id'],'name':b['name'],'transform_m':np.eye(4).tolist(),'asset_ids':[a['id'] for a in assets if a['owner_body_id']==b['id']]} for b in graph['bodies']],
        'joints':joints,'geometry_assets':assets,'mounts':attachments,'assembly_dependencies':assembly,
        'geometry_check_scope':'实体与 STL、安装面和承力连通、闭合材料恢复、声明装配状态下的工具通道和连续拆壳路径、静态碰撞及离散关节运动；不含强度、电子、线束或实物验证',
        'checks':checks,'warnings':warnings,'pending':list(dict.fromkeys([w['reason'] for w in warnings]+pending)),'hardware_requests':hardware_requests,'elapsed_s':round(time.monotonic()-started,3),
        'template_sha256':hashlib.sha256(template_raw).hexdigest()}
    result['generator_sources_sha256']=generator_sources
    # Closed original body domains are spatial constraints, never manufacturing parts.
    result['shell_assembly_interfaces']=[{'owner_body_id':p['owner_body_id'],
        'structural_part_id':p['owner_body_id']+'_shell','removable_part_id':p['cover_asset_id'],
        'opening_direction':p['normal'],'opening_distance_mm':p['extraction_distance_mm'],
        'assembly_state':'cover_removed_for_internal_installation','split_method':p['method'],
        'electronics_fit_verified':False,'requires_revalidation_after_layout':True} for p in lid_planes]
    result['layout_domains']=[]
    for owner,region in outer_regions.items():
        file='layout-domain-'+owner+'.stl';region.export(directory/file)
        result['layout_domains'].append({'owner_body_id':owner,'file':file,
            'sha256':hashlib.sha256((directory/file).read_bytes()).hexdigest(),'export_length_unit':'mm',
            'purpose':'original_body_volume_for_internal_layout_only'})
    atomic(directory/'fastener-access.json',{'interfaces':fastening_reports,'physical_fit_verified':False})
    atomic(directory/'manifest.json',result);atomic(directory/'joint-templates.json',library);atomic(directory/'input-project.json',project)
    atomic(directory/'preview.json',{'parts':preview,'graph':graph,'joint':{'limits_rad':[-1,1]}})
    scene.export(directory/'assembly.glb');(directory/'source.raw').write_bytes(Path(source_path).read_bytes())
    for built in parts:
        reference=built['template'].get('reference_cad')
        if reference:
            folder=(TEMPLATE_PATH.parent/reference['file']).parent
            for name in ['LICENSE.txt','README.md']:
                (directory/('sg90-reference-'+name)).write_bytes((folder/name).read_bytes())
    result['artifacts']=sorted(p.name for p in directory.iterdir() if p.is_file() and p.suffix!='.zip' and p.name not in {'run.json','input.json','worker.log','result.json'})
    with zipfile.ZipFile(directory/'mechanical-review.zip','w',zipfile.ZIP_DEFLATED) as package:
        for filename in result['artifacts']:package.write(directory/filename,filename)
    result['artifacts'].append('mechanical-review.zip');atomic(directory/'result.json',result)
    return result
