"""Template-driven screw, head and tool access; no model-specific coordinates.

All tool envelopes are explicit engineering candidates until hardware is measured.
Checks concern a stated assembly stage, never a fictitious transparent shell.
"""
import math
import numpy as np
import trimesh as tm
from .solid import overlap, union, linear_sweep
from .horn_interface import registration as horn_registration


def interfaces(template, design):
    t=template;sites=[]
    def add(identity,side,purpose,tools,removed_child=False):
        sites.append({'id':identity,'side':side,'purpose':purpose,'tools':tools,
                      'detach_child_subtree':removed_child,'physical_fit_verified':False,
                      'evidence':'template_geometry_and_candidate_tool_envelopes'})
    def tool(point,direction,radius,kind):
        return {'point_mm':point,'direction':direction,'radius_mm':radius,'kind':kind}
    if t['branch']=='text2robot_rectangular':
        ref=t['reference_cad'];gap=t.get('ear_contact_clearance_mm',.05);wall=design['wall']
        for i,(x,y) in enumerate(ref['ear_hole_centres_mm']):
            nut=tool([x,y,ref['ear_seat_z_mm']-gap-wall+.75],[0,1,0],2.,'captive_nut_insertion')
            nut.update(shape='hex_nut',thickness_mm=1.3)
            nut.update(sections=6,assembly_stage='preload_nuts_before_body_joining',removal_scope='owner_body',
                       removed_module_roles=['motor','retainer','rotor','output_mount'])
            add('ear_'+str(i+1),'parent','Clamp motor mounting ear',[
                tool([x,y,ref['ear_top_z_mm']+gap+wall+.05],[0,0,1],2.1,'screw_head_and_driver'),
                nut],True)
        add('horn_center','child','Stock horn centre screw',[
            tool([0,0,t['horn_thickness']+.05],[0,0,1],1.5,'centre_screw_and_driver')])
        # Shared registration drives both actual slots and tool envelopes.
        for i,site in enumerate(horn_registration(t)):
            x,y,_=site['point_mm']
            add('horn_adaptor_'+str(i+1),'child','Printed output adaptor to stock horn',[
                tool([x,y,t['horn_thickness']+wall+.05],[0,0,1],site['head_radius_mm'],'screw_head_and_driver')])
    else:
        for side,normal,z in [('fixed',-1,-t['body_depth']-t['output_depth']-t['plate_thickness']-.05),
                              ('output',1,t['plate_thickness']+.05)]:
            for i in range(t[side+'_holes']):
                angle=math.pi/2+i*2*math.pi/t[side+'_holes'];r=t[side+'_pcd']/2
                add(side+'_'+str(i+1),'parent' if side=='fixed' else 'child',side+' face screw',[
                    tool([r*math.cos(angle),r*math.sin(angle),z],[0,0,normal],2.8,'screw_head_and_driver')],side=='fixed')
    return sites


def cylinder_path(point,direction,length,radius,sections=24):
    point=np.asarray(point);direction=np.asarray(direction,dtype=float)
    direction/=np.linalg.norm(direction)
    # Reversing travel must not rotate a polygonal nut relative to its seat.
    transform=np.eye(4) if abs(direction[2])>1-1e-12 else tm.geometry.align_vectors([0,0,1],direction)
    transform[:3,3]=point+direction*length/2
    return tm.creation.cylinder(radius=radius,height=length,sections=sections,transform=transform)


def placed_access(built, outer_regions):
    """Tool paths start above their seats and end outside all source bodies."""
    transform=built['transform'];joint=built['joint'];result=[]
    vertices=np.vstack([m.vertices for m in outer_regions.values()])
    for site in built['fastening_interfaces']:
        owner=joint[site['side']];record={k:v for k,v in site.items() if k!='tools'}
        # Complete the incoming joint before installing actuators further
        # down its child subtree. The assembly DAG enforces this declaration.
        later_bodies=set(descendants(joint['child'],built['graph']))
        deferred=[j['id'] for j in built['graph'].get('joints',[]) if j.get('id') and j['parent'] in later_bodies] if site['side']=='child' else []
        record.update(assembly_stage='before_descendant_actuators' if deferred else 'joint_assembly',deferred_joint_ids=deferred)
        record.update(id=built['id']+'_'+site['id'],joint_id=built['id'],owner_body_id=owner,
                      open_cover_body_ids=list(dict.fromkeys([joint['parent'],joint['child']])),
                      detach_body_ids=descendants(joint['child'],built['graph']) if site['detach_child_subtree'] else [],tools=[])
        for local in site['tools']:
            point=tm.transform_points([local['point_mm']],transform)[0]
            direction=transform[:3,:3]@np.asarray(local['direction']);direction/=np.linalg.norm(direction)
            length=max(20.,float((vertices@direction).max()-point@direction)+20.)
            # Build in the same hardware frame as the captive seat. A direction
            # alone loses axial roll, which matters for a hexagonal cross section.
            if local.get('shape')=='hex_nut':
                nut=tm.creation.cylinder(local['radius_mm'],local['thickness_mm'],sections=6).apply_translation(local['point_mm'])
                mesh=linear_sweep(nut,np.asarray(local['direction']),length).apply_transform(transform)
            else:mesh=cylinder_path(local['point_mm'],local['direction'],length,local['radius_mm'],local.get('sections',24)).apply_transform(transform)
            detached=[b['id'] for b in built['graph'].get('bodies',[]) if b['id']!=owner] if local.get('removal_scope')=='owner_body' else []
            record['tools'].append({**local,'point_mm':point.tolist(),'direction':direction.tolist(),'detach_body_ids':detached,
                                    'hardware_frame_mm':transform.tolist(),'length_mm':length,'mesh':mesh})
        result.append(record)
    return result


def descendants(root,graph):
    result=[root]
    for current in result:
        result.extend(j['child'] for j in graph['joints'] if j['parent']==current)
    return result


def check_access(records,pieces,lids,modules,tolerance=.1):
    """Check complete linear paths with local covers off, other bodies present."""
    reports=[]
    for site in records:
        opened=set(site['open_cover_body_ids'])
        tools=[]
        for tool in site['tools']:
            detached=set(site['detach_body_ids'])|set(tool.get('detach_body_ids',[]))
            obstacles=[(owner+'_shell',mesh) for owner,mesh in pieces.items() if owner not in detached]
            obstacles.extend((owner+'_lid',mesh) for owner,mesh in lids.items() if owner not in detached and owner not in opened and not mesh.is_empty)
            present=obstacles+[(p['id'],p['mesh']) for p in modules if p['owner'] not in detached and not p.get('merged') and p.get('joint_id') not in site.get('deferred_joint_ids',[])
                and not ((p.get('joint_id')==site.get('joint_id') or (tool.get('removal_scope')=='owner_body' and p['owner']==site['owner_body_id']))
                         and p.get('role') in tool.get('removed_module_roles',[]))]
            hits=[{'asset_id':identity,'overlap_mm3':float(overlap(tool['mesh'],mesh))} for identity,mesh in present
                  if np.all(tool['mesh'].bounds[1]>mesh.bounds[0]) and np.all(mesh.bounds[1]>tool['mesh'].bounds[0])]
            hits=[hit for hit in hits if hit['overlap_mm3']>tolerance]
            tools.append({k:v for k,v in tool.items() if k!='mesh'}|{'passed':not hits,'blockers':hits})
        reports.append({k:v for k,v in site.items() if k!='tools'}|{'tools':tools,'passed':all(t['passed'] for t in tools),
                        'scope':'continuous candidate screw/head/tool paths at recorded assembly stage; covers open; hardware fit and tightening torque unverified'})
    return reports


def order_assembly(steps, reports, graph):
    """Enforce declared actuator stages, then topologically order real steps.

    No postponed hardware is allowed without an installation step and a
    dependency on the fastening operation that needs it absent.
    """
    nodes={s['id']:dict(s,depends_on=list(s.get('depends_on',[]))) for s in steps}
    if len(nodes)!=len(steps):raise ValueError('Duplicate assembly step identifiers')
    def depend(after,before):
        if after not in nodes or before not in nodes:raise ValueError('Missing registered assembly operation: '+after+' / '+before)
        if before not in nodes[after]['depends_on']:nodes[after]['depends_on'].append(before)
    module_steps={}
    for joint in graph['joints']:
        identity=joint['id'];prefix=identity+'_step_'
        sequence=sorted([key for key in nodes if key.startswith(prefix)],key=lambda key:int(key[len(prefix):]))
        if len(sequence)<2:raise ValueError('Motor template lacks fixed/output assembly operations')
        module_steps[identity]=sequence
        for before,after in zip(sequence,sequence[1:]):depend(after,before)
    for report in reports:
        fasten=report['id']+'_fasten';identity=report['joint_id']
        sequence=module_steps[identity]
        depend(fasten,sequence[-2] if report['side']=='parent' else sequence[-1])
        if report['side']=='parent':depend(sequence[-1],fasten)
        output_summary=report['owner_body_id']+'_output_attachment'
        if report['side']=='child' and output_summary in nodes:depend(output_summary,fasten)
        for deferred in report.get('deferred_joint_ids',[]):
            depend(deferred+'_step_0',fasten)
            nodes[fasten].setdefault('install_after_fastening_joint_ids',[]).append(deferred)
    ordered=[];done=set()
    while len(done)<len(nodes):
        ready=[s for identity,s in nodes.items() if identity not in done and set(s['depends_on'])<=done]
        if not ready:raise ValueError('Assembly dependencies form a cycle or reference a missing operation')
        ordered.extend(ready);done.update(s['id'] for s in ready)
    return ordered


def apply_shell_assembly_stages(records):
    """Output screws are operated after the parent's shell has been closed."""
    for site in records:
        if site['side']=='child':
            site['open_cover_body_ids']=[site['owner_body_id']]
            site['closed_ancestor_covers']=True
    return records
