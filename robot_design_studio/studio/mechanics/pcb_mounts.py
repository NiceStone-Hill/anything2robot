"""Registered PCB screw interfaces, following the legacy hardware.py convention.

Source CAD, hole axes and supports use one rigid transform. Native CAD holes
are measured from archived STEP, not inferred from a box or scaled to a tray.
Fastener dimensions remain explicitly uncalibrated prototype assumptions.
"""
import numpy as np
import trimesh as tm
from .solid import box,cylinder,union,cut,regions
from . import component_visuals as visuals


def attach(catalog_id,dimensions,orientation,angle,effective_height,width,depth):
    ref=visuals.reference(catalog_id)
    if ref is None:return None
    entry,cad=ref;spec=entry.get('mounting')
    if not spec:return None
    if spec['source_sha256']!=entry['source_sha256']:raise ValueError('PCB mounting source hash mismatch')
    dims=np.asarray(dimensions,dtype=float)
    registration=visuals.registration(entry,cad,dims)
    transformed=cad.copy().apply_transform(registration)
    if np.any(transformed.bounds[0]<-dims/2-1e-5) or np.any(transformed.bounds[1]>dims/2+1e-5):
        raise ValueError('Reference PCB and standoffs do not fit the declared envelope; increase its height explicitly.')
    pose=tm.transformations.rotation_matrix(angle,[0,0,1])@orientation
    pose[:3,3]=[0,0,2+effective_height/2]
    source_to_tray=pose@registration
    b=spec['pcb_bottom_mm'];top=spec['pcb_top_mm'];f=spec['fastener'];h=f['standoff_height_mm'];t=f['plate_thickness_mm']
    lower=np.min(spec['outline_mm'],axis=0)-1;upper=np.max(spec['outline_mm'],axis=0)+1
    plate=box([*lower,b-h-t],[*upper,b-h])
    posts=[cylinder(*hole['xy_mm'],b-h-.05,b,f['nut_circumradius_mm']+.45) for hole in spec['holes']]
    bracket=union(plate,*posts).apply_transform(source_to_tray)
    # A connected L-foot joins an upright PCB backplate to the removable lid tray.
    bounds=bracket.bounds
    foot=box([bounds[0,0],bounds[0,1],1.9],[bounds[1,0],bounds[1,1],max(2.1,bounds[0,2]+.15)])
    tray=box([-width/2-8,-depth/2-5,0],[width/2+8,depth/2+5,2])
    tray=cut(tray,*[cylinder(x,0,-1,3,1.2) for x in [-width/2-5,width/2+5]])
    tray=union(tray,bracket,foot)
    holes=[];nut_entries=[];tools=[];records=[]
    for hole in spec['holes']:
        x,y=hole['xy_mm']
        bore=cylinder(x,y,b-h-t-1,top+1,f['clearance_diameter_mm']/2).apply_transform(source_to_tray)
        # Nut enters from the underside of the PCB backplate, before mounting
        # the independent carrier in the robot. Leave a shoulder at its top.
        nut=cylinder(x,y,b-h-t-35,b-h-.4,f['nut_circumradius_mm'],6).apply_transform(source_to_tray)
        tool=cylinder(x,y,top+.05,top+35,f['head_diameter_mm']/2+.3).apply_transform(source_to_tray)
        holes.append(bore);nut_entries.append(nut);tools.append(tool)
        records.append({'id':hole['id'],'source_center_mm':[x,y,b],'source_hole_diameter_mm':hole['diameter_mm'],
                        'tray_center_mm':tm.transform_points([[x,y,b]],source_to_tray)[0].tolist(),
                        'tray_axis':source_to_tray[:3,2].tolist(),'source_step_surface':hole['step_surface']})
    tray=cut(tray,*holes,*nut_entries,*tools)
    if not tray.is_volume or regions(tray)!=1:raise ValueError('Registered PCB carrier is not one connected solid')
    # Reserve a full upper envelope, but leave the known PCB underside free for
    # standoffs. This is an explicit registered mounting zone, not deleted hardware.
    envelope=box(-dims/2,dims/2)
    native=envelope.copy().apply_transform(np.linalg.inv(registration));nb=native.bounds
    keepout=box([nb[0,0],nb[0,1],b],[nb[1,0],nb[1,1],nb[1,2]])
    # Preserve the PCB annulus around its actual measured hole. The larger head
    # and driver clearance starts above the board, never through board material.
    keepout=cut(keepout,*[cylinder(*hole['xy_mm'],b-1,top,hole['diameter_mm']/2) for hole in spec['holes']],
                *[cylinder(*hole['xy_mm'],top,nb[1,2]+1,f['head_diameter_mm']/2+.3) for hole in spec['holes']])
    keepout.apply_transform(source_to_tray)
    return tray,keepout,{'model_id':entry['id'],'method':'registered_pcb_screws','holes':records,
        'source_to_tray_mm':source_to_tray.tolist(),'source_sha256':entry['source_sha256'],'source_url':entry['source_url'],
        'reference_variant':entry['variant'],'fastener':f,'pcb_bottom_mm':b,'pcb_top_mm':top,
        'standoff_height_mm':h,'plate_thickness_mm':t,'physical_fit_verified':False,
        'collision_scope':'Conservative upper hardware envelope with registered PCB underside and screw-access keepouts.'}


def world_record(local,frame,xyz,side):
    transform=np.eye(4)
    if side==-1:transform=tm.transformations.rotation_matrix(np.pi,[1,0,0])
    transform[:3,3]=xyz
    world=frame@transform
    record={**local,'holes':[]}
    record['source_to_world_mm']=(world@np.asarray(local['source_to_tray_mm'])).tolist()
    for hole in local['holes']:
        record['holes'].append({**hole,'center_world_mm':tm.transform_points([hole['tray_center_mm']],world)[0].tolist(),
            'axis_world':(world[:3,:3]@hole['tray_axis']).tolist()})
    return record


def fasteners(mount):
    """Uncalibrated screw/washer/nut solids reserve the installed hardware space."""
    matrix=np.asarray(mount['source_to_world_mm']);f=mount['fastener'];b=mount['pcb_bottom_mm'];top=mount['pcb_top_mm'];result=[]
    for hole in mount['holes']:
        x,y,_=hole['source_center_mm'];seat=top+.5
        bolt=union(cylinder(x,y,seat-f['candidate_length_mm'],seat+.01,f['shaft_diameter_mm']/2),
                   cylinder(x,y,top,seat+1.5,f['head_diameter_mm']/2)).apply_transform(matrix)
        nut=cut(cylinder(x,y,b-5.9,b-4.5,f['nut_circumradius_mm']-.05,6),
                cylinder(x,y,b-6,b-4.4,f['shaft_diameter_mm']/2+.03)).apply_transform(matrix)
        result.extend([(hole['id']+'_screw',bolt),(hole['id']+'_nut',nut)])
    return result
