"""Pin-aware assembly guides, distinct from checked bundle occupancy solids.

Reference pads use the very same source-to-world matrix as the displayed CAD.
Unselected hardware never acquires a fictitious measured connector. Coloured
lanes and dashed terminal associations are assembly aids, not released wires.
"""
import colorsys
import numpy as np
import trimesh as tm
from .harness import astar, tube
from .electronics_layout import overlap
from ..catalog import Catalog

PICO_PINOUT='https://pip-assets.raspberrypi.com/categories/1005-raspberry-pi-pico-2/documents/RP-008301-DS-1-Pico-2-Pinout.pdf'
MPU_FOOTPRINT='https://github.com/usini/usini_kicad_sensors/blob/e2d03afffcb95ee2b2ff4183b6573658f2684229/usini_sensors.pretty/module_mpu6050.kicad_mod'


def wire_color(index):
    # Stable by wire number; never encode electrical polarity in display colour.
    rgb=colorsys.hsv_to_rgb((.57+index*.61803398875)%1,.76,.76)
    return '#'+''.join(f'{round(c*255):02x}' for c in rgb)


def reference_pad(asset, port, physical_pin):
    """Return a reference STEP pad and outward normal; millimetres."""
    if asset['model_id']=='pico2' and physical_pin:
        pin=int(physical_pin)
        if not 1<=pin<=40:return None
        # USB is at source +Y. Header holes, not castellated edge cut-outs.
        point=[-8.89 if pin<=20 else 8.89,24.13-(pin-1)*2.54 if pin<=20 else -24.13+(pin-21)*2.54,.95]
        evidence='manufacturer_reference_pad';url=PICO_PINOUT
    elif asset['model_id']=='mpu6050' and port in ['VCC','GND','SCL','SDA']:
        index=['VCC','GND','SCL','SDA'].index(port)
        # The supplied STEP has 18/7 mm pitch, different from the footprint's
        # nominal 2.54 mm. Preserve the STEP; do not rescale or claim real fit.
        point=[9-index*18/7,-7,0];pin=None;evidence='reference_pad_assumed_label_order';url=MPU_FOOTPRINT
    else:return None
    matrix=np.asarray(asset['source_to_world_mm'])
    return {'position_m':(tm.transform_points([point],matrix)[0]/1000).tolist(),
            'exit_direction':(matrix[:3,:3]@np.array([0.,0.,1.])).tolist(),
            'source_position_mm':point,'source_sha256':asset['source_sha256'],
            'physical_pin':str(pin) if pin else None,'evidence':evidence,'source_url':url,
            'physical_verified':False,'pin_label_verified':asset['model_id']=='pico2'}


def endpoint_record(endpoint, project, layout, meshes, aggregation, sibling_index, sibling_count, catalog):
    identity=endpoint['instance_id'];instance=next(i for i in project['instances'] if i['id']==identity)
    spec=catalog.resolve(instance)['ports'].get(endpoint.get('port_id'),{})
    asset=next((a for a in layout.get('visual_assets',[]) if a['instance_id']==identity),None)
    registered=reference_pad(asset,endpoint.get('port_id'),spec.get('physical_pin')) if asset else None
    placement=next((p for p in layout['component_placements'] if p['instance_id']==identity),None)
    if registered is None:
        # Separate labelled placeholders on a component face. Explicitly not a
        # proposed connector layout for supply/distributor/servo candidates.
        bounds=meshes[identity].bounds;center=bounds.mean(0);direction=aggregation-center
        axis=int(np.argmax(np.abs(direction)/np.maximum(bounds[1]-bounds[0],1)))
        normal=np.zeros(3);normal[axis]=1 if direction[axis]>=0 else -1
        point=center.copy();point[axis]=bounds[1 if normal[axis]>0 else 0,axis]
        other=(axis+1)%3
        point[other]+=(sibling_index-(sibling_count-1)/2)*min(2.,(bounds[1,other]-bounds[0,other])/(sibling_count+1))
        registered={'position_m':(point/1000).tolist(),'exit_direction':normal.tolist(),
                    'physical_pin':spec.get('physical_pin'),'evidence':'unregistered_component_face_placeholder',
                    'physical_verified':False,'pin_label_verified':False}
    return {**endpoint,**registered,'component_label':instance['label'],'coordinate_frame':'world_rest','length_unit':'m',
            'label':endpoint.get('port_id') or ((endpoint.get('junction_net_id','Unregistered')+' contact '+str(sibling_index+1)) if endpoint.get('junction_id') else 'Unassigned contact'),
            'junction_net_id':endpoint.get('junction_net_id'),
            'support_asset_id':placement.get('support_asset_id') if placement else None}


def segment_lane(segment, wire_id):
    """A full-diameter wire lane inside the reserved straight cross sections."""
    points=np.asarray(segment['points_m'])*1000;out=[]
    x,y,_=segment['packing_circles_xy_radius_mm'][segment['packing_wire_order'].index(wire_id)]
    for index,point in enumerate(points):
        tangent=points[min(index+1,len(points)-1)]-points[max(0,index-1)]
        tangent/=np.linalg.norm(tangent)
        basis=np.eye(3)[np.argmin(abs(tangent))];u=np.cross(tangent,basis);u/=np.linalg.norm(u);v=np.cross(tangent,u)
        out.append(point+x*u+y*v)
    return out


def envelope_exit(pin, normal, bounds, allowance):
    """First outward intersection of a ray with an enclosing AABB."""
    distances=[(bounds[1 if n>0 else 0,a]-pin[a])/n for a,n in enumerate(normal) if abs(n)>1e-6]
    distance=max(0.,min(distances,default=0.))
    return pin+normal*(distance+allowance)


def add_connections(project, layout, parts, meshes, terminals, segments, routes, free, lo, pitch):
    """Attach evidence and individually selectable wire guides to current routes."""
    catalog=Catalog();records={};keys={}
    for route in routes:
        route['color']=wire_color(len(keys)) if route['id'] not in keys else keys[route['id']]
        keys[route['id']]=route['color']
        for side in ['from','to']:
            ep=route[side];key=(ep['instance_id'],ep.get('port_id'),ep.get('junction_id'))
            records.setdefault(key,ep)
    groups={identity:[k for k in records if k[0]==identity] for identity in meshes}
    resolved={}
    for key,ep in records.items():
        siblings=groups[key[0]]
        resolved[key]=endpoint_record(ep,project,layout,meshes,np.asarray(terminals[key[0]]['position_m'])*1000,siblings.index(key),len(siblings),catalog)
    single_radius=project['harness_config']['wire_diameter_mm']/2
    # Full tube tests supplement the grid. The sole exception is the short
    # outward pad exit through its own conservative component envelope.
    obstacles=[p['mesh'] for p in parts]
    def blockers(a,b,exclude=None):
        if np.linalg.norm(np.asarray(a)-b)<1e-6:return []
        mesh=tube(np.asarray(a),np.asarray(b),single_radius+.15)
        return [p['record']['id'] for p in parts if p['mesh'] is not exclude and overlap(mesh,p['mesh'])>.01]
    def clear(a,b,exclude=None):
        if np.linalg.norm(np.asarray(a)-b)<1e-6:return True
        mesh=tube(np.asarray(a),np.asarray(b),single_radius+.15)
        return not any(other is not exclude and overlap(mesh,other)>.01 for other in obstacles)
    cells=np.argwhere(free);xyz=lo+cells*pitch;segments_by_id={s['id']:s for s in segments}
    cache={};failures={}
    def fanout(ep, target):
        key=(ep['instance_id'],ep.get('port_id'),ep.get('junction_id'),tuple(target))
        if key in cache:return cache[key]
        pin=np.asarray(ep['position_m'])*1000;normal=np.asarray(ep['exit_direction']);own=meshes[ep['instance_id']]
        # Egress is straight along the registered board normal until outside
        # the component envelope. Never search through the PCB or its carrier.
        exit_point=envelope_exit(pin,normal,own.bounds,single_radius+1.)
        result=None
        blocked=blockers(pin,exit_point,own)
        if not blocked:
            if clear(exit_point,target):result=[pin,exit_point,target]
            else:
                end=tuple(cells[np.argmin(np.linalg.norm(xyz-target,axis=1))])
                for index in np.argsort(np.linalg.norm(xyz-exit_point,axis=1))[:24]:
                    if np.linalg.norm(xyz[index]-exit_point)>18:break
                    if not clear(exit_point,xyz[index]):continue
                    try:path=astar(free,tuple(cells[index]),end)
                    except ValueError:continue
                    path=[exit_point]+[lo+np.asarray(c)*pitch for c in path]+[target]
                    smooth=[path[0]];cursor=0
                    while cursor<len(path)-1:
                        nxt=len(path)-1
                        while nxt>cursor+1 and not clear(path[cursor],path[nxt]):nxt-=1
                        if not clear(path[cursor],path[nxt]):break
                        smooth.append(path[nxt]);cursor=nxt
                    if cursor==len(path)-1:result=[pin]+smooth;break
        cache[key]=result
        failures[key]={'reason':'outward_exit_blocked' if blocked else 'no_clear_terminal_path','blocking_assets':blocked} if result is None else None
        return result
    for route in routes:
        lanes=[]
        for item in route['segments']:
            lane=segment_lane(segments_by_id[item['segment_id']],route['id'])
            if item['reverse']:lane=lane[::-1]
            lanes.extend(lane)
        sides={};leads={}
        for side,target in [('from',lanes[0]),('to',lanes[-1])]:
            old=route[side];key=(old['instance_id'],old.get('port_id'),old.get('junction_id'))
            ep=dict(resolved[key]);lead=fanout(ep,target)
            ep['fanout_clearance_checked']=lead is not None
            failure_key=(ep['instance_id'],ep.get('port_id'),ep.get('junction_id'),tuple(target))
            ep['fanout_failure']=failures.get(failure_key)
            leads[side]=(np.asarray(lead)/1000).tolist() if lead is not None else [ep['position_m'],(np.asarray(target)/1000).tolist()]
            sides[side]=ep
        route.update(endpoints=sides,guide_points_m=(np.asarray(lanes)/1000).tolist(),fanout_points_m=leads,
                     guide_role='individual_wire_assembly_guide',fanout_clearance_checked=all(e['fanout_clearance_checked'] for e in sides.values()),
                     lane_transitions_verified=False,physical_connection_verified=False)
    return list(resolved.values())


def guide_mesh(points, radius, dashed=False):
    pieces=[]
    for a,b in zip(np.asarray(points)*1000,np.asarray(points)[1:]*1000):
        length=np.linalg.norm(b-a)
        if length<1e-6:continue
        if dashed:
            for start in np.arange(0,length,2.5):
                pieces.append(tube(a+(b-a)*start/length,a+(b-a)*min(start+1.4,length)/length,radius))
        else:pieces.append(tube(a,b,radius))
    return tm.util.concatenate(pieces) if pieces else tm.Trimesh()
