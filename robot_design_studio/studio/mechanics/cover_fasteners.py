"""Re-cut cover screw/head access after adding lid-mounted electronics supports.

Follows legacy shell fastening: shared axes, a head entry, nut seat, bearing
material, and an external straight-tool corridor. Thread-specific M2/M3 clearances; physical fit remains unverified.
"""
import numpy as np
import trimesh as tm
from .solid import cylinder,cut,union


def _finish(parts,analysis,overlap):
    records=[]
    for panel in analysis.get('lid_planes',[]):
        owner=panel['owner_body_id'];cap=next((p for p in parts if p['record']['id']==panel.get('cover_asset_id',owner+'_lid')),None)
        base=next((p for p in parts if p['record']['id']==owner+'_shell'),None)
        if cap is None or base is None:continue
        detached=set(panel.get('detached_body_ids',[]))
        present=[p for p in parts if p['record'].get('owner_body_id') not in detached]
        transform=np.asarray(panel['transform_mm']);inv=np.linalg.inv(transform);plane=panel['plane_z_mm'];top=panel['top_local_mm'];seat=plane+1.5
        for index,fastener in enumerate(panel['fasteners']):
            x,y,_=tm.transform_points([fastener['position_mm']],inv)[0]
            thread=fastener.get('candidate_thread','M2')
            bore,head_r,probe_r,tool_r,boss,washer_r,nut_r=(1.7,3.2,1.5,3.1,4.4,1.8,3.45) if thread=='M3' else (1.2,2.2,1.,2.1,3.4,1.3,2.6)
            shaft=cylinder(x,y,plane-5,top+30,bore).apply_transform(transform)
            entry=cylinder(x,y,seat,top+30,head_r).apply_transform(transform)
            cap['mesh']=cut(cap['mesh'],shaft,entry)
            base['mesh']=cut(base['mesh'],shaft)
            axis=transform[:3,2]
            # Check a slightly smaller full screw/head corridor, independent of
            # the cutting cylinders, and actual remaining material on each side.
            probe=cylinder(x,y,seat-6,top+20,probe_r).apply_transform(transform)
            head=cylinder(x,y,seat+.01,top+20,tool_r).apply_transform(transform)
            washer=cut(cylinder(x,y,plane+.1,seat-.1,boss),cylinder(x,y,plane,seat,washer_r)).apply_transform(transform)
            mate=cut(cylinder(x,y,plane-2.8,plane-.1,boss),cylinder(x,y,plane-3,plane,nut_r)).apply_transform(transform)
            screw_block=overlap(probe,cap['mesh'])+overlap(probe,base['mesh']);head_block=overlap(head,cap['mesh'])
            upper=overlap(washer,cap['mesh']);lower=overlap(mate,base['mesh'])
            tool=cylinder(x,y,seat+.01,top+30,tool_r).apply_transform(transform)
            blocked=[p['record']['id'] for p in present if p is not cap and p is not base and overlap(tool,p['mesh'])>.02]
            shaft_blocked=[p['record']['id'] for p in present if p is not cap and p is not base and overlap(probe,p['mesh'])>.02]
            installed=union(cylinder(x,y,seat-6,seat,probe_r),cylinder(x,y,seat,seat+2.,tool_r)).apply_transform(transform)
            installed_blocked=[p['record']['id'] for p in present if p is not cap and p is not base and overlap(installed,p['mesh'])>.02]
            records.append({'installed_screw_blocked_assets':installed_blocked,'id':panel.get('cover_asset_id',owner)+'_cover_screw_'+str(index+1),'owner_body_id':owner,'cover_asset_id':cap['record']['id'],
                'mate_asset_id':base['record']['id'],'axis_world':axis.tolist(),'head_seat_world_mm':tm.transform_points([[x,y,seat]],transform)[0].tolist(),
                'shared_hole_center_world_mm':fastener['position_mm'],'shaft_clearance_diameter_mm':2*bore,'head_entry_diameter_mm':2*head_r,
                'candidate_thread':thread,'nut_seat':'existing candidate hexagonal seat','candidate_underhead_length_mm':6.,'actual_part_number':None,
                'shaft_obstruction_mm3':screw_block,'head_obstruction_mm3':head_block,'cover_bearing_mm3':upper,'mate_bearing_mm3':lower,
                'tool_blocked_assets':blocked,'shaft_blocked_assets':shaft_blocked,'passed':screw_block<.02 and head_block<.02 and upper>1 and lower>1 and not blocked and not shaft_blocked,
                'detached_body_ids':sorted(detached),'assembly_stage':panel.get('assembly_stage','final_closed_assembly'),'physical_fit_verified':False})
    return records


def finish(parts,analysis,overlap,domains=None,assembly_steps=None):
    from .fastener_search import relocate, sequence
    records=relocate(parts,analysis,domains or {},_finish,overlap)
    if assembly_steps is not None:
        plan=sequence(records,analysis.get('lid_planes',[]),assembly_steps)
        analysis['cover_assembly_plan']=plan
    return records


def reserved_channels(panel):
    """Reserve final closure cuts before ribs are routed to the shell."""
    frame=np.asarray(panel['transform_mm']);inverse=np.linalg.inv(frame)
    plane=panel['plane_z_mm'];top=panel['top_local_mm'];channels=[]
    for fastener in panel['fasteners']:
        x,y,_=tm.transform_points([fastener['position_mm']],inverse)[0]
        bore,head,nut=(1.7,3.2,3.35) if fastener.get('candidate_thread')=='M3' else (1.2,2.2,2.6)
        channels.extend(m.apply_transform(frame) for m in [
            cylinder(x,y,plane-5,top+30,bore),
            cylinder(x,y,plane+1.5,top+30,head),
            cylinder(x,y,plane-3.1,plane+.1,nut,6)])
    return channels
