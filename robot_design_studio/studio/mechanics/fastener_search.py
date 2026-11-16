"""Geometry-derived screw relocation and cycle-checked cover closure proposals.

Never infer material for repairing an old hole: relocation requires the exact
stock removed when that fastening was generated. Historical assets without it
remain immutable and may only use the assembly-order search.
"""
import copy
import numpy as np
import trimesh as tm
from scipy.spatial import ConvexHull, QhullError
from .solid import cylinder, cut, union, intersect, regions


def packed(mesh):
    return {'vertices':mesh.vertices.tolist(),'faces':mesh.faces.tolist()}


def unpacked(data):
    return tm.Trimesh(data['vertices'],data['faces'],process=False)


def fastening_span(points):
    if len(points)<3:return 0.
    try:return float(ConvexHull(np.asarray(points)).volume)
    except QhullError:return 0.


def ordered_steps(steps, edges):
    nodes={s['id']:copy.deepcopy(s) for s in steps}
    if len(nodes)!=len(steps):return None
    for before,after in edges:
        if before not in nodes or after not in nodes:return None
        nodes[after].setdefault('depends_on',[])
        if before not in nodes[after]['depends_on']:nodes[after]['depends_on'].append(before)
    done=set();result=[]
    while len(done)<len(nodes):
        ready=[s for k,s in nodes.items() if k not in done and set(s.get('depends_on',[]))<=done]
        if not ready:return None
        result.extend(ready);done.update(s['id'] for s in ready)
    return result


def sequence(records, panels, steps):
    """Only removable, explicitly registered covers may be deferred.

    All existing motor/join/closure dependencies are preserved. A shaft that
    collides at its final installed length is never excused by assembly order.
    """
    covers={p.get('cover_asset_id',p['owner_body_id']+'_lid') for p in panels}
    edges=[];changed=[]
    for r in records:
        if r['passed']:continue
        blockers=set(r['tool_blocked_assets']+r['shaft_blocked_assets'])
        sound=(r['shaft_obstruction_mm3']<.02 and r['head_obstruction_mm3']<.02 and
               r['cover_bearing_mm3']>1 and r['mate_bearing_mm3']>1)
        if not sound or not blockers or not blockers<=covers or r.get('installed_screw_blocked_assets'):continue
        proposal=edges+[(r['cover_asset_id'],other) for other in sorted(blockers)]
        if ordered_steps(steps,proposal) is None:continue
        edges=proposal;changed.append((r,sorted(blockers)))
    ordered=ordered_steps(steps,edges)
    if ordered is None:return None
    for r,blockers in changed:
        r.update(passed=True,assembly_stage='before_obstructing_covers',deferred_cover_ids=blockers,
                 access_in_final_closed_state=False,access_with_deferred_covers=True)
    return {'steps':ordered,'closure_dependencies':[{'before':a,'after':b} for a,b in edges],
            'scope':'Existing operation dependencies preserved; straight cover extraction remains a separate check.'}


def relocate(parts, analysis, domains, evaluate, overlap, limit=48):
    """Bounded nearest-first search; commit paired geometry and metadata atomically."""
    records=evaluate(parts,analysis,overlap);events=[]
    for failed in list(records):
        if failed['passed']:continue
        panel=next(p for p in analysis['lid_planes'] if p.get('cover_asset_id',p['owner_body_id']+'_lid')==failed['cover_asset_id'])
        index=int(failed['id'].rsplit('_',1)[1])-1;old=panel['fasteners'][index]
        stock=old.get('restore_cut_material')
        if stock is None or old.get('candidate_thread')!='M3' or panel['owner_body_id'] not in domains:
            events.append({'id':failed['id'],'status':'no_registered_restoration_stock'});continue
        frame=np.asarray(panel['transform_mm']);inv=np.linalg.inv(frame);plane=panel['plane_z_mm'];top=panel['top_local_mm']
        old_position=list(old['position_mm'])
        old_xy=tm.transform_points([old_position],inv)[0,:2]
        points=panel.get('fastener_candidate_xy_mm',[])
        others=[tm.transform_points([f['position_mm']],inv)[0,:2] for j,f in enumerate(panel['fasteners']) if j!=index]
        points=sorted(points,key=lambda xy:float(np.linalg.norm(np.asarray(xy)-old_xy)))
        attempts=0;accepted=False
        for xy in points:
            if np.linalg.norm(np.asarray(xy)-old_xy)<2 or any(np.linalg.norm(np.asarray(xy)-p)<9 for p in others):continue
            if len(others)>=2 and fastening_span([xy,*others])<max(1.,.6*fastening_span([old_xy,*others])):continue
            if attempts>=limit:break
            attempts+=1;x,y=xy
            trial=[{'record':p['record'].copy(),'mesh':p['mesh'].copy()} for p in parts]
            cap=next(p for p in trial if p['record']['id']==failed['cover_asset_id'])
            base=next(p for p in trial if p['record']['id']==failed['mate_asset_id'])
            a=union(base['mesh'],unpacked(stock['base']));b=union(cap['mesh'],unpacked(stock['cap']))
            domain=domains[panel['owner_body_id']]
            lower=intersect(cylinder(x,y,plane-3,plane,4.6).apply_transform(frame),domain)
            upper=intersect(cylinder(x,y,plane,plane+3.5,4.6).apply_transform(frame),domain)
            # New seats cannot intersect any installed hardware or neighboring shell.
            other=[p['mesh'] for p in trial if p is not cap and p is not base]
            if any(overlap(seat,m)>.02 for seat in [lower,upper] for m in other):continue
            pre_a=union(a,lower);pre_b=union(b,upper)
            shaft=cylinder(x,y,plane-4,top+30,1.7).apply_transform(frame)
            head=cylinder(x,y,plane+1.5,top+30,3.2).apply_transform(frame)
            nut=cylinder(x,y,plane-3.1,plane+.1,3.35,6).apply_transform(frame)
            base['mesh']=cut(pre_a,shaft,nut);cap['mesh']=cut(pre_b,shaft,head)
            if any(not p['mesh'].is_volume or regions(p['mesh'])!=1 for p in [base,cap]):continue
            if overlap(base['mesh'],cap['mesh'])>.02:continue
            protected=[unpacked(c['mesh']) for c in analysis.get('support_core_records',[]) if c['owner_body_id']==panel['owner_body_id']]
            if any(float(core.volume)-overlap(core,base['mesh'])>.1 for core in protected):continue
            # Refuse cuts that break the bearing seats of any unchanged screw.
            config=copy.deepcopy(analysis)
            target=next(p for p in config['lid_planes'] if p.get('cover_asset_id',p['owner_body_id']+'_lid')==failed['cover_asset_id'])
            replacement=target['fasteners'][index]
            replacement.update(position_mm=tm.transform_points([[x,y,plane]],frame)[0].tolist(),
                restore_cut_material={'base':packed(cut(pre_a,base['mesh'])),'cap':packed(cut(pre_b,cap['mesh']))})
            nut_entry=cylinder(x,y,plane-3.1,top+30,3.35,6).apply_transform(frame)
            if overlap(nut_entry,base['mesh'])>.02:continue
            checked=evaluate(trial,config,overlap)
            moved=next(r for r in checked if r['id']==failed['id'])
            formerly_passed={r['id'] for r in records if r['passed']}
            if not moved['passed'] or any(not r['passed'] for r in checked if r['id'] in formerly_passed):continue
            # Restoring an old hole must not collide with later-added parts either.
            if any(overlap(p['mesh'],m)>.1 for p in [base,cap] for m in other):continue
            parts[:]=trial
            for live,updated in zip(analysis['lid_planes'],config['lid_planes']):
                original_fasteners=live['fasteners']
                for before,after in zip(original_fasteners,updated['fasteners']):before.clear();before.update(after)
                live.clear();live.update(updated);live['fasteners']=original_fasteners
            records=checked;accepted=True
            events.append({'id':failed['id'],'status':'relocated','old_position_mm':old_position,
                'position_mm':replacement['position_mm'],'attempts':attempts,'old_hole_restored':True})
            break
        if not accepted:events.append({'id':failed['id'],'status':'no_feasible_relocation','attempts':attempts})
    analysis['fastener_search']={'relocations':events,'candidate_budget_per_fastener':limit}
    return records
