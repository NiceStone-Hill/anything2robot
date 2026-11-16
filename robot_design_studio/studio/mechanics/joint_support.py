"""Bounded shell attachments, checked before accepting an exposed motor pose."""
import time
import numpy as np
import trimesh as tm
from scipy.spatial import cKDTree
from .solid import union, cut, intersect, overlap, regions


def beam(a,b,radius):
    direction=b-a;length=np.linalg.norm(direction)
    if length<.01:return tm.creation.icosphere(radius=radius).apply_translation(a)
    transform=tm.geometry.align_vectors([0,0,1],direction/length);transform[:3,3]=(a+b)/2
    return tm.creation.cylinder(radius,length+radius,sections=16,transform=transform)


def _attachment(shell,mount,anchor,obstacles,shape,design,exposed,broaden=False):
    """Return a real connecting rib, or None; never infer fit from distance alone."""
    started=time.monotonic()
    if shell.is_empty:return None
    if exposed:
        # Every candidate beam contains this inscribed ball at its anchor.
        # A blocked anchor cannot be repaired by trying hundreds of endpoints.
        common=tm.creation.icosphere(subdivisions=1,radius=design['open_mount_min_core_radius_mm']/2).apply_translation(anchor)
        if any(overlap(common,o)>.01 for o in obstacles):return None
    _,ids=cKDTree(shell.vertices).query(anchor,k=min(80,len(shell.vertices)))
    radius=design['beam_radius'];contact_min=design['open_mount_min_contact_mm3'] if exposed else .05
    def attempt(target,middle=None):
        points=[anchor,target] if middle is None else [anchor,middle,target]
        length=sum(float(np.linalg.norm(b-a)) for a,b in zip(points,points[1:]))
        if length>design['max_bridge_span_mm']:return None
        rib=union(*[beam(a,b,radius) for a,b in zip(points,points[1:])])
        local=[o for o in obstacles if not o.is_empty and np.all(o.bounds[1]>rib.bounds[0]) and np.all(o.bounds[0]<rib.bounds[1])]
        core=None
        if exposed:
            core=union(*[beam(a,b,design['open_mount_min_core_radius_mm']) for a,b in zip(points,points[1:])])
            if any(overlap(core,o)>.01 for o in local):return None
        rib=cut(rib,*local)
        if not exposed:rib=intersect(rib,shape)
        if rib.is_empty or regions(rib)!=1:return None
        contacts=[overlap(shell,rib),overlap(mount,rib)]
        if min(contacts)<=contact_min:return None
        intrusion=sum(overlap(rib,o) for o in local)
        if intrusion>.1:return None
        return {'mesh':rib,'core':core,'target':target,'length_mm':length,'via':middle,
                'contacts_mm3':contacts,'intrusion_mm3':intrusion}
    # A nearest vertex is not the nearest shell surface (especially on a
    # large, flat triangle). Try the actual closest point first, and retain
    # the existing collision-aware alternatives when that route is blocked.
    closest=tm.proximity.closest_point(shell,np.asarray(anchor,float).reshape(1,3))[0][0]
    found=attempt(closest)
    if found is not None:return found
    # Dense tessellation can put all nearest 80 vertices on one blocked patch.
    # Also inspect directionally distinct shell surfaces within the same span.
    delta=shell.vertices-anchor;distance=np.linalg.norm(delta,axis=1)
    eligible=np.flatnonzero((distance>1e-6)&(distance<=design['max_bridge_span_mm']))
    diverse=[]
    if broaden and len(eligible):
        directions=delta[eligible]/distance[eligible,None]
        bins=np.round(directions*2).astype(int)
        for key in np.unique(bins,axis=0):
            group=eligible[np.all(bins==key,axis=1)]
            diverse.extend([int(group[np.argmin(distance[group])]),int(group[np.argmax(distance[group])])])
    # Bound solid-boolean work while retaining angular coverage.
    pool=list(dict.fromkeys(diverse));spread=[]
    while pool and len(spread)<24:
        if not spread:chosen=min(pool,key=lambda i:distance[i])
        else:chosen=max(pool,key=lambda i:min(np.linalg.norm(delta[i]/distance[i]-delta[j]/distance[j]) for j in spread))
        spread.append(chosen);pool.remove(chosen)
    targets=list(dict.fromkeys([*np.atleast_1d(ids)[::4].tolist(),*spread]))
    for vertex in targets:
        if time.monotonic()-started>8:return None
        found=attempt(shell.vertices[vertex])
        if found is not None:return found
    for vertex in np.atleast_1d(ids)[::10]:
        target=shell.vertices[vertex]
        for offset in [np.eye(3)[axis]*sign*radius*2 for axis in range(3) for sign in [-1,1]]:
            if time.monotonic()-started>8:return None
            found=attempt(target,(anchor+target)/2+offset)
            if found is not None:return found
    return None


def attachment(shell,mount,anchor,obstacles,shape,design,exposed,broaden=False,relocate_anchor=False):
    found=_attachment(shell,mount,anchor,obstacles,shape,design,exposed,broaden)
    if found is not None:
        found['anchor']=np.asarray(anchor);return found
    if not relocate_anchor:return None
    # A fixed actuator pose does not imply a fixed rib attachment point.
    # Search points in the registered mounting solid, without moving the motor.
    started=time.monotonic()
    axes=[np.linspace(lo+.2,hi-.2,7) for lo,hi in zip(*mount.bounds)]
    points=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    points=points[mount.contains(points)]
    if not len(points):return None
    distances=tm.proximity.closest_point(shell,points)[1]
    selected=points[np.argsort(distances)]
    for candidate in selected[::max(1,len(selected)//48)]:
        if time.monotonic()-started>25:break
        found=_attachment(shell,mount,candidate,obstacles,shape,design,exposed,True)
        if found is not None:
            found['anchor']=candidate;return found
    return None
