"""Small, deterministic parting-plane proposals; no model-specific coordinates.

Semantic points are proposals, not feasibility evidence. Projected mounting
intervals add planes that retain whole interfaces on the main (-normal) side.
The solid generator must still verify access, attachment and extraction.
"""
import numpy as np


def plane_levels(bounds, semantic_z, protected_bounds, wall, gap, contact_z=()):
    lo, hi = float(bounds[0][2]), float(bounds[1][2])
    if not np.isfinite([lo, hi, semantic_z, wall, gap]).all():
        raise ValueError('分型面参数必须有限')
    if wall <= 0 or gap < 0 or hi <= lo:
        raise ValueError('分型面尺寸无效')
    # Start with the requested seam. If it crosses an interface, the next
    # proposal sits beyond all mounting surfaces, retaining them as a whole.
    candidates = [float(semantic_z)]
    if protected_bounds:
        candidates.append(max(float(b[1][2]) for b in protected_bounds) + gap + .6)
    candidates.extend([lo + (hi-lo)*f for f in (.5, .65, .35, .2, .1)])
    if len(contact_z):
        candidates.append(min(contact_z)-.5)
    return list(dict.fromkeys(round(v, 3) for v in candidates
                             if lo+wall < v < hi-wall))


def longitudinal_normals(vertices, preferred):
    """Vertical seams first (Z is up), then local longitudinal seams.

    A semantic end-cap suggestion must not outrank a longitudinal opening.
    Stable axis order avoids PCA noise changing the front/back split.
    """
    vertices=np.asarray(vertices,float)
    _,axes=np.linalg.eigh(np.cov(vertices.T))
    preferred=np.asarray(preferred,float);preferred=preferred/np.linalg.norm(preferred)
    candidates=[np.array(v,float) for v in ([0,1,0],[0,-1,0],[1,0,0],[-1,0,0])]
    if abs(preferred[2])<1e-6:
        candidates.insert(0,preferred)
    candidates.extend([*axes[:,:2].T,*-axes[:,:2].T,preferred,np.array([0,0,1.]),np.array([0,0,-1.])])
    result=[]
    for n in candidates:
        if not any(np.dot(n,v)>.999 for v in result):result.append(n)
    return result
