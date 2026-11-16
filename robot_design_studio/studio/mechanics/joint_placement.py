"""Installation policy shared by motor recommendations and assembly placement.

Exterior dimensions are bounded using the module bounding-box corners and
surface vertices. These are placement checks, not strength or motion proofs.
"""
import numpy as np
import trimesh as tm
from .solid import overlap


def placement_metrics(candidate, shape, exposed=False):
    outside=max(0.,float(candidate.volume)-overlap(candidate,shape))
    result={'outside_mm3':outside,'outside_fraction':outside/max(float(candidate.volume),1e-9)}
    if exposed:
        probes=np.vstack([tm.bounds.corners(candidate.bounds),candidate.convex_hull.vertices])
        distances=tm.proximity.signed_distance(shape,probes)
        result['sampled_protrusion_mm']=float(np.maximum(-distances,0).max())
    return result


def room_fits(metrics, built, design):
    if built['allow_exterior']:
        return metrics['sampled_protrusion_mm']<=design['open_mount_max_protrusion_mm']
    return metrics['outside_fraction']<=design['module_max_outside_fraction']


def placement_score(metrics, built, support_distances, collision=0., shift=0.):
    support=sum(d*d for d in support_distances)
    if built['allow_exterior']:
        # Prefer accessible exposure while keeping both shell connections short.
        return support+800*(1-metrics['outside_fraction'])**2+collision*5+abs(shift)*.001
    return metrics['outside_mm3']+support+collision*5+abs(shift)*.001


def registration_metrics(transform, joint, built):
    """Native output origin, not motor bounding-box centre, defines placement."""
    axis=np.asarray(joint['axis'],dtype=float);axis/=np.linalg.norm(axis)
    delta=np.asarray(transform)[:3,3]-np.asarray(joint['position_mm'])
    axial=float(delta@axis);radial=float(np.linalg.norm(delta-axial*axis))
    alignment=float(abs(np.asarray(transform)[:3,2]@axis))
    error=float(np.linalg.norm(delta))
    return {'output_datum_error_mm':error,'axis_line_error_mm':radial,
            'output_registration_passed':bool(radial<1e-5 and alignment>1-1e-8 and
                (built['registration_policy']!='output_datum_at_joint' or error<1e-5))}


def mounting_surface_checks(built, pieces):
    """Each mating face must survive on its declared rigid body's final shell.

    The witnesses have finite thickness to allow exact solid intersection. A
    floating seat or seat trimmed away by a tool opening cannot pass. This is
    geometric retention/connectivity only, not a load or real fastener fit test.
    """
    from .solid import regions
    reports=[]
    for site in built.get('mounting_surfaces',[]):
        owner=built['joint'][site['side']]
        witness=site['mesh'].copy().apply_transform(built['transform'])
        expected=abs(float(witness.volume));retained=overlap(witness,pieces[owner])
        connected=regions(pieces[owner])==1
        reports.append({'id':built['id']+'_'+site['id']+'_mating_surface','owner_body_id':owner,
            'passed':bool(expected>1e-3 and retained>=expected-.001 and connected),
            'required_mm3':expected,'retained_mm3':retained,'owner_connected':connected,
            'point_mm':tm.transform_points([site['point']],built['transform'])[0].tolist(),
            'normal':(built['transform'][:3,:3]@site['normal']).tolist(),
            'assembly_gap_mm':site['assembly_gap_mm'],'physical_fit_verified':False})
    return reports
