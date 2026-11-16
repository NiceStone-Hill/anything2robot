"""Convex-source solid operations and A2R link assignment / sweep adaptation.

Distance-to-segment assignment: auto_design/modules/mesh_decomp.py.
Rigid rotation / sampled interference: auto_design/modules/motor_opt.py.
Overlap and conservative extraction sweep: shell_partition/geometry.py.
Reimplemented without importing legacy application state. See attribution document.
"""
import numpy as np
import trimesh as tm
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.optimize import linprog


def box(lo, hi):
    lo, hi = np.array(lo), np.array(hi)
    return tm.creation.box(hi-lo, transform=tm.transformations.translation_matrix((lo+hi)/2))


def cylinder(x, y, z0, z1, radius, sections=32):
    return tm.creation.cylinder(radius, z1-z0, sections=sections,
                               transform=tm.transformations.translation_matrix([x,y,(z0+z1)/2]))


def manifold_solid(mesh):
    import manifold3d as md
    if mesh.is_empty:return md.Manifold()
    solid=md.Manifold(md.Mesh64(np.array(mesh.vertices,dtype=np.float64,order="C",copy=True),np.array(mesh.faces,dtype=np.uint64,order="C",copy=True)))
    if solid.status()!=md.Error.NoError:raise ValueError('实体布尔运算输入不是闭合流形')
    return solid


def from_manifold(solid):
    import manifold3d as md
    if solid.status()!=md.Error.NoError:raise ValueError('实体布尔运算失败')
    m=solid.to_mesh64()
    return tm.Trimesh(np.asarray(m.vert_properties)[:,:3],np.asarray(m.tri_verts),process=False)


def union(*parts):
    import manifold3d as md
    return from_manifold(md.Manifold.batch_boolean([manifold_solid(p) for p in parts],md.OpType.Add))


def cut(a, *b):
    import manifold3d as md
    return from_manifold(md.Manifold.batch_boolean([manifold_solid(p) for p in [a,*b]],md.OpType.Subtract))


def intersect(a, b):
    return from_manifold(manifold_solid(a)^manifold_solid(b))


def overlap(a, b):
    if a.is_empty or b.is_empty:return 0.
    if np.any(a.bounds[1] <= b.bounds[0]) or np.any(b.bounds[1] <= a.bounds[0]): return 0.
    m=intersect(a,b)
    return 0. if m.is_empty else abs(float(tm.triangles.mass_properties(m.triangles,center_mass=np.zeros(3),skip_inertia=True).volume))


def convex_sweep(m, distance):
    return tm.convex.convex_hull(np.vstack([m.vertices,m.vertices+[0,0,distance]]))


def rotate(m, angle):
    return m.copy().apply_transform(tm.transformations.rotation_matrix(angle,[0,0,1]))


def regions(m):
    return sum(float(b.volume)>1e-3 for b in m.split(only_watertight=False))


class ComponentGroupingError(ValueError):
    error_kind='internal_geometry_processing'


def remove_planar_boolean_boundaries(mesh):
    """Remove only closed, consistently wound, zero-volume planar boundaries.

    Exact Boolean tangencies can return a double-sided planar tetrahedron.
    This is no material component. Non-planar thin solids and open surfaces
    remain untouched and must still pass the ordinary volume checks.
    """
    surfaces=list(mesh.split(only_watertight=False,repair=False))
    kept=[];records=[]
    for surface in surfaces:
        thickness=float(np.linalg.svd(surface.vertices-surface.vertices.mean(axis=0),compute_uv=False)[-1])
        if (surface.is_watertight and surface.is_winding_consistent
                and abs(float(surface.volume))<1e-9 and thickness<1e-9):
            records.append({'faces':len(surface.faces),'area_mm2':float(surface.area),
                            'volume_mm3':float(surface.volume),'planarity_residual_mm':thickness})
        else:kept.append(surface)
    if not records:return mesh,[]
    cleaned=tm.util.concatenate(kept) if kept else tm.Trimesh()
    if abs(float(cleaned.volume)-float(mesh.volume))>1e-8:
        raise ComponentGroupingError('Planar boundary cleanup changed material volume')
    return cleaned,records


def volume_components(mesh):
    """Group closed cavity boundaries with their enclosing solid component.

    Trimesh.split returns connected *surfaces*. A negative-volume inner wall is
    a void boundary, not a separate solid to reconnect or fill. Preserve every
    surface and its winding; only associate inner surfaces with outer shells.
    """
    # Group the original indexed faces without repair or re-import into the
    # Boolean kernel. Kernel construction can collapse thin intersections even
    # before simplify(), so it is not an identity-preserving grouping operation.
    surfaces=list(mesh.split(only_watertight=False,repair=False))
    if not surfaces:return []
    if any(not s.is_watertight or not s.is_winding_consistent or abs(float(s.volume))<1e-9 for s in surfaces):
        raise ComponentGroupingError('承力连接输入含非闭合或零体积表面，不能作为独立实体合并')
    positive=[s for s in surfaces if s.volume>0]
    groups=[[s] for s in positive]
    for inner in (s for s in surfaces if s.volume<0):
        void=inner.copy();void.invert()
        enclosing=[]
        for i,outer in enumerate(positive):
            if np.any(inner.bounds[0]<outer.bounds[0]-1e-6) or np.any(inner.bounds[1]>outer.bounds[1]+1e-6):continue
            # Exact solid subtraction avoids thousands of slow ray casts on a
            # detailed shell. Preserve the void itself; use this copy only to
            # check that no void material lies outside the enclosing surface.
            outside=cut(void,outer)
            if outside.is_empty or abs(float(outside.volume))<=1e-6:
                enclosing.append(i)
        if not enclosing:raise ComponentGroupingError('空腔内壁没有完整包围它的正体积外壁')
        owner=min(enclosing,key=lambda i:float(positive[i].volume));groups[owner].append(inner)
    components=[tm.util.concatenate(g) for g in groups]
    if any(not c.is_volume for c in components):raise ComponentGroupingError('保留空腔后刚体组件不是有效实体')
    if abs(sum(float(c.volume) for c in components)-float(mesh.volume))>max(.001,abs(float(mesh.volume))*1e-6):
        raise ComponentGroupingError('实体组件分组改变了体积')
    return components


def bevel_shape(dimensions):
    half=np.array(dimensions)/2; bevel=min(half)*.15
    vertices=[]
    for sign in np.array(np.meshgrid(*[[-1,1]]*3)).T.reshape(-1,3):
        for axis in range(3):
            v=half.copy();v[axis]-=bevel;vertices.append(v*sign)
    return tm.convex.convex_hull(vertices)


def inset_convex(mesh, wall):
    planes=ConvexHull(mesh.vertices).equations.copy();planes[:,3]+=wall
    # Chebyshev centre is strictly inside all shifted halfspaces.
    a=np.column_stack([planes[:,:3],np.ones(len(planes))])
    result=linprog([0,0,0,-1],A_ub=a,b_ub=-planes[:,3],bounds=[(None,None)]*4,method='highs')
    if not result.success or result.x[3] <= .1: raise ValueError('外观过薄，无法生成指定壁厚')
    points=HalfspaceIntersection(planes,result.x[:3]).intersections
    return tm.convex.convex_hull(points)


def link_labels(points, joint_x, lo, hi):
    """A2R closest finite link assignment; single straight joint specialization."""
    points=np.asarray(points);dist=[]
    for a,b in [(lo,joint_x),(joint_x,hi)]:
        p=np.array([a,0,0]);d=np.array([b-a,0,0]);t=np.clip((points-p)@d/(d@d),0,1)
        dist.append(np.linalg.norm(points-(p+t[:,None]*d),axis=1))
    return np.argmin(dist,axis=0)


def normalize(mesh, unit, target_length):
    if not isinstance(mesh,tm.Trimesh) or not mesh.is_volume or regions(mesh)!=1:
        raise ValueError('需要单个闭合、方向一致的实体 STL/OBJ；没有自动修补或替换外观')
    if len(mesh.faces)>40000: raise ValueError('首个机械流程上限为 40000 个三角面，请先简化外观')
    if not mesh.is_convex: raise ValueError('当前单关节流程支持凸形外观；凹形分解仍待实现，不以凸包替换输入')
    factor={'mm':1,'cm':10,'m':1000}[unit]
    ext=mesh.extents*factor
    axes=np.argsort(-ext,kind='stable');rotation=np.eye(3)[axes]
    if np.linalg.det(rotation)<0: rotation[2]*=-1
    scale=target_length/ext.max()
    transform=np.eye(4);transform[:3,:3]=rotation*factor*scale
    transform[:3,3]=-transform[:3,:3]@mesh.bounds.mean(axis=0)
    normalized=mesh.copy().apply_transform(transform)
    if normalized.extents[1]<65 or normalized.extents[2]<65:
        raise ValueError('外观横截面不足 65 mm，无法容纳当前完整模块；电机不会随外观缩小')
    if normalized.extents[1]>140 or normalized.extents[2]>140:
        raise ValueError('当前已验证的单关节截面范围为 65–140 mm')
    return normalized,transform


def dilate(mesh, clearance):
    """Local surface dilation; conservative faceted spherical offset, mm.

    Circumscribe an icosahedron around the requested ball. The radial envelope
    is [clearance, clearance / cos(face angle)], never a scaled motor or AABB.
    """
    import manifold3d as manifold
    if clearance <= 0: return mesh.copy()
    ball=tm.creation.icosphere(subdivisions=0, radius=1.)
    apothem=float(np.min(np.einsum('ij,ij->i',ball.face_normals,ball.triangles_center)))
    ball.apply_scale(clearance/apothem)
    def solid(m):
        return manifold.Manifold(manifold.Mesh(np.asarray(m.vertices,dtype=np.float32),np.asarray(m.faces,dtype=np.uint32)))
    result=solid(mesh).minkowski_sum(solid(ball))
    if result.status()!=manifold.Error.NoError: raise ValueError('局部间隙扩张失败')
    out=result.to_mesh()
    return tm.Trimesh(np.asarray(out.vert_properties)[:,:3],np.asarray(out.tri_verts),process=True)


def linear_sweep(mesh,direction,distance,epsilon=.0001):
    """Conservative non-convex translation sweep with a thin segment box.

    Minkowski sum preserves concave slide profiles. The box adds epsilon mm
    transverse clearance, so this contains the exact straight translation.
    """
    direction=np.asarray(direction,dtype=float);direction/=np.linalg.norm(direction)
    segment=box([-epsilon,-epsilon,0],[epsilon,epsilon,distance])
    segment.apply_transform(tm.geometry.align_vectors([0,0,1],direction))
    return from_manifold(manifold_solid(mesh).minkowski_sum(manifold_solid(segment)))


def placed_sweep(mesh, transform, local_direction, distance):
    """Sweep native hardware before rigid placement, retaining its concavity.

    Minkowski sums commute with rigid transforms. Avoid rotating a detailed
    motor and its thin sweep segment independently into oblique world planes.
    """
    return linear_sweep(mesh,np.asarray(local_direction,dtype=float),distance).apply_transform(transform)
