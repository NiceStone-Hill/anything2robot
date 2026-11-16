"""Bounded voxel A* ribs, adapting A2R's protected-connection path method.

All fragments remain accounted for. No path means rejection, never deleting a
small disconnected part to make connectivity appear valid.
"""
import heapq
import math
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
import trimesh as tm
from .solid import union, volume_components, regions


def astar(free,starts,goals,budget=150000):
    shape=np.array(free.shape);goalset={tuple(p) for p in goals};goalbox=np.array(goals)
    lower=goalbox.min(0);upper=goalbox.max(0)
    def heuristic(node):return int(np.maximum(lower-node,0).sum()+np.maximum(node-upper,0).sum())
    heap=[];costs={};parent={}
    for node in starts:
        key=tuple(int(x) for x in node);costs[key]=0;parent[key]=None;heapq.heappush(heap,(heuristic(node),0,key))
    expanded=0
    while heap and expanded<budget:
        _,distance,current=heapq.heappop(heap)
        if distance!=costs[current]:continue
        expanded+=1
        if current in goalset:
            path=[]
            while current is not None:path.append(current);current=parent[current]
            return np.array(path[::-1]),expanded
        for axis in range(3):
            for direction in [-1,1]:
                candidate=list(current);candidate[axis]+=direction;candidate=tuple(candidate)
                if candidate[axis]<0 or candidate[axis]>=shape[axis] or not free[candidate]:continue
                new=distance+1
                if new>=costs.get(candidate,math.inf):continue
                costs[candidate]=new;parent[candidate]=current
                heapq.heappush(heap,(new+heuristic(np.array(candidate)),new,candidate))
    return None,expanded


def obstructed_grid(voxels,meshes,radius):
    blocked=np.zeros(voxels.shape,dtype=bool);pitch=float(voxels.pitch[0])
    for mesh in meshes:
        if mesh.is_empty:continue
        # The source grid bounds this computation; far-away bodies need no voxel map.
        lo=voxels.transform[:3,3]-pitch;hi=lo+np.array(voxels.shape)*pitch+pitch
        if np.any(mesh.bounds[1]<lo) or np.any(mesh.bounds[0]>hi):continue
        occupied=mesh.voxelized(pitch).fill().points
        indices=voxels.points_to_indices(occupied)
        valid=np.all((indices>=0)&(indices<np.array(voxels.shape)),axis=1);indices=indices[valid]
        if len(indices):blocked[tuple(indices.T)]=True
    # Conservative full-neighbour inflation at voxel resolution.
    return binary_dilation(blocked,structure=np.ones((3,3,3),dtype=bool),iterations=max(1,math.ceil(radius/pitch)))


def reconnect(mesh,domain,voxels,obstacles,radius):
    fragments=sorted(volume_components(mesh),key=lambda m:float(m.volume),reverse=True)
    if len(fragments)<=1:return mesh,{'required':False,'paths':[],'passed':True}
    free=domain&~obstructed_grid(voxels,obstacles,radius)
    indices=np.argwhere(free);points=voxels.indices_to_points(indices)
    if not len(points):return mesh,{'required':True,'passed':False,'reason':'本刚体内没有避开运动和模块的承力路径空间','paths':[]}
    connected=fragments[0];paths=[]
    def cylinder_between(a,b):
        direction=b-a;length=np.linalg.norm(direction)
        if length<1e-6:return tm.creation.icosphere(subdivisions=1,radius=radius).apply_translation(a)
        transform=tm.geometry.align_vectors([0,0,1],direction/length);transform[:3,3]=(a+b)/2
        return tm.creation.cylinder(radius,length+radius,sections=12,transform=transform)
    for fragment in fragments[1:]:
        start_distance,start_vertex=cKDTree(connected.vertices).query(points)
        goal_distance,goal_vertex=cKDTree(fragment.vertices).query(points)
        # Multiple entry cells avoid requiring hand-selected attachment coordinates.
        start_order=np.argsort(start_distance)[:32];goal_order=np.argsort(goal_distance)[:32]
        path,expanded=astar(free,indices[start_order],indices[goal_order])
        if path is None:
            return mesh,{'required':True,'passed':False,'reason':'在原外观区域与运动障碍约束内找不到连通路径','expanded':expanded,'paths':paths}
        coords=voxels.indices_to_points(path)
        source=connected.vertices[cKDTree(connected.vertices).query(coords[0])[1]]
        target=fragment.vertices[cKDTree(fragment.vertices).query(coords[-1])[1]]
        # Collapse collinear grid steps; keep every turn and exact contact endpoints.
        keep=[0]
        for k in range(1,len(coords)-1):
            if not np.array_equal(path[k]-path[k-1],path[k+1]-path[k]):keep.append(k)
        if len(coords)>1:keep.append(len(coords)-1)
        polyline=np.vstack([source,coords[keep],target])
        ribs=[cylinder_between(a,b) for a,b in zip(polyline[:-1],polyline[1:])]
        connected=union(connected,fragment,*ribs)
        paths.append({'polyline_mm':polyline.tolist(),'radius_mm':radius,'expanded':expanded})
    return connected,{'required':True,'passed':regions(connected)==1,'paths':paths,
                      'method':'bounded_voxel_astar','max_expanded_per_fragment':150000}
