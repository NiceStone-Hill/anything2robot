"""Focused regression checks for issues exposed by the custom robot run."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import json
import sys
import tempfile
import numpy as np
import trimesh
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'auto_design/modules'), str(ROOT/'auto_design')]
from motor_opt import a_star_search, set_diff_numpy
from mesh_loader import Custom_Mesh_Loader

def shortest(grid, start, goals):
    queue = deque([(start, 0)]); seen = {start}
    while queue:
        p, d = queue.popleft()
        if p in goals: return d
        for axis in range(3):
            for step in [-1, 1]:
                q = list(p); q[axis] += step; q = tuple(q)
                if all(0 <= v < n for v, n in zip(q, grid.shape)) and grid[q] and q not in seen:
                    seen.add(q); queue.append((q, d+1))
    return None

rng = np.random.default_rng(42)
for trial in range(30):
    grid = rng.random((7, 6, 5)) > .3
    start = (0, 0, 0); ends = np.array([[6,5,4],[6,0,4]])
    grid[start] = True; grid[tuple(ends.T)] = True
    path = a_star_search(grid, start, ends, [False])
    expected = shortest(grid, start, {tuple(v) for v in ends})
    assert (len(path)-1 if path else None) == expected
    if path:
        assert path[0] == start and path[-1] in {tuple(v) for v in ends}
        assert all(grid[p] for p in path)
        assert all(sum(abs(a-b) for a,b in zip(p,q)) == 1 for p,q in zip(path,path[1:]))
assert a_star_search(np.ones((2,2,2)), (0,0,0), np.empty((0,3)), [0]) == []
a = rng.integers(0, 5, (100,3)); b = rng.integers(0,5,(50,3))
old = a[~np.any(np.all(a[:,None,:] == b, axis=2), axis=1)]
assert np.array_equal(set_diff_numpy(a,b),old)
loader = Custom_Mesh_Loader(SimpleNamespace(headless=True))
loader.load_joint_positions(str(ROOT/'data/custom_dog/dog_joints.pkl'))
assert len(loader.link_tree.get_all_children()[0]) == 5
assert len(loader.joint_dict) == 10
with tempfile.TemporaryDirectory() as tmp:
    mesh = trimesh.creation.box([7,3,5]); mesh.apply_translation([12,0,3])
    mesh.export(Path(tmp)/'offset.stl'); loader.load_mesh(str(Path(tmp)/'offset.stl'))
    loader.scale(60); assert np.isclose(np.ptp(np.asarray(loader.scaled_mesh.mesh_o3d.vertices)[:,0]),60)
    previous = loader.link_tree.children[0].val.axis[0].copy()
    loader.scale(66); assert np.isclose(np.ptp(np.asarray(loader.scaled_mesh.mesh_o3d.vertices)[:,0]),66)
    assert np.allclose(loader.link_tree.children[0].val.axis[0],previous*1.1)
result={'status':'PASS','shortest_path_cases':30,'empty_target_case':True,'row_difference_equivalence':True,'headless_five_joint_import':True,'offset_mesh_width_and_retry_scaling':True}
(ROOT/'data/custom_dog/regression_checks.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
