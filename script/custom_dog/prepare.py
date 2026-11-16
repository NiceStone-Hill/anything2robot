"""Prepare the user's dog mesh and five manually interpreted joint annotations."""
import hashlib
import json
import pickle
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'auto_design/modules'))
from mesh_loader import Link
from data_struct import TreeNode

out = ROOT / 'data/custom_dog'
out.mkdir(exist_ok=True)
source = Path('/home/qinxuan/.codex/attachments/eddc86b8-4687-4cdc-9019-a4785913363e/cute dog 3d model.stl')
shutil.copy2(source, out / 'original.stl')
mesh = trimesh.load(source, force='mesh')
# Image left/right follows the supplied labels, not anatomical left/right.
points = {
    'neck': [0, -0.01, 0.525],
    'left_shoulder': [-0.135, -0.015, 0.49],
    'right_shoulder': [0.135, -0.015, 0.49],
    'left_hip': [-0.085, 0.0, 0.205],
    'right_hip': [0.085, 0.0, 0.205],
    'head_tip': [0, 0, 0.85],
    'left_hand': [-0.35, -0.01, 0.465],
    'right_hand': [0.35, -0.01, 0.465],
    'left_foot': [-0.095, -0.025, 0.01],
    'right_foot': [0.095, -0.025, 0.01],
}
body = TreeNode(Link('BODY'))
body.val.joints = {k: np.array(points[k]) for k in list(points)[:5]}
body.val.axis = [np.zeros(3), np.zeros(3)]
nodes = {'BODY': body}
for name, start, end, axis in [
    ('HEAD', 'neck', 'head_tip', [0, 0, 1]),
    ('L_ARM', 'left_shoulder', 'left_hand', [0, 1, 0]),
    ('R_ARM', 'right_shoulder', 'right_hand', [0, 1, 0]),
    ('L_LEG', 'left_hip', 'left_foot', [1, 0, 0]),
    ('R_LEG', 'right_hip', 'right_foot', [1, 0, 0]),
]:
    node = TreeNode(Link(name))
    node.val.joints = {k: np.array(points[k]) for k in [start, end]}
    node.val.axis = [np.array(points[start]), np.array(axis)]
    body.add_child(node)
    nodes[name] = node
with (out / 'dog_joints.pkl').open('wb') as stream:
    pickle.dump(nodes, stream)
mesh.export(out / 'dog.stl')
scale = 60 / mesh.extents[0]
counts = np.bincount(mesh.edges_unique_inverse)
meta = {
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'source_units': 'unspecified; geometry normalized to X width 60 cm for test',
    'expected_x_cm': 60, 'voxel_size_cm': 0.5,
    'dimensions_cm_at_test_scale': (mesh.extents * scale).tolist(),
    'faces': len(mesh.faces), 'watertight': bool(mesh.is_watertight),
    'nonmanifold_edges': int(sum(counts > 2)), 'boundary_edges': int(sum(counts == 1)),
    'points_source_coordinates': points,
    'points_cm_at_test_scale': {k: (np.array(v) * scale).tolist() for k, v in points.items()},
    'axes': {k: n.val.axis[1].tolist() for k, n in nodes.items()},
    'assumptions': ['Single view does not specify depth or axes. Depth manually inferred from side mesh.',
                    'Image left/right follows supplied J2/J3/J4/J5 labels.',
                    'Neck yaw about Z; shoulder swing about Y; hip swing about X.',
                    'Five actuated joints; hand/head tips and foot contacts are decomposition annotations only.',
                    'Original input mesh retained; no surface repair before original-pipeline test.'],
}
(out / 'annotation.json').write_text(json.dumps(meta, indent=2))
fig, axes = plt.subplots(1, 2, figsize=(11, 8))
for ax, dims, title in zip(axes, [(0, 2), (1, 2)], ['Front: X/Z', 'Side: Y/Z']):
    v = mesh.vertices * scale
    ax.triplot(v[:, dims[0]], v[:, dims[1]], mesh.faces, lw=.25, color='gray', alpha=.45)
    for i, name in enumerate(list(points)[:5], 1):
        p = np.array(points[name]) * scale
        ax.scatter(p[dims[0]], p[dims[1]], color='#934ae8', s=60, zorder=4)
        ax.annotate(f'J{i}', (p[dims[0]], p[dims[1]]), xytext=(7, 4), textcoords='offset points', fontsize=11)
    ax.set_aspect('equal'); ax.set_title(title); ax.set_xlabel('cm'); ax.grid(alpha=.2)
fig.tight_layout(); fig.savefig(out / 'joint_annotation.png', dpi=180)
print(json.dumps(meta, indent=2))
