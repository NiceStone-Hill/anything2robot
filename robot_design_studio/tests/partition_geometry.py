"""Regression for binary saddle contacts and STL deliverables."""
import io
import unittest
import numpy as np
import trimesh as tm
from studio.mechanics.voxel_surface import surface

class SurfaceTests(unittest.TestCase):
    def test_seam_core_transfers_appendage_without_dropping_detached_source(self):
        from unittest.mock import patch
        from studio.mechanics.articulation import connected_partition
        mask=np.zeros((26,26,12),dtype=bool)
        mask[1:18,1:9,1:9]=True
        mask[10:16,10:15,4:8]=True
        mask[12:14,8:11,5:7]=True
        mask[22:24,22:24,2:4]=True
        initial=np.ones(mask.shape,dtype=int)
        initial[:9]=0;initial[10:16,10:15,4:8]=0
        initial[7:13,7,6]=0;initial[12,7:12,6]=0
        initial[22:24,22:24,2:4]=0
        voxels=tm.voxel.VoxelGrid(tm.voxel.encoding.DenseEncoding(mask))
        def labels(points,graph):
            return initial[tuple(np.rint(points).astype(int).T)]
        report={}
        with patch('studio.mechanics.articulation.link_partition',side_effect=labels):
            result=connected_partition(voxels,{'bodies':[{'id':'body'},{'id':'head'}]},report)
        indices=voxels.points_to_indices(voxels.points);grid=np.full(mask.shape,-1,dtype=int)
        grid[tuple(indices.T)]=result
        self.assertEqual(len(result),mask.sum())
        self.assertTrue((grid[~mask]==-1).all())
        self.assertTrue((grid[22:24,22:24,2:4]==0).all())
        self.assertGreater((grid[10:16,10:15,4:8]==1).mean(),.9)
        self.assertLessEqual(len(report['reassigned_voxels_per_pass']),3)

    def test_ambiguous_binary_partition_and_scale_change(self):
        rng=np.random.default_rng(42)
        for _ in range(7):mask=rng.random((4,4,4))>.5
        self.assertFalse(tm.voxel.ops.matrix_to_marching_cubes(mask).is_volume)
        outputs=[]
        for pitch in [1.,5.]:
            mesh,report=surface(mask,pitch,[10,-20,30])
            self.assertTrue(mesh.is_volume);self.assertTrue(mesh.is_watertight)
            reopened=tm.load(io.BytesIO(mesh.export(file_type='stl')),file_type='stl')
            self.assertTrue(reopened.is_volume)
            self.assertIn(report['level'],[.499,.501])
            self.assertEqual(report['occupied_voxels'],mask.sum())
            self.assertLessEqual(report['boundary_surface_sample_deviation_mm'],pitch*.4)
            # Every original occupied cell centre remains inside its own region.
            points=np.argwhere(mask)*pitch+[10,-20,30]
            self.assertTrue(mesh.contains(points).all())
            outputs.append(mesh.volume)
        self.assertAlmostEqual(outputs[1]/outputs[0],125,places=4)
    def test_simple_mask_uses_original_boundary(self):
        mesh,report=surface(np.ones((4,5,6),bool),2.)
        self.assertEqual(report['level'],.5)
        self.assertEqual(report['boundary_surface_sample_deviation_mm'],0)
        self.assertTrue(mesh.is_volume)
    def test_empty_mask_is_rejected(self):
        with self.assertRaises(ValueError):surface(np.zeros((4,4,4),bool),1.)

if __name__=='__main__':unittest.main(verbosity=2)
