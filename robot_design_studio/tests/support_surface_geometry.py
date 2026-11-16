"""Reachable support surfaces and native-frame nonconvex installation sweeps."""
import json
import unittest
import numpy as np
import trimesh as tm
from studio.mechanics.solid import box, union, overlap, placed_sweep
from studio.mechanics.joint_support import attachment
from studio.mechanics.joint_modules import TEMPLATE_PATH


class SupportSurfaceTests(unittest.TestCase):
    def test_reachable_side_wall_when_dense_nearest_surface_is_blocked(self):
        shell=union(box([-10,-10,5],[23,10,7]),box([20,-10,-10],[23,10,7]))
        for _ in range(4):shell=shell.subdivide()
        mount=box([-3,-3,-2],[3,3,0]);anchor=np.array([0.,0.,-1.])
        obstacles=[box([-11,-11,1],[11,11,4])]
        design=json.loads(TEMPLATE_PATH.read_text())['design']
        args=(shell,mount,anchor,obstacles,box([-30,-30,-30],[30,30,30]),design,True)
        self.assertIsNone(attachment(*args))
        result=attachment(*args,broaden=True)
        self.assertIsNotNone(result)
        self.assertGreater(overlap(result['mesh'],shell),design['open_mount_min_contact_mm3'])
        self.assertGreater(overlap(result['mesh'],mount),design['open_mount_min_contact_mm3'])
        self.assertLess(overlap(result['core'],obstacles[0]),.01)
        self.assertLessEqual(result['length_mm'],design['max_bridge_span_mm'])

    def test_fixed_motor_allows_a_different_rib_anchor(self):
        shell=box([18,-15,-6],[21,15,6]);mount=box([-8,-7,-3],[8,7,0])
        obstacle=box([-2,-2,-5],[2,2,5]);anchor=np.array([0.,0.,-1.])
        original=mount.vertices.copy();design=json.loads(TEMPLATE_PATH.read_text())['design']
        args=(shell,mount,anchor,[obstacle],box([-30,-30,-30],[30,30,30]),design,True)
        self.assertIsNone(attachment(*args))
        found=attachment(*args,relocate_anchor=True)
        self.assertIsNotNone(found)
        self.assertGreater(overlap(found['mesh'],mount),design['open_mount_min_contact_mm3'])
        self.assertLess(overlap(found['core'],obstacle),.01)
        np.testing.assert_array_equal(mount.vertices,original)

    def test_cover_finish_cutters_are_reserved_for_support_search(self):
        from studio.mechanics.cover_fasteners import reserved_channels
        panel={'transform_mm':np.eye(4).tolist(),'plane_z_mm':0,'top_local_mm':10,
               'fasteners':[{'position_mm':[0,0,0],'candidate_thread':'M3'}]}
        reserved=reserved_channels(panel)
        # The final re-drill starts 5 mm behind the seam, not at the seam.
        probe=box([-.2,-.2,-4.8],[.2,.2,-4.2])
        self.assertAlmostEqual(overlap(probe,reserved[0]),probe.volume)

    def test_rotated_nonconvex_sweep_preserves_gap_and_contains_travel(self):
        guide=union(box([-10,-10,0],[-5,10,10]),box([5,-10,0],[10,10,10]),box([-10,-10,0],[10,-5,10]))
        transform=tm.transformations.rotation_matrix(.7,[1,2,3]);transform[:3,3]=[15,-8,30]
        swept=placed_sweep(guide,transform,[0,0,1],20)
        self.assertAlmostEqual(overlap(swept,box([-3,-3,12],[3,3,25]).apply_transform(transform)),0)
        for z in [0,1,7.5,20]:
            moved=guide.copy().apply_translation([0,0,z]).apply_transform(transform)
            self.assertAlmostEqual(overlap(swept,moved),moved.volume,places=3)


if __name__=='__main__':unittest.main()
