"""Physical counterexamples for closed, removable shells (no permanent ports)."""
import unittest
import numpy as np
from studio.mechanics.solid import box,cut,union,overlap,linear_sweep,regions
from studio.mechanics.assembly_split import split_shell,split_body,connected_interface_frame
from studio.mechanics.shell_access import PanelSearchError

class AssemblySplitTests(unittest.TestCase):
    def test_split_precedes_fixed_pose_mount_connection(self):
        from studio.mechanics.joint_support import attachment
        outer,shell,hint=self.fixture()
        # An internal mounting plate is not yet part of the hollow shell.
        # Its fixed position constrains the seam, without demanding that the
        # shell already contain the plate or a pre-generated support network.
        mount=box([-6,-6,-10],[6,6,-7])
        base,cap,panel=split_shell(shell,outer,hint,[mount],[],3,[],budget_s=20)
        before=mount.vertices.copy()
        found=attachment(base,mount,np.array([0.,0.,-8.]),[cap],outer,
            {'beam_radius':2.,'max_bridge_span_mm':40.},False)
        self.assertIsNotNone(found)
        np.testing.assert_allclose(found['target'],[0,-panel['clearance_mm'],-15],atol=1e-5)
        self.assertAlmostEqual(found['length_mm'],float(np.hypot(7,panel['clearance_mm'])))
        self.assertTrue(all(f['candidate_thread']=='M3' for f in panel['fasteners']))
        connected=union(base,mount,found['mesh'])
        self.assertEqual(regions(connected),1)
        self.assertLess(overlap(found['mesh'],cap),.01)
        np.testing.assert_array_equal(mount.vertices,before)
        self.assertTrue(panel['continuous_extraction'])

    def test_output_template_void_survives_union_with_source_shell(self):
        from studio.mechanics.joint_modules import module
        from studio.mechanics.solid import cylinder
        built=module('sg90_digital_towerpro')
        mount=next(p[3] for p in built['parts'] if p[0]=='output_mount')
        source=cylinder(0,0,0,8,12)
        result=union(cut(source,built['cavity']),mount)
        for void in (v['mesh'] for v in built['interface_voids'] if v['side']=='child'):
            self.assertLess(overlap(result,void),.01)
        # The bounded interface cutter must not perforate the opposite wall.
        roof=box([-10,-10,25],[10,10,28])
        self.assertLess(overlap(built['cavity'],roof),.01)

    def test_interface_frame_uses_source_material_and_avoids_tool_path(self):
        outer,shell,_=self.fixture()
        mounts=[box([-24,-5,-5],[-21,5,5]),box([21,-5,-5],[24,5,5])]
        tool=box([-4,-4,0],[4,4,30])
        protected,report=connected_interface_frame(shell,mounts,[tool],3)
        frame=union(*protected)
        self.assertEqual(report['method'],'retained_source_material_frame')
        self.assertEqual(regions(frame),1)
        self.assertLess(cut(frame,shell).volume,.01)
        self.assertLess(overlap(frame,tool),.01)
        self.assertFalse(report['physical_strength_verified'])
        base,covers=split_body(shell,outer,{'normal':[0,0,1]},[],mounts,3,[tool])
        self.assertEqual(regions(base),1)
        self.assertLess(overlap(base,tool),.1)
        self.assertGreater(sum(overlap(cap,tool) for cap,_ in covers),100)
        for _,panel in covers:
            self.assertTrue(panel['continuous_extraction'])
            self.assertFalse(panel['permanent_assembly_ports'])

    def fixture(self):
        outer=box([-24,-20,-18],[24,20,18])
        shell=cut(outer,box([-21,-17,-15],[21,17,15]))
        return outer,shell,{'normal':[0,0,1],'center_mm':[0,0,0],'reason':'test'}

    def test_closed_cover_restores_tool_corridor_surface(self):
        outer,shell,hint=self.fixture()
        tool=box([-3,-3,0],[3,3,30])
        base,cap,p=split_shell(shell,outer,hint,[],[],3,[tool],budget_s=30)
        self.assertLess(overlap(base,tool),.1)
        self.assertGreater(overlap(cap,tool),100)
        self.assertFalse(p['permanent_assembly_ports'])
        self.assertTrue(p['continuous_extraction'])
        self.assertLess(p['closed_unregistered_missing_mm3'],.1)
        self.assertGreaterEqual(len(p['fasteners']),3)

    def test_joint_end_and_integral_rib_stay_with_main_shell(self):
        outer,shell,hint=self.fixture()
        end=box([-24,-20,-18],[-19,20,18])
        rib=box([-22,-2,-5],[0,2,-1])
        shell=union(shell,end,rib)
        tool=box([3,-3,0],[9,3,30])
        base,cap,p=split_shell(shell,outer,hint,[],[end,rib],3,[tool],budget_s=45)
        for protected in [end,rib]:
            self.assertAlmostEqual(overlap(base,protected),protected.volume,places=3)
            self.assertLess(overlap(cap,protected),.01)
        self.assertEqual(p['method'],'protected_stepped_split')

    def test_curved_shell_restores_surface_after_tool_access(self):
        import trimesh as tm
        outer=tm.creation.icosphere(subdivisions=2,radius=28)
        shell=cut(outer,tm.creation.icosphere(subdivisions=2,radius=24))
        tool=box([-3,-3,0],[3,3,40])
        base,cap,p=split_shell(shell,outer,{'normal':[0,0,1],'center_mm':[0,0,0],'reason':'curved shell'},[],[],4,[tool],budget_s=30)
        self.assertLess(overlap(base,tool),.1)
        self.assertGreater(overlap(cap,tool),20)
        self.assertLess(p['closed_unregistered_missing_mm3'],.1)
        self.assertGreaterEqual(len(p['fasteners']),3)

    def test_output_screw_cannot_rely_on_reopening_parent_cover(self):
        import trimesh as tm
        from studio.mechanics.fastening import apply_shell_assembly_stages,check_access
        site={'id':'output','joint_id':'joint','side':'child','owner_body_id':'child',
              'open_cover_body_ids':['parent','child'],'detach_body_ids':[],
              'tools':[{'mesh':box([-1,-1,-2],[1,1,5]),'removed_module_roles':[]}]}
        pieces={'parent':box([10,10,10],[12,12,12]),'child':box([20,20,20],[22,22,22])}
        lids={'parent':box([-5,-5,0],[5,5,2]),'child':tm.Trimesh()}
        self.assertTrue(check_access([site],pieces,lids,[])[0]['passed'])
        apply_shell_assembly_stages([site])
        report=check_access([site],pieces,lids,[])[0]
        self.assertFalse(report['passed'])
        self.assertEqual(report['tools'][0]['blockers'][0]['asset_id'],'parent_lid')

    def test_impossible_protected_tool_path_fails_without_port(self):
        outer,shell,hint=self.fixture()
        protected=box([-4,-4,15],[4,4,18]);tool=box([-3,-3,10],[3,3,30])
        with self.assertRaises(PanelSearchError):
            split_shell(shell,outer,hint,[],[protected],3,[tool],budget_s=5)

if __name__=='__main__':unittest.main(verbosity=2)
