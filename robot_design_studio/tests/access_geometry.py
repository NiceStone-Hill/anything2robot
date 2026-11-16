import unittest
import numpy as np
from studio.mechanics.solid import box,cut,overlap,regions
from studio.mechanics.shell_access import local_panel,validate_suggestions

class AccessTests(unittest.TestCase):
    def test_only_output_leaf_can_omit_service_panel(self):
        from studio.mechanics.shell_access import access_requirement
        graph={'root_body_id':'base','joints':[{'id':'a','parent':'base','child':'tip'}]}
        output=[{'owner':'tip','role':'rotor'},{'owner':'tip','role':'output_mount'}]
        self.assertFalse(access_requirement('tip',graph,output,{})['required'])
        self.assertTrue(access_requirement('base',graph,output,{})['required'])
        self.assertTrue(access_requirement('tip',graph,output+[{'owner':'tip','role':'motor'}],{})['required'])
        self.assertTrue(access_requirement('tip',graph,output,{'layout_config':{'components':[{'owner_body_id':'tip'}]}})['required'])
        self.assertTrue(access_requirement('tip',graph,output,{'instances':[{'owner_body_id':'tip'}]})['required'])
        graph['joints'].append({'id':'b','parent':'tip','child':'end'})
        self.assertTrue(access_requirement('tip',graph,output,{})['required'])

    def test_voxel_artifacts_cannot_remove_source_material_or_mounts(self):
        from studio.mechanics.shell_access import separate_voxel_artifacts
        from studio.mechanics.solid import union
        main=box([0,0,0],[30,30,30]);artifact=box([30.5,0,0],[31,.5,.5])
        mesh=union(main,artifact)
        clean,removed,report=separate_voxel_artifacts(mesh,main,[],2,1)
        self.assertEqual(len(removed),1);self.assertAlmostEqual(overlap(clean,artifact),0)
        self.assertEqual(report['components'][0]['source_overlap_mm3'],0)
        for source,protected,count in [(union(main,artifact),[],1),(main,[artifact],1),(main,[],2)]:
            clean,removed,_=separate_voxel_artifacts(mesh,source,protected,2,count)
            self.assertFalse(removed);self.assertAlmostEqual(overlap(clean,artifact),artifact.volume)
        substantial=box([30.5,0,0],[32.5,2,2])
        clean,removed,_=separate_voxel_artifacts(union(main,substantial),main,[],1,1)
        self.assertFalse(removed)

    def test_connected_local_panel_preserves_remote_limbs(self):
        from studio.mechanics.solid import union
        outer=union(box([-50,-40,-40],[50,40,40]),box([-70,-15,-40],[-45,15,-10]),box([45,-15,-40],[70,15,-10]))
        shell=cut(outer,box([-47,-37,-37],[47,37,37]))
        motor=box([-10,-6,-10],[10,6,10])
        a,b,r=local_panel(shell,outer,{'body_id':'body','center_mm':[0,0,0],'normal':[0,0,1],'reason':'test'},[motor],[],3)
        self.assertEqual(regions(a),1);self.assertEqual(regions(b),1)
        self.assertLess(overlap(a,b),.01);self.assertGreaterEqual(len(r['fasteners']),3)
        # Both limbs stay with main shell, never sliced into the lid.
        for probe in [box([-65,-5,-35],[-55,5,-20]),box([55,-5,-35],[65,5,-20])]:
            self.assertAlmostEqual(overlap(probe,a),probe.volume,places=3)
            self.assertAlmostEqual(overlap(probe,b),0,places=3)
    def test_nonconvex_insertion_sweep_keeps_slide_clearance(self):
        from studio.mechanics.solid import union,linear_sweep
        guide=union(box([-10,-10,0],[-5,10,10]),box([5,-10,0],[10,10,10]),box([-10,-10,0],[10,-5,10]))
        sweep=linear_sweep(guide,[0,0,1],20)
        self.assertAlmostEqual(overlap(sweep,box([-3,-3,12],[3,3,25])),0)
        for z in [0,1,7.5,20]:
            moved=guide.copy().apply_translation([0,0,z])
            self.assertAlmostEqual(overlap(sweep,moved),moved.volume,places=3)

    def test_cutting_cleanup_is_bounded_and_preserves_interfaces(self):
        from studio.mechanics.shell_access import separate_cutting_remnants
        from studio.mechanics.solid import union
        main=box([0,0,0],[30,30,30]);chip=box([31,0,0],[31.5,.5,.5]);far=box([80,0,0],[80.5,.5,.5])
        mesh=union(main,chip,far);cutter=box([31.5,0,0],[33,3,3])
        cleaned,removed,report=separate_cutting_remnants(mesh,[],[cutter])
        self.assertEqual(len(removed),1)
        self.assertAlmostEqual(overlap(cleaned,far),far.volume)
        self.assertAlmostEqual(overlap(cleaned,chip),0)
        self.assertLess(report['fraction'],.005)
        protected,removed,_=separate_cutting_remnants(mesh,[chip],[cutter])
        self.assertFalse(removed)
        self.assertAlmostEqual(overlap(protected,chip),chip.volume)

    def test_semantic_suggestions_reject_foreign_ids_and_bad_normals(self):
        graph={'bodies':[{'id':'body'}]};bounds=np.array([[-50]*3,[50]*3])
        good={'regions':[{'body_id':'body','center_mm':[0,0,0],'normal':[0,1,0],'reason':'侧面检修口'}]}
        validate_suggestions(good,bounds,graph)
        good['regions'][0]['normal']=[1,1,0]
        with self.assertRaises(ValueError):validate_suggestions(good,bounds,graph)

if __name__=='__main__':unittest.main(verbosity=2)
