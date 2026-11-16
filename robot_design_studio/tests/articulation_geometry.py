import io
"""Actual geometric tests; run in the separate geometry interpreter."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
import trimesh as tm
from studio.catalog import Catalog
from studio.project import Store
from studio.mechanical_service import appearance
from studio.articulation_service import configure
from studio.contracts import ContractError
from studio.mechanics.articulation import load_source, reference_graph, validate_graph, link_partition
from studio.mechanics.planning import render_views
from studio.mechanics.joint_modules import module
from studio.mechanics.graph_assembly import poses, interior_mask, stl_deliverable
from studio.mechanics.solid import overlap, cylinder, volume_components, union
from studio.mechanics.structural_paths import astar, reconnect


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name);store=Store(root,Catalog());p=store.create('graph')
        p=appearance(store,p['id'],{'revision':1,'builtin':'box'})
        p=configure(store,p['id'],{'revision':p['revision'],'mode':'reference','intent':'','length_m':.24,'up_axis':'+Y','support':'fixed_base','max_joints':3})
        self.mesh,self.voxels,self.analysis=load_source(p,store.directory(p['id'])/'sources'/p['source_shapes'][0]['file'])
        render_views(self.mesh,root);self.cameras=json.loads((root/'views.json').read_text())
        self.graph=reference_graph(self.mesh,self.voxels,3,self.cameras)
    def tearDown(self):self.tmp.cleanup()
    def test_reprojection_graph_and_volume_partition(self):
        checks=validate_graph(self.graph,self.mesh,self.voxels,self.cameras,3)
        self.assertEqual(len(checks),9);self.assertTrue(all(c['error_px']<1e-8 for c in checks))
        labels=link_partition(self.voxels.points,self.graph)
        self.assertEqual(set(labels),{0,1,2,3});self.assertEqual(len(labels),len(self.voxels.points))
        self.assertEqual(self.analysis['up_axis'],'+Y');self.assertFalse(self.analysis['global_convex_hull_used'])
    def test_invalid_vlm_geometry_is_rejected(self):
        for key,value in [('axis',[0,0,0]),('position_mm',[9999,0,0]),('limits_deg',[True,10]),('dof',True)]:
            g=copy.deepcopy(self.graph);g['joints'][0][key]=value
            with self.assertRaises(ContractError):validate_graph(g,self.mesh,self.voxels,self.cameras,3)
        g=copy.deepcopy(self.graph);g['joints'][0]['observations_px']['top.png']=[0,0]
        with self.assertRaises(ContractError):validate_graph(g,self.mesh,self.voxels,self.cameras,3)
        g=copy.deepcopy(self.graph);g['joints'][0]['child']=g['root_body_id']
        with self.assertRaises(ContractError):validate_graph(g,self.mesh,self.voxels,self.cameras,3)
    def test_complete_module_solids_do_not_interpenetrate(self):
        for catalog_id in ['sg90_digital_towerpro','mg4005v2_candidate']:
            built=module(catalog_id);parts=built['parts']
            self.assertTrue(all(p[3].is_volume for p in parts))
            for index,a in enumerate(parts):
                for b in parts[index+1:]:self.assertLess(overlap(a[3],b[3]),.05,(catalog_id,a[0],b[0]))
        square=module('sg90_digital_towerpro');self.assertIn('retainer',[p[0] for p in square['parts']]);self.assertNotIn('cartridge',[p[0] for p in square['parts']])
        round_module=module('mg4005v2_candidate');fixed=round_module['parts'][0][3]
        # Independently probe the rear M3 bolt circle from supplier drawing.
        self.assertLess(overlap(cylinder(0,24,-38,-32,1.6),fixed),.01)
    def test_graph_pose_propagates_parent_rotation(self):
        graph={'root_body_id':'root','joints':[{'id':'a','parent':'root','child':'arm','axis':[1,0,0],'position_mm':[0,0,0]},
             {'id':'b','parent':'arm','child':'tip','axis':[0,1,0],'position_mm':[0,10,0]}]}
        result=poses(graph,{'a':np.pi/2,'b':np.pi/2})
        np.testing.assert_allclose(result['root'],np.eye(4))
        # Rest tip on joint b's axis inherits joint a's rotation.
        np.testing.assert_allclose(result['tip']@np.array([0,10,0,1]),[0,0,10,1],atol=1e-8)
    def test_coarse_grid_preserves_a_real_shell(self):
        mask=np.ones((10,10,10),dtype=bool)
        inside,wall=interior_mask(mask,3.2,3.)
        self.assertEqual(wall,3.2);self.assertFalse(inside[0].any());self.assertTrue(inside[1:-1,1:-1,1:-1].all())
        self.assertGreater(np.count_nonzero(mask&~inside),0)
    def test_structural_path_routes_around_obstacle_and_rejects_disconnected_domain(self):
        free=np.ones((12,12,3),dtype=bool);free[5,:,1]=False;free[5,10,1]=True
        path,_=astar(free,[[1,1,1]],[[10,1,1]])
        self.assertIsNotNone(path);self.assertTrue(all(free[tuple(p)] for p in path))
        free[5,:,:]=False
        path,_=astar(free,[[1,1,1]],[[10,1,1]])
        self.assertIsNone(path)
    def test_storage_aliases_are_repaired_locally_and_reload_closed(self):
        from pathlib import Path
        data=np.load(Path(__file__).parent/'fixtures/stl_precision/closed_body_aliases.npz')
        mesh=tm.Trimesh(data['vertices'],data['faces'],process=False)
        vertices=mesh.vertices.copy();faces=mesh.faces.copy()
        deliverable,report=stl_deliverable(mesh)
        self.assertTrue(report['passed'])
        self.assertEqual(report['method'],'local_quantization_separation')
        self.assertLessEqual(report['attempts'][-1]['maximum_vertex_displacement_mm'],.001)
        self.assertLess(report['attempts'][-1]['volume_change_mm3'],mesh.volume*1e-6)
        reopened=tm.load(io.BytesIO(deliverable.export(file_type='stl')),file_type='stl',force='mesh')
        self.assertTrue(reopened.is_watertight and reopened.is_volume)
        self.assertTrue(np.array_equal(mesh.vertices,vertices));self.assertTrue(np.array_equal(mesh.faces,faces))

    def test_si_export_keeps_closed_mesh_and_bounded_roundoff(self):
        from pathlib import Path
        data=np.load(Path(__file__).parent/'fixtures/stl_precision/si_export.npz')
        for key in ['alias','roundoff']:
            mesh=tm.Trimesh(data[key+'_vertices'],data[key+'_faces'],process=False)
            vertices=mesh.vertices.copy()
            delivered,report=stl_deliverable(mesh,millimeters_per_unit=1000.)
            self.assertTrue(report['passed'],report)
            reopened=tm.load(io.BytesIO(delivered.export(file_type='stl')),file_type='stl',force='mesh')
            self.assertTrue(reopened.is_volume and reopened.is_watertight)
            self.assertLessEqual(report['attempts'][-1]['maximum_vertex_displacement_mm'],.001)
            self.assertLessEqual(abs(reopened.volume-mesh.volume),max(1e-12,abs(mesh.volume)*1e-5))
            self.assertTrue(np.array_equal(mesh.vertices,vertices))

    def test_open_cradle_boolean_edges_and_aliases_are_repaired_together(self):
        data=np.load(Path(__file__).parent/'fixtures/stl_precision/open_cradle_boolean.npz')
        mesh=tm.Trimesh(data['vertices'],data['faces'],process=False)
        original=mesh.vertices.copy();delivered,report=stl_deliverable(mesh)
        self.assertTrue(report['passed'],report)
        reopened=tm.load(io.BytesIO(delivered.export(file_type='stl')),file_type='stl',force='mesh')
        self.assertTrue(reopened.is_watertight and reopened.is_volume)
        self.assertLess(abs(reopened.volume-mesh.volume),mesh.volume*1e-6)
        self.assertLessEqual(report['attempts'][-1]['maximum_vertex_displacement_mm'],.001)
        self.assertTrue(np.array_equal(mesh.vertices,original))

    def test_large_open_surface_is_not_silently_approved(self):
        mesh=tm.creation.box([40,30,20]);mesh.update_faces(np.arange(len(mesh.faces)-2))
        with self.assertRaises(ValueError):stl_deliverable(mesh)

    def test_stl_cleanup_preserves_volume_and_real_closed_surface(self):
        mesh=tm.creation.box([40,30,20]);original_volume=mesh.volume
        mesh.faces=np.vstack([mesh.faces,[0,0,1]])
        deliverable,report=stl_deliverable(mesh)
        self.assertTrue(report['passed']);self.assertTrue(deliverable.is_watertight)
        self.assertAlmostEqual(deliverable.volume,original_volume)
        self.assertFalse(report['source_model_modified'])


class CavityComponentsTests(unittest.TestCase):
    def test_thin_shell_grouping_preserves_original_faces_and_volume(self):
        from pathlib import Path
        data=np.load(Path(__file__).parent/'fixtures/volume_grouping/thin_shell.npz')
        original=tm.Trimesh(data['vertices'],data['faces'],process=False)
        components=volume_components(original)
        self.assertEqual(len(components),5)
        self.assertEqual(sum(len(c.faces) for c in components),len(original.faces))
        self.assertAlmostEqual(sum(float(c.volume) for c in components),float(original.volume),places=7)
        self.assertTrue(all(c.is_volume and c.is_watertight for c in components))
        def rows(mesh):return sorted(map(tuple,np.asarray(mesh.triangles).reshape(-1,9)))
        self.assertEqual(rows(tm.util.concatenate(components)),rows(original))

    def test_closed_void_stays_with_outer_shell_without_filling(self):
        outer=tm.creation.box([20,20,20]);inner=tm.creation.box([10,10,10]);inner.invert()
        shell=tm.util.concatenate([outer,inner]);separate=tm.creation.box([5,5,5]).apply_translation([30,0,0])
        components=volume_components(tm.util.concatenate([shell,separate]))
        self.assertEqual(len(components),2)
        self.assertTrue(all(m.is_volume for m in components))
        self.assertAlmostEqual(sum(m.volume for m in components),7125)
        combined=union(*components)
        self.assertAlmostEqual(overlap(combined,tm.creation.box([6,6,6])),0)
        # A single hollow solid must not be handed to A* as two objects.
        unchanged,report=reconnect(shell,None,None,[],3)
        self.assertIs(unchanged,shell);self.assertFalse(report['required'])
    def test_unenclosed_negative_shell_is_rejected(self):
        inner=tm.creation.box([5,5,5]).apply_translation([30,0,0]);inner.invert()
        with self.assertRaises(ValueError):volume_components(tm.util.concatenate([tm.creation.box([20,20,20]),inner]))


if __name__=='__main__':unittest.main(verbosity=2)
