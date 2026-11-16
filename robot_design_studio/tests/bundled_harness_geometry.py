"""Shared routing must retain electrical identities and reserve actual bundle capacity."""
import tempfile,unittest
from pathlib import Path
import numpy as np
from studio.mechanics import bundle_network as network,component_visuals

class BundledHarnessGeometryTests(unittest.TestCase):
    def test_distinct_wires_share_a_corridor_without_becoming_one_net(self):
        paths={'power':[0,1,2,3],'signal':[4,1,2,5],'ground':[0,1,4]}
        segments=network.shared_segments(paths,{0,3,4,5})
        for i,s in enumerate(segments):s['id']=str(i)
        trunk=next(s for s in segments if s['nodes']==[1,2]);self.assertEqual(trunk['cables'],['power','signal'])
        for wire,path in paths.items():
            sequence=network.cable_segment_path(segments,wire,path[0],path[-1]);restored=[]
            for item in sequence:
                segment=next(s for s in segments if s['id']==item['segment_id']);values=segment['nodes'][::-1] if item['reverse'] else segment['nodes']
                if restored:self.assertEqual(restored[-1],values[0])
                restored+=values if not restored else values[1:]
            self.assertEqual(restored,path)
        with self.assertRaises(ValueError):network.cable_segment_path(segments,'unregistered',0,3)
    def test_packing_reserves_every_insulated_wire_and_grows_with_diameter(self):
        radius,circles=network.pack((1.2,)*11,.2,.5)
        self.assertEqual(len(circles),11)
        for i,(x,y,r) in enumerate(circles):
            self.assertLessEqual(np.hypot(x,y)+r+.5,radius+1e-8)
            for a,b,s in circles[:i]:self.assertGreaterEqual(np.hypot(x-a,y-b),r+s+.2-1e-8)
        self.assertGreater(network.pack((1.4,)*11,.2,.5)[0],radius)
        with self.assertRaises(ValueError):network.pack((0.,),.2,.5)
    def test_growing_tree_connects_devices_around_a_blocked_wall(self):
        free=np.ones((16,12,5),bool);free[7,2:10,:]=False;clearance=np.ones(free.shape)*10
        cells,ids,_,_=network.rooted_grid_tree(free,clearance,(1,6,2))
        terminals=[int(ids[p]) for p in [(1,6,2),(14,6,2),(14,2,2)]]
        parent=network.growing_tree(free,clearance,set(terminals),terminals[0])
        for end in terminals[1:]:
            path=network.tree_path(terminals[0],end,parent)
            self.assertTrue(free[tuple(cells[path].T)].all())
        free[7,:,:]=False;cells,ids,_,_=network.rooted_grid_tree(free,clearance,(1,6,2))
        with self.assertRaises(ValueError):network.growing_tree(free,clearance,{int(ids[1,6,2]),int(ids[14,6,2])},int(ids[1,6,2]))

class ReferenceCadTests(unittest.TestCase):
    def placement(self,catalog='pico2_unknown',dims=None):
        matrix=np.eye(4);matrix[:3,3]=[.1,.2,.3]
        return {'instance_id':'test_board','catalog_id':catalog,'owner_body_id':'body','envelope_asset_id':'test_envelope','envelope_dimensions_m':dims or [.055,.025,.012],'transform_m':matrix.tolist()}
    def test_reference_models_stay_full_size_and_inside_the_assumed_envelope(self):
        for catalog,dims in [('pico2_unknown',[.055,.025,.012]),('mpu6050_lab',[.025,.02,.01])]:
            with tempfile.TemporaryDirectory() as temp:
                assets,reports=component_visuals.generate([self.placement(catalog,dims)],temp)
                self.assertEqual(len(assets),1);self.assertEqual(assets[0]['scale_factor'],1.)
                self.assertFalse(assets[0]['physical_variant_verified'])
                mesh=component_visuals.load(temp,assets[0]);native=__import__('trimesh').load(component_visuals.ROOT/assets[0]['model_id']/'model.ply',force='mesh').apply_transform(assets[0]['source_to_world_mm']);np.testing.assert_allclose(native.bounds,mesh.bounds,atol=2e-5);lo=np.array([100,200,300])-np.array(dims)*500;hi=np.array([100,200,300])+np.array(dims)*500
                self.assertTrue(np.all(mesh.bounds[0]>=lo-1e-5));self.assertTrue(np.all(mesh.bounds[1]<=hi+1e-5))
                if catalog.startswith('pico'):self.assertAlmostEqual(mesh.extents[0],52.3,places=4)
                self.assertTrue(component_visuals.preview_parts(temp,assets))
    def test_small_envelope_is_reported_instead_of_scaling_a_board(self):
        with tempfile.TemporaryDirectory() as temp:
            assets,reports=component_visuals.generate([self.placement(dims=[.02,.02,.002])],temp)
            self.assertEqual(assets,[]);self.assertEqual(reports[0]['status'],'model_outside_envelope')
    def test_unknown_power_module_keeps_an_envelope(self):
        with tempfile.TemporaryDirectory() as temp:
            assets,reports=component_visuals.generate([self.placement('supply_5_candidate')],temp)
            self.assertEqual(assets,[]);self.assertEqual(reports[0]['status'],'envelope_only')

if __name__=='__main__':unittest.main()
