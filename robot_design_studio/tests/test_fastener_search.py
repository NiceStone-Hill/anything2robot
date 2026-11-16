import copy
import unittest
import numpy as np
from studio.mechanics.solid import box,cylinder,cut,union,overlap
from studio.mechanics.fastener_search import packed,ordered_steps,sequence
from studio.mechanics.cover_fasteners import finish

class FastenerSearchTests(unittest.TestCase):
    def scene(self,shift=(0,0,0),half_width=30):
        frame=np.eye(4);frame[:3,3]=shift
        stock_a=box([-half_width,-10,-5],[half_width,10,-.1]);stock_b=box([-half_width,-10,0],[half_width,10,4])
        shaft=cylinder(0,0,-6,34,1.7);head=cylinder(0,0,1.5,34,3.2);nut=cylinder(0,0,-3.1,.1,3.35,6)
        a=cut(stock_a,shaft,nut);b=cut(stock_b,shaft,head)
        f={'position_mm':list(shift),'candidate_thread':'M3','restore_cut_material':{
            'base':packed(cut(stock_a,a).apply_transform(frame)), 'cap':packed(cut(stock_b,b).apply_transform(frame))}}
        panel={'owner_body_id':'body','cover_asset_id':'body_lid','transform_mm':frame.tolist(),
               'plane_z_mm':0,'top_local_mm':4,'fasteners':[f],'fastener_candidate_xy_mm':[[0,0],[12,0],[-12,0]]}
        parts=[{'record':{'id':i,'owner_body_id':o},'mesh':m.apply_transform(frame)} for i,o,m in [
            ('body_shell','body',a),('body_lid','body',b),('obstacle','neighbor',box([-4,-4,8],[4,4,12]))]]
        return parts,{'lid_planes':[panel]},{'body':box([-half_width,-10,-5],[half_width,10,4]).apply_transform(frame)}

    def test_moves_shared_axis_restores_old_hole_in_translated_models(self):
        for shift,width in [((0,0,0),30),((40,-25,80),23)]:
            parts,analysis,domains=self.scene(shift,width)
            records=finish(parts,analysis,overlap,domains=domains)
            self.assertTrue(records[0]['passed'])
            self.assertEqual(analysis['fastener_search']['relocations'][0]['status'],'relocated')
            self.assertGreater(abs(records[0]['shared_hole_center_world_mm'][0]-shift[0]),9)
            former=box([-.5,-.5,.2],[.5,.5,1.2]).apply_translation(shift)
            self.assertAlmostEqual(overlap(former,parts[1]['mesh']),former.volume,places=5)

    def test_no_stock_does_not_invent_hole_filling(self):
        parts,analysis,domains=self.scene();analysis['lid_planes'][0]['fasteners'][0].pop('restore_cut_material')
        records=finish(parts,analysis,overlap,domains=domains)
        self.assertFalse(records[0]['passed'])
        self.assertEqual(analysis['fastener_search']['relocations'][0]['status'],'no_registered_restoration_stock')

    def test_exhausted_search_keeps_geometry_and_failure(self):
        parts,analysis,domains=self.scene()
        parts[2]['mesh']=box([-28,-8,8],[28,8,12])
        volumes=[p['mesh'].volume for p in parts]
        r=finish(parts,analysis,overlap,domains=domains)
        self.assertFalse(r[0]['passed'])
        self.assertEqual(analysis['fastener_search']['relocations'][0]['status'],'no_feasible_relocation')
        self.assertTrue(np.allclose(volumes,[p['mesh'].volume for p in parts]))

    def test_sequence_preserves_dependencies_and_rejects_cycle(self):
        self.assertIsNone(ordered_steps([{'id':'a','depends_on':['b']},{'id':'b'}],[('a','b')]))
        self.assertIsNone(ordered_steps([{'id':'a','depends_on':['missing']}],[]))
        report={'passed':False,'cover_asset_id':'a','tool_blocked_assets':['b'],'shaft_blocked_assets':['b'],
                'installed_screw_blocked_assets':[],'shaft_obstruction_mm3':0,'head_obstruction_mm3':0,
                'cover_bearing_mm3':10,'mate_bearing_mm3':10}
        panels=[{'owner_body_id':x,'cover_asset_id':x} for x in ['a','b']]
        for deps,expected in [([],True),(['b'],False)]:
            r=copy.deepcopy(report);sequence([r],panels,[{'id':'a','depends_on':deps},{'id':'b'}])
            self.assertEqual(r['passed'],expected)
        r=copy.deepcopy(report);r['installed_screw_blocked_assets']=['b']
        sequence([r],panels,[{'id':'a'},{'id':'b'}]);self.assertFalse(r['passed'])

if __name__=='__main__':unittest.main()
