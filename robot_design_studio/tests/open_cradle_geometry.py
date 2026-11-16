"""Real solids: external mounts, installation order, and unchanged round policy."""
import json
import unittest
import numpy as np
from studio.mechanics.joint_modules import module, TEMPLATE_PATH
from studio.mechanics.joint_placement import placement_metrics, room_fits
from studio.mechanics.solid import box, cylinder, overlap, linear_sweep, regions
from studio.mechanics.shell_access import access_requirement
from studio.mechanics.articulation import compare_motors


class OpenCradleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.built=module('sg90_digital_towerpro')
        cls.parts={p[0]:p[3] for p in cls.built['parts']}
        cls.design=json.loads(TEMPLATE_PATH.read_text())['design']

    def test_open_plates_are_connected_and_keep_real_ear_holes(self):
        self.assertEqual(set(self.parts),{'fixed_mount','retainer','output_mount','motor','rotor'})
        for name,mesh in self.parts.items():
            self.assertTrue(mesh.is_watertight and mesh.is_volume,name)
            self.assertEqual(regions(mesh),1,name)
        for i,(name,a) in enumerate(self.parts.items()):
            for other,b in list(self.parts.items())[i+1:]:
                self.assertLess(overlap(a,b),.05,(name,other))
        # Independent dimensions from the registered CAD's two ear slots.
        for x in [-18.95,8.25]:
            probe=cylinder(x,0,-18,-7,.8)
            for role in ['fixed_mount','retainer']:
                self.assertLess(overlap(probe,self.parts[role]),.01)
                self.assertGreater(overlap(cylinder(x,0,-18,-7,2.8),self.parts[role]),30)
        # The motor's long sides and bottom have no enclosing box.
        for role in ['fixed_mount','retainer']:
            self.assertLess(overlap(box([-14,-5,-29],[4,5,-19]),self.parts[role]),.01)
            self.assertLess(self.parts[role].extents[1],13)
        self.assertLess(self.built['cavity'].extents[0],36)
        self.assertLess(self.built['cavity'].extents[1],22)

    def test_motor_side_entry_and_plate_top_entry_with_real_blockers(self):
        for role,direction,blockers in [('motor',[0,1,0],['fixed_mount']),
                                         ('retainer',[0,-1,0],['fixed_mount','motor']),
                                         ('retainer',[0,0,1],['fixed_mount','motor'])]:
            swept=linear_sweep(self.parts[role],direction,60)
            for blocker in blockers:
                self.assertLess(overlap(swept,self.parts[blocker]),.05,(role,blocker))
        # A deliberately obstructed side route must not be declared clear.
        swept=linear_sweep(self.parts['motor'],[0,1,0],60)
        obstruction=box([-14,20,-25],[3,23,-18])
        self.assertGreater(overlap(swept,obstruction),100)
        self.assertEqual(self.built['installation_steps'][0]['removed_roles'],['retainer','rotor','output_mount'])
        # Sliding the U-shaped plate toward the open front would drive its
        # rear bar through the motor; only the registered rear route is valid.
        self.assertGreater(overlap(linear_sweep(self.parts['retainer'],[0,1,0],60),self.parts['motor']),100)

    def test_exposure_is_allowed_but_far_away_mounts_are_rejected(self):
        shape=box([-50,-5,-40],[50,5,20])
        metrics=placement_metrics(self.built['cavity'],shape,True)
        self.assertGreater(metrics['outside_fraction'],.03)
        self.assertTrue(room_fits(metrics,self.built,self.design))
        far=self.built['cavity'].copy().apply_translation([0,100,0])
        self.assertFalse(room_fits(placement_metrics(far,shape,True),self.built,self.design))
        round_module=module('mg4005v2_candidate')
        self.assertFalse(round_module['allow_exterior'])
        self.assertFalse(room_fits(placement_metrics(round_module['cavity'],shape),round_module,self.design))
        self.assertEqual(round_module['installation_steps'][0]['direction'],[0,0,1])

    def test_recommendation_does_not_call_exposed_servo_insufficient_space(self):
        shape=box([-50,-5,-40],[50,5,20])
        graph={'root_body_id':'base','bodies':[{'id':'base','seed_mm':[-25,0,0]},{'id':'arm','seed_mm':[25,0,0]}],
               'joints':[{'id':'hinge','name':'hinge','parent':'base','child':'arm','axis':[0,0,1],'position_mm':[0,0,0]}]}
        choices={c['catalog_id']:c for c in compare_motors(shape,graph)[0]['choices']}
        square=choices['sg90_digital_towerpro']
        self.assertGreater(square['minimum_outside_fraction'],.03)
        self.assertTrue(square['preliminary_space_fit'])
        self.assertEqual(square['fit_policy'],'bounded_external_support')
        self.assertFalse(choices['mg4005v2_candidate']['preliminary_space_fit'])

    def test_external_motor_does_not_force_an_internal_access_panel(self):
        graph={'root_body_id':'base','joints':[{'id':'a','parent':'base','child':'arm'},{'id':'b','parent':'arm','child':'tip'}]}
        modules=[{'owner':'arm','joint_id':'b','role':r,'external_service':True} for r in ['motor','fixed_mount','retainer']]
        self.assertFalse(access_requirement('arm',graph,modules,{})['required'])
        self.assertTrue(access_requirement('base',graph,modules,{})['required'])
        for key in ['component_placements','service_opening_requirements']:
            self.assertTrue(access_requirement('arm',graph,modules,{key:[{'owner_body_id':'arm'}]})['required'])
        for item in modules:item['external_service']=False
        self.assertTrue(access_requirement('arm',graph,modules,{})['required'])


if __name__=='__main__':unittest.main(verbosity=2)
