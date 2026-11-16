import copy
import json
import unittest
from studio.mechanics.joint_modules import module,TEMPLATE_PATH
from studio.mechanics.horn_interface import registration
from studio.mechanics.solid import cylinder,overlap

class HornTests(unittest.TestCase):
    def test_cross_and_bar_slots_and_tools_share_axis(self):
        for shape,width,span in [('cross',6.,9.),('bar',5.,10.)]:
            library=json.loads(TEMPLATE_PATH.read_text());t=library['templates']['sg90_digital_towerpro']
            t['horn_profile']['shape']=shape;t['horn_profile']['arm_width_mm']=width;t['horn_radius']=span
            built=module('sg90_digital_towerpro',library)
            parts={p[0]:p[3] for p in built['parts']};out=parts['output_mount'];horn=parts['rotor']
            self.assertTrue(out.is_watertight);self.assertTrue(horn.is_watertight)
            sites=registration(t)
            tools=[s['tools'][0] for s in built['fastening_interfaces'] if s['id'].startswith('horn_adaptor_')]
            for site,tool in zip(sites,tools):
                x,y,z=site['point_mm'];self.assertEqual(tool['point_mm'][:2],[x,y])
                self.assertLess(overlap(out,cylinder(x,y,z-.1,z+library['design']['wall']+.1,.9)),1e-4)
                self.assertLess(overlap(horn,cylinder(x,y,-1,z+1,.5)),1e-4)
                self.assertGreater(overlap(horn,cylinder(x,y,0,z,1.5)),1)
            self.assertGreater(next(s['mesh'].volume for s in built['mounting_surfaces'] if s['id']=='horn_seat'),1)
    def test_reject_slot_outside_actual_arm(self):
        t=json.loads(TEMPLATE_PATH.read_text())['templates']['sg90_digital_towerpro']
        t['horn_profile']['arm_width_mm']=1.5
        with self.assertRaises(ValueError):registration(t)

if __name__=='__main__':unittest.main()
