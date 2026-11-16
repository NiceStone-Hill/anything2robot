"""Hardware offsets and real mounting interfaces, independent physical probes."""
import unittest
import numpy as np
import trimesh as tm
from studio.mechanics.joint_modules import module
from studio.mechanics.solid import dilate,box,cylinder,overlap,cut

class ClearanceTests(unittest.TestCase):
    def test_offset_preserves_concavity_and_physical_gap(self):
        from studio.mechanics.solid import union
        l=union(box([0,0,0],[20,4,5]),box([0,0,0],[4,20,5]))
        expanded=dilate(l,.4)
        self.assertLess(overlap(expanded,box([8,8,1],[15,15,4])),.001)
        self.assertLess(float(cut(l,expanded).volume),.001)
        growth=expanded.bounds-l.bounds
        self.assertTrue(np.all(np.abs(growth)>=.399))
        self.assertTrue(np.all(np.abs(growth)<.505))
    def test_sg90_mounting_ear_screws_and_bearing_faces(self):
        built=module('sg90_digital_towerpro');p={p[0]:p[3] for p in built['parts']}
        # CAD slots measured at 27.2 mm pitch, not invented as official SG90 data.
        for x in [-18.95,8.25]:
            self.assertLess(overlap(cylinder(x,0,-18,-11,.85),p['cartridge']),.001)
            self.assertLess(overlap(cylinder(x,0,-14,-11.5,.85),p['motor']),.001)
            seat=cut(cylinder(x,0,-14.1,-14.01,2.5),cylinder(x,0,-15,-13,1.1))
            self.assertGreater(overlap(seat,p['cartridge']),.5)
        self.assertLess(overlap(p['motor'],p['cartridge']),.01)
        # Concave module space is materially smaller than its enclosing box.
        self.assertLess(built['cavity'].volume,np.prod(built['cavity'].extents)*.8)
        self.assertIsNone(built['template']['reference_cad']['actual_dimensions_mm'])

if __name__=='__main__':unittest.main(verbosity=2)
