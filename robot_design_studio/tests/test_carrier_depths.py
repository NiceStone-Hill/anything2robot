import unittest
import numpy as np
from studio.mechanics.electronics_layout import carrier_depths
class DepthTests(unittest.TestCase):
 def test_offset_split_cavity(self):
  bounds=np.array([[-40,-30,-50],[40,30,52]])
  for side in [-1,1]:
   zs=carrier_depths(bounds,18,side)
   self.assertTrue(any(z>20 for z in zs))
   for z in zs:
    ends=[z,z+side*20.5]
    self.assertGreaterEqual(min(ends),-48-1e-8)
    self.assertLessEqual(max(ends),50+1e-8)
 def test_translation_and_too_shallow(self):
  a=np.array([[-10,-10,-20],[10,10,20]])
  for side in [-1,1]:
   self.assertTrue(np.allclose(np.array(carrier_depths(a+[0,0,97],10,side)),np.array(carrier_depths(a,10,side))+97))
   self.assertEqual(carrier_depths(a,40,side),[])
if __name__ == '__main__': unittest.main()
