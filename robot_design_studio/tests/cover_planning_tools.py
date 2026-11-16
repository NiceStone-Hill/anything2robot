"""External screw/tool obstacles must reject a cover during planning."""
import unittest
import numpy as np
import trimesh as tm
from studio.mechanics.shell_access import cover_tools_clear, cover_seats_supported
from studio.mechanics.solid import box
class CoverPlanningTools(unittest.TestCase):
 def test_external_body_blocks_driver(self):
  fasteners=[{'position_mm':[0,0,0]}];pose=np.eye(4)
  self.assertTrue(cover_tools_clear(fasteners,pose,0,8,[box([5,5,10],[7,7,15])]))
  self.assertFalse(cover_tools_clear(fasteners,pose,0,8,[box([-1,-1,10],[1,1,15])]))
 def test_tapered_cover_edge_cannot_support_a_screw(self):
  base=box([-4,-4,-3],[4,4,0]);cap=box([-4,-4,0],[4,4,2])
  self.assertTrue(cover_seats_supported(cap,base,0,0,0))
  edge=box([3.39,-4,0],[4,4,2])
  self.assertFalse(cover_seats_supported(edge,base,0,0,0))
  self.assertFalse(cover_seats_supported(cap,box([3.39,-4,-3],[4,4,0]),0,0,0))
 def test_probe_rotates_with_cover_and_checks_shaft(self):
  pose=tm.transformations.rotation_matrix(np.pi/2,[1,0,0]);pose[:3,3]=[11,23,19]
  fasteners=[{'position_mm':tm.transform_points([[0,0,0]],pose)[0].tolist()}]
  obstacle=box([-.5,-.5,-4],[.5,.5,-3]).apply_transform(pose)
  self.assertFalse(cover_tools_clear(fasteners,pose,0,8,[obstacle]))
  self.assertTrue(cover_tools_clear(fasteners,pose,0,8,[]))
if __name__=='__main__':unittest.main()
