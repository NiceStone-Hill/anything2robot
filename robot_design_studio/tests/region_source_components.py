"""Remote source decorations must not disable safe owner-scoped cleanup."""
import unittest
from studio.mechanics.graph_assembly import region_source_component_count
class RegionSourceComponents(unittest.TestCase):
 def test_remote_tip_is_excluded_from_base_only(self):
  main=[[-10,-10,-20],[10,10,20]];tip=[[-1,-1,21],[1,1,23]]
  self.assertEqual(region_source_component_count([[-11,-11,-21],[11,11,-5]],[main,tip]),1)
  self.assertEqual(region_source_component_count([[-11,-11,-5],[11,11,24]],[main,tip]),2)
 def test_overlapping_bounds_disable_cleanup_conservatively(self):
  self.assertEqual(region_source_component_count([[-2,-2,-2],[2,2,2]],[[[-3,-3,-3],[3,3,3]],[[-1,-1,-1],[1,1,1]]]),2)
 def test_no_source_overlap_never_enables_cleanup(self):
  self.assertEqual(region_source_component_count([[5,5,5],[6,6,6]],[[[-1,-1,-1],[1,1,1]]]),0)
if __name__=='__main__':unittest.main()
