import unittest
import numpy as np
from studio.mechanics.solid import box,cut,overlap,regions
from studio.mechanics.assembly_split import split_shell

class ManualExtractionTests(unittest.TestCase):
 def test_blocked_translation_remains_unverified_in_manual_mode(self):
  outer=box([-25,-20,-20],[25,20,20]);shell=cut(outer,box([-22,-17,-17],[22,17,17]))
  obstacle=box([-3,24,-3],[3,28,3])
  base,cap,panel=split_shell(shell,outer,{'normal':[0,1,0]},[obstacle],[],3,[],manual_extraction_review=True)
  np.testing.assert_allclose(panel['normal'],[0,1,0])
  self.assertFalse(panel['continuous_extraction'])
  self.assertTrue(panel['manual_extraction_review_required'])
  self.assertGreater(max(panel['extraction_overlap_mm3']),.1)
  self.assertLess(overlap(cap,obstacle),.01)
  self.assertEqual(regions(cap),1)
  self.assertLess(overlap(base,cap),.01)
  _,_,strict=split_shell(shell,outer,{'normal':[0,1,0]},[obstacle],[],3,[])
  self.assertTrue(strict['continuous_extraction'])
  self.assertFalse(strict['manual_extraction_review_required'])

if __name__=='__main__':unittest.main()
