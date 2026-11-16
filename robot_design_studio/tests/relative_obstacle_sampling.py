"""Exact pose deduplication preserves every branch and material removal."""
import unittest
import numpy as np
from studio.mechanics.graph_assembly import poses,sample_angles,unique_relative_obstacles
from studio.mechanics.solid import box,cut
class RelativeSamplingTests(unittest.TestCase):
 def graph(self):
  return {'root_body_id':'body','joints':[{'id':f'j{i}','parent':'body','child':f'leg{i}','axis':[0,1,0],'position_mm':[i*10,0,0],'limits_deg':[-5,5]} for i in range(4)]}
 def test_exact_coverage_and_reduction(self):
  samples=[poses(self.graph(),a) for a in sample_angles(self.graph())]
  for owner in samples[0]:
   raw=[(other,(np.linalg.inv(m[owner])@matrix).tobytes()) for m in samples for other,matrix in m.items() if other!=owner]
   unique=[(other,matrix.tobytes()) for other,matrix in unique_relative_obstacles(owner,samples)]
   self.assertEqual(set(raw),set(unique));self.assertEqual(len(unique),len(set(unique)))
   self.assertLess(len(unique),len(raw))
 def test_nested_tree_preserves_coverage(self):
  graph=self.graph();graph['joints'][1]['parent']='leg0'
  samples=[poses(graph,a) for a in sample_angles(graph)]
  for owner in samples[0]:
   raw={(other,(np.linalg.inv(m[owner])@matrix).tobytes()) for m in samples for other,matrix in m.items() if other!=owner}
   self.assertEqual(raw,{(other,matrix.tobytes()) for other,matrix in unique_relative_obstacles(owner,samples)})
 def test_distinct_transforms_are_preserved(self):
  a={'body':np.eye(4),'leg':np.eye(4)};b={k:v.copy() for k,v in a.items()};b['leg'][0,3]=1e-12
  self.assertEqual(len(list(unique_relative_obstacles('body',[a,b,a]))),2)
 def test_subtractions_preserve_volume(self):
  samples=[poses(self.graph(),a) for a in sample_angles(self.graph())]
  source=box([-4,-4,-4],[4,4,4]);obstacle=box([1,-5,-5],[3,5,5]);raw=source.copy();dedup=source.copy()
  for m in samples:
   for other,matrix in m.items():
    if other!='body':raw=cut(raw,obstacle.copy().apply_transform(matrix))
  for other,matrix in unique_relative_obstacles('body',samples):dedup=cut(dedup,obstacle.copy().apply_transform(matrix))
  self.assertTrue(raw.is_volume and dedup.is_volume);self.assertAlmostEqual(raw.volume,dedup.volume,places=7)
if __name__=='__main__':unittest.main()
