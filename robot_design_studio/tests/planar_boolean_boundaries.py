"""Conservative cleanup of closed planar Boolean tangencies."""
import unittest
import numpy as np
import trimesh as tm
from studio.mechanics.solid import remove_planar_boolean_boundaries, volume_components, ComponentGroupingError
class PlanarBoundaryTests(unittest.TestCase):
 def sheet(self):
  return tm.Trimesh(vertices=[[0,0,0],[1,0,0],[0,1,0],[.2,.2,0]],faces=[[0,2,1],[0,1,3],[1,2,3],[2,0,3]],process=False)
 def test_zero_volume_planar_boundary(self):
  source=tm.creation.box([10,10,10]);cleaned,records=remove_planar_boolean_boundaries(tm.util.concatenate([source,self.sheet()]))
  self.assertEqual(len(records),1);self.assertAlmostEqual(cleaned.volume,source.volume,places=8);self.assertEqual(len(volume_components(cleaned)),1)
 def test_thin_real_material_preserved(self):
  source=tm.creation.box([1,1,1e-6]);cleaned,records=remove_planar_boolean_boundaries(source)
  self.assertFalse(records);np.testing.assert_array_equal(cleaned.faces,source.faces)
 def test_multiple_real_sources_keep_material_and_clean_only_planar_boundary(self):
  from studio.mechanics.shell_access import separate_voxel_artifacts
  a=tm.creation.box([10,10,10]);b=tm.creation.box([2,2,2]).apply_translation([20,0,0])
  clean,removed,report=separate_voxel_artifacts(tm.util.concatenate([a,b,self.sheet()]),tm.util.concatenate([a,b]),[],1,2)
  self.assertFalse(removed);self.assertEqual(len(report['planar_boundary_cleanup']),1)
  self.assertEqual(len(volume_components(clean)),2);self.assertAlmostEqual(clean.volume,a.volume+b.volume,places=8)
 def test_open_surface_rejected(self):
  source=self.sheet();source.update_faces([True,True,True,False]);cleaned,records=remove_planar_boolean_boundaries(source)
  self.assertFalse(records)
  with self.assertRaises(ComponentGroupingError):volume_components(cleaned)
if __name__=='__main__':unittest.main()
