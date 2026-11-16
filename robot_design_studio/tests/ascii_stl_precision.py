"""Actual cactus topology must survive storage without changing the surface."""
import unittest,tempfile
from pathlib import Path
import numpy as np,trimesh as tm
from studio.mechanics.graph_assembly import stl_deliverable
class AsciiStlPrecision(unittest.TestCase):
 def test_actual_failed_binary_mesh_preserves_surface_in_ascii(self):
  d=np.load(Path(__file__).parent/'fixtures/cactus_precision/right_arm.npz');m=tm.Trimesh(d['vertices'],d['faces'],process=False);original=m.vertices.copy();n,r=stl_deliverable(m)
  self.assertTrue(r['passed']);self.assertEqual(r.get('stl_encoding'),'ascii')
  with tempfile.TemporaryDirectory() as folder:
   path=Path(folder)/'arm.stl';n.export(path,file_type='stl_ascii');reopened=tm.load(path,force='mesh')
   self.assertTrue(reopened.is_volume and reopened.is_watertight);self.assertAlmostEqual(m.volume,reopened.volume,places=7)
  np.testing.assert_array_equal(m.vertices,original)
if __name__=='__main__':unittest.main()
