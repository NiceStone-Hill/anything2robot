import hashlib,json,tempfile,unittest,zipfile,io
from pathlib import Path
import trimesh as tm
from studio.mechanics.print_package import generate
class PrintPackageTests(unittest.TestCase):
 def test_only_verified_printables_and_on_bed(self):
  with tempfile.TemporaryDirectory() as temp:
   folder=Path(temp);raw=tm.creation.box([20,30,10]).export(file_type='stl');(folder/'body.stl').write_bytes(raw)
   a={'id':'body','name':'body','file':'body.stl','sha256':hashlib.sha256(raw).hexdigest(),'printable_candidate':True}
   r={'project_id':'test','design_revision':1,'mechanical_run_id':'run','layout_valid':True,'structure_verified':False,'checks':[],
      'geometry_assets':[a,{**a,'id':'board','printable_candidate':False}],'assembly_dependencies':[]}
   self.assertIsNone(generate(r,folder));r['structure_verified']=True
   result=generate(r,folder);self.assertEqual(result['part_count'],1)
   with zipfile.ZipFile(folder/result['file']) as z:
    mesh=tm.load(io.BytesIO(z.read('STL/body.stl')),file_type='stl')
    self.assertTrue(mesh.is_volume);self.assertAlmostEqual(mesh.bounds[0,2],0)
    self.assertFalse(json.loads(z.read('manifest.json'))['manufacturing_ready'])
   r['checks']=[{'name':'collision','passed':False}];self.assertIsNone(generate(r,folder))
if __name__=='__main__':unittest.main()
