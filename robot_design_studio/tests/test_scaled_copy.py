import hashlib
import tempfile
import unittest
from pathlib import Path
from studio.catalog import Catalog
from studio.project import Store
from studio.articulation_service import clone_at_height
from studio.contracts import ContractError


class ScaledCopyTests(unittest.TestCase):
    def test_height_uses_z_extent_and_keeps_original_and_hardware_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            store=Store(temp,Catalog());p=store.create('Arbitrary source')
            source=store.directory(p['id'])/'sources';source.mkdir();raw=b'original source bytes';(source/'test.stl').write_bytes(raw)
            p['source_shapes']=[{'id':'shape_test','name':'test.stl','file':'test.stl','format':'stl','unit':'mm','sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)}]
            p['articulation_config']={'length_m':.6,'up_axis':'+Z','mode':'reference','support':'fixed_base','max_joints':1,'intent':'test'}
            p['articulation']={'analysis':{'bounds_mm':[[-300,-100,-250],[300,100,250]]},'source':{'kind':'test'},'run_id':'run_original',
                'graph':{'bodies':[{'id':'body_a','seed_mm':[0,0,100]}],'joints':[{'id':'joint_a','position_mm':[50,0,0],'axis':[0,1,0]}]}}
            p['motor_bindings']=[{'joint_id':'joint_a','catalog_id':'sg90_digital_towerpro'}];store._save(p)
            before=(store.directory(p['id'])/'project.json').read_bytes()
            new,config,recovery=clone_at_height(store,p['id'],{'revision':p['revision'],'name':'Resized','height_m':.2})
            self.assertEqual(before,(store.directory(p['id'])/'project.json').read_bytes())
            self.assertAlmostEqual(config['length_m'],.24)
            self.assertEqual(recovery['graph']['joints'][0]['position_mm'],[20,0,0])
            self.assertEqual(recovery['graph']['joints'][0]['axis'],[0,1,0])
            self.assertFalse(recovery['resize_provenance']['hardware_scaled'])
            self.assertIsNone(new.get('mechanical_result'))
            self.assertEqual((store.directory(new['id'])/'sources'/new['source_shapes'][0]['file']).read_bytes(),raw)


    def test_complete_local_transfer_preserves_resources_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as left,tempfile.TemporaryDirectory() as right:
            a=Store(left,Catalog());b=Store(right,Catalog());p=a.create('Any model')
            original=(a.directory(p['id'])/'project.json').read_bytes()
            a.copy_complete_project_to(b,p['id'])
            self.assertEqual((b.directory(p['id'])/'project.json').read_bytes(),original)
            self.assertEqual((a.directory(p['id'])/'project.json').read_bytes(),original)
            with self.assertRaises(ContractError):a.copy_complete_project_to(b,p['id'])


if __name__=='__main__':unittest.main()
