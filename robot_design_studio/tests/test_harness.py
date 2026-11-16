import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from studio.catalog import Catalog
from studio.project import Store
from studio.contracts import ContractError
from studio import harness_service as harness
from studio.layout_service import invalidate
from studio.export import bundle


class HarnessServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.catalog=Catalog();self.store=Store(self.temp.name,self.catalog)
        self.p=self.store.create('排线版本回归')
        self.p['layout_result']={'run_id':'run_layout','layout_valid':True,'input_revision':1}
        self.p['circuit']={'nets':[]};self.p['validation']={'logical_valid':True};self.store._save(self.p)
        self.data={'revision':1,'mode':'reference','wire_diameter_mm':1.2,'grid_mm':3}
    def tearDown(self):self.temp.cleanup()
    def test_stale_layout_and_invalid_diameter_rejected(self):
        for value in [0,float('nan'),True,4]:
            with self.assertRaises(ContractError):harness.configure(self.store,self.p['id'],{**self.data,'wire_diameter_mm':value})
        self.p['layout_result']['input_revision']=0;self.store._save(self.p)
        with self.assertRaises(ContractError):harness.configure(self.store,self.p['id'],self.data)
    def test_result_only_adopted_for_current_run_and_invalidated_by_layout(self):
        p=harness.configure(self.store,self.p['id'],self.data);p=self.store.begin_run(p['id'],1,'run_harness')
        result={'routing_valid':True,'routes':[]}
        self.assertFalse(harness.finish(self.store,p,'run_old',result))
        self.assertTrue(harness.finish(self.store,p,'run_harness',result))
        p=self.store.load(p['id']);self.assertFalse(p['harness_result']['physical_verified'])
        invalidate(p);self.assertIsNone(p['harness_result']);self.assertEqual(p['harnesses'],[])
    def test_export_rejects_modified_harness_asset(self):
        p=self.store.create('asset check');p['layout_result']={'run_id':'run_layout'}
        directory=Path(self.temp.name)/'route';directory.mkdir();(directory/'wire-list.csv').write_text('modified')
        manifest={'project_id':p['id'],'design_revision':p['revision'],'layout_run_id':'run_layout','routing_valid':True,'terminal_assumptions':[],'routes':[],
                  'artifacts':['wire-list.csv'],'artifact_sha256':{'wire-list.csv':hashlib.sha256(b'original').hexdigest()}}
        (directory/'result.json').write_text(json.dumps(manifest))
        with self.assertRaises(ContractError):bundle(p,self.catalog,harness_directory=directory)
    def test_export_keeps_reference_pads_separate_from_aggregation_exits(self):
        import io,zipfile
        p=self.store.create('pin guide contract');p['layout_result']={'run_id':'run_layout'}
        directory=Path(self.temp.name)/'pin_routes';directory.mkdir()
        pad={'instance_id':'board','port_id':'GP0','position_m':[.01,.02,.03],'physical_verified':False}
        exit={'instance_id':'board','position_m':[.04,.05,.06],'port_id':None}
        manifest={'project_id':p['id'],'design_revision':p['revision'],'layout_run_id':'run_layout','routing_valid':True,
            'terminal_assumptions':[exit],'endpoint_registration':[pad],'routes':[],'artifacts':[],
            'fanout_checks':[{'passed':False,'failure':{'blocking_assets':['carrier']}}]}
        (directory/'result.json').write_text(json.dumps(manifest))
        with zipfile.ZipFile(io.BytesIO(bundle(p,self.catalog,harness_directory=directory))) as package:
            contract=json.loads(package.read('routing_contract.json'))
        self.assertEqual(contract['ports'],[pad]);self.assertEqual(contract['aggregation_exits'],[exit])
        self.assertFalse(contract['full_connection_verified']);self.assertFalse(contract['fanout_checks'][0]['passed'])

if __name__=='__main__':unittest.main()
