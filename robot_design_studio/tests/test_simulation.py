import tempfile
import unittest
from unittest.mock import patch
from studio.catalog import Catalog
from studio.project import Store,Conflict
from studio.contracts import ContractError
from studio import simulation_service as simulation
from studio.layout_service import invalidate


class SimulationServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.store=Store(self.temp.name,Catalog());self.p=self.store.create('仿真')
        self.p['layout_result']={'run_id':'run_layout','layout_valid':True,'input_revision':self.p['revision']}
        self.p['articulation_config']={'support':'fixed_base'};self.store._save(self.p)
        self.data={'revision':1,'mode':'reference','printed_density_kg_m3':1200,'hardware_density_kg_m3':1000,'assumed_torque_nm':1}
        self.mock=patch.object(simulation,'interpreter',return_value='test-interpreter');self.mock.start()
    def tearDown(self):self.mock.stop();self.temp.cleanup()
    def test_requires_current_layout_fixed_base_and_finite_inputs(self):
        for key,value in [('printed_density_kg_m3',float('nan')),('assumed_torque_nm',True),('hardware_density_kg_m3',0)]:
            with self.assertRaises(ContractError):simulation.configure(self.store,self.p['id'],{**self.data,key:value})
        self.p['articulation_config']['support']='tabletop';self.store._save(self.p)
        with self.assertRaises(ContractError):simulation.configure(self.store,self.p['id'],self.data)
        self.p['articulation_config']['support']='fixed_base';self.p['layout_result']['input_revision']=0;self.store._save(self.p)
        with self.assertRaises(ContractError):simulation.configure(self.store,self.p['id'],self.data)
    def test_derived_job_preserves_design_and_stale_commit_rejected(self):
        p=simulation.configure(self.store,self.p['id'],self.data)
        self.assertEqual(p['revision'],1);self.assertTrue(p['layout_result']['layout_valid'])
        p=self.store.begin_run(p['id'],1,'run_simulation')
        self.assertFalse(simulation.finish(self.store,p,'run_old',{'export_valid':True}))
        self.assertTrue(simulation.finish(self.store,p,'run_simulation',{'export_valid':True}))
        current=self.store.load(p['id']);self.assertFalse(current['simulation_result']['physical_verified'])
        self.assertFalse(current['simulation_result']['simulation_performed'])
        self.assertEqual(current['downstream']['simulation'],'model_exported')
        invalidate(current);self.assertIsNone(current['simulation_result'])


if __name__=='__main__':unittest.main()
