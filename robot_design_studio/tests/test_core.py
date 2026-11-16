import copy
import io
import json
import tempfile
import threading
import time
import unittest
import zipfile
from pathlib import Path
from studio.catalog import Catalog
from studio.contracts import ContractError, loads, metres, transform_point
from studio.electrical import review, voltage_range
from studio.export import bundle
from studio.jobs import Jobs
from studio.legacy import read_evidence
from studio.project import Store, Conflict
from studio.providers import reference

class CoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='机器人 项目 ')
        self.catalog=Catalog();self.store=Store(Path(self.tmp.name)/'中文 路径',self.catalog)
        self.p=self.store.create('身体 表达')
        self.p=self.select(['pico2_unknown','mpu6050_lab','sg90_digital_towerpro','supply_5_candidate','supply_48_candidate'])
        self.proposal=reference(self.p,self.catalog)
    def tearDown(self):self.tmp.cleanup()
    def select(self,ids):
        instances=[{'id':'part_'+str(n),'catalog_id':key,'catalog_revision':'1','label':self.catalog.parts[key]['name'],'locked':False,'owner_body_id':None} for n,key in enumerate(ids)]
        old={i['id']:i for i in self.p['instances']}
        for i in instances:
            if i['id'] in old and old[i['id']]['catalog_id']!=i['catalog_id']:i['id']+='_new'
        return self.store.select(self.p['id'],{'revision':self.p['revision'],'instances':instances,'intent':'轻轻摆动'})
    def codes(self,p=None):return {e['code'] for e in review(self.p,p or self.proposal,self.catalog)['errors']}
    def net(self,name):return next(n for n in self.proposal['nets'] if n['id']==name)
    def test_reference_is_traceable_but_not_released(self):
        r=review(self.p,self.proposal,self.catalog)
        self.assertEqual(r['errors'],[]);self.assertTrue(r['logical_valid']);self.assertFalse(r['manufacturing_released'])
        self.assertIn('unknown_voltage',{e['code'] for e in r['pending']})
    def test_unknown_pin_rejected(self):
        self.net('bus_sda')['endpoints'][0]['port_id']='GP99'
        self.assertIn('unknown_port',self.codes())
    def test_unselected_component_rejected(self):
        self.net('bus_sda')['endpoints'][1]['instance_id']='phantom'
        self.assertIn('unknown_port',self.codes())
    def test_source_cannot_lie_about_voltage(self):
        self.net('supply_5_candidate_rail')['voltage_v']=[3.3,3.3]
        self.assertIn('source_voltage_mismatch',self.codes())
    def test_servo_cannot_use_pico_3v3(self):
        self.net('supply_48_candidate_rail')['endpoints'][0]={'instance_id':'part_0','port_id':'3V3'}
        self.assertIn('voltage_out_of_range',self.codes())
    def test_polarity_short_rejected(self):
        self.net('ground')['endpoints'].append({'instance_id':'part_3','port_id':'OUT'})
        self.assertIn('role_mismatch',self.codes())
    def test_multiple_sources_rejected(self):
        self.net('supply_48_candidate_rail')['endpoints'].append({'instance_id':'part_3','port_id':'OUT'})
        self.assertIn('multiple_power_sources',self.codes())
    def test_missing_common_ground_rejected(self):
        self.net('ground')['endpoints']=[e for e in self.net('ground')['endpoints'] if e['instance_id']!='part_1']
        self.assertIn('missing_common_ground',self.codes())
    def test_duplicate_port_rejected(self):
        self.net('bus_scl')['endpoints'][0]['port_id']='GP0'
        self.assertIn('port_reused',self.codes());self.assertIn('protocol_mismatch',self.codes())
    def test_missing_allocation_rejected(self):
        self.proposal['allocations']=[]
        self.assertIn('missing_allocation',self.codes())
    def test_wrong_peripheral_rejected(self):
        self.proposal['allocations'][0]['function']='UART0_TX'
        self.assertIn('invalid_allocation',self.codes())
    def test_duplicate_allocation_rejected(self):
        self.proposal['allocations'].append(copy.deepcopy(self.proposal['allocations'][0]))
        self.assertIn('invalid_allocation',self.codes())
    def test_duplicate_i2c_address_rejected(self):
        self.p=self.select(['pico2_unknown','mpu6050_lab','mpu6050_lab'])
        self.proposal=reference(self.p,self.catalog);self.proposal['addresses'][1]['value']=104
        self.assertIn('address_conflict',self.codes())
    def test_missing_address_rejected(self):
        self.proposal['addresses']=[];self.assertIn('missing_address',self.codes())
    def test_address_on_wrong_bus_rejected(self):
        self.proposal['addresses'][0]['bus_net_id']='ground';self.assertIn('invalid_address',self.codes())
    def test_protocol_spoof_rejected(self):
        self.net('bus_sda')['kind']='can';self.assertIn('protocol_mismatch',self.codes())
    def test_known_peak_current_budget_rejected(self):
        self.catalog.parts['sg90_digital_towerpro']['electrical']['peak_current_a']=2
        self.catalog.parts['supply_48_candidate']['ports']['OUT']['current_limit_a']=.5
        self.assertIn('power_budget_exceeded',self.codes())
    def test_missing_required_stays_pending(self):
        self.proposal['nets']=[n for n in self.proposal['nets'] if n['id']!='supply_5_candidate_rail']
        r=review(self.p,self.proposal,self.catalog)
        self.assertIn('required_port_unconnected',{i['code'] for i in r['pending']});self.assertFalse(r['manufacturing_released'])
    def test_no_hidden_hardware_addition(self):
        self.p=self.select(['pico2_unknown','mpu6050_lab','sg90_digital_towerpro'])
        before=copy.deepcopy(self.p['instances']);proposal=reference(self.p,self.catalog)
        self.assertEqual(before,self.p['instances']);self.assertIn('supply_48_candidate',{r['catalog_id'] for r in proposal['missing_parts']})
    def test_unknown_addition_rejected(self):
        self.proposal['missing_parts'].append({'catalog_id':'imaginary','category':'power','quantity':1,'reason':'invented'})
        self.assertIn('unknown_requested_part',self.codes())
    def test_unknown_motor_not_implicitly_supported(self):
        self.p=self.select(['pico2_unknown','mpu6050_lab','st3215_12v_candidate']);self.proposal=reference(self.p,self.catalog)
        self.assertIn('unsupported_actuator',self.codes())
    def test_invalid_output_shapes_rejected(self):
        for value in [[],None,{'run_shell':'rm -rf'},True,{'schema_version':'2.0'}]:
            with self.subTest(value=value):self.assertEqual(review(self.p,value,self.catalog)['errors'][0]['code'],'invalid_schema')
    def test_extra_field_rejected(self):
        self.proposal['execute']='anything';self.assertIn('invalid_schema',self.codes())
    def test_nonfinite_reversed_boolean_ranges_rejected(self):
        for span in [[5,2],[False,5],[0,float('nan')],[0,float('inf')]]:
            with self.subTest(span=span),self.assertRaises(ContractError):voltage_range(span)
    def test_duplicate_json_fields_rejected(self):
        for raw in ['{"a":1,"a":2}','{"a":NaN}']:
            with self.assertRaises(ContractError):loads(raw)
    def test_stale_and_other_project_rejected(self):
        self.proposal['project_id']='project_other';self.assertIn('stale_proposal',self.codes())
    def test_catalog_digest_rejected(self):
        self.proposal['catalog_digest']='fake';self.assertIn('stale_proposal',self.codes())
    def test_project_reopens_and_history_is_retained(self):
        reopened=Store(self.store.root,self.catalog).load(self.p['id']);self.assertEqual(reopened,self.p)
        self.assertGreaterEqual(len(list((self.store.directory(self.p['id'])/'history').glob('*.json'))),2)
    def test_optimistic_conflict(self):
        with self.assertRaises(Conflict):self.store.select(self.p['id'],{'revision':1,'instances':self.p['instances'],'intent':'new'})
    def test_concurrent_selection_only_one_wins(self):
        results=[]
        def change():
            try:self.store.select(self.p['id'],{'revision':self.p['revision'],'instances':self.p['instances'],'intent':'new'});results.append('ok')
            except Conflict:results.append('conflict')
        threads=[threading.Thread(target=change) for _ in range(2)]
        for t in threads:t.start()
        for t in threads:t.join(2);self.assertFalse(t.is_alive())
        self.assertCountEqual(results,['ok','conflict'])
    def test_selection_invalidates_and_old_job_cannot_commit(self):
        self.store.begin_run(self.p['id'],self.p['revision'],'run_old')
        self.p=self.select(['pico2_unknown','mpu6050_lab'])
        self.assertIsNone(self.p['circuit']);self.assertTrue(all(v=='stale' for v in self.p['downstream'].values()))
        self.assertFalse(self.store.finish_run(self.p['id'],self.p['revision']-1,'run_old',self.proposal,{'logical_valid':True,'status':'passed'}))
    def test_newer_job_wins_same_revision(self):
        self.store.begin_run(self.p['id'],self.p['revision'],'run_old');self.store.begin_run(self.p['id'],self.p['revision'],'run_new')
        self.assertFalse(self.store.finish_run(self.p['id'],self.p['revision'],'run_old',{},{}))
    def test_duplicate_instance_and_unknown_catalog_rejected(self):
        for instances in [[self.p['instances'][0]]*2,[{**self.p['instances'][0],'catalog_id':'unknown'}]]:
            with self.assertRaises(ContractError):self.store.select(self.p['id'],{'revision':self.p['revision'],'instances':instances,'intent':''})
    def test_unknown_not_zero_and_assembly_states_distinct(self):
        self.assertIsNone(self.catalog.parts['mpu6050_lab']['geometry']['dimensions_m']['value'])
        self.assertEqual({self.catalog.parts['pico2_'+s]['assembly_state'] for s in ['unknown','bare','straight','angled']},{'unknown','bare','straight','angled'})
    def test_explicit_units_and_coordinates(self):
        self.assertAlmostEqual(metres(100,'mm'),.1);self.assertAlmostEqual(metres(10,'cm'),.1)
        t={'from_frame':'cad','to_frame':'body','units':'m','rotation':[[1,0,0],[0,1,0],[0,0,1]],'translation_m':[.1,0,0]}
        self.assertEqual(transform_point([.1,0,0],t),[.2,0,0])
        t['rotation'][0][0]=-1
        with self.assertRaises(ContractError):transform_point([0,0,0],t)
        with self.assertRaises(ContractError):metres(1,'inch')
        with self.assertRaises(ContractError):transform_point([0,0,0],{})
    def test_import_does_not_trust_passed_claims(self):
        self.p['validation']={'manufacturing_released':True};p=self.store.import_design(self.p)
        self.assertNotEqual(p['id'],self.p['id']);self.assertIsNone(p['validation']);self.assertIsNone(p['circuit'])
    def test_import_refuses_silent_loss_of_future_geometry(self):
        self.p['joints']=[{'id':'joint_1'}]
        with self.assertRaises(ContractError):self.store.import_design(self.p)
    def test_path_traversal_rejected(self):
        with self.assertRaises(ContractError):self.store.load('../other')
    def test_export_contains_bom_ports_and_blocked_routing(self):
        self.p['circuit']=self.proposal
        with zipfile.ZipFile(io.BytesIO(bundle(self.p,self.catalog))) as z:
            self.assertIn('bom.csv',z.namelist());r=json.loads(z.read('routing_contract.json'))
            self.assertEqual(r['status'],'blocked_pending_registration');self.assertTrue(all(p['position_m'] is None for p in r['ports']))
            self.assertFalse(json.loads(z.read('validation.json'))['manufacturing_released'])
    def test_legacy_failures_not_upgraded(self):
        path=Path(self.tmp.name)/'legacy.json';path.write_text('{"status":"failed","error":"Insufficient shell connections: 0/3"}')
        r=read_evidence(path);self.assertEqual(r['historical_status'],'failed');self.assertFalse(r['manufacturing_released'])

class JobTests(CoreTests):
    # This class does not inherit CoreTests in the final discovery set below.
    pass

class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.t=tempfile.TemporaryDirectory();c=Catalog();self.s=Store(self.t.name,c);self.p=self.s.create('任务测试')
        instances=[{'id':'part_'+str(n),'catalog_id':key,'catalog_revision':'1','label':key,'locked':False,'owner_body_id':None} for n,key in enumerate(['pico2_unknown','mpu6050_lab'])]
        self.p=self.s.select(self.p['id'],{'revision':1,'instances':instances,'intent':''});self.j=Jobs(self.s)
    def tearDown(self):self.j.close();self.t.cleanup()
    def wait(self,identity):
        for _ in range(100):
            r=self.j.get(identity)
            if r['status'] in {'completed','cancelled','timed_out','failed','rejected','stale'}:return r
            time.sleep(.05)
        self.fail('worker did not finish')
    def test_actual_subprocess_completes_and_archives(self):
        r=self.wait(self.j.start(self.p['id'],self.p['revision'],'reference')['id'])
        self.assertEqual(r['status'],'completed');self.assertFalse(r['result']['live_model_called'])
        self.assertIsNotNone(self.s.load(self.p['id'])['circuit'])
    def test_immediate_cancel_does_not_commit(self):
        with self.j.lock:
            r=self.j.start(self.p['id'],self.p['revision'],'reference');self.j.cancel(r['id'])
        self.assertEqual(self.wait(r['id'])['status'],'cancelled');self.assertIsNone(self.s.load(self.p['id'])['circuit'])
    def test_timeout_does_not_commit(self):
        self.j.timeout=0
        r=self.wait(self.j.start(self.p['id'],self.p['revision'],'reference')['id'])
        self.assertEqual(r['status'],'timed_out');self.assertIsNone(self.s.load(self.p['id'])['circuit'])
    def test_restart_marks_abandoned_job_interrupted(self):
        from studio.project import atomic
        identity='run_abandoned';self.s.begin_run(self.p['id'],self.p['revision'],identity)
        atomic(self.j.root/identity/'run.json',{'id':identity,'project_id':self.p['id'],'status':'running','events':[]})
        j=Jobs(self.s);self.assertEqual(j.get(identity)['status'],'interrupted');self.assertIsNone(self.s.load(self.p['id'])['active_run_id'])

# Avoid duplicating the core suite through inheritance.
del JobTests
if __name__=='__main__':unittest.main()
