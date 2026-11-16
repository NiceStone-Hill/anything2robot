import copy
import json
import tempfile
import unittest
from studio.catalog import Catalog
from studio.project import Store,Conflict
from studio.contracts import ContractError
from studio.mechanical_service import dependency_digest,appearance
from studio import layout_service as layout
from studio.providers import reference


class LayoutServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.store=Store(self.temp.name,Catalog());self.p=self.store.create('内部装配')
        self.p=appearance(self.store,self.p['id'],{'revision':1,'builtin':'box'})
        self.p['instances']=[{'id':'pico','catalog_id':'pico2_unknown','catalog_revision':'1','label':'Pico','locked':False,'owner_body_id':None},
                             {'id':'imu','catalog_id':'mpu6050_lab','catalog_revision':'1','label':'IMU','locked':False,'owner_body_id':None}]
        self.p['rigid_bodies']=[{'id':'body','name':'身体'}];self.p['articulation']={'graph':{'root_body_id':'body','bodies':self.p['rigid_bodies'],'joints':[]}}
        self.p['mechanical_result']={'run_id':'run_structure','geometry_valid':True,'dependency_digest':dependency_digest(self.p)};self.store._save(self.p)
    def tearDown(self):self.temp.cleanup()
    def config(self):return {'revision':self.p['revision'],'mode':'reference','components':[
        {'instance_id':i['id'],'dimensions_m':[.055,.025,.012] if i['id']=='pico' else [.025,.02,.01],
         'owner_body_id':'body','evidence':'engineering_assumption','note':'test only, hardware unmeasured'} for i in self.p['instances']]}
    def test_explicit_layout_draft_preserves_failed_structure_status(self):
        from studio.layout_readiness import draft_layout_ready
        result={'geometry_valid':False,'geometry_assets':[{'id':'body_shell'}],
                'layout_domains':[{'owner_body_id':'body'}],
                'checks':[{'name':name,'passed':True} for name in
                          ['assembly_collisions','joint_sweep','structural_paths','solid_body_shell','stl_roundtrip_body_shell','body_final_connections']]+[
                          {'name':'support_core_retained','passed':False}]}
        self.assertTrue(draft_layout_ready(result))
        folder=self.store.root/'runs'/'run_structure';folder.mkdir(parents=True)
        (folder/'result.json').write_text(json.dumps(result))
        self.p['mechanical_result']['geometry_valid']=False;self.store._save(self.p)
        with self.assertRaises(ContractError):layout.configure(self.store,self.p['id'],self.config())
        config=self.config();config['allow_unverified_structure']=True
        changed=copy.deepcopy(result);changed['checks'][0]['passed']=False
        self.assertFalse(draft_layout_ready(changed))
        (folder/'result.json').write_text(json.dumps(changed))
        with self.assertRaises(ContractError):layout.configure(self.store,self.p['id'],config)
        (folder/'result.json').write_text(json.dumps(result))
        updated=layout.configure(self.store,self.p['id'],config)
        self.assertTrue(updated['layout_config']['allow_unverified_structure'])
        self.assertFalse(updated['mechanical_result']['geometry_valid'])
        self.assertEqual(json.loads((folder/'result.json').read_text()),result)

    def test_strict_complete_profile_and_unknown_evidence(self):
        for field,value in [('dimensions_m',[True,.02,.01]),('owner_body_id','missing'),('evidence','measured'),('note','')]:
            config=self.config();config['components'][0][field]=value
            with self.assertRaises(ContractError):layout.configure(self.store,self.p['id'],config)
        config=self.config();config['components'].pop()
        with self.assertRaises(ContractError):layout.configure(self.store,self.p['id'],config)
        config=self.config();config['components'][1]=config['components'][0]
        with self.assertRaises(ContractError):layout.configure(self.store,self.p['id'],config)
    def test_manual_extraction_policy_invalidates_strict_structure_inputs(self):
        strict=dependency_digest(self.p)
        self.p['assembly_policy']='manual_extraction_review'
        self.assertNotEqual(strict,dependency_digest(self.p))
        self.p['assembly_policy']='strict'
        self.assertEqual(strict,dependency_digest(self.p))

    def test_automatic_body_ownership_is_left_for_geometry_solver(self):
        config=self.config()
        for v in config['components']:v['owner_body_id']='auto'
        result=layout.configure(self.store,self.p['id'],config)
        self.assertTrue(all(v['owner_body_id']=='auto' for v in result['layout_config']['components']))
        self.assertTrue(all(i['owner_body_id'] is None for i in result['instances']))

    def test_preserves_structure_and_invalidates_layout(self):
        self.p['layout_result']={'layout_valid':True};self.p['component_placements']=[{'instance_id':'pico','owner_body_id':'body'}]
        self.p['instances'][0]['owner_body_id']='body';self.store._save(self.p)
        result=layout.configure(self.store,self.p['id'],self.config())
        self.assertTrue(result['mechanical_result']['geometry_valid']);self.assertIsNone(result['layout_result'])
        self.assertIsNone(result['instances'][0]['owner_body_id']);self.assertEqual(result['revision'],self.p['revision']+1)
    def test_stale_and_rejected_layout_never_adopted(self):
        p=layout.configure(self.store,self.p['id'],self.config());p=self.store.begin_run(p['id'],p['revision'],'run_layout')
        with self.assertRaises(Conflict):layout.configure(self.store,p['id'],self.config())
        self.assertTrue(layout.finish(self.store,p,'run_layout',{'layout_valid':False}))
        self.assertFalse(self.store.load(p['id'])['layout_result']['layout_valid'])
        self.assertFalse(layout.finish(self.store,p,'run_other',{'layout_valid':True}))
    def test_valid_ownership_and_selection_change(self):
        p=layout.configure(self.store,self.p['id'],self.config());p=self.store.begin_run(p['id'],p['revision'],'run_layout')
        result={'layout_valid':True,'component_placements':[{'instance_id':'pico','owner_body_id':'body'}],'geometry_assets':[]}
        layout.finish(self.store,p,'run_layout',result);p=self.store.load(p['id'])
        self.assertEqual(p['instances'][0]['owner_body_id'],'body')
        p=self.store.select(p['id'],{'revision':p['revision'],'instances':p['instances'],'intent':p['intent']})
        self.assertIsNone(p['layout_result']);self.assertIsNone(p['instances'][0]['owner_body_id'])
    def test_source_invalidates_ownership_and_old_geometry(self):
        self.p['component_placements']=[{'instance_id':'pico','owner_body_id':'body'}];self.p['instances'][0]['owner_body_id']='body';self.store._save(self.p)
        p=appearance(self.store,self.p['id'],{'revision':self.p['revision'],'builtin':'box'})
        self.assertIsNone(p['layout_result']);self.assertIsNone(p['instances'][0]['owner_body_id']);self.assertEqual(p['rigid_bodies'],[])
    def test_changed_motor_inputs_cannot_reuse_structure(self):
        self.p['motor_bindings']=[{'joint_id':'other'}];self.store._save(self.p)
        with self.assertRaises(ContractError):layout.configure(self.store,self.p['id'],self.config())
    def test_layout_revalidates_electrical_copy_without_changing_old_proposal(self):
        from studio.electrical import review
        proposal=reference(self.p,self.store.catalog);old=copy.deepcopy(proposal)
        self.p['circuit']=proposal;self.p['validation']=review(self.p,proposal,self.store.catalog)
        self.assertTrue(self.p['validation']['logical_valid']);self.store._save(self.p)
        p=layout.configure(self.store,self.p['id'],self.config())
        self.assertTrue(p['validation']['logical_valid']);self.assertEqual(p['circuit']['design_revision'],p['revision'])
        self.assertEqual(old,proposal);self.assertEqual(old['nets'],p['circuit']['nets'])

    def test_access_feedback_keeps_revalidated_wiring_and_invalidates_old_geometry(self):
        from studio.articulation_service import configure_assembly
        from studio.electrical import review
        values=self.config()['components']
        self.p['articulation_config']={'mode':'reference'}
        self.p['articulation']['source_sha256']=self.p['source_shapes'][0]['sha256']
        self.p['articulation']['graph']['joints']=[{'id':'neck'}]
        motor=self.store.catalog.parts['sg90_digital_towerpro']
        self.p['instances'].append({'id':'servo','catalog_id':motor['id'],'catalog_revision':motor['revision'],
            'label':'Servo','locked':False,'owner_body_id':None})
        self.p['motor_bindings']=[{'joint_id':'neck','motor_instance_id':'servo','catalog_id':motor['id']}]
        proposal=reference(self.p,self.store.catalog);old=copy.deepcopy(proposal)
        self.p['circuit']=proposal;self.p['validation']=review(self.p,proposal,self.store.catalog)
        self.assertTrue(self.p['validation']['logical_valid']);self.store._save(self.p)
        p=configure_assembly(self.store,self.p['id'],{'revision':self.p['revision'],'mode':'reference','service_components':values})
        self.assertIsNone(p['mechanical_result']);self.assertIsNone(p['layout_result'])
        self.assertTrue(p['validation']['logical_valid']);self.assertEqual(p['circuit']['design_revision'],p['revision'])
        self.assertEqual(old,proposal);self.assertEqual(old['nets'],p['circuit']['nets'])
        self.assertEqual(p['service_opening_requirements'],values)
        self.assertEqual(p['events'][-2]['type'],'electrical_revalidated_for_access')
        # Historical surface advice is reusable after an interrupted rebuild,
        # but only when its archived source, config and graph still match.
        folder=self.store.root/'runs'/'run_structure';folder.mkdir(parents=True)
        result={'geometry_valid':True,'analysis':{'access_suggestions':{'regions':[]}}}
        (folder/'result.json').write_text(json.dumps(result))
        archived=copy.deepcopy(self.p);archived['source_shapes'][0]['sha256']='different-source'
        (folder/'input.json').write_text(json.dumps(archived))
        p['events'].append({'type':'mechanical_checked','run_id':'run_structure'});self.store._save(p)
        p=configure_assembly(self.store,p['id'],{'revision':p['revision'],'mode':'reference','service_components':values})
        self.assertNotIn('surface_plan_reuse',p)
        (folder/'input.json').write_text(json.dumps(self.p))
        p=configure_assembly(self.store,p['id'],{'revision':p['revision'],'mode':'reference','service_components':values})
        self.assertEqual(p['surface_plan_reuse']['source_run_id'],'run_structure')
        # Explicit parting proposals replace historical access advice, while
        # remaining proposals (never an assertion that geometry is valid).
        p['articulation']['analysis']={'bounds_mm':[[-50,-50,-50],[50,50,50]]}
        self.store._save(p)
        regions=[{'body_id':'body','center_mm':[0,0,0],'normal':[0,1,0],'reason':'rear seam'}]
        p=configure_assembly(self.store,p['id'],{'revision':p['revision'],'mode':'reference','parting_regions':regions})
        plan=p['surface_plan_reuse']['plan']
        self.assertEqual(plan['plan_kind'],'assembly_parting_v1')
        self.assertEqual(plan['regions'],regions)
        self.assertFalse(plan['live_model_called'])
        self.assertIsNone(p['mechanical_result'])
        bad=copy.deepcopy(regions);bad[0]['center_mm']=[0,500,0]
        with self.assertRaises(ContractError):
            configure_assembly(self.store,p['id'],{'revision':p['revision'],'mode':'reference','parting_regions':bad})
        self.assertEqual(self.store.load(p['id'])['revision'],p['revision'])


if __name__=='__main__':unittest.main()
