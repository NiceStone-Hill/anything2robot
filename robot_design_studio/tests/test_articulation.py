import copy
import tempfile
import unittest
from studio.catalog import Catalog
from studio.project import Store, Conflict
from studio.contracts import ContractError
from studio.mechanical_service import appearance
from studio import articulation_service as service


class ArticulationServiceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.store=Store(self.tmp.name,Catalog());self.p=self.store.create('无电机起步')
        self.p=appearance(self.store,self.p['id'],{'revision':1,'builtin':'box'})
    def tearDown(self):self.tmp.cleanup()
    def config(self):
        return {'revision':self.p['revision'],'mode':'reference','intent':'','length_m':.24,'up_axis':'+Z','support':'fixed_base','max_joints':2}
    def annotate(self):
        self.p=service.configure(self.store,self.p['id'],self.config())
        self.p=self.store.begin_run(self.p['id'],self.p['revision'],'run_annotation')
        result={'annotation_valid':True,'source_sha256':self.p['source_shapes'][0]['sha256'],
            'graph':{'joints':[{'id':'joint_a','name':'头部'},{'id':'joint_b','name':'手臂'}]},'analysis':{},'source':{},'motor_candidates':[],'status':'checked'}
        self.assertTrue(service.finish(self.store,self.p,'run_annotation',result));self.p=self.store.load(self.p['id'])
    def choose(self):
        self.p=service.bind_motors(self.store,self.p['id'],{'revision':self.p['revision'],'choices':[
            {'joint_id':'joint_a','catalog_id':'sg90_digital_towerpro'},{'joint_id':'joint_b','catalog_id':'mg4005v2_candidate'}]})
    def test_no_preselected_motor_and_correct_order(self):
        with self.assertRaises(ContractError):service.bind_motors(self.store,self.p['id'],{'revision':self.p['revision'],'choices':[{'joint_id':'joint_a','catalog_id':'sg90_digital_towerpro'}]})
        self.annotate();self.assertEqual(self.p['instances'],[]);self.choose()
        self.assertEqual(len(self.p['instances']),2)
        self.assertEqual({b['branch'] for b in self.p['motor_bindings']},{'text2robot_rectangular','anything2robot_round'})
    def test_binding_idempotence_and_locked_motor(self):
        self.annotate();self.choose();ids=[i['id'] for i in self.p['instances']];self.choose()
        self.assertEqual(ids,[i['id'] for i in self.p['instances']])
        self.p['instances'][0]['locked']=True;self.store._save(self.p)
        with self.assertRaises(ContractError):service.bind_motors(self.store,self.p['id'],{'revision':self.p['revision'],'choices':[
            {'joint_id':'joint_a','catalog_id':'mg4005v2_candidate'},{'joint_id':'joint_b','catalog_id':'mg4005v2_candidate'}]})
    def test_appearance_invalidates_graph_and_bindings(self):
        self.annotate();self.choose();self.p=appearance(self.store,self.p['id'],{'revision':self.p['revision'],'builtin':'box','dimensions_mm':[280,95,85]})
        self.assertIsNone(self.p['articulation']);self.assertEqual(self.p['motor_bindings'],[])
    def test_stale_task_cannot_replace_new_source(self):
        original=service.configure(self.store,self.p['id'],self.config());original=self.store.begin_run(original['id'],original['revision'],'run_old')
        appearance(self.store,original['id'],{'revision':original['revision'],'builtin':'box'})
        self.assertFalse(service.finish(self.store,original,'run_old',{'annotation_valid':True}))
    def test_duplicate_binding_unknown_motor_and_bad_config(self):
        self.annotate()
        for choices in [[{'joint_id':'joint_a','catalog_id':'sg90_digital_towerpro'}]*2,
                        [{'joint_id':'joint_a','catalog_id':'pico2_unknown'},{'joint_id':'joint_b','catalog_id':'mg4005v2_candidate'}]]:
            with self.assertRaises(ContractError):service.bind_motors(self.store,self.p['id'],{'revision':self.p['revision'],'choices':choices})
        for key,value in [('length_m',True),('up_axis','Y'),('max_joints',True),('support','unknown')]:
            config=self.config();config[key]=value
            with self.assertRaises(ContractError):service.configure(self.store,self.p['id'],config)
    def test_assembly_requires_current_instances(self):
        self.annotate();self.choose();self.p['instances'].pop();self.store._save(self.p)
        with self.assertRaises(ContractError):service.configure_assembly(self.store,self.p['id'],{'revision':self.p['revision'],'mode':'reference'})
    def test_electronics_changes_preserve_mechanics_and_removing_motor_invalidates(self):
        self.annotate();self.choose()
        self.p['mechanical_result']={'run_id':'run_geometry','geometry_valid':True}
        self.store._save(self.p)
        instances=copy.deepcopy(self.p['instances']);part=self.store.catalog.parts['pico2_unknown']
        instances.append({'id':'pico','catalog_id':part['id'],'catalog_revision':part['revision'],'label':part['name'],'locked':False,'owner_body_id':None})
        self.p=self.store.select(self.p['id'],{'revision':self.p['revision'],'instances':instances,'intent':self.p['intent']})
        self.assertTrue(self.p['mechanical_result']['geometry_valid']);self.assertEqual(len(self.p['motor_bindings']),2)
        self.p=self.store.select(self.p['id'],{'revision':self.p['revision'],'instances':instances[1:],'intent':self.p['intent']})
        self.assertIsNone(self.p['mechanical_result']);self.assertEqual(len(self.p['motor_bindings']),1)


if __name__=='__main__':unittest.main()
