import copy
import unittest
from studio.size_search import targets, baseline, needs_space, run


def project(h=.2):
    return {'id':'original','revision':1,'name':'Any robot','intent':'test','events':[],
            'articulation':{'analysis':{'bounds_mm':[[0,0,-10],[80,40,h*1000-10]]}},
            'instances':[{'id':'motor','owner_body_id':'body','catalog_id':'servo'},
                         {'id':'board','owner_body_id':'body','catalog_id':'pcb'}],
            'motor_bindings':[{'joint_id':'joint','motor_instance_id':'motor','catalog_id':'servo'}],
            'power_config':{'mode':'single_external'}}


def no_space():
    return {'layout_valid':False,'checks':[{'name':'all_selected_components_placed','passed':False,
            'unplaced':[{'instance_id':'board'}]}], 'searches':[{'instance_id':'board','attempts':4}]}


class FakeAPI:
    def __init__(self, p, fit_at=999, bad_structure=False):
        self.projects={p['id']:copy.deepcopy(p)};self.runs={};self.requests=[]
        self.fit_at=fit_at;self.bad_structure=bad_structure;self.heights=[]
    def get(self,path):
        group,identity=path.split('/')
        return copy.deepcopy((self.projects if group=='projects' else self.runs)[identity])
    def mark(self,identity,origin):self.projects[identity]['events'].append(copy.deepcopy(origin))
    def post(self,path,data):
        self.requests.append((path,copy.deepcopy(data)))
        group,identity,action=path.split('/');p=self.projects[identity]
        if data.get('revision') != p['revision']:raise AssertionError('stale revision')
        if action=='clone-at-height':
            identity='copy'+str(len(self.projects));p=project(data['height_m']);p['id']=identity
            p['instances']=[];p['motor_bindings']=[];self.projects[identity]=p
            self.heights.append(data['height_m']);result={}
        elif action=='joint-motors':
            p['instances']=[{'id':'newmotor','catalog_id':'servo','owner_body_id':None}]
            p['revision']+=1;return copy.deepcopy(p)
        elif action=='selection':
            p.update(copy.deepcopy(data));p['revision']+=1;return copy.deepcopy(p)
        elif action=='assembly':
            p['revision']+=1;result={'geometry_valid':not self.bad_structure}
        elif action=='layout':
            p['revision']+=1;result={'layout_valid':True} if len(self.projects)>=self.fit_at else no_space()
        else:result={}
        rid='run'+str(len(self.runs));job={'id':rid,'project_id':identity,'status':'completed','result':result}
        self.runs[rid]=job;return copy.deepcopy(job)


class SizeSearchTests(unittest.TestCase):
    def test_bound_and_no_compounding_for_multiple_inputs(self):
        for h in [.12,.2,.48]:
            p=project(h);self.assertAlmostEqual(targets(p)[-1],h*1.25)
            derived=project(h*1.15);derived['events']=[baseline(p)]
            self.assertAlmostEqual(targets(derived)[-1],h*1.25)
            self.assertEqual(len(targets(derived)),3)
            derived['articulation']['analysis']['bounds_mm'][1][2]=h*1.26*1000-10
            with self.assertRaises(ValueError):targets(derived)
    def test_requires_real_packing_failure(self):
        self.assertTrue(needs_space(no_space()))
        p=no_space();p['searches']=[];self.assertFalse(needs_space(p))
        self.assertFalse(needs_space({'checks':[{'name':'cover_screw_access','passed':False}]}))
    def test_search_stops_first_fit_preserves_hardware(self):
        p=project();api=FakeAPI(p,fit_at=3)
        components=[{'instance_id':'board','dimensions_m':[.055,.025,.016]}]
        result=run(api,p,components,lambda _:None)
        self.assertEqual(result['reason'],'fit');self.assertEqual(len(result['attempts']),3)
        self.assertEqual([round(h,3) for h in api.heights],[.21,.22])
        for path,data in api.requests:
            if path.endswith('/layout'):self.assertEqual(data['components'],components)
            if path.endswith('/selection'):
                self.assertEqual(data['power_config'],p['power_config'])
                self.assertEqual(data['instances'][1],{**p['instances'][1],'owner_body_id':None})
        self.assertEqual(api.projects['copy2']['events'][0]['height_m'],.2)
    def test_exhaustion_never_exceeds_limit(self):
        p=project(.32);api=FakeAPI(p)
        result=run(api,p,[{'instance_id':'board'}],lambda _:None)
        self.assertEqual(result['reason'],'limit_reached')
        self.assertEqual(len(result['attempts']),6)
        self.assertLessEqual(max(api.heights),.4)
    def test_structure_failure_does_not_trigger_growth(self):
        p=project();api=FakeAPI(p,bad_structure=True)
        result=run(api,p,[{'instance_id':'board'}],lambda _:None)
        self.assertEqual(result['reason'],'structure_failed');self.assertFalse(api.heights)
    def test_actuator_room_failure_can_trigger_growth(self):
        class TightMotorAPI(FakeAPI):
            def post(self,path,data):
                job=super().post(path,data)
                if path.endswith('/assembly') and len(self.projects)==1:
                    job['result']={'geometry_valid':False,'checks':[
                        {'name':'arbitrary_joint_module_room','passed':False}]}
                return job
        p=project();api=TightMotorAPI(p,fit_at=2)
        result=run(api,p,[{'instance_id':'board'}],lambda _:None)
        self.assertEqual(result['reason'],'fit')
        self.assertEqual(len(api.heights),1)
        self.assertEqual(result['attempts'][0]['reason'],'actuator_space_search_exhausted')

    def test_cancel_before_any_mutation(self):
        p=project();api=FakeAPI(p)
        with self.assertRaises(InterruptedError):run(api,p,[],lambda _:None,lambda:True)
        self.assertFalse(api.requests)


if __name__=='__main__':unittest.main()
