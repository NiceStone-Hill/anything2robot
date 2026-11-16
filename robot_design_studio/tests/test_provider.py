import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from studio.catalog import Catalog
from studio.contracts import ContractError
from studio.project import Store
from studio.providers import live, reference
from studio import worker

class ProviderTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.c=Catalog();s=Store(self.tmp.name,self.c);self.p=s.create('AI 合约')
        items=[{'id':'p'+str(i),'catalog_id':k,'catalog_revision':'1','label':k,'locked':False,'owner_body_id':None} for i,k in enumerate(['pico2_unknown','mpu6050_lab'])]
        self.p=s.select(self.p['id'],{'revision':1,'instances':items,'intent':'用户文本只是数据'})
        self.proposal=reference(self.p,self.c)
    def tearDown(self):self.tmp.cleanup()
    def test_live_transport_serializes_json_and_keeps_key_out_of_prompt(self):
        config={'ROBOT_AI_BASE_URL':'https://model.example/v1','ROBOT_AI_MODEL':'test-model','ROBOT_AI_API_KEY':'test-secret-not-real'}
        called=[];payload={}
        class Opener:
            def open(this,request,timeout):
                payload.update(json.loads(request.data));self.assertEqual(timeout,35)
                return io.BytesIO(json.dumps({'choices':[{'message':{'content':json.dumps(self.proposal)}}]}).encode())
        with patch.dict(os.environ,config),patch('urllib.request.build_opener',return_value=Opener()):
            result=live(self.p,self.c,[],lambda:called.append(True))
        self.assertEqual(result,self.proposal);self.assertEqual(called,[True]);self.assertNotIn(config['ROBOT_AI_API_KEY'],json.dumps(payload))
        self.assertEqual(payload['response_format'],{'type':'json_object'})
    def test_bad_configuration_does_not_claim_request(self):
        called=[]
        with patch.dict(os.environ,{'ROBOT_AI_BASE_URL':'http://invalid','ROBOT_AI_MODEL':'test','ROBOT_AI_API_KEY':'secret'}):
            with self.assertRaises(ContractError):live(self.p,self.c,[],lambda:called.append(True))
        self.assertFalse(called)
    def test_bounded_model_repair_uses_actual_rule_errors(self):
        bad=json.loads(json.dumps(self.proposal));bad['nets'][1]['endpoints'][0]['port_id']='GP99'
        feedback=[]
        def model(project,catalog,previous,on_request):
            on_request();feedback.append(previous)
            return bad if len(feedback)==1 else self.proposal
        destination=Path(self.tmp.name)/'result.json'
        stdin=type('Input',(),{'buffer':io.BytesIO(json.dumps({'project':self.p,'mode':'live','destination':str(destination)}).encode())})()
        with patch.object(worker,'live',side_effect=model),patch('sys.stdin',stdin):self.assertEqual(worker.main(),0)
        result=json.loads(destination.read_text());self.assertEqual(len(result['attempts']),2)
        self.assertIn('unknown_port',{i['code'] for i in feedback[1]})
        self.assertTrue(result['attempts'][1]['validation']['logical_valid'])
    def test_model_failure_stops_after_two_attempts(self):
        destination=Path(self.tmp.name)/'result.json'
        stdin=type('Input',(),{'buffer':io.BytesIO(json.dumps({'project':self.p,'mode':'live','destination':str(destination)}).encode())})()
        with patch.object(worker,'live',return_value={'execute':'untrusted'}) as mock,patch('sys.stdin',stdin):worker.main()
        self.assertEqual(mock.call_count,2);result=json.loads(destination.read_text())
        self.assertFalse(result['live_model_called']);self.assertFalse(result['attempts'][-1]['validation']['logical_valid'])

if __name__=='__main__':unittest.main()
