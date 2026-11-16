"""Real loopback HTTP integration tests. Run explicitly when sockets are permitted."""
import json
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from studio.server import make_server

class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.server=make_server(self.tmp.name,0)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.base='http://127.0.0.1:'+str(self.server.server_port);self.token=None
        self.token=self.request('/api/session')[1]['token']
    def tearDown(self):
        self.server.jobs.close();self.server.shutdown();self.server.server_close();self.thread.join();self.tmp.cleanup()
    def request(self,path,data=None,headers=None):
        h={'Content-Type':'application/json','X-Studio-Token':self.token or ''};h.update(headers or {})
        req=urllib.request.Request(self.base+path,data=json.dumps(data).encode() if data is not None else None,headers=h)
        try:r=urllib.request.urlopen(req,timeout=5)
        except urllib.error.HTTPError as exc:r=exc
        raw=r.read();content=r.headers.get('Content-Type','')
        return r.status,json.loads(raw) if 'application/json' in content else raw
    def test_actual_http_roundtrip_generation_and_export(self):
        status,p=self.request('/api/projects',{'name':'HTTP 中文 项目'});self.assertEqual(status,201)
        items=[{'id':'part_'+str(n),'catalog_id':key,'catalog_revision':'1','label':key,'locked':False,'owner_body_id':None} for n,key in enumerate(['pico2_unknown','mpu6050_lab','sg90_digital_towerpro'])]
        _,p=self.request('/api/projects/'+p['id']+'/selection',{'revision':1,'instances':items,'intent':'摆动'})
        _,run=self.request('/api/projects/'+p['id']+'/generate',{'revision':p['revision'],'mode':'reference'})
        for _ in range(100):
            _,r=self.request('/api/runs/'+run['id'])
            if r['status'] in {'completed','failed','rejected'}:break
            time.sleep(.05)
        self.assertEqual(r['status'],'completed')
        _,p=self.request('/api/projects/'+p['id']);self.assertTrue(p['circuit']);self.assertFalse(p['validation']['manufacturing_released'])
        status,archive=self.request('/api/projects/'+p['id']+'/export');self.assertEqual(status,200);self.assertTrue(archive.startswith(b'PK'))
        status,clone=self.request('/api/import',p);self.assertEqual(status,201);self.assertIsNone(clone['circuit'])
        self.assertEqual(self.request('/api/projects/'+p['id']+'/selection',{'revision':1,'instances':items,'intent':''})[0],409)
    def test_second_server_cannot_interrupt_same_workspace(self):
        from studio.contracts import ContractError
        with self.assertRaises(ContractError):make_server(self.tmp.name,0)

    def test_no_mutations_without_session_token_or_cross_origin(self):
        self.assertEqual(self.request('/api/projects',{'name':'bad'},{'X-Studio-Token':''})[0],403)
        self.assertEqual(self.request('/api/projects',{'name':'bad'},{'Origin':'https://other.example'})[0],403)
    def test_no_arbitrary_files_or_bad_host(self):
        self.assertEqual(self.request('/../../README.md')[0],404)
        self.assertEqual(self.request('/api/session',headers={'Host':'other.example'})[0],403)
        self.assertEqual(self.request('/')[0],200)
    def test_malformed_request_does_not_create_project(self):
        self.assertEqual(self.request('/api/projects',{'name':[]} )[0],400)
        self.assertEqual(self.request('/api/projects')[1],[])

if __name__=='__main__':unittest.main()
