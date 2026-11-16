"""Actual formal geometry jobs; change electronic envelope and verify regeneration."""
import hashlib
import io
import json
import shutil
import threading
import time
import urllib.request
import zipfile
from pathlib import Path
import trimesh as tm
from studio.server import make_server
from studio.project import atomic


def run(source,destination):
    destination=Path(destination).resolve();destination.mkdir(parents=True,exist_ok=False)
    root=destination/'workspace';shutil.copytree(source,root)
    server=make_server(root,0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    token='';base=f'http://127.0.0.1:{server.server_port}/api/'
    def request(path,data=None,raw=False):
        req=urllib.request.Request(base+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(req,timeout=15) as response:return response.read() if raw else json.load(response)
    def wait(r):
        deadline=time.monotonic()+190
        while time.monotonic()<deadline:
            r=request('runs/'+r['id'])
            if r['status'] in {'completed','rejected','failed','timed_out','stale'}:return r
            time.sleep(.25)
        raise AssertionError('job did not finish')
    summaries=[]
    try:
        token=request('session')['token'];p=request('projects/'+request('projects')[0]['id'])
        r=wait(request('projects/'+p['id']+'/assembly',{'revision':p['revision'],'mode':'reference'}))
        atomic(destination/'structure.json',r);assert r['status']=='completed',r
        for index,dims in enumerate([[.055,.025,.012],[.06,.025,.014],[.3,.3,.3]]):
            p=request('projects/'+p['id']);root_body=p['articulation']['graph']['root_body_id']
            profiles=[{'instance_id':i['id'],'dimensions_m':dims if i['catalog_id'].startswith('pico2') else [.025,.020,.010],
                'owner_body_id':root_body,'evidence':'engineering_assumption','note':'Integration test envelope; actual hardware dimensions unknown.'}
                for i in p['instances'] if i['catalog_id'].startswith('pico2') or i['catalog_id']=='mpu6050_lab']
            r=wait(request('projects/'+p['id']+'/layout',{'revision':p['revision'],'mode':'reference','components':profiles}))
            atomic(destination/f'layout-{index+1}.json',r)
            assert r['status'] in {'completed','rejected'},r
            result=r['result'];assert result.get('pipeline')=='electronics-carrier-0.2',r
            current=request('projects/'+p['id']);assert current['mechanical_result']['geometry_valid']
            if index==2:
                assert not result['layout_valid'] and not current.get('component_placements'),r
            else:
                assert result['layout_valid'],[(c['name'],c) for c in result['checks'] if not c['passed']]
                assert len(result['component_placements'])==2
                for a in result['geometry_assets']:
                    path=root/'runs'/r['id']/a['file'];assert hashlib.sha256(path.read_bytes()).hexdigest()==a['sha256']
                    m=tm.load(path,force='mesh');assert m.is_volume and m.is_watertight,a
                data=request('projects/'+p['id']+'/export',raw=True);(destination/f'layout-{index+1}-review.zip').write_bytes(data)
                with zipfile.ZipFile(io.BytesIO(data)) as package:assert 'internal-assembly/manifest.json' in package.namelist()
            summaries.append({'run_id':r['id'],'layout_valid':result['layout_valid'],'elapsed_s':r['elapsed_s'],
                'input_dimensions_m':dims,'placements':result['component_placements'],'checks_failed':[c['name'] for c in result['checks'] if not c['passed']],
                'carrier_hashes':[a['sha256'] for a in result['geometry_assets'] if a['id'].endswith('_carrier')]})
        assert summaries[0]['carrier_hashes']!=summaries[1]['carrier_hashes']
        atomic(destination/'summary.json',{'passed':True,'cases':summaries,'manufacturing_verified':False})
        print(json.dumps(summaries,ensure_ascii=False))
    finally:server.jobs.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('source');parser.add_argument('destination');a=parser.parse_args();run(Path(a.source),Path(a.destination))
