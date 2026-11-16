"""Re-run current code on archived VLM annotations + raw sources, never old meshes."""
import argparse
import hashlib
import json
import shutil
import threading
import time
import urllib.request
from pathlib import Path
import trimesh as tm
from studio.server import make_server
from studio.project import atomic


def run(source,destination):
    destination=Path(destination).resolve();destination.mkdir(parents=True,exist_ok=True)
    root=destination/'workspace';shutil.copytree(Path(source).resolve(),root)
    server=make_server(root,0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    token='';base=f'http://127.0.0.1:{server.server_port}/api/'
    def call(path,data=None):
        req=urllib.request.Request(base+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(req,timeout=10) as response:return json.load(response)
    results=[]
    try:
        token=call('session')['token']
        for index,item in enumerate(call('projects')):
            p=call('projects/'+item['id'])
            if not p.get('articulation') or not p.get('motor_bindings'):continue
            run=call('projects/'+p['id']+'/assembly',{'revision':p['revision'],'mode':'reference'})
            deadline=time.monotonic()+185
            while time.monotonic()<deadline:
                run=call('runs/'+run['id'])
                if run['status'] in {'completed','rejected','failed','timed_out'}:break
                time.sleep(.2)
            atomic(destination/f'case-{index+1}.json',run)
            assert run['status'] in {'completed','rejected'},run
            result=run['result'];assert result.get('pipeline')=='graph_assembly_0.3',run
            assert len(result['geometry_assets'])>=8
            directory=root/'runs'/run['id']
            invalid_exports=[]
            for asset in result['geometry_assets']:
                data=(directory/asset['file']).read_bytes();assert hashlib.sha256(data).hexdigest()==asset['sha256']
                mesh=tm.load(directory/asset['file'],force='mesh')
                actual=bool(mesh.is_volume and mesh.is_watertight)
                reported=next(c['passed'] for c in result['checks'] if c['name']=='stl_roundtrip_'+asset['id'])
                assert actual==reported,(asset,actual,reported)
                if not actual:invalid_exports.append(asset['id'])
            if result['geometry_valid']:assert not invalid_exports
            failed=[c['name'] for c in result['checks'] if not c['passed']]
            assert result['geometry_valid']==(not failed)
            current=call('projects/'+p['id'])
            assert current['mechanical_result']['geometry_valid']==result['geometry_valid']
            if failed:assert not current['geometry_assets'],'Failed assembly cannot be adopted'
            results.append({'project_id':p['id'],'run_id':run['id'],'source_sha256':result['source_sha256'],
                'geometry_valid':result['geometry_valid'],'failed_checks':failed,'invalid_exports':invalid_exports,'assets':len(result['geometry_assets']),
                'first_asset_sha256':result['geometry_assets'][0]['sha256'],'elapsed_s':run['elapsed_s']})
        assert len(results)>=2 and len({r['source_sha256'] for r in results})>=2
        assert len({r['first_asset_sha256'] for r in results})>=2
        atomic(destination/'summary.json',{'execution_and_rejection_guards_passed':True,'cases':results,'manufacturing_verified':False,
            'scope':'正式入口重新生成两种不同源模型；逐件重新加载和摘要校核。geometry_valid 单独反映候选是否通过，不把执行成功当制造通过。'})
        print(json.dumps(results,ensure_ascii=False))
    finally:server.jobs.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('destination');a=p.parse_args();run(a.source,a.destination)
