"""Formal HTTP model-only export; no simulation runtime."""
import io
import json
import shutil
import threading
import time
import urllib.request
import zipfile
from pathlib import Path
from studio.server import make_server
from studio.project import atomic


def main():
    destination=Path('examples/simulator_export_2026-11-15/formal-1').resolve();destination.mkdir(parents=True,exist_ok=False)
    root=destination/'workspace';shutil.copytree('workspace/internal_layout_demo_2026_11_15',root)
    server=make_server(root,0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();token=''
    base=f'http://127.0.0.1:{server.server_port}/api/'
    def call(path,data=None,raw=False):
        req=urllib.request.Request(base+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(req,timeout=15) as response:return response.read() if raw else json.load(response)
    cases=[]
    try:
        token=call('session')['token'];p=call('projects/'+call('projects')[0]['id']);before=p['layout_result']
        for density in [1000,1500]:
            r=call('projects/'+p['id']+'/simulation',{'revision':p['revision'],'mode':'reference','printed_density_kg_m3':1200,
                'hardware_density_kg_m3':density,'assumed_torque_nm':1})
            deadline=time.monotonic()+45
            while time.monotonic()<deadline:
                r=call('runs/'+r['id'])
                if r['status'] in {'completed','rejected','failed','timed_out'}:break
                time.sleep(.1)
            atomic(destination/f'case-{density}.json',r)
            assert r['status']=='completed',r
            result=r['result'];assert result['export_valid'] and not result['simulation_performed'] and not result['physical_verified']
            assert len(result['joint_mapping'])==2 and all(c['passed'] for c in result['checks'])
            current=call('projects/'+p['id']);assert current['layout_result']==before
            assert all(not v['hardware_ready'] for v in result['joint_mapping'])
            data=call('projects/'+p['id']+'/export',raw=True)
            with zipfile.ZipFile(io.BytesIO(data)) as package:
                assert {'simulation/robot.xml','simulation/robot.urdf','internal-assembly/manifest.json'}<=set(package.namelist())
                assert not any(n.endswith(('.py','trajectory.csv')) for n in package.namelist())
                assert 'simulation/README.txt' in package.namelist()
            (destination/f'review-{density}.zip').write_bytes(data)
            cases.append({'run_id':r['id'],'mass_kg':result['total_assumed_mass_kg'],'hardware_density_kg_m3':density,
                'elapsed_s':r['elapsed_s'],'checks':result['checks'],'model_sha256':result['artifact_sha256']['robot.xml']})
        assert cases[1]['mass_kg']>cases[0]['mass_kg']
        assert cases[1]['model_sha256']!=cases[0]['model_sha256']
        atomic(destination/'summary.json',{'passed':True,'cases':cases,'physical_verified':False,'simulation_performed':False})
        print(json.dumps(cases,ensure_ascii=False))
    finally:server.jobs.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':main()
