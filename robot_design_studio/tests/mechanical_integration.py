"""Real formal HTTP entry, fresh inputs, isolated process, artifact round-trip.

Run with studio .venv. --live-env is opt-in and sends the generated views to the
configured model. Evidence records model/output, never provider credentials.
"""
import argparse
import base64
import hashlib
import json
import threading
import time
import urllib.request
import zipfile
import io
from pathlib import Path
from studio.server import make_server
from studio.providers import load_ai_env
from studio.project import atomic


def run(destination,live_env=None):
    destination=Path(destination).resolve();destination.mkdir(parents=True,exist_ok=True)
    if live_env:load_ai_env(live_env)
    server=make_server(destination/'workspace',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base=f'http://127.0.0.1:{server.server_port}/api/'
    def call(path,data=None):
        request=urllib.request.Request(base+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(request,timeout=10) as response:return json.load(response)
    def wait(task):
        deadline=time.monotonic()+185
        while time.monotonic()<deadline:
            task=call('runs/'+task['id'])
            if task['status'] in {'completed','rejected','failed','cancelled','stale','timed_out'}:return task
            time.sleep(.1)
        raise AssertionError('Task did not finish')
    token=call('session')['token'];catalog=call('catalog');servo=next(p for p in catalog['parts'] if p['id']=='sg90_digital_towerpro')
    try:
        project=call('projects',{'name':'单关节机械验收 · 不同外观再次生成'})
        instances=[]
        for identity,catalog_id in [('servo_1','sg90_digital_towerpro'),('pico_1','pico2_unknown'),('imu_1','mpu6050_lab')]:
            entry=next(p for p in catalog['parts'] if p['id']==catalog_id)
            instances.append({'id':identity,'catalog_id':catalog_id,'catalog_revision':entry['revision'],'label':entry['name'],'locked':False,'owner_body_id':None})
        project=call('projects/'+project['id']+'/selection',{'revision':project['revision'],'intent':'一端稳定，另一端轻轻左右摆动。','instances':instances})
        runs=[]
        for index,(dims,angle) in enumerate([([240,90,80],20),([280,98,84],15)]):
            if index==0:
                source_data={'revision':project['revision'],'builtin':'box','dimensions_mm':dims}
            else:
                # A different external-file input: bevelled octagonal prism, closed OBJ.
                x,y,z=[d/2 for d in dims];bevel=6
                outline=[[-x+bevel,-y],[x-bevel,-y],[x,-y+bevel],[x,y-bevel],[x-bevel,y],[-x+bevel,y],[-x,y-bevel],[-x,-y+bevel]]
                vertices=[[*p,h] for h in [-z,z] for p in outline]
                faces=[[1,i+2,i+1] for i in range(1,7)]+[[9,9+i,10+i] for i in range(1,7)]
                for i in range(8):
                    a=i+1;b=(i+1)%8+1;faces.extend([[a,b,b+8],[a,b+8,a+8]])
                obj='\n'.join('v '+' '.join(map(str,p)) for p in vertices)+'\n'+'\n'.join('f '+' '.join(map(str,f)) for f in faces)
                source_data={'revision':project['revision'],'filename':'改变外观 凸形.obj','unit':'mm','data_base64':base64.b64encode(obj.encode()).decode()}
            project=call('projects/'+project['id']+'/appearance',source_data)
            task=wait(call('projects/'+project['id']+'/mechanics',{'revision':project['revision'],'length_m':dims[0]/1000,'amplitude_deg':angle,
                 'motor_instance_id':'servo_1','mode':'live' if live_env else 'reference','intent':'保持一侧稳定，让另一侧温和摆动，像在回应观众。'}))
            atomic(destination/f'run-{index+1}.json',task)
            assert task['status']=='completed',task.get('result',task)
            result=task['result'];assert result['geometry_valid'] and not result['manufacturing_ready']
            if live_env:assert result['plan']['live_model_called']
            with urllib.request.urlopen(base+'runs/'+task['id']+'/artifacts/mechanical-review.zip') as response:package=response.read()
            (destination/f'case-{index+1}.zip').write_bytes(package)
            with zipfile.ZipFile(io.BytesIO(package)) as z:
                manifest=json.loads(z.read('manifest.json'))
                for asset in manifest['geometry_assets']:assert hashlib.sha256(z.read(asset['file'])).hexdigest()==asset['sha256']
            project=call('projects/'+project['id']);assert len(project['rigid_bodies'])==2 and len(project['joints'])==1
            with urllib.request.urlopen(base+'projects/'+project['id']+'/export') as response:combined=response.read()
            with zipfile.ZipFile(io.BytesIO(combined)) as z:
                assert 'mechanics/fixed_shell.stl' in z.namelist() and 'bom.csv' in z.namelist()
            runs.append(task)
        a,b=[r['result'] for r in runs]
        assert a['source_sha256']!=b['source_sha256']
        assert a['geometry_assets'][0]['sha256']!=b['geometry_assets'][0]['sha256']
        assert a['geometry_assets'][0]['bounds_m']!=b['geometry_assets'][0]['bounds_m']
        # A VLM can legitimately recommend the same conservative amplitude twice.
        if not live_env:assert a['joints'][0]['limits_rad']!=b['joints'][0]['limits_rad']
        wiring=wait(call('projects/'+project['id']+'/generate',{'revision':project['revision'],'mode':'reference'}))
        assert wiring['status']=='completed'
        project=call('projects/'+project['id'])
        assert project['circuit'] and project['joints'][0]['motor_instance_id']=='servo_1'
        with urllib.request.urlopen(base+'projects/'+project['id']+'/export') as response:combined=response.read()
        (destination/'combined-review.zip').write_bytes(combined)
        with zipfile.ZipFile(io.BytesIO(combined)) as z:
            package_project=json.loads(z.read('project.json'));manifest=json.loads(z.read('mechanics/manifest.json'))
            assert package_project['circuit']['design_revision']==manifest['design_revision']==package_project['revision']
        # Exercise actual mechanical rejection, cancellation and stale-result guards.
        rejected_project=call('projects',{'name':'机械任务失败与取消验收'})
        rejected_project=call('projects/'+rejected_project['id']+'/selection',{'revision':1,'intent':'测试','instances':instances})
        rejected_project=call('projects/'+rejected_project['id']+'/appearance',{'revision':rejected_project['revision'],'builtin':'box'})
        def config(p,length=.24):return {'revision':p['revision'],'length_m':length,'amplitude_deg':20,'motor_instance_id':'servo_1','mode':'reference','intent':'测试'}
        rejected=wait(call('projects/'+rejected_project['id']+'/mechanics',config(rejected_project,.18)))
        assert rejected['status']=='rejected' and not rejected['result']['geometry_valid']
        rejected_project=call('projects/'+rejected_project['id']);assert not rejected_project['rigid_bodies']
        cancelled=call('projects/'+rejected_project['id']+'/mechanics',config(rejected_project))
        call('runs/'+cancelled['id']+'/cancel',{});cancelled=wait(cancelled);assert cancelled['status']=='cancelled'
        rejected_project=call('projects/'+rejected_project['id'])
        stale=call('projects/'+rejected_project['id']+'/mechanics',config(rejected_project))
        rejected_project=call('projects/'+rejected_project['id'])
        rejected_project=call('projects/'+rejected_project['id']+'/appearance',{'revision':rejected_project['revision'],'builtin':'box','dimensions_mm':[280,90,80]})
        stale=wait(stale);assert stale['status']=='stale'
        assert not call('projects/'+rejected_project['id'])['rigid_bodies']
        atomic(destination/'lifecycle.json',{'rejected':rejected,'cancelled':cancelled,'stale':stale})
        # State guard: do not overwrite changed project with previous result.
        from studio.mechanical_service import finish
        assert not finish(server.store,{'id':project['id'],'revision':project['revision']-1},runs[0]['id'],a)
        summary={'passed':True,'project_id':project['id'],'same_revision_wiring_and_mechanics':True,'wiring_live_model_called':False,'cases':[{'run_id':r['id'],'source_sha256':r['result']['source_sha256'],
            'shape_hash':r['result']['geometry_assets'][0]['sha256'],'elapsed_s':r['result']['elapsed_s'],'checks':len(r['result']['checks']),
            'bounds_m':r['result']['geometry_assets'][0]['bounds_m'],'live_model_called':r['result']['plan']['live_model_called'],
            'model':r['result']['plan'].get('model'),'joint_limits_rad':r['result']['joints'][0]['limits_rad']} for r in runs]}
        atomic(destination/'summary.json',summary);print(json.dumps(summary,ensure_ascii=False,indent=2))
    finally:
        server.jobs.close();server.shutdown();server.server_close();thread.join(timeout=3)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--destination',required=True);parser.add_argument('--live-env')
    args=parser.parse_args();run(args.destination,args.live_env)
