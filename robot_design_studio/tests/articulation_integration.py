"""Real HTTP + geometry worker + optional real VLM; only synthetic views sent."""
import argparse
import base64
import json
import threading
import time
import urllib.request
from pathlib import Path
import trimesh as tm
from studio.server import make_server
from studio.providers import load_ai_env
from studio.project import atomic
from studio.mechanics.solid import box, union


def run(destination, live_env=None, assembly=False):
    destination=Path(destination).resolve();destination.mkdir(parents=True,exist_ok=True)
    if live_env: load_ai_env(live_env)
    server=make_server(destination/'workspace',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base=f'http://127.0.0.1:{server.server_port}/api/';token=''
    def call(path,data=None):
        request=urllib.request.Request(base+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(request,timeout=10) as response:return json.load(response)
    def wait(task):
        deadline=time.monotonic()+185
        while time.monotonic()<deadline:
            task=call('runs/'+task['id'])
            if task['status'] in {'completed','rejected','failed','cancelled','timed_out','stale'}:return task
            time.sleep(.15)
        raise AssertionError('annotation timeout')
    token=call('session')['token'];results=[]
    try:
        # Concave branching silhouette, then a U shape and detached small solid.
        shapes=[union(box([-35,-25,-100],[35,25,100]),box([-105,-25,15],[105,25,65])),
                tm.util.concatenate([union(box([-90,-25,-80],[-35,25,80]),box([35,-25,-80],[90,25,80]),box([-90,-25,-80],[90,25,-25])),box([103,-10,-60],[115,10,-45])])]
        for index,shape in enumerate(shapes):
            project=call('projects',{'name':f'多关节外观验收 {index+1}'})
            assert project['instances']==[]
            payload=shape.export(file_type='stl')
            project=call('projects/'+project['id']+'/appearance',{'revision':project['revision'],'filename':f'分叉与凹形-{index}.stl','unit':'mm','data_base64':base64.b64encode(payload).decode()})
            task=wait(call('projects/'+project['id']+'/articulation',{'revision':project['revision'],'mode':'live' if live_env else 'reference',
                'intent':'底部固定，识别左右上方两条分支，各安排一个关节让它们轻轻前后摆动。独立小块保留在底座侧，不作为关节。',
                'length_m':(.36 if index else .32) if assembly else (.28 if index else .24),'up_axis':'+Z','support':'fixed_base','max_joints':3}))
            atomic(destination/f'case-{index+1}.json',task)
            assert task['status']=='completed',task.get('result',task)
            result=task['result'];assert result['annotation_valid']
            assert result['analysis']['all_components_preserved'] and not result['analysis']['global_convex_hull_used']
            assert result['analysis']['source_components']==index+1
            if live_env:assert result['source']['live_model_called']
            project=call('projects/'+project['id']); assert project['instances']==[], 'VLM must not require preselected motors'
            choices=[{'joint_id':j['id'],'catalog_id':'sg90_digital_towerpro' if i%2==0 else 'mg4005v2_candidate'} for i,j in enumerate(result['graph']['joints'])]
            project=call('projects/'+project['id']+'/joint-motors',{'revision':project['revision'],'choices':choices})
            assert len(project['motor_bindings'])==len(choices)
            assert all(b['annotation_run_id']==task['id'] for b in project['motor_bindings'])
            assembly_task=None
            if assembly:
                assembly_task=wait(call('projects/'+project['id']+'/assembly',{'revision':project['revision'],'mode':'reference'}))
                atomic(destination/f'assembly-{index+1}.json',assembly_task)
                assert assembly_task['status'] in {'completed','rejected'},assembly_task
                assert assembly_task['result'].get('pipeline')=='graph_assembly_0.3',assembly_task
                assert len(assembly_task['result']['geometry_assets'])>=8
                assert len(assembly_task['result']['placement_candidates'])==len(choices)
            results.append({'assembly_run_id':assembly_task['id'] if assembly_task else None,'assembly_geometry_valid':assembly_task['result']['geometry_valid'] if assembly_task else None,'run_id':task['id'],'project_id':project['id'],'source_sha256':result['source_sha256'],'joints':len(result['graph']['joints']),
                'axes':[j['axis'] for j in result['graph']['joints']],'components':result['analysis']['source_components'],
                'partitions':[p['sha256'] for p in result['partitions']],'source':result['source'],'elapsed_s':task['elapsed_s']})
        assert results[0]['source_sha256']!=results[1]['source_sha256']
        assert results[0]['partitions']!=results[1]['partitions']
        atomic(destination/'summary.json',{'passed':True,'cases':results,'hardware_preselected':False,'manufacturing_verified':False})
        print(json.dumps({'passed':True,'destination':str(destination),'cases':results},ensure_ascii=False))
    finally:server.jobs.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('destination');p.add_argument('--live-env');p.add_argument('--assembly',action='store_true');a=p.parse_args();run(a.destination,a.live_env,a.assembly)
