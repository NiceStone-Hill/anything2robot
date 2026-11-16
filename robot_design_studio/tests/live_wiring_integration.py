"""Opt-in real HTTPS wiring provider through the official local HTTP task entry."""
import argparse
import json
import threading
import time
import urllib.request
from pathlib import Path
from studio.server import make_server
from studio.providers import load_ai_env
from studio.project import atomic


def run(destination,env):
    destination=Path(destination).resolve();load_ai_env(env)
    server=make_server(destination/'workspace',0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    token='';base=f'http://127.0.0.1:{server.server_port}/api/'
    def call(path,data=None):
        req=urllib.request.Request(base+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(req,timeout=10) as response:return json.load(response)
    try:
        token=call('session')['token'];catalog={p['id']:p for p in call('catalog')['parts']}
        project=call('projects',{'name':'真实在线接线校核'})
        selected=['pico2_unknown','mpu6050_lab','sg90_digital_towerpro','supply_48_candidate','supply_5_candidate','wire_candidate','splitter_candidate']
        instances=[{'id':f'part_{index}','catalog_id':identity,'catalog_revision':catalog[identity]['revision'],'label':catalog[identity]['name'],'locked':False,'owner_body_id':None} for index,identity in enumerate(selected)]
        project=call('projects/'+project['id']+'/selection',{'revision':project['revision'],'instances':instances,'intent':'读取姿态并温和摆动。仅形成候选逻辑接线，实际电源、接头和电平待核实。'})
        task=call('projects/'+project['id']+'/generate',{'revision':project['revision'],'mode':'live'})
        deadline=time.monotonic()+95
        while time.monotonic()<deadline:
            task=call('runs/'+task['id'])
            if task['status'] in {'completed','rejected','failed','timed_out'}:break
            time.sleep(.2)
        atomic(destination/'run.json',task)
        assert task['status']=='completed',task
        assert task['result']['live_model_called']
        validation=task['result']['attempts'][-1]['validation']
        assert validation['logical_valid'] and not validation['manufacturing_released']
        atomic(destination/'summary.json',{'passed':True,'model':task['result']['model'],'run_id':task['id'],'attempts':len(task['result']['attempts']),
            'logical_valid':True,'physical_verified':False,'scope':'真实在线电气逻辑单模块验收；不代表完整机器人贯通'})
        print(json.dumps({'passed':True,'model':task['result']['model'],'attempts':len(task['result']['attempts'])}))
    finally:server.jobs.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('destination');parser.add_argument('--live-env',required=True);a=parser.parse_args();run(a.destination,a.live_env)
