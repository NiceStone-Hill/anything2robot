"""Run an actual user STL through formal HTTP tasks; archive failures honestly."""
import argparse
import base64
import hashlib
import json
import time
import urllib.request
from pathlib import Path
from studio.project import atomic


def run(source, destination, size_mm=300, port=8768):
    source=Path(source);out=Path(destination);out.mkdir(parents=True,exist_ok=False)
    base=f'http://127.0.0.1:{port}/api/';token=''
    def call(path,data=None,raw=False):
        req=urllib.request.Request(base+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(req,timeout=30) as response:return response.read() if raw else json.load(response)
    def task(stage,path,data):
        r=call(path,data);print(stage,r['id'],flush=True)
        deadline=time.monotonic()+210
        while time.monotonic()<deadline:
            r=call('runs/'+r['id'])
            if r['status'] in {'completed','rejected','failed','cancelled','timed_out','stale','interrupted'}:break
            time.sleep(.4)
        else:raise TimeoutError(stage+' still running; no output adopted')
        atomic(out/(stage+'.json'),r)
        summary['stages'][stage]={'run_id':r['id'],'status':r['status'],'elapsed_s':r.get('elapsed_s'),
            'failed_checks':[c for c in (r.get('result') or {}).get('checks',[]) if not c.get('passed')],
            'error':(r.get('result') or {}).get('error') or r.get('error')}
        atomic(out/'summary.json',summary);print(stage,r['status'],flush=True);return r
    token=call('session')['token'];raw=source.read_bytes()
    p=call('projects',{'name':f'Tripo 用户模型实测 · {size_mm}mm · 单颈关节'})
    identity=p['id'];path='projects/'+identity
    summary={'project_id':identity,'project_name':p['name'],'source_sha256':hashlib.sha256(raw).hexdigest(),
        'source_filename':source.name,'size_mm':size_mm,'stages':{},'full_workflow_complete':False,
        'physical_verified':False,'simulation_performed':False,'control_generated':False,'harness_implemented':False}
    atomic(out/'summary.json',summary)
    p=call(path+'/appearance',{'revision':p['revision'],'filename':source.name,'unit':'mm','data_base64':base64.b64encode(raw).decode()})
    r=task('articulation',path+'/articulation',{'revision':p['revision'],'mode':'live','length_m':size_mm/1000,
        'up_axis':'+Z','support':'fixed_base','max_joints':1,
        'intent':'身体固定，只让整个头部相对身体左右转动，只生成一个颈部关节。耳朵随头一起运动；四肢和尾巴均保持固定。保持原外观。'})
    if not (r.get('result') or {}).get('annotation_valid'):
        print(json.dumps(summary,ensure_ascii=False),flush=True);return
    p=call(path)
    p=call(path+'/joint-motors',{'revision':p['revision'],'choices':[{'joint_id':j['id'],'catalog_id':'sg90_digital_towerpro'} for j in p['articulation']['graph']['joints']]})
    r=task('assembly',path+'/assembly',{'revision':p['revision'],'mode':'reference'})
    structure_valid=bool((r.get('result') or {}).get('geometry_valid'))
    for filename in ['mechanical-review.zip','assembly.glb','manifest.json']:
        if filename in (r.get('result') or {}).get('artifacts',[]):
            (out/filename).write_bytes(call('runs/'+r['id']+'/artifacts/'+filename,raw=True))
    # Electrical testing is independent. Never imply it repairs rejected geometry.
    p=call(path);catalog=call('catalog');parts={part['id']:part for part in catalog['parts']}
    instances=p['instances'][:]
    for name in ['pico2_unknown','mpu6050_lab']:
        instances.append({'id':'part_'+name,'catalog_id':name,'catalog_revision':parts[name]['revision'],
            'label':parts[name]['name'],'locked':False,'owner_body_id':None})
    p=call(path+'/selection',{'revision':p['revision'],'instances':instances,'intent':'Pico 2 控制一个 SG90 颈部关节，MPU6050 用于感知；供电与实物端口未知，明确列出缺件。'})
    r=task('wiring',path+'/generate',{'revision':p['revision'],'mode':'live'})
    if structure_valid:
        p=call(path);root=p['articulation']['graph']['root_body_id']
        profiles=[{'instance_id':i['id'],'dimensions_m':[.055,.025,.012] if i['catalog_id']=='pico2_unknown' else [.025,.020,.010],
            'owner_body_id':root,'evidence':'engineering_assumption','note':'用户模型软件测试完整包络假设，实物板卡、连接器及紧固参数未知。'}
            for i in p['instances'] if i['catalog_id'] in {'pico2_unknown','mpu6050_lab'}]
        r=task('layout',path+'/layout',{'revision':p['revision'],'mode':'reference','components':profiles})
        if (r.get('result') or {}).get('layout_valid'):
            p=call(path)
            r=task('model_export',path+'/simulation',{'revision':p['revision'],'mode':'reference',
                'printed_density_kg_m3':1200,'hardware_density_kg_m3':1000,'assumed_torque_nm':1})
            if (r.get('result') or {}).get('export_valid'):
                (out/'simulation-model.zip').write_bytes(call('runs/'+r['id']+'/artifacts/simulation-model.zip',raw=True))
    else:summary['blocked_stages']=['内部电子装配：机械候选未通过','仿真模型导出：无有效内部装配']
    (out/'project-review.zip').write_bytes(call(path+'/export',raw=True))
    atomic(out/'project.json',call(path));atomic(out/'summary.json',summary)
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source');parser.add_argument('destination');parser.add_argument('--size-mm',type=int,default=300)
    args=parser.parse_args();run(args.source,args.destination,args.size_mm)
