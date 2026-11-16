"""Create a real software-only example in a new output directory."""
import argparse
import json
import time
from pathlib import Path
from .catalog import Catalog
from .contracts import now
from .electrical import review
from .export import bundle
from .jobs import Jobs, TERMINAL
from .project import Store, atomic


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():parser.error('输出路径已存在；请选择新目录，保留旧结果')
    args.output.mkdir(parents=True)
    c=Catalog();s=Store(args.output/'workspace',c);p=s.create('Pico 2 + MPU6050 + SG90 接线阶段样例')
    ids=['pico2_unknown','mpu6050_lab','sg90_digital_towerpro','supply_5_candidate','supply_48_candidate','splitter_candidate']+['wire_candidate']*7
    instances=[{'id':f'component_{n+1:02}','catalog_id':k,'catalog_revision':'1','label':c.parts[k]['name'],'locked':False,'owner_body_id':None} for n,k in enumerate(ids)]
    p=s.select(p['id'],{'revision':1,'instances':instances,'intent':'桌面支撑、轻型附肢慢速摆动；负载与校准尚未验证'})
    j=Jobs(s);run=j.start(p['id'],p['revision'],'reference')
    try:
        for _ in range(200):
            run=j.get(run['id'])
            if run['status'] in TERMINAL:break
            time.sleep(.05)
        if run['status']!='completed':raise RuntimeError('样例任务失败')
    finally:j.close()
    p=s.load(p['id']);atomic(args.output/'project.json',p)
    atomic(args.output/'run.json',run)
    (args.output/'design-review.zip').write_bytes(bundle(p,c))
    bad=json.loads(json.dumps(p['circuit']));bad['nets'][1]['endpoints'][0]['port_id']='GP99'
    rejection=review(p,bad,c);assert not rejection['logical_valid']
    atomic(args.output/'invalid-proposal.json',bad);atomic(args.output/'rejection.json',rejection)
    atomic(args.output/'summary.json',{'at':now(),'status':'software_workflow_passed','project_id':p['id'],
        'run_id':run['id'],'instance_count':len(p['instances']),'net_count':len(p['circuit']['nets']),
        'logical_errors':len(p['validation']['errors']),'pending_count':len(p['validation']['pending']),
        'invalid_pin_rejected':True,'live_model_called':False,'manufacturing_released':False,'physical_tested':False})
    print(json.dumps(json.loads((args.output/'summary.json').read_text()),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
