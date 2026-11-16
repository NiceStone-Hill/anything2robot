"""Formal API workflow on an isolated copy; reuse annotated graph and reviewed nets.

Creates new geometry jobs, independent export checks, and a local test page.
Never requests an online model or changes the original example workspace.
"""
import argparse
import copy
import hashlib
import io
import json
import os
import shutil
import threading
import time
import urllib.request
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
import trimesh as tm
from studio.server import make_server
from studio.project import atomic
from studio.electrical import review
from studio.catalog import Catalog
from studio.contracts import now

APP=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8770)
    parser.add_argument('--keep-serving',action='store_true')
    args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    root=out/'workspace';source=APP/'workspace/internal_layout_demo_2026_11_15'
    identity='project_95d1e73d971c4ba1';source_project=source/identity
    original=(source_project/'project.json').read_bytes();project=json.loads(original)
    if root.exists():raise ValueError('Use a new output directory; test evidence is immutable')
    root.mkdir();shutil.copytree(source_project,root/identity)
    annotation=project['articulation']['run_id']
    shutil.copytree(source/'runs'/annotation,root/'runs'/annotation)
    atomic(out/'source-project.json',project)
    catalog=Catalog()
    previous_catalog=json.loads((APP/'catalog/history'/(project['catalog_digest']+'.json')).read_text())
    old_parts={(p['id'],p['revision']):p for p in previous_catalog['parts']}
    compared=[]
    for instance in project['instances']:
        old=old_parts[(instance['catalog_id'],instance['catalog_revision'])]
        resolved=catalog.resolve(instance)
        assert old==resolved,('Selected catalog revision changed',instance['id'])
        compared.append({'instance_id':instance['id'],'catalog_id':instance['catalog_id'],
                         'revision':instance['catalog_revision'],'content_unchanged':True})
    atomic(out/'catalog-migration.json',{'from_digest':project['catalog_digest'],'to_digest':catalog.digest,
        'selection_revision_content_unchanged':compared,'scope':'Explicit test-copy migration; preserve selected part revisions and revalidate electrical nets.'})
    project['catalog_digest']=catalog.digest;project['circuit']['catalog_digest']=catalog.digest
    requirements=copy.deepcopy(project['layout_config']['components'])
    harness_config=copy.deepcopy(project['harness_config']);sim_config=copy.deepcopy(project['simulation_config'])
    old_mech=json.loads((source/'runs'/project['mechanical_result']['run_id']/'result.json').read_text())
    project['name']='Full test · open SG90 · 500 mm'
    project['revision']+=1;project['active_run_id']=None;project['runs']=[annotation]
    project['articulation_config']['mode']='reference'
    project['surface_plan_reuse']={'source_sha256':project['source_shapes'][0]['sha256'],
       'graph':copy.deepcopy(project['articulation']['graph']),'articulation_config':copy.deepcopy(project['articulation_config']),
       'source_run_id':old_mech.get('run_id','archived-mechanical-regions'),'plan':old_mech['analysis']['access_suggestions']}
    project['events']=[{'at':now(),'type':'full_workflow_test_copy','source_project_id':identity,
                       'source_revision':json.loads(original)['revision'],'annotation_reused':True,'electrical_nets_reused':True}]
    for key in ['mechanical_result','layout_result','harness_result','simulation_result']:project[key]=None
    for key in ['geometry_assets','rigid_bodies','joints','mounts','component_placements','layout_assets','harnesses','assembly_dependencies']:project[key]=[]
    for instance in project['instances']:instance['owner_body_id']=None
    project.pop('layout_config',None)
    project['circuit']['design_revision']=project['revision']
    project['downstream']={k:'pending' for k in project['downstream']}
    server=make_server(root,args.port);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    project['validation']=review(project,project['circuit'],server.store.catalog)
    assert project['validation']['logical_valid'],project['validation']
    with server.store.transaction():server.store._save(project)
    base=f'http://127.0.0.1:{server.server_port}/';token=''
    summary={'status':'running','url':base,'project_id':identity,'name':project['name'],
       'scope':'Formal API test: reused existing joint annotation and electrical nets; freshly validated wiring and regenerated assembly, electronics layout, bundles, model export and review package.',
       'online_model_called':False,'source_project_sha256':hashlib.sha256(original).hexdigest(),
       'stages':[],'manufacturing_verified':False,'simulation_performed':False}
    atomic(out/'summary.json',summary)
    def call(path,data=None,raw=False):
        request=urllib.request.Request(base+'api/'+path,data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type':'application/json','X-Studio-Token':token} if data is not None else {})
        with urllib.request.urlopen(request,timeout=60) as response:return response.read() if raw else json.load(response)
    def stage(action,configuration,flag):
        p=call('projects/'+identity)
        run=call('projects/'+identity+'/'+action,{'revision':p['revision'],'mode':'reference',**configuration})
        print('START',action,run['id'],flush=True)
        atomic(out/(action+'-request.json'),run)
        deadline=time.monotonic()+650
        while time.monotonic()<deadline:
            run=call('runs/'+run['id'])
            if run['status'] in {'completed','rejected','failed','timed_out','stale','cancelled'}:break
            time.sleep(.5)
        atomic(out/(action+'-run.json'),run)
        result=run.get('result') or {};failed=[c['name'] for c in result.get('checks',[]) if not c['passed']]
        summary['stages'].append({'stage':action,'run_id':run['id'],'status':run['status'],
           'valid':bool(result.get(flag)),'checks':len(result.get('checks',[])),'failed_checks':failed,'elapsed_s':run.get('elapsed_s')})
        atomic(out/'summary.json',summary)
        assert run['status']=='completed' and result.get(flag),(action,run['status'],failed,result.get('error'),run.get('error'))
        print('PASS',action,len(result.get('checks',[])),'checks',flush=True)
        return result,root/'runs'/run['id']
    try:
        token=call('session')['token']
        validation=call('projects/'+identity+'/review',project['circuit'])
        atomic(out/'electrical-review.json',validation);assert validation['logical_valid']
        summary['stages'].append({'stage':'electrical_review','valid':True,'pending':len(validation['pending'])})
        atomic(out/'summary.json',summary)
        mechanics,mechanical_dir=stage('assembly',{'service_components':requirements},'geometry_valid')
        layout,layout_dir=stage('layout',{'components':requirements},'layout_valid')
        harness,harness_dir=stage('harness',harness_config,'routing_valid')
        simulation,simulation_dir=stage('simulation',sim_config,'export_valid')
        p=call('projects/'+identity);assert p['circuit']['nets']==json.loads(original)['circuit']['nets']
        assert p['validation']['logical_valid'] and len(p['component_placements'])==5
        independent=[]
        for label,result,directory in [('assembly',mechanics,mechanical_dir),('layout',layout,layout_dir)]:
            for asset in result['geometry_assets']:
                file=directory/asset['file'];assert hashlib.sha256(file.read_bytes()).hexdigest()==asset['sha256']
                mesh=tm.load(file,force='mesh');assert mesh.is_volume and mesh.is_watertight,(label,asset['id'])
                independent.append({'stage':label,'file':file.name,'watertight':True,'positive_volume':True,'sha256_matches':True})
        for file in simulation_dir.glob('*.stl'):
            mesh=tm.load(file,force='mesh');assert mesh.is_volume and mesh.is_watertight,file.name
            independent.append({'stage':'simulation','file':file.name,'watertight':True,'positive_volume':True})
        for name in ['robot.urdf','robot.xml']:
            tree=ET.parse(simulation_dir/name)
            for node in tree.iter():
                for key in ['filename','file']:
                    resource=node.get(key)
                    if resource and not resource.startswith(('package:','http:','https:')):
                        assert (simulation_dir/resource).is_file(),(name,resource)
        review_bytes=call('projects/'+identity+'/export',raw=True)
        (out/'full-test-review.zip').write_bytes(review_bytes)
        with zipfile.ZipFile(io.BytesIO(review_bytes)) as package:
            assert package.testzip() is None
            names=package.namelist()
            assert {'internal-assembly/manifest.json','simulation/robot.xml','simulation/robot.urdf'}<=set(names)
        for folder,files in [(harness_dir,['assembly-with-wires.glb','wire-list.csv']),(simulation_dir,['simulation-model.zip'])]:
            for name in files:
                if (folder/name).exists():shutil.copy2(folder/name,out/name)
        atomic(out/'project.json',p);atomic(out/'independent-verification.json',{'assets':independent,'zip_entries':len(names),'xml_resources_valid':True,'electrical_nets_preserved':True})
        assert (source_project/'project.json').read_bytes()==original,'Original project changed during test'
        summary.update(status='passed',revision=p['revision'],zip_entries=len(names),
            layout_components=len(p['component_placements']),reference_models=len(layout.get('visual_assets',[])),
            routes=len(harness.get('routes',[])),routing_mode=harness.get('routing_mode'),
            original_project_unchanged=True,completed_at=now())
        atomic(out/'summary.json',summary)
        print('FULL_WORKFLOW_PASSED',base,identity,flush=True)
    except Exception as exc:
        summary.update(status='failed',error=str(exc));atomic(out/'summary.json',summary)
        print('FULL_WORKFLOW_FAILED',repr(exc),flush=True)
        if not args.keep_serving:raise
    finally:
        if args.keep_serving:
            print('Test workspace remains available at',base,flush=True)
            try:thread.join()
            except KeyboardInterrupt:pass
        server.jobs.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':main()
