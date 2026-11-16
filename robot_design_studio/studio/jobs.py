import json
import subprocess
import sys
import threading
import time
from pathlib import Path
from .contracts import ContractError, now, uid, loads, validate, ID
from .project import atomic
from .providers import live_config
from . import mechanical_service
from . import articulation_service
from . import layout_service
from . import simulation_service
from . import harness_service

APP=Path(__file__).resolve().parents[1]
TERMINAL={'completed','rejected','failed','cancelled','timed_out','stale','interrupted'}

def job_time_budget(project,kind,timeout):
    if kind not in {'mechanical','articulation','assembly','layout','simulation','harness'}:return timeout
    count=len((project.get('articulation') or {}).get('graph',{}).get('joints',[]))
    extra=90*(max(1,min(6,count))-1) if kind in {'assembly','layout'} else 0
    # Closed-shell access searches each body's removable parts and final
    # extraction paths. Bound the extra time by the declared graph size.
    bodies=len((project.get('articulation') or {}).get('graph',{}).get('bodies',[]))
    split_extra=60*min(12,bodies) if kind=='assembly' else 0
    return max(timeout,180+extra+split_extra)


class Jobs:
    def __init__(self,store,timeout=85):
        self.store=store;self.root=store.root/'runs';self.root.mkdir(exist_ok=True)
        self.timeout=timeout;self.lock=threading.RLock();self.processes={};self.cancelled=set()
        for f in self.root.glob('*/run.json'):
            r=loads(f.read_text())
            if r['status'] not in TERMINAL:
                r['status']='interrupted';r['events'].append({'at':now(),'type':'server_restarted'})
                atomic(f,r);self.store.detach_run(r['project_id'],r['id'])

    def get(self,identity):
        validate(identity,ID)
        with self.lock:
            r=loads((self.root/identity/'run.json').read_text())
            progress=self.root/identity/'progress.json'
            if progress.exists():r['progress']=loads(progress.read_text())
            return r

    def clone_at_height(self,identity,data):
        project,config,recovery=articulation_service.clone_at_height(self.store,identity,data)
        return self.start(project['id'],project['revision'],'reference',kind='articulation',
                          configuration={'revision':project['revision'],**config},recovery=recovery)

    def start_size_search(self, identity, data):
        from .size_search import targets
        if set(data) != {'revision', 'components'}:
            raise ContractError('尺寸搜索需要版本和完整部件包络')
        with self.lock:
            p = self.store.load(identity)
            if p['revision'] != data['revision'] or p.get('active_run_id'):
                raise ContractError('项目版本已改变或已有生成任务')
            if self.processes:
                raise ContractError('请先等待当前生成任务完成')
            targets(p)
            if not isinstance(data['components'], list) or not data['components']:
                raise ContractError('请先填写全部元件尺寸')
            run_id = uid('run')
            r = {'id': run_id, 'project_id': identity, 'design_revision': p['revision'],
                 'mode': 'reference', 'kind': 'size_search', 'status': 'queued',
                 'events': [{'at': now(), 'type': 'queued'}]}
            atomic(self.root/run_id/'run.json', r)
            self.processes[run_id] = None
            threading.Thread(target=self._size_search, args=(r,p,data['components']), daemon=True).start()
            return r

    def _size_search(self, run, project, components):
        from . import size_search
        jobs = self
        class LocalAPI:
            def get(self, path):
                kind, identity = path.split('/')
                if kind != 'runs': return jobs.store.load(identity)
                value = jobs.get(identity)
                if value['status'] in TERMINAL:
                    while identity in jobs.processes: time.sleep(.01)
                return value
            def post(self, path, data):
                kind, identity, action = path.split('/')
                if kind == 'runs': return jobs.cancel(identity)
                if action == 'clone-at-height': return jobs.clone_at_height(identity, data)
                if action == 'joint-motors': return articulation_service.bind_motors(jobs.store, identity, data)
                if action == 'selection': return jobs.store.select(identity, data)
                return jobs.start(identity, data['revision'], 'reference',
                                  kind='wiring' if action == 'generate' else action, configuration=data)
            def mark(self, identity, origin):
                with jobs.store.transaction():
                    p = jobs.store._load(identity)
                    p['events'].append({'at': now(), **origin})
                    jobs.store._save(p)
        identity = run['id']
        try:
            self.update(identity, 'running')
            result = size_search.run(LocalAPI(), project, components,
                lambda result: self.update(identity, 'running', result=result),
                lambda: identity in self.cancelled)
            atomic(self.root/identity/'result.json', result)
            self.update(identity, 'completed' if result['reason']=='fit' else 'rejected', result=result)
        except InterruptedError:
            self.update(identity, 'cancelled')
        except Exception as exc:
            self.update(identity, 'failed', error=str(exc) if isinstance(exc,ContractError) else type(exc).__name__)
        finally:
            with self.lock:
                self.processes.pop(identity, None); self.cancelled.discard(identity)

    def update(self,identity,status,**data):
        with self.lock:
            r=self.get(identity);r.update(status=status,**data)
            r['events'].append({'at':now(),'type':status});atomic(self.root/identity/'run.json',r)
            return r

    def retry_partition(self,project_id,data):
        if set(data)!={'revision','run_id'}:raise ContractError('重新分区参数无效')
        with self.lock:
            old=self.get(data['run_id']);p=self.store.load(project_id)
            result=old.get('result') or {}
            if old['project_id']!=project_id or old.get('kind')!='articulation' or old['status']!='rejected':raise ContractError('只能重算本项目已拒绝的标注分区')
            checks={c['name']:c['passed'] for c in result.get('checks',[])}
            if not checks.get('tree_and_internal_points') or not checks.get('multiview_reprojection'):raise ContractError('原关节标注未通过位置检查，不能直接复用')
            original=loads((self.root/old['id']/'input-project.json').read_text())
            if original['articulation_config']!=p.get('articulation_config') or result['source_sha256']!=p['source_shapes'][0]['sha256']:raise ContractError('源模型或标注参数已改变，请重新识别')
            configuration={**p['articulation_config'],'revision':data['revision']}
            recovery={'run_id':old['id'],'graph':result['graph'],'source':result['source'],'source_sha256':result['source_sha256']}
            return self.start(project_id,data['revision'],'reference',kind='articulation',configuration=configuration,recovery=recovery)

    def start(self,project_id,revision,mode,kind='wiring',configuration=None,recovery=None):
        if kind not in {'wiring','mechanical','articulation','assembly','layout','simulation','harness'}:raise ContractError('任务类型未知')
        if mode not in {'reference','live'}: raise ContractError('生成模式未知')
        if mode=='live' and not all(live_config().values()): raise ContractError('尚未配置在线 AI；可以使用明确标注的本地参考方案')
        if kind in {'mechanical','articulation','assembly','layout'}:mechanical_service.geometry_python()
        with self.lock:
            if len(self.processes)>=2: raise ContractError('最多同时运行两个任务，请等待或取消')
            if kind=='mechanical':revision=mechanical_service.configure(self.store,project_id,configuration)['revision']
            if kind=='articulation':revision=articulation_service.configure(self.store,project_id,configuration)['revision']
            if kind=='assembly':revision=articulation_service.configure_assembly(self.store,project_id,configuration)['revision']
            if kind=='layout':revision=layout_service.configure(self.store,project_id,configuration)['revision']
            if kind=='harness':revision=harness_service.configure(self.store,project_id,configuration)['revision']
            if kind=='simulation':revision=simulation_service.configure(self.store,project_id,configuration)['revision']
            run_id=uid('run');p=self.store.begin_run(project_id,revision,run_id)
            r={'id':run_id,'project_id':project_id,'design_revision':revision,'mode':mode,'kind':kind,'status':'queued','events':[{'at':now(),'type':'queued'}]}
            if recovery is not None:r['annotation_recovery']=recovery
            atomic(self.root/run_id/'run.json',r)
            atomic(self.root/run_id/'input.json',p)
            self.processes[run_id]=None
            threading.Thread(target=self._work,args=(r,p),daemon=True).start()
            return r

    def _work(self,run,project):
        identity=run['id'];directory=self.root/identity;process=None
        try:
            mechanical=run.get('kind') in {'mechanical','articulation','assembly','layout','simulation','harness'}
            articulation=run.get('kind')=='articulation'
            interpreter=mechanical_service.geometry_python() if mechanical else sys.executable
            if run.get('kind')=='simulation':interpreter=simulation_service.interpreter()
            worker='studio.mechanical_worker' if mechanical else 'studio.worker'
            process=subprocess.Popen([interpreter,'-m',worker],cwd=APP,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            with self.lock: self.processes[identity]=process
            budget=job_time_budget(project,run.get('kind'),self.timeout)
            self.update(identity,'running',time_budget_s=budget)
            request={'project':project,'mode':run['mode'],'kind':run.get('kind'),'destination':str(directory/'result.json')}
            if run.get('annotation_recovery'):request['annotation_recovery']=run['annotation_recovery']
            if mechanical:request['source_path']=str(self.store.directory(project['id'])/'sources'/project['source_shapes'][0]['file'])
            if run.get('kind')=='layout':
                source_run=project['layout_config']['mechanical_run_id'];validate(source_run,ID)
                request['mechanical_directory']=str(self.root/source_run)
            if run.get('kind') in {'simulation','harness'}:
                source_run=project['layout_result']['run_id'];validate(source_run,ID)
                request['layout_directory']=str(self.root/source_run)
            payload=json.dumps(request).encode()
            start=time.monotonic();first=True;status=None
            while True:
                with self.lock:
                    if identity in self.cancelled: status='cancelled'
                if time.monotonic()-start>budget: status='timed_out'
                if status:
                    process.kill();process.communicate();break
                try:
                    stdout,stderr=process.communicate(input=payload if first else None,timeout=.1)
                    if mechanical:(directory/'worker.log').write_bytes(stderr[-32000:])
                    break
                except subprocess.TimeoutExpired:
                    first=False
            if status:
                self.update(identity,status,exit_code=process.returncode)
            elif process.returncode!=0:
                self.update(identity,'failed',exit_code=process.returncode,error='生成进程异常退出；未采用任何输出')
            else:
                result=loads((directory/'result.json').read_text())
                if not mechanical:last=result['attempts'][-1]
                # Cancel and commit serialize through the same lock.
                with self.lock:
                    if identity in self.cancelled:
                        self.update(identity,'cancelled',exit_code=process.returncode)
                    else:
                        if run.get('kind')=='harness':
                            accepted=harness_service.finish(self.store,project,identity,result)
                            valid=result.get('routing_valid',False)
                        elif run.get('kind')=='simulation':
                            accepted=simulation_service.finish(self.store,project,identity,result)
                            valid=result.get('export_valid',False)
                        elif run.get('kind')=='layout':
                            accepted=layout_service.finish(self.store,project,identity,result)
                            valid=result.get('layout_valid',False)
                        elif articulation:
                            accepted=articulation_service.finish(self.store,project,identity,result)
                            valid=result['annotation_valid']
                        elif mechanical:
                            accepted=mechanical_service.finish(self.store,project,identity,result)
                            valid=result['geometry_valid'] or result.get('shell_partition_valid',False)
                        else:
                            accepted=self.store.finish_run(project['id'],project['revision'],identity,last['proposal'],last['validation'])
                            valid=last['validation']['logical_valid']
                        self.update(identity,('completed' if valid else 'rejected') if accepted else 'stale',
                                    exit_code=process.returncode,result=result,elapsed_s=round(time.monotonic()-start,3))
        except Exception as exc:
            if process and process.poll() is None: process.kill();process.communicate()
            self.update(identity,'failed',error='任务异常: '+type(exc).__name__)
        finally:
            self.store.detach_run(project['id'],identity)
            with self.lock:
                self.processes.pop(identity,None);self.cancelled.discard(identity)

    def cancel(self,identity):
        with self.lock:
            r=self.get(identity)
            if r['status'] not in TERMINAL:
                self.cancelled.add(identity)
            return r

    def close(self):
        with self.lock: self.cancelled.update(self.processes)
        deadline=time.monotonic()+3
        while self.processes and time.monotonic()<deadline: time.sleep(.05)
