"""Human-reviewable JSON snapshots, optimistic concurrency and atomic persistence."""
import copy
import fcntl
import json
import os
import threading
import hashlib
import shutil
import tempfile
from contextlib import contextmanager
from pathlib import Path
from .contracts import ContractError, ID, SELECTION_SCHEMA, now, uid, validate, loads
from .electrical import review

class Conflict(ContractError):
    pass


def atomic(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name('.'+path.name+'.'+uid('write'))
    try:
        with tmp.open('x',encoding='utf-8') as f:
            json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
        os.replace(tmp,path)
    finally:
        tmp.unlink(missing_ok=True)


class Store:
    def __init__(self, root, catalog):
        self.root=Path(root).resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.catalog=catalog;self.lock=threading.RLock()

    @contextmanager
    def transaction(self):
        with self.lock, (self.root/'.lock').open('a') as f:
            fcntl.flock(f,fcntl.LOCK_EX)
            try: yield
            finally: fcntl.flock(f,fcntl.LOCK_UN)

    def directory(self, identity):
        validate(identity,ID)
        p=self.root/identity
        if p.is_symlink(): raise ContractError('项目路径不得为符号链接')
        return p

    def _save(self,p):
        p['updated_at']=now()
        directory=self.directory(p['id'])
        atomic(directory/'history'/f"r{p['revision']}_{uid('snapshot')}.json",p)
        atomic(directory/'project.json',p)

    def create(self,name):
        if not isinstance(name,str) or not name.strip() or len(name)>120:
            raise ContractError('项目名称须为 1–120 个字符')
        p={'schema_version':'1.0','id':uid('project'),'name':name.strip(),'revision':1,
           'catalog_digest':self.catalog.digest,'created_at':now(),'updated_at':now(),
           'units':{'length':'m','mass':'kg','time':'s','angle':'rad','handedness':'right'},
           'intent':'A lightweight robot with fixed support; test sensing and wiring first.','instances':[],
           'source_shapes':[],'rigid_bodies':[],'joints':[],'mounts':[],'geometry_assets':[],
           'motion_requirements':[],'assembly_dependencies':[],'harnesses':[],
           'circuit':None,'validation':None,'active_run_id':None,'runs':[],
           'downstream':{k:'pending' for k in ['layout','harness','manufacturing','simulation','control']},
           'events':[{'at':now(),'type':'project_created','revision':1}]}
        with self.transaction(): self._save(p)
        return p

    def _load(self,identity):
        p=loads((self.directory(identity)/'project.json').read_text())
        if p['catalog_digest']!=self.catalog.digest:
            raise Conflict('目录版本已改变；请从旧项目导出并显式迁移，不能沿用旧校核')
        return p

    def load(self,identity):
        with self.transaction():
            return self._load(identity)

    def copy_complete_project_to(self,destination,identity):
        """Transfer an intact, idle project between local workspaces.

        Includes source bytes, immutable run artifacts and history. Never merge
        or overwrite existing identities, or import only the project JSON while
        silently losing its generated resources.
        """
        if self.root==destination.root:raise ContractError('来源与目标工作区相同')
        if self.catalog.digest!=destination.catalog.digest:raise Conflict('工作区部件目录不同，不能隐式迁移')
        stores=sorted([self,destination],key=lambda s:str(s.root))
        with stores[0].transaction(),stores[1].transaction():
            project=self._load(identity)
            if project.get('active_run_id'):raise Conflict('请等待项目任务结束后再复制')
            if destination.directory(identity).exists():raise Conflict('目标工作区已有同一项目，不会覆盖')
            runs=[]
            for run_id in project.get('runs',[]):
                validate(run_id,ID);folder=self.root/'runs'/run_id
                run=loads((folder/'run.json').read_text())
                if run['project_id']!=identity or run['status'] not in {'completed','rejected','failed','cancelled','timed_out','stale','interrupted'}:
                    raise Conflict('项目任务尚未结束或引用归属不一致')
                if (destination.root/'runs'/run_id).exists():raise Conflict('目标已有同一任务，不会覆盖')
                for asset in (run.get('result') or {}).get('geometry_assets',[]):
                    name=asset['file']
                    if Path(name).name!=name or hashlib.sha256((folder/name).read_bytes()).hexdigest()!=asset['sha256']:
                        raise ContractError('任务几何资源摘要不一致')
                runs.append((run_id,folder))
            for source in project.get('source_shapes',[]):
                if Path(source['file']).name!=source['file'] or hashlib.sha256((self.directory(identity)/'sources'/source['file']).read_bytes()).hexdigest()!=source['sha256']:
                    raise ContractError('源模型资源摘要不一致')
            incoming=destination.root/'.imports';incoming.mkdir(exist_ok=True)
            published=[]
            with tempfile.TemporaryDirectory(dir=incoming) as temporary:
                stage=Path(temporary)
                for name,folder in [(identity,self.directory(identity)),*runs]:
                    if any(f.is_symlink() for f in folder.rglob('*')):raise ContractError('不复制包含符号链接的项目资源')
                    shutil.copytree(folder,stage/name)
                try:
                    target_runs=destination.root/'runs';target_runs.mkdir(exist_ok=True)
                    for run_id,_ in runs:
                        target=target_runs/run_id;(stage/run_id).rename(target);published.append(target)
                    (stage/identity).rename(destination.directory(identity))
                except Exception:
                    for target in published:shutil.rmtree(target)
                    raise
            return project

    def list(self):
        with self.transaction():
            return [{'id':p['id'],'name':p['name'],'revision':p['revision'],'updated_at':p['updated_at']} for p in
                    (loads(f.read_text()) for f in sorted(self.root.glob('*/project.json')))]

    def select(self,identity,data):
        validate(data,SELECTION_SCHEMA)
        if data.get('power_config') and data['power_config']['voltage_v'] is not None:
            from .electrical import voltage_range
            voltage_range(data['power_config']['voltage_v'])
        ids=set()
        for instance in data['instances']:
            if instance['id'] in ids: raise ContractError('部件实例 ID 重复')
            ids.add(instance['id']);self.catalog.resolve(instance)
        with self.transaction():
            p=self._load(identity)
            if p['revision']!=data['revision']: raise Conflict('项目已在另一窗口修改；请重新打开最新版本')
            bodies={b['id'] for b in p['rigid_bodies']}
            if any(i['owner_body_id'] is not None and i['owner_body_id'] not in bodies for i in data['instances']):
                raise ContractError('安装归属引用不存在的刚体')
            from .mechanical_service import dependency_digest
            previous_mechanical_inputs=dependency_digest(p)
            old={i['id']:i for i in p['instances']}
            for i in data['instances']:
                if i['id'] in old and (i['catalog_id'],i['catalog_revision'])!=(old[i['id']]['catalog_id'],old[i['id']]['catalog_revision']):
                    raise ContractError('更换部件型号必须使用新实例 ID')
            p.update(instances=copy.deepcopy(data['instances']),intent=data['intent'],revision=p['revision']+1,
                     circuit=None,validation=None,active_run_id=None)
            if 'power_config' in data:p['power_config']=copy.deepcopy(data['power_config'])
            from .layout_service import invalidate as invalidate_layout
            invalidate_layout(p)
            if p.get('mechanical_result') and dependency_digest(p)!=previous_mechanical_inputs:
                from .mechanical_service import invalidate
                invalidate(p)
            # Motor bindings cannot retain dangling references after library edits.
            current_ids={i['id'] for i in p['instances']}
            p['motor_bindings']=[b for b in p.get('motor_bindings',[]) if b['motor_instance_id'] in current_ids]
            p['downstream']={k:'stale' for k in p['downstream']}
            p['events'].append({'at':now(),'type':'selection_changed','revision':p['revision'],'invalidated':['circuit',*p['downstream']]})
            self._save(p)
            return p

    def begin_run(self,identity,revision,run_id):
        with self.transaction():
            p=self._load(identity)
            if p['revision']!=revision: raise Conflict('任务输入版本已过期')
            p['active_run_id']=run_id;p['runs'].append(run_id)
            self._save(p);return p

    def finish_run(self,identity,revision,run_id,proposal,validation):
        with self.transaction():
            p=self._load(identity)
            if p['revision']!=revision or p['active_run_id']!=run_id:
                return False
            p['active_run_id']=None
            # Invalid proposals stay in run evidence, never enter the current circuit.
            p['harness_result']=None;p['harnesses']=[];p['downstream']['harness']='stale'
            p['validation']=validation
            p['circuit']=proposal if validation['logical_valid'] else None
            p['events'].append({'at':now(),'type':'wiring_checked','run_id':run_id,'revision':revision,'status':validation['status']})
            self._save(p);return True

    def detach_run(self,identity,run_id):
        with self.transaction():
            p=self._load(identity)
            if p['active_run_id']==run_id:
                p['active_run_id']=None;self._save(p)

    def import_design(self,raw):
        if not isinstance(raw,dict) or raw.get('schema_version')!='1.0' or raw.get('catalog_digest')!=self.catalog.digest:
            raise ContractError('仅接受当前目录版本的 1.0 项目；旧输出须通过专用只读适配器')
        if raw.get('units')!={'length':'m','mass':'kg','time':'s','angle':'rad','handedness':'right'}:
            raise ContractError('项目单位或坐标系不一致')
        for key in ['source_shapes','rigid_bodies','joints','mounts','geometry_assets','motion_requirements','assembly_dependencies','harnesses']:
            if raw.get(key)!=[]: raise ContractError('第一阶段尚不支持导入非空 '+key+'；禁止静默丢失数据')
        selection={'revision':1,'instances':raw.get('instances'),'intent':raw.get('intent')}
        validate(selection,SELECTION_SCHEMA)
        for i in selection['instances']: self.catalog.resolve(i)
        if len({i['id'] for i in selection['instances']})!=len(selection['instances']) or any(i['owner_body_id'] is not None for i in selection['instances']):
            raise ContractError('部件 ID 重复或引用不存在的刚体')
        p=self.create(raw.get('name'))
        p=self.select(p['id'],selection)
        # Imported claimed pass states are not trusted; a fresh run is required.
        return p
