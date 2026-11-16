"""Standard-library boundary for project inputs and isolated geometry tasks."""
import base64
import hashlib
import math
import os
import json
from pathlib import Path
from .contracts import ContractError, now, uid, validate, INTEGER
from .project import Conflict

APP=Path(__file__).resolve().parents[1]


def dependency_digest(project):
    """Mechanical inputs exclude later electronics choices and global revision."""
    actuator_ids={'sg90_digital_towerpro','mg4005v2_candidate','st3215_12v_candidate'}
    inputs={'source_shapes':project.get('source_shapes',[]),
        'actuators':sorted((i['id'],i['catalog_id'],i['catalog_revision']) for i in project['instances'] if i['catalog_id'] in actuator_ids),
        'mechanical_config':project.get('mechanical_config'),
        'articulation_config':project.get('articulation_config'),
        'graph':(project.get('articulation') or {}).get('graph'),
        'motor_bindings':project.get('motor_bindings',[])}
    if project.get('service_opening_requirements'):inputs['service_opening_requirements']=project['service_opening_requirements']
    if project.get('assembly_stage','complete')!='complete':inputs['assembly_stage']=project['assembly_stage']
    if project.get('assembly_policy','strict')!='strict':inputs['assembly_policy']=project['assembly_policy']
    plan=(project.get('surface_plan_reuse') or {}).get('plan') or {}
    if plan.get('plan_kind')=='assembly_parting_v1':inputs['parting_regions']=plan.get('regions')
    return hashlib.sha256(json.dumps(inputs,sort_keys=True,ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def geometry_python():
    candidate=Path(os.environ.get('ROBOT_GEOMETRY_PYTHON',str(APP.parent/'.conda-env/voxel_tool/bin/python')))
    if not candidate.is_file():raise ContractError('几何环境不存在；请按 README 配置 ROBOT_GEOMETRY_PYTHON')
    return str(candidate.resolve())


def number(value,lo,hi,label):
    if type(value) not in (float,int) or not math.isfinite(value) or not lo<=value<=hi:
        raise ContractError(label+f' 必须在 {lo}–{hi} 之间')
    return value


def appearance(store,identity,data):
    validate(data.get('revision'),INTEGER)
    if set(data)-{'revision','filename','unit','data_base64','builtin','dimensions_mm'}:raise ContractError('未知外观字段')
    unit=data.get('unit','mm')
    if unit not in {'m','mm','cm'}:raise ContractError('外观单位须为 m、cm 或 mm')
    if data.get('builtin')=='box':
        dims=data.get('dimensions_mm',[240,90,80])
        if not isinstance(dims,list) or len(dims)!=3:raise ContractError('需要三项外观尺寸')
        for v in dims:number(v,20,500,'外观尺寸 mm')
        # A real closed source OBJ, later processed exactly like uploaded input.
        xyz=[[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]
        faces=[[1,3,2],[1,4,3],[5,6,7],[5,7,8],[1,2,6],[1,6,5],[4,8,7],[4,7,3],[1,5,8],[1,8,4],[2,3,7],[2,7,6]]
        raw=('\n'.join('v '+' '.join(str(v*d/2) for v,d in zip(p,dims)) for p in xyz)+'\n'+'\n'.join('f '+' '.join(map(str,f)) for f in faces)).encode()
        name='简单外观.obj';suffix='obj';unit='mm'
    else:
        name=data.get('filename')
        if not isinstance(name,str) or len(name)>200 or '/' in name or '\\' in name:raise ContractError('外观文件名无效')
        suffix=Path(name).suffix.lower().lstrip('.')
        if suffix not in {'stl','obj'}:raise ContractError('首版支持 STL 或 OBJ 外观')
        try:raw=base64.b64decode(data.get('data_base64',''),validate=True)
        except Exception:raise ContractError('外观文件编码无效') from None
        if not 1<=len(raw)<=40_000_000:raise ContractError('外观文件须为 1–40000000 字节；读取后另按面数和体素预算检查')
        # OBJ is geometry-only: reject external materials; no network or file references.
        if suffix=='obj' and any(line.split() and line.split()[0] in {b'mtllib',b'call',b'include'} for line in raw.splitlines()):
            raise ContractError('请导出不含外部材料或脚本引用的几何 OBJ')
    with store.transaction():
        p=store._load(identity)
        if p['revision']!=data['revision']:raise Conflict('外观输入版本已过期')
        asset_id=uid('shape');file=store.directory(identity)/'sources'/(asset_id+'.'+suffix)
        file.parent.mkdir(exist_ok=True);file.write_bytes(raw)
        p['source_shapes']=[{'id':asset_id,'name':name,'file':file.name,'format':suffix,'unit':unit,'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)}]
        invalidate(p);p['articulation']=None;p['motor_bindings']=[];p['revision']+=1
        p['events'].append({'at':now(),'type':'appearance_imported','revision':p['revision'],'source_id':asset_id})
        store._save(p);return p


def invalidate(p):
    from .layout_service import invalidate as invalidate_layout
    invalidate_layout(p)
    p['mechanical_result']=None;p['active_run_id']=None
    # Global project revision is the validation boundary; snapshots retain old wiring.
    p['circuit']=None;p['validation']=None
    for key in ['rigid_bodies','joints','mounts','geometry_assets','assembly_dependencies']:p[key]=[]
    p['downstream']={k:'stale' for k in p['downstream']}


def configure(store,identity,data):
    if set(data)!={'revision','length_m','amplitude_deg','motor_instance_id','mode','intent'}:raise ContractError('机械任务参数不完整或有未知字段')
    validate(data['revision'],INTEGER)
    number(data['length_m'],.18,.4,'外观长度 m');number(data['amplitude_deg'],5,25,'摆动角度')
    if data['mode'] not in {'reference','live'}:raise ContractError('推荐来源无效')
    if not isinstance(data['intent'],str) or len(data['intent'])>4000:raise ContractError('运动意图过长')
    with store.transaction():
        p=store._load(identity)
        if p['revision']!=data['revision']:raise Conflict('机械输入版本已过期')
        if not p['source_shapes']:raise ContractError('请先导入外观')
        motor=next((i for i in p['instances'] if i['id']==data['motor_instance_id']),None)
        if not motor or motor['catalog_id']!='sg90_digital_towerpro':raise ContractError('请从部件库明确选择一个 SG90 Digital 候选实例')
        invalidate(p);p['mechanical_config']={k:v for k,v in data.items() if k!='revision'};p['revision']+=1
        p['events'].append({'at':now(),'type':'mechanical_configured','revision':p['revision']})
        store._save(p);return p


def finish(store,project,run_id,result):
    with store.transaction():
        p=store._load(project['id'])
        if p['revision']!=project['revision'] or p['active_run_id']!=run_id:return False
        p['active_run_id']=None
        p['mechanical_result']={'run_id':run_id,'status':result['status'],'geometry_valid':result['geometry_valid'],'manufacturing_ready':False,
                                'shell_partition_valid':result.get('shell_partition_valid',False),
                                'input_revision':project['revision'],'dependency_digest':dependency_digest(project)}
        # Failed geometry remains inspectable in the run, but is not committed as assembly.
        if result['geometry_valid']:
            for key in ['rigid_bodies','joints','mounts','geometry_assets','assembly_dependencies']:p[key]=result[key]
            p['downstream']['layout']='engineering_fixture'
        p['events'].append({'at':now(),'type':'mechanical_checked','run_id':run_id,'status':result['status']})
        store._save(p);return True
