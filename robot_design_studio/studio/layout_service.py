"""Versioned component envelopes and isolated internal assembly jobs."""
import copy
import json
from .layout_readiness import draft_layout_ready
from .contracts import ContractError, INTEGER, validate, now
from .project import Conflict
from .mechanical_service import number, dependency_digest

EXCLUDED = {'actuator', 'wire', 'fastener', 'connector'}


def components(project, catalog):
    return [i for i in project['instances'] if catalog.resolve(i)['category'] not in EXCLUDED]


def invalidate(project):
    owners = {p['instance_id'] for p in project.get('component_placements', [])}
    for instance in project['instances']:
        if instance['id'] in owners: instance['owner_body_id'] = None
    project['harness_result'] = None
    project['harnesses'] = []
    project['layout_result'] = None
    project['simulation_result'] = None
    project['component_placements'] = []
    project['layout_assets'] = []
    for key in ['layout','harness','manufacturing','simulation','control']:
        project['downstream'][key] = 'stale'


def configure(store, identity, data):
    if set(data)-{'revision','mode','components','allow_unverified_structure'} or not {'revision','mode','components'}<=set(data) or data['mode'] != 'reference':
        raise ContractError('布局需要版本、参考求解模式和完整部件包络')
    if type(data.get('allow_unverified_structure',False)) is not bool:raise ContractError('布局草案选项须为布尔值')
    validate(data['revision'], INTEGER)
    with store.transaction():
        p = store._load(identity)
        if p['revision'] != data['revision']: raise Conflict('布局输入版本已过期')
        m = p.get('mechanical_result') or {}
        if m.get('dependency_digest') != dependency_digest(p):raise ContractError('当前结构已过期，请重新生成')
        draft=bool(data.get('allow_unverified_structure',False) and not m.get('geometry_valid'))
        if not m.get('geometry_valid'):
            source=store.root/'runs'/m['run_id']/'result.json'
            if not draft or not source.is_file() or not draft_layout_ready(json.loads(source.read_text())):
                raise ContractError('结构未通过；布局草案也需要完整、连通且无静态/采样运动碰撞的实体')
        if not p.get('articulation'): raise ContractError('内部布局需要正式关节图结构')
        expected = {i['id'] for i in components(p, store.catalog)}
        values = data['components']
        if not isinstance(values,list) or not values or len(values)>16:
            raise ContractError('内部布局支持 1–16 个已选择的控制器、传感器、电源或分线部件')
        ids = set()
        bodies = {b['id'] for b in p['rigid_bodies']}
        for v in values:
            if not isinstance(v,dict) or set(v) != {'instance_id','dimensions_m','owner_body_id','evidence','note'}:
                raise ContractError('部件包络字段不完整')
            if v['instance_id'] not in expected or v['instance_id'] in ids: raise ContractError('部件未选择或重复登记')
            ids.add(v['instance_id'])
            if v['owner_body_id'] != 'auto' and v['owner_body_id'] not in bodies: raise ContractError('安装刚体不存在')
            if v['evidence'] != 'engineering_assumption': raise ContractError('本入口仅登记待核实的工程包络，不将填写尺寸冒充实测')
            if not isinstance(v['note'],str) or not 1<=len(v['note'])<=1000: raise ContractError('需要说明包络来源与未知')
            if not isinstance(v['dimensions_m'],list) or len(v['dimensions_m'])!=3: raise ContractError('需要长宽高三项包络尺寸')
            instance=next(i for i in p['instances'] if i['id']==v['instance_id'])
            for axis,x in zip(['长','宽','高'],v['dimensions_m']):
                try:number(x,.002,.3,'部件完整包络 m')
                except ContractError:raise ContractError(instance['label']+'：'+axis+'度须填写 2–300 mm；空白、零和非数字不能用于布局') from None
        if ids != expected: raise ContractError('全部已选择的内部实体部件都须登记；不能静默遗漏电源或分线板')
        invalidate(p)
        p['layout_config'] = {'components':copy.deepcopy(values),'mechanical_run_id':m['run_id'],'mode':'reference','allow_unverified_structure':draft}
        p['revision'] += 1
        p['active_run_id'] = None
        # Layout metadata changes no electrical net, port, model or selected count.
        # Keep the archived proposal intact; independently revalidate this copy.
        if p.get('circuit'):
            from .electrical import review
            previous = p['circuit']['design_revision']
            p['circuit'] = copy.deepcopy(p['circuit']); p['circuit']['design_revision'] = p['revision']
            p['validation'] = review(p,p['circuit'],store.catalog)
            if not p['validation']['logical_valid']: p['circuit'] = None
            p['events'].append({'at':now(),'type':'electrical_revalidated_for_layout','source_revision':previous,'revision':p['revision']})
        p['events'].append({'at':now(),'type':'layout_configured','revision':p['revision']})
        store._save(p)
        return p


def finish(store, project, run_id, result):
    with store.transaction():
        p = store._load(project['id'])
        if p['revision']!=project['revision'] or p['active_run_id']!=run_id: return False
        p['active_run_id'] = None
        valid = result.get('layout_valid',False)
        p['layout_result'] = {'run_id':run_id,'layout_valid':valid,'input_revision':p['revision'],
            'mechanical_run_id':project['layout_config']['mechanical_run_id'],'manufacturing_ready':False,
            'structure_verified':result.get('structure_verified',True)}
        if valid:
            p['component_placements'] = result['component_placements']
            p['layout_assets'] = result['geometry_assets']
            owners = {v['instance_id']:v['owner_body_id'] for v in p['component_placements']}
            for i in p['instances']:
                if i['id'] in owners:i['owner_body_id'] = owners[i['id']]
            p['downstream']['layout'] = 'candidate_checked' if result.get('structure_verified',True) else 'draft_structure_unverified'
        else:p['downstream']['layout'] = 'rejected'
        p['events'].append({'at':now(),'type':'layout_checked','run_id':run_id,'accepted':valid})
        store._save(p)
        return True
