"""Version-bound, explicit engineering wire routing; never measured port claims."""
import copy
from .contracts import ContractError,INTEGER,validate,now
from .project import Conflict
from .mechanical_service import number

def configure(store,identity,data):
    if set(data)-{'revision','mode','wire_diameter_mm','grid_mm','routing_mode'} or not {'revision','mode','wire_diameter_mm','grid_mm'}<=set(data) or data['mode']!='reference':raise ContractError('排线参数无效')
    if data.get('routing_mode','bundled') not in {'bundled','individual'}:raise ContractError('Unknown routing mode')
    validate(data['revision'],INTEGER);number(data['wire_diameter_mm'],.5,3,'试算线径 mm');number(data['grid_mm'],2,6,'排线格距 mm')
    with store.transaction():
        p=store._load(identity)
        if p['revision']!=data['revision']:raise Conflict('排线输入版本已改变')
        if not (p.get('layout_result') or {}).get('layout_valid') or p['layout_result']['input_revision']!=p['revision']:raise ContractError('先完成当前版本内部布局')
        if not p.get('circuit') or not (p.get('validation') or {}).get('logical_valid'):raise ContractError('先完成当前版本接线检查')
        p['harness_config']={k:v for k,v in data.items() if k not in {'revision','mode'}}
        p['harness_config'].setdefault('routing_mode','bundled')
        p['harness_result']=None;p['harnesses']=[];p['downstream']['harness']='pending'
        store._save(p);return p

def finish(store,project,run_id,result):
    with store.transaction():
        p=store._load(project['id'])
        if p['revision']!=project['revision'] or p['active_run_id']!=run_id:return False
        p['active_run_id']=None
        p['harness_result']={'run_id':run_id,'input_revision':p['revision'],'layout_run_id':project['layout_result']['run_id'],'structure_verified':result.get('structure_verified',True),'routing_valid':bool(result.get('routing_valid')),'physical_verified':False,'routing_mode':result.get('routing_mode','individual')}
        if result.get('routing_valid'):p['harnesses']=copy.deepcopy(result['routes'])
        p['downstream']['harness']='engineering_routes_checked' if result.get('routing_valid') else 'rejected'
        p['events'].append({'at':now(),'type':'harness_checked','run_id':run_id,'accepted':bool(result.get('routing_valid'))})
        store._save(p);return True
