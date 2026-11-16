"""Export a derived simulator model without running a simulation engine."""
from .contracts import ContractError,INTEGER,validate,now
from .project import Conflict
from .mechanical_service import number,geometry_python


def interpreter():
    return geometry_python()


def configure(store,identity,data):
    if set(data)!={'revision','mode','printed_density_kg_m3','hardware_density_kg_m3','assumed_torque_nm'} or data['mode']!='reference':
        raise ContractError('模型导出参数字段无效')
    validate(data['revision'],INTEGER)
    for key,lo,hi in [('printed_density_kg_m3',100,5000),('hardware_density_kg_m3',100,10000),('assumed_torque_nm',.01,20)]:
        number(data[key],lo,hi,key)
    interpreter()
    with store.transaction():
        p=store._load(identity)
        if p['revision']!=data['revision']:raise Conflict('模型导出输入版本已过期')
        layout=p.get('layout_result') or {}
        if not layout.get('layout_valid') or layout['input_revision']!=p['revision']:raise ContractError('先完成当前版本的内部装配候选')
        if p.get('articulation_config',{}).get('support')!='fixed_base':raise ContractError('当前导出仅支持固定底座模型')
        p['simulation_config']={k:v for k,v in data.items() if k not in {'revision','mode'}}
        p['simulation_result']=None;p['downstream']['simulation']='pending'
        p['events'].append({'at':now(),'type':'model_export_parameters_set','revision':p['revision'],'evidence':'engineering_assumption'})
        store._save(p);return p


def finish(store,project,run_id,result):
    with store.transaction():
        p=store._load(project['id'])
        if p['revision']!=project['revision'] or p['active_run_id']!=run_id:return False
        p['active_run_id']=None
        p['simulation_result']={'run_id':run_id,'input_revision':p['revision'],'export_valid':result.get('export_valid',False),
            'layout_run_id':project['layout_result']['run_id'],'physical_verified':False,'simulation_performed':False}
        p['downstream']['simulation']='model_exported' if result.get('export_valid') else 'rejected'
        p['events'].append({'at':now(),'type':'simulator_model_exported','run_id':run_id,'simulation_performed':False})
        store._save(p);return True
