"""Upstream joint planning, independent of hardware selection and assembly."""
import copy
import json
import base64
import hashlib
from .contracts import ContractError, INTEGER, ID, validate, now, uid
from .project import Conflict
from .mechanical_service import number, invalidate

MOTOR_BRANCHES = {'sg90_digital_towerpro':'text2robot_rectangular',
                  'st3215_12v_candidate':'text2robot_rectangular',
                  'mg4005v2_candidate':'anything2robot_round'}


def clone_at_height(store,identity,data):
    """Copy source and rescale the existing graph; never scale motor fixtures."""
    if set(data)!={'revision','name','height_m'}:raise ContractError('需要版本、新项目名称和目标高度')
    validate(data['revision'],INTEGER);number(data['height_m'],.05,2.,'目标高度 m')
    original=store.load(identity)
    if original['revision']!=data['revision']:raise Conflict('复制来源版本已过期')
    annotation=original.get('articulation')
    if not annotation or not original.get('articulation_config'):raise ContractError('先完成源模型关节标注再按高度复制')
    bounds=annotation['analysis']['bounds_mm'];height=bounds[1][2]-bounds[0][2]
    if height<=0:raise ContractError('源模型高度无效')
    scale=data['height_m']*1000/height
    config=copy.deepcopy(original['articulation_config']);config['length_m']*=scale
    number(config['length_m'],.05,2.,'缩放后最大尺寸 m')
    config.update(mode='reference',intent='Uniformly resized source and inherited joint graph; target height '+str(data['height_m']*1000)+' mm. Recompute mounts, access and loads at native hardware scale.')
    source=original['source_shapes'][0];raw=(store.directory(identity)/'sources'/source['file']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=source['sha256']:raise ContractError('源模型摘要不匹配')
    graph=copy.deepcopy(annotation['graph'])
    for body in graph['bodies']:body['seed_mm']=[v*scale for v in body['seed_mm']]
    for joint in graph['joints']:joint['position_mm']=[v*scale for v in joint['position_mm']]
    created=store.create(data['name'])
    from .mechanical_service import appearance
    created=appearance(store,created['id'],{'revision':created['revision'],'filename':source['name'],
        'unit':source['unit'],'data_base64':base64.b64encode(raw).decode()})
    recovery={'source_sha256':source['sha256'],'graph':graph,'source':annotation['source'],
        'run_id':annotation['run_id'],'reproject_observations':True,
        'resize_provenance':{'project_id':identity,'revision':original['revision'],'uniform_factor':scale,
                             'target_height_m':data['height_m'],'hardware_scaled':False}}
    with store.transaction():
        created=store._load(created['id'])
        created['events'].append({'at':now(),'type':'scaled_copy_created',**recovery['resize_provenance']})
        created['inherited_motor_choices']=[{'joint_id':b['joint_id'],'catalog_id':b['catalog_id']} for b in original.get('motor_bindings',[])]
        store._save(created)
    return created,config,recovery


def configure(store, identity, data):
    if set(data) != {'revision','mode','intent','length_m','up_axis','support','max_joints'}:
        raise ContractError('关节标注参数不完整或有未知字段')
    validate(data['revision'], INTEGER)
    number(data['length_m'], .05, 2, '外观最大尺寸 m')
    if data['mode'] not in {'reference','live'}: raise ContractError('标注来源无效')
    if data['up_axis'] not in {'+X','-X','+Y','-Y','+Z','-Z'}: raise ContractError('朝上方向无效')
    if data['support'] not in {'fixed_base','tabletop'}: raise ContractError('支撑方式无效')
    if type(data['max_joints']) is not int or not 1 <= data['max_joints'] <= 6: raise ContractError('当前标注支持 1–6 个关节')
    if not isinstance(data['intent'], str) or len(data['intent']) > 4000: raise ContractError('动作意图过长')
    with store.transaction():
        p = store._load(identity)
        if p['revision'] != data['revision']: raise Conflict('标注输入版本已过期')
        if not p['source_shapes']: raise ContractError('请先导入外观；无需提前选择电机')
        invalidate(p)
        p['articulation'] = None
        p['motor_bindings'] = []
        p['articulation_config'] = {k:v for k,v in data.items() if k != 'revision'}
        p['revision'] += 1
        p['events'].append({'at':now(),'type':'articulation_configured','revision':p['revision']})
        store._save(p)
        return p


def finish(store, project, run_id, result):
    with store.transaction():
        p = store._load(project['id'])
        if p['revision'] != project['revision'] or p['active_run_id'] != run_id: return False
        p['active_run_id'] = None
        if result['annotation_valid']:
            p['articulation'] = {'run_id':run_id, 'source_sha256':result['source_sha256'],
                'input_revision':project['revision'], 'graph':result['graph'],
                'analysis':result['analysis'], 'source':result['source'],
                'motor_candidates':result['motor_candidates']}
        p['events'].append({'at':now(),'type':'articulation_checked','run_id':run_id,'status':result['status']})
        store._save(p)
        return True


def configure_assembly(store, identity, data):
    if not {'revision','mode'}<=set(data) or set(data)-{'revision','mode','service_components','parting_regions','stage','assembly_policy'} or data['mode'] != 'reference':
        raise ContractError('结构生成使用已经校核的关节与电机，不重新请求模型')
    if data.get('stage','complete') not in {'complete','shell_partition'}:raise ContractError('结构阶段无效')
    if data.get('assembly_policy','strict') not in {'strict','manual_extraction_review'}:raise ContractError('装配检查策略无效')
    validate(data['revision'], INTEGER)
    with store.transaction():
        p=store._load(identity)
        if p['revision']!=data['revision']:raise Conflict('项目已更新，当前页面版本落后；请刷新页面后重新生成结构，已有历史结果会保留')
        annotation=p.get('articulation')
        if not annotation or not p.get('motor_bindings'):raise ContractError('请先完成关节标注和逐关节电机选择')
        if annotation['source_sha256']!=p['source_shapes'][0]['sha256']:raise Conflict('关节与源外观不一致')
        bindings=p['motor_bindings'];instances={i['id']:i for i in p['instances']}
        if {b['joint_id'] for b in bindings}!={j['id'] for j in annotation['graph']['joints']}:raise ContractError('部分关节缺少电机')
        for b in bindings:
            if b['motor_instance_id'] not in instances or instances[b['motor_instance_id']]['catalog_id']!=b['catalog_id']:raise ContractError('关节电机实例已改变，请重新选择')
            if b['catalog_id']=='st3215_12v_candidate':raise ContractError('ST3215 完整模块模板尚未接入；不会缩放 SG90 孔位替代')
        requirements=data.get('service_components',p.get('layout_config',{}).get('components',p.get('service_opening_requirements',[])))
        if not isinstance(requirements,list) or len(requirements)>16:raise ContractError('检修口部件要求无效')
        body_ids={b['id'] for b in annotation['graph']['bodies']}
        for c in requirements:
            if not isinstance(c,dict) or c.get('instance_id') not in instances or c.get('owner_body_id') not in body_ids|{'auto'}:raise ContractError('检修口要求必须引用已选择部件和当前刚体')
            if c.get('evidence')!='engineering_assumption':raise ContractError('检修口包络须标明试算假设')
            dims=c.get('dimensions_m')
            if not isinstance(dims,list) or len(dims)!=3:raise ContractError('检修口包络需要长宽高')
            for value in dims:number(value,.002,.3,'检修口部件包络 m')
        # This operation changes shell access, not the selected electrical hardware
        # or its ports. Revalidate a copy at the new revision instead of forcing a
        # second AI request after the electronics-to-shell feedback step.
        circuit=copy.deepcopy(p.get('circuit'))
        candidates=[e['run_id'] for e in reversed(p['events']) if e['type']=='mechanical_checked'][:20]
        for run_id in candidates:
            validate(run_id,ID);folder=store.root/'runs'/run_id
            if not (folder/'result.json').is_file() or not (folder/'input.json').is_file():continue
            result=json.loads((folder/'result.json').read_text());archived=json.loads((folder/'input.json').read_text())
            surface=result.get('analysis',{}).get('access_suggestions')
            if (result.get('geometry_valid') and surface and archived['source_shapes']==p['source_shapes']
                    and p.get('articulation_config') is not None and archived.get('articulation_config')==p['articulation_config']
                    and (archived.get('articulation') or {}).get('graph')==annotation['graph']):
                p['surface_plan_reuse']={'source_sha256':p['source_shapes'][0]['sha256'],
                    'articulation_config':copy.deepcopy(p['articulation_config']),'graph':copy.deepcopy(annotation['graph']),
                    'source_run_id':run_id,'plan':surface}
                break
        if 'parting_regions' in data:
            bounds=(annotation.get('analysis') or {}).get('bounds_mm')
            if bounds is None:raise ContractError('缺少当前模型尺寸，不能接受分型面建议')
            regions=data['parting_regions']
            if not isinstance(regions,list) or any(not isinstance(r,dict) or set(r)!={'body_id','center_mm','normal','reason'} for r in regions):
                raise ContractError('分型面建议字段无效')
            ids=[r['body_id'] for r in regions]
            if any(not isinstance(i,str) for i in ids) or len(ids)!=len(set(ids)) or set(ids)!={b['id'] for b in annotation['graph']['bodies']}:
                raise ContractError('每个刚体必须恰好有一个分型面建议')
            for r in regions:
                if not isinstance(r['center_mm'],list) or len(r['center_mm'])!=3 or not isinstance(r['normal'],list) or len(r['normal'])!=3:
                    raise ContractError('分型面坐标无效')
                for axis,value in enumerate(r['center_mm']):number(value,bounds[0][axis],bounds[1][axis],'分型点')
                for value in r['normal']:number(value,-1,1,'分型方向')
                if sum(v!=0 for v in r['normal'])!=1 or max(abs(v) for v in r['normal'])!=1:
                    raise ContractError('分型方向必须为一个坐标轴方向')
                if not isinstance(r['reason'],str) or not 1<=len(r['reason'])<=1500:raise ContractError('分型依据无效')
            # Provided advice is a geometry proposal, never a declaration of
            # assembly validity or proof that an online model was called.
            p['surface_plan_reuse']={'source_sha256':p['source_shapes'][0]['sha256'],
                'articulation_config':copy.deepcopy(p['articulation_config']),
                'graph':copy.deepcopy(annotation['graph']),'source_run_id':'provided-parting-regions',
                'plan':{'plan_kind':'assembly_parting_v1','source':'provided_parting_regions',
                        'live_model_called':False,'regions':copy.deepcopy(data['parting_regions'])}}
        p['service_opening_requirements']=copy.deepcopy(requirements)
        p['assembly_stage']=data.get('stage','complete')
        p['assembly_policy']=data.get('assembly_policy',p.get('assembly_policy','strict'))
        invalidate(p);p['revision']+=1
        if circuit:
            from .electrical import review
            previous=circuit['design_revision'];circuit['design_revision']=p['revision']
            validation=review(p,circuit,store.catalog)
            if validation['logical_valid']:
                p['circuit']=circuit;p['validation']=validation
            p['events'].append({'at':now(),'type':'electrical_revalidated_for_access',
                'source_revision':previous,'revision':p['revision'],'logical_valid':validation['logical_valid']})
        p['events'].append({'at':now(),'type':'graph_assembly_configured','revision':p['revision']})
        store._save(p);return p


def bind_motors(store, identity, data):
    if set(data) != {'revision','choices'}: raise ContractError('电机选择参数无效')
    validate(data['revision'], INTEGER)
    choices = data['choices']
    if not isinstance(choices, list) or not 1 <= len(choices) <= 6: raise ContractError('需要每个关节的电机选择')
    with store.transaction():
        p = store._load(identity)
        if p['revision'] != data['revision']: raise Conflict('电机选择版本已过期')
        annotation = p.get('articulation')
        if not annotation: raise ContractError('先完成外观关节标注，再选择电机')
        joints = {j['id']:j for j in annotation['graph']['joints']}
        if any(not isinstance(c,dict) or set(c) != {'joint_id','catalog_id'} for c in choices): raise ContractError('关节电机选择格式无效')
        if len(choices) != len(joints) or {c['joint_id'] for c in choices} != set(joints): raise ContractError('每个关节必须恰好选择一个电机')
        old = {b['joint_id']:b for b in p.get('motor_bindings',[])}
        instances = copy.deepcopy(p['instances']); bindings = []
        for c in choices:
            validate(c['joint_id'], ID)
            catalog_id = c['catalog_id']
            if catalog_id not in MOTOR_BRANCHES: raise ContractError('该型号尚未登记电机分支')
            previous = old.get(c['joint_id'])
            instance = next((i for i in instances if previous and i['id'] == previous['motor_instance_id']), None)
            if instance and instance['catalog_id'] != catalog_id:
                if instance['locked']: raise ContractError('关节电机已锁定；先解除锁定再更换')
                instances.remove(instance); instance = None
            if instance is None:
                part = store.catalog.parts[catalog_id]
                instance = {'id':uid('motor'),'catalog_id':catalog_id,'catalog_revision':part['revision'],
                            'label':joints[c['joint_id']]['name']+' · '+part['name'],'locked':False,'owner_body_id':None}
                instances.append(instance)
            candidates=next((entry['choices'] for entry in annotation.get('motor_candidates',[]) if entry['joint_id']==c['joint_id']),[])
            candidate=next((entry for entry in candidates if entry['catalog_id']==catalog_id),{})
            bindings.append({'joint_id':c['joint_id'],'motor_instance_id':instance['id'],'catalog_id':catalog_id,
                'branch':MOTOR_BRANCHES[catalog_id], 'annotation_run_id':annotation['run_id'],
                'template_revision':candidate.get('template_revision'),'preliminary_space_fit':candidate.get('preliminary_space_fit'),
                'load_verified':False,'reason':'按已标注关节明确选择；安装与负载由后续结构求解检查'})
        if len(instances) > 64: raise ContractError('部件实例超过 64 个')
        invalidate(p)
        p['instances'] = instances; p['motor_bindings'] = bindings
        p['revision'] += 1
        p['events'].append({'at':now(),'type':'joint_motors_selected','revision':p['revision']})
        store._save(p)
        return p
