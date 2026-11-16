"""Deterministic review. Catalog facts, never model-supplied port capabilities."""
from collections import defaultdict
from .contracts import CIRCUIT_SCHEMA, ContractError, validate


def voltage_range(value):
    # Adapted from robot_interior_postprocess/electrical_review.py::voltage_range.
    # Original repository Non-Commercial Research License applies (../LICENSE).
    from math import isfinite
    if (not isinstance(value, list) or len(value) != 2 or
        any(type(v) not in (int, float) or not isfinite(v) for v in value) or
        value[0] < 0 or value[1] < value[0]):
        raise ContractError('电压区间须为有限、非负、从小到大的两项数值')
    return value


def review(project, proposal, catalog):
    errors, pending = [], []
    def issue(target, code, reason, **data):
        target.append({'code': code, 'reason': reason, **data})
    try:
        validate(proposal, CIRCUIT_SCHEMA)
        for net in proposal['nets']:
            if net['voltage_v'] is not None:
                voltage_range(net['voltage_v'])
    except (ContractError, TypeError, KeyError) as exc:
        return report([{'code':'invalid_schema','reason':str(exc)}], [], [])
    if (proposal['project_id'] != project['id'] or proposal['design_revision'] != project['revision'] or
        proposal['catalog_digest'] != catalog.digest):
        issue(errors, 'stale_proposal', '接线方案对应的项目或目录版本已过期')
    parts = {i['id']: catalog.resolve(i) for i in project['instances']}
    ports = {(i,p):s for i,c in parts.items() for p,s in c['ports'].items()}
    assigned, nets, allocations = {}, {}, {}
    for net in proposal['nets']:
        n, kind, span = net['id'], net['kind'], net['voltage_v']
        if n in nets:
            issue(errors, 'duplicate_net', '网络 ID 重复', net_id=n)
        nets[n] = net
        keys = [(e['instance_id'],e['port_id']) for e in net['endpoints']]
        specs = []
        for key in keys:
            if key not in ports:
                issue(errors,'unknown_port','引用了未选择的部件或不存在的端口',terminal=list(key));continue
            if key in assigned:
                issue(errors,'port_reused','同一端口重复连接',terminal=list(key))
            assigned[key] = n
            spec = ports[key]; specs.append((key,spec))
            bounds = spec['voltage_v']
            if bounds is None or span is None:
                issue(pending,'unknown_voltage','端口工作范围或网络电平待核实',terminal=list(key))
            elif span[0] < bounds[0] or span[1] > bounds[1]:
                issue(errors,'voltage_out_of_range','网络电压超出部件允许范围',terminal=list(key),net_id=n)
            if spec['kind'] == 'power_out' and bounds and span and (bounds[0] < span[0] or bounds[1] > span[1]):
                issue(errors,'source_voltage_mismatch','不能用模型指定的电压掩盖电源实际输出',net_id=n)
            allowed = {'power':{'power_in','power_out'},'ground':{'ground'},
                       'i2c_sda':{'bus'},'i2c_scl':{'bus'},'pwm':{'signal_in','signal_out'},
                       'ttl':{'bus'},'can':{'bus'},'rs485':{'bus'}}[kind]
            if spec['kind'] not in allowed:
                issue(errors,'role_mismatch','端口角色与网络不符（电源、地或信号误接）',terminal=list(key),net_id=n)
            functions = spec['functions']
            protocol_ok = (kind not in ('i2c_sda','i2c_scl','pwm','ttl','can','rs485') or
                (kind == 'i2c_sda' and any(f.endswith('_SDA') for f in functions)) or
                (kind == 'i2c_scl' and any(f.endswith('_SCL') for f in functions)) or
                (kind == 'pwm' and any(f.startswith('PWM') for f in functions)) or kind.upper() in functions)
            if not protocol_ok:
                issue(errors,'protocol_mismatch','引脚不支持该协议或信号角色',terminal=list(key))
        sources = [(k,s) for k,s in specs if s['kind']=='power_out']
        if len(sources)>1:
            issue(errors,'multiple_power_sources','多个未隔离电源相连',net_id=n)
        if sum(s['kind']=='signal_out' for _,s in specs)>1:
            issue(errors,'multiple_signal_drivers','多个推挽输出相连',net_id=n)
        if kind=='pwm' and sum(s['kind']=='signal_out' for _,s in specs)!=1:
            issue(errors,'missing_signal_driver','PWM 网络必须有一个已登记输出',net_id=n)
        if kind=='power':
            external=project.get('power_config') or {}
            boundary=external.get('mode')=='single_external' and n=='external_power'
            if boundary and span!=external.get('voltage_v'):issue(errors,'external_voltage_mismatch','外部供电网络与项目声明不一致',net_id=n)
            if boundary and sources:issue(errors,'multiple_power_sources','外部供电边界不能再并联未隔离内部电源',net_id=n)
            if not sources:
                if boundary:issue(pending,'external_power_unverified','单一外接电源入口；实际电源、引线、分支与电流能力待核实，不占用内部模块载板',net_id=n)
                else:issue(errors,'missing_power_source','供电网络没有电源',net_id=n)
            loads = [parts[k[0]]['electrical']['peak_current_a'] for k,s in specs if s['kind']=='power_in']
            limit = sources[0][1]['current_limit_a'] if sources else None
            if any(x is None for x in loads) or limit is None:
                issue(pending,'power_budget_unknown','电源能力或同时负载峰值未知，不能放行上电',net_id=n)
            elif sum(loads)>limit:
                issue(errors,'power_budget_exceeded','同时峰值需求超过电源能力',net_id=n)
        if len(keys)>2:
            issue(pending,'physical_branch_required','多端网络需要明确分线器端口和实体线束；曲线相交不构成导通',net_id=n)
        if kind=='ground' and span != [0,0]:
            issue(errors,'ground_voltage','参考地必须为 0 V',net_id=n)
    for net in proposal['nets']:
        if net['kind'] not in ('power','ground'):
            grounds={assigned.get((e['instance_id'],'GND')) for e in net['endpoints']}
            if len(grounds)!=1 or None in grounds:
                issue(errors,'missing_common_ground','信号两端缺少共同参考地',net_id=net['id'])
    used_resources = {}
    for a in proposal['allocations']:
        key=(a['instance_id'],a['port_id']); spec=ports.get(key)
        if key in allocations or not spec or a['function'] not in spec['functions'] or assigned.get(key)!=a['net_id'] or parts.get(key[0],{}).get('category')!='controller':
            issue(errors,'invalid_allocation','MCU 引脚分配不存在、重复或不匹配网络',terminal=list(key));continue
        allocations[key]=a
        resource=(key[0],a['function'])
        if resource in used_resources:
            issue(errors,'resource_conflict','MCU 外设资源被重复占用',terminal=list(key))
        used_resources[resource]=key
    for key, n in assigned.items():
        if parts[key[0]]['category']=='controller' and ports[key]['functions'] and key not in allocations:
            issue(errors,'missing_allocation','已使用的 MCU 引脚没有资源分配',terminal=list(key))
    addresses={}; address_instances=set()
    for a in proposal['addresses']:
        instance=a['instance_id']; net=nets.get(a['bus_net_id'])
        if instance not in parts or not net or net['kind']!='i2c_sda' or assigned.get((instance,'SDA'))!=a['bus_net_id'] or a['value'] not in parts[instance]['electrical']['address_options']:
            issue(errors,'invalid_address','I²C 地址不属于已连接的设备和总线');continue
        key=(a['bus_net_id'],a['value'])
        if key in addresses or instance in address_instances:
            issue(errors,'address_conflict','同一 I²C 总线地址重复或设备被重复分配地址')
        addresses[key]=instance;address_instances.add(instance)
        issue(pending,'address_strap_unknown','需要核实实物地址跳线状态及扫描地址',instance_id=instance)
    for instance,p in parts.items():
        if p['category']!='controller' and {'SDA','SCL'} <= set(p['ports']):
            if instance not in address_instances:
                issue(errors,'missing_address','I²C 设备缺少地址分配',instance_id=instance)
            channels=[]
            for suffix in ('SDA','SCL'):
                n=nets.get(assigned.get((instance,suffix)),{})
                controllers=[allocations.get((e['instance_id'],e['port_id'])) for e in n.get('endpoints',[]) if parts.get(e['instance_id'],{}).get('category')=='controller']
                controllers=[a for a in controllers if a]
                if len(controllers)!=1:
                    issue(errors,'i2c_controller_missing','I²C 需要唯一已分配的控制器接口',instance_id=instance)
                else:
                    a=controllers[0];channels.append((a['instance_id'],a['function'].rsplit('_',1)[0]))
            if len(channels)==2 and channels[0]!=channels[1]:
                issue(errors,'i2c_bus_mismatch','SDA/SCL 必须来自同一 MCU I²C 外设',instance_id=instance)
            issue(pending,'pullup_unknown','上拉电阻与上拉电源待核实',instance_id=instance)
        for port_id,spec in p['ports'].items():
            if spec['required'] and (instance,port_id) not in assigned:
                issue(pending,'required_port_unconnected','必需端口未连接，需补齐供电或接口部件',instance_id=instance,port_id=port_id)
        for reason in p['pending']:
            issue(pending,'catalog_evidence_missing',reason,instance_id=instance)
        if not p['ports'] and p['category']=='actuator':
            issue(errors,'unsupported_actuator','此电机候选尚未登记可用于接线的端口',instance_id=instance)
    requests=list(proposal['missing_parts'])
    for request in requests:
        if request['catalog_id'] is not None and request['catalog_id'] not in catalog.parts:
            issue(errors,'unknown_requested_part','新增部件须先登记入库')
        issue(pending,'selection_change_requested',request['reason'])
    for dep in catalog.selection_issues(project['instances']):
        if (project.get('power_config') or {}).get('mode')=='single_external' and dep['code']=='missing_dependency' and catalog.parts[dep['catalog_id']]['category']=='power':continue
        issue(pending,dep['code'],dep['reason'])
    issue(pending,'harness_unverified','线材承载、接头、长度、分支和空间路线尚未验证；逻辑网络不能用作制造线束')
    return report(errors,pending,requests)


def report(errors,pending,requests):
    return {'status':'failed' if errors else 'pending' if pending else 'passed',
            'logical_valid':not errors,'manufacturing_released':False,
            'errors':errors,'pending':pending,'requested_parts':requests,
            'checks':{'electrical':'failed' if errors else 'pending' if pending else 'passed',
                      'geometry':'pending','assembly':'pending','motion_wiring':'pending','simulation':'pending','physical':'pending'},
            'scope':'已登记端口、供电/信号角色、范围、共地、MCU、I²C、已知电流预算；未登记协议和实物项目不放行'}
