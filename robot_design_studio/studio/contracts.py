"""Strict JSON contracts. No UI objects or executable model output."""
import json
import math
import re
from datetime import datetime, timezone
from uuid import uuid4

class ContractError(ValueError):
    pass


def uid(prefix):
    return prefix + '_' + uuid4().hex[:16]


def now():
    return datetime.now(timezone.utc).isoformat()


def loads(text):
    def pairs(items):
        result = {}
        for k, v in items:
            if k in result:
                raise ContractError('重复 JSON 字段: ' + k)
            result[k] = v
        return result
    def invalid(value):
        raise ContractError('非有限 JSON 数值: ' + value)
    return json.loads(text, object_pairs_hook=pairs, parse_constant=invalid)


def obj(properties):
    return {'type': 'object', 'properties': properties, 'required': list(properties), 'additionalProperties': False}


def arr(items, max_items=500):
    return {'type': 'array', 'items': items, 'maxItems': max_items}


def nullable(spec):
    return {'anyOf': [spec, {'type': 'null'}]}

TEXT = {'type': 'string', 'maxLength': 4000}
ID = {'type': 'string', 'pattern': r'^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$'}
NUMBER = {'type': 'number'}
INTEGER = {'type': 'integer', 'minimum': 0}
RANGE = {'type': 'array', 'items': NUMBER, 'minItems': 2, 'maxItems': 2}
ENDPOINT = obj({'instance_id': ID, 'port_id': ID})
NET = obj({'id': ID, 'kind': {'enum': ['power', 'ground', 'i2c_sda', 'i2c_scl', 'pwm', 'ttl', 'can', 'rs485']},
           'voltage_v': nullable(RANGE), 'endpoints': {**arr(ENDPOINT), 'minItems': 2}})
ALLOCATION = obj({'instance_id': ID, 'port_id': ID, 'function': ID, 'net_id': ID})
REQUEST = obj({'catalog_id': nullable(ID), 'category': TEXT, 'quantity': {'type': 'integer', 'minimum': 1, 'maximum': 64}, 'reason': TEXT})
CIRCUIT_SCHEMA = obj({'schema_version': {'enum': ['1.0']}, 'project_id': ID, 'design_revision': INTEGER,
                      'catalog_digest': TEXT, 'nets': arr(NET), 'allocations': arr(ALLOCATION),
                      'addresses': arr(obj({'instance_id': ID, 'bus_net_id': ID, 'value': {'type': 'integer', 'minimum': 0, 'maximum': 127}})),
                      'assumptions': arr(TEXT), 'missing_parts': arr(REQUEST)})
INSTANCE = obj({'id': ID, 'catalog_id': ID, 'catalog_revision': TEXT, 'label': TEXT,
                'locked': {'type': 'boolean'}, 'owner_body_id': nullable(ID)})
SELECTION_SCHEMA = obj({'revision': INTEGER, 'instances': arr(INSTANCE, 64), 'intent': TEXT})
SELECTION_SCHEMA['properties']['power_config']=nullable(obj({'mode':{'type':'string','enum':['single_external']},'voltage_v':nullable(arr({'type':'number'},2)),'distribution':{'type':'string','enum':['wire_splice']}}))


def validate(value, schema, path='$'):
    """Validate the deliberately small subset used by our published schemas."""
    def fail(message):
        raise ContractError(path + ': ' + message)
    if 'anyOf' in schema:
        for choice in schema['anyOf']:
            try:
                validate(value, choice, path)
                return
            except ContractError:
                pass
        fail('类型不匹配')
    if 'enum' in schema and not any(type(value) is type(v) and value == v for v in schema['enum']):
        fail('不在允许值内')
    kind = schema.get('type')
    types = {'object': dict, 'array': list, 'string': str, 'integer': int, 'boolean': bool, 'null': type(None)}
    if kind == 'number':
        if type(value) not in (int, float) or not math.isfinite(value):
            fail('需要有限数值')
    elif kind and type(value) is not types[kind]:
        fail('类型应为 ' + kind)
    if kind == 'object':
        if set(schema['required']) - value.keys():
            fail('缺少字段 ' + ', '.join(sorted(set(schema['required']) - value.keys())))
        if schema.get('additionalProperties') is False and value.keys() - schema['properties'].keys():
            fail('含未知字段')
        for key, child in value.items():
            validate(child, schema['properties'][key], path + '.' + key)
    if kind == 'array':
        if not schema.get('minItems', 0) <= len(value) <= schema.get('maxItems', 500):
            fail('数组长度超出范围')
        for i, child in enumerate(value):
            validate(child, schema['items'], f'{path}[{i}]')
    if kind == 'string':
        if len(value) > schema.get('maxLength', 4000):
            fail('文字过长')
        if 'pattern' in schema and not re.fullmatch(schema['pattern'], value):
            fail('无效 ID')
    if kind in ('number', 'integer'):
        if value < schema.get('minimum', -math.inf) or value > schema.get('maximum', math.inf):
            fail('数值超出范围')


def metres(value, unit):
    if unit not in {'m', 'cm', 'mm'} or type(value) not in (int, float) or not math.isfinite(value):
        raise ContractError('需要明确单位和有限数值')
    return value * {'m': 1, 'cm': .01, 'mm': .001}[unit]


def transform_point(point, transform):
    """Rigid right-handed transform; source and target frames are mandatory."""
    if not transform.get('from_frame') or not transform.get('to_frame') or transform.get('units') != 'm':
        raise ContractError('变换缺少坐标语义或 SI 单位')
    r, t = transform['rotation'], transform['translation_m']
    if len(point) != 3 or len(t) != 3 or len(r) != 3 or any(len(row) != 3 for row in r):
        raise ContractError('变换尺寸错误')
    for v in [*point, *t, *(x for row in r for x in row)]:
        validate(v, NUMBER)
    if any(abs(sum(r[k][i]*r[k][j] for k in range(3)) - (i == j)) > 1e-6 for i in range(3) for j in range(3)):
        raise ContractError('旋转矩阵非正交')
    det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    if abs(det - 1) > 1e-6:
        raise ContractError('必须为右手旋转')
    return [sum(r[i][j]*point[j] for j in range(3)) + t[i] for i in range(3)]
