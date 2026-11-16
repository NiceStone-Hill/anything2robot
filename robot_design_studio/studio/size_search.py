"""Bounded, native-hardware size trials using the ordinary generation services."""
import copy
import time
from .contracts import ContractError
from .layout_readiness import draft_layout_ready

FACTORS = (1., 1.05, 1.10, 1.15, 1.20, 1.25)


def height(project):
    bounds = project['articulation']['analysis']['bounds_mm']
    return (bounds[1][2] - bounds[0][2]) / 1000


def baseline(project):
    # Persist the first search's baseline in every derived candidate. Starting
    # another search on a candidate must not compound the 25% allowance.
    origin = next((e for e in reversed(project.get('events', []))
                   if e['type'] == 'size_search_origin'), None)
    return copy.deepcopy(origin) if origin else {
        'type': 'size_search_origin', 'project_id': project['id'],
        'revision': project['revision'], 'height_m': height(project)}


def targets(project):
    base = baseline(project)['height_m']
    current = height(project)
    if base <= 0 or current > base * 1.25 + 1e-7:
        raise ContractError('尺寸已超过原始模型的 125%，不能继续自动放大')
    return [current] + [base*f for f in FACTORS[1:] if base*f > current+1e-7]


def needs_space(result):
    """Packing search exhaustion, not a proof of mathematical infeasibility."""
    check = next((c for c in result.get('checks', [])
                  if c['name'] == 'all_selected_components_placed'), {})
    # Missing service openings, bad dimensions and worker errors are not space
    # evidence. Require an actual unsuccessful placement search for each item.
    missing = {i['instance_id'] for i in check.get('unplaced', [])}
    searched = {s['instance_id'] for s in result.get('searches', []) if s.get('attempts', 0)}
    return check.get('passed') is False and bool(missing) and missing <= searched


def run(api, source, components, report, cancelled=lambda: False, sleep=time.sleep):
    """api.get/post use existing studio endpoints; api.mark persists provenance."""
    original = copy.deepcopy(source)
    origin = baseline(original)
    result = {'baseline': origin, 'max_factor': 1.25, 'hardware_scaled': False,
              'attempts': [], 'manufacturing_ready': False}

    def publish():
        report(copy.deepcopy(result))

    def wait(job):
        result['active_run_id'] = job['id']; publish()
        while job['status'] in {'queued', 'running'}:
            if cancelled():
                api.post('runs/'+job['id']+'/cancel', {})
                raise InterruptedError('Size search cancelled')
            sleep(.2); job = api.get('runs/'+job['id'])
        if cancelled(): raise InterruptedError('Size search cancelled')
        result['active_run_id'] = None
        return job

    def stage(identity, action, config):
        if cancelled(): raise InterruptedError('Size search cancelled')
        p = api.get('projects/'+identity)
        return wait(api.post('projects/'+identity+'/'+action,
                             {'revision': p['revision'], 'mode': 'reference', **config}))

    for target in targets(original):
        if cancelled(): raise InterruptedError('Size search cancelled')
        # Abort if another editor changes the source between trials. Do not
        # silently generate against a different component selection/geometry.
        p = api.get('projects/'+original['id'])
        if p['revision'] != source['revision']:
            raise ContractError('原项目已改变，尺寸搜索停止；已有候选保留')
        identity = original['id']
        attempt = {'height_m': target, 'factor': target/origin['height_m'],
                   'project_id': identity, 'runs': {}}
        result['attempts'].append(attempt); publish()
        if target > height(original)+1e-7:
            job = wait(api.post('projects/'+identity+'/clone-at-height', {
                'revision': p['revision'], 'name': original['name'][:80]+' · '+str(round(target*1000, 1))+' mm fit',
                'height_m': target}))
            identity = job['project_id']; attempt['project_id'] = identity
            attempt['runs']['articulation'] = job['id']
            api.mark(identity, origin)
            if job['status'] != 'completed':
                attempt['reason'] = 'annotation_failed'; publish(); continue
            p = api.get('projects/'+identity)
            p = api.post('projects/'+identity+'/joint-motors', {'revision': p['revision'],
                'choices': [{'joint_id': b['joint_id'], 'catalog_id': b['catalog_id']}
                            for b in original['motor_bindings']]})
            motors = {b['motor_instance_id'] for b in original['motor_bindings']}
            extras = [{**i, 'owner_body_id': None} for i in original['instances'] if i['id'] not in motors]
            selection = {'revision': p['revision'], 'instances': p['instances']+extras, 'intent': original['intent']}
            if original.get('power_config') is not None: selection['power_config'] = original['power_config']
            api.post('projects/'+identity+'/selection', selection)
            electrical = stage(identity, 'generate', {})
            attempt['runs']['generate'] = electrical['id']
            if electrical['status'] != 'completed':
                result['reason'] = 'electrical_failed'; break
        assembly = stage(identity, 'assembly', {'stage': 'complete',
            'assembly_policy': original.get('assembly_policy', 'strict'), 'service_components': components})
        attempt['runs']['assembly'] = assembly['id']
        if assembly['status'] not in {'completed', 'rejected'}:
            result['reason'] = 'generation_'+assembly['status']; break
        mechanical = assembly.get('result') or {}
        attempt['geometry_valid'] = mechanical.get('geometry_valid', False)
        room_failures = [c['name'] for c in mechanical.get('checks', [])
                         if c['name'].endswith('_module_room') and c.get('passed') is False]
        if room_failures:
            attempt['reason'] = 'actuator_space_search_exhausted'
            attempt['failed_room_checks'] = room_failures
            if identity == original['id']: source = api.get('projects/'+identity)
            publish(); continue
        if not attempt['geometry_valid'] and not draft_layout_ready(mechanical):
            attempt['reason'] = 'structure_failed'
            # No new evidence of packing failure: never mask a geometry error
            # by calling it insufficient volume.
            result['reason'] = 'structure_failed'; break
        layout = stage(identity, 'layout', {'components': components, 'allow_unverified_structure': True})
        attempt['runs']['layout'] = layout['id']
        if layout['status'] not in {'completed', 'rejected'}:
            result['reason'] = 'generation_'+layout['status']; break
        fitted = layout.get('result') or {}
        attempt['layout_valid'] = fitted.get('layout_valid', False)
        if attempt['layout_valid']:
            result.update(selected_project_id=identity,
                          reason='fit' if attempt['geometry_valid'] else 'fit_structure_unverified')
            break
        if not needs_space(fitted):
            result['reason'] = 'non_packing_failure'; break
        attempt['reason'] = 'packing_search_exhausted'
        # Original-size trial changes its revision through normal generation.
        if identity == original['id']:
            source = api.get('projects/'+identity)
        publish()
    else:
        result['reason'] = 'limit_reached'
    result['active_run_id'] = None; publish()
    return result
