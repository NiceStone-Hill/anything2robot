"""Export a design review bundle; never a released manufacturing package."""
import csv
import io
import json
import zipfile
import hashlib
from pathlib import Path
from .contracts import ContractError
from .electrical import review


def bundle(project,catalog,mechanical_directory=None,layout_directory=None,simulation_directory=None,harness_directory=None):
    stream=io.BytesIO()
    def encoded(value): return json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)
    def table(rows):
        result=io.StringIO();writer=csv.writer(result)
        # Prevent spreadsheet formula interpretation for user/model labels.
        writer.writerows([["'"+str(x) if str(x).startswith(('=','+','-','@')) else x for x in row] for row in rows]);return result.getvalue()
    circuit=project['circuit'];validation=review(project,circuit,catalog) if circuit else project['validation']
    bom=[['instance_id','catalog_id','catalog_revision','name','stock_quantity','physical_status']]
    for i in project['instances']:
        p=catalog.resolve(i);bom.append([i['id'],p['id'],p['revision'],p['name'],p['inventory']['quantity'],'pending'])
    connections=[['net_id','kind','instance_id','port_id','physical_pin','position_m','release']]
    ports=[]
    for n in (circuit or {}).get('nets',[]):
        for e in n['endpoints']:
            i=next(i for i in project['instances'] if i['id']==e['instance_id']);p=catalog.resolve(i)['ports'][e['port_id']]
            connections.append([n['id'],n['kind'],e['instance_id'],e['port_id'],p['physical_pin'],'unknown','blocked'])
            ports.append({**e,'net_id':n['id'],'position_m':None,'coordinate_frame':None,'exit_direction':None,'physical_connector_id':None})
    contract={'schema_version':'1.0','project_id':project['id'],'design_revision':project['revision'],
              'catalog_digest':catalog.digest,'status':'blocked_pending_registration','units':'m',
              'components':project['instances'],'electrical_nets':(circuit or {}).get('nets',[]),
              'ports':ports,'harness_topology':[],'spatial_routes':[],
              'reason':'此接口只传递逻辑连接；须登记实物端口、归属和线束后才允许路由'}
    if harness_directory:
        h=json.loads((Path(harness_directory)/'result.json').read_text())
        if h['project_id']!=project['id'] or h['design_revision']!=project['revision'] or h['layout_run_id']!=(project.get('layout_result') or {}).get('run_id') or not h['routing_valid']:raise ContractError('线束版本与当前装配不一致')
        contract.update(status='engineering_routes_only',ports=h['terminal_assumptions'],spatial_routes=h['routes'],harness_topology=[{'from':r['from'],'to':r['to'],'net_id':r['net_id']} for r in h['routes']],routing_mode=h.get('routing_mode','individual'),bundle_segments=h.get('bundle_segments',[]),reason='端口位置、线径、裁线余量为试算假设；实物、弯曲与线夹未放行')
        if h.get('endpoint_registration'):
            contract.update(ports=h['endpoint_registration'],aggregation_exits=h['terminal_assumptions'],
                fanout_checks=h.get('fanout_checks',[]),full_connection_verified=False,
                reason='共享主通道已检查；参考针脚与未知实物端口分别标注。端部碰撞、接头、弯曲和线夹未全部放行。')
    with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as z:
        if harness_directory:
            for file in h['artifacts']:
                if Path(file).name!=file:raise ContractError('线束资产路径无效')
                raw=(Path(harness_directory)/file).read_bytes()
                if hashlib.sha256(raw).hexdigest()!=h.get('artifact_sha256',{}).get(file):raise ContractError('线束资产摘要无效')
                if not file.endswith('.zip'):z.writestr('harness/'+file,raw)

        for name,value in [('project.json',project),('catalog.json',catalog.document),('validation.json',validation),('routing_contract.json',contract)]:
            z.writestr(name,encoded(value))
        z.writestr('bom.csv','\ufeff'+table(bom));z.writestr('connections.csv','\ufeff'+table(connections))
        if simulation_directory:
            directory=Path(simulation_directory);manifest=json.loads((directory/'result.json').read_text())
            if manifest.get('project_id')!=project['id'] or manifest.get('design_revision')!=project['revision'] or not manifest.get('export_valid'):
                raise ContractError('模型导出并非当前版本有效的候选')
            if manifest['layout_run_id']!=(project.get('layout_result') or {}).get('run_id'):raise ContractError('物理模型的内部装配已改变')
            for filename in manifest['artifacts']:
                if filename.endswith('.zip'):continue
                if Path(filename).name!=filename:raise ContractError('仿真产物路径无效')
                raw=(directory/filename).read_bytes()
                expected=manifest.get('artifact_sha256',{}).get(filename)
                if not expected or hashlib.sha256(raw).hexdigest()!=expected:raise ContractError('仿真资产摘要无效')
                z.writestr('simulation/'+filename,raw)
        if layout_directory:
            directory=Path(layout_directory);manifest=json.loads((directory/'result.json').read_text())
            if manifest.get('project_id')!=project['id'] or manifest.get('design_revision')!=project['revision'] or not manifest.get('layout_valid'):
                raise ContractError('内部布局不是当前版本通过检查的候选')
            if manifest['mechanical_run_id']!=(project.get('mechanical_result') or {}).get('run_id'):
                raise ContractError('内部布局所用结构已改变')
            for asset in manifest['geometry_assets']+manifest.get('visual_assets',[]):
                if Path(asset['file']).name!=asset['file']:raise ContractError('布局资产路径无效')
                if hashlib.sha256((directory/asset['file']).read_bytes()).hexdigest()!=asset['sha256']:
                    raise ContractError('内部布局产物摘要校验失败')
            for filename in manifest['artifacts']:
                if filename.endswith('.zip'):continue
                if Path(filename).name!=filename:raise ContractError('布局产物路径无效')
                z.write(directory/filename,'internal-assembly/'+filename)
            z.writestr('internal-assembly/README.txt','此目录是包含电子包络和新增固定柱的派生装配；mechanics/ 保留原机械装配。不可混用两个版本的壳体。所有电子包络为工程假设，尚非制造输出。\n')
        if mechanical_directory:
            directory=Path(mechanical_directory);manifest=json.loads((directory/'result.json').read_text())
            if manifest.get('project_id')!=project['id']:raise ContractError('机械产物不属于当前项目')
            if manifest.get('design_revision')!=project['revision']:
                from .mechanical_service import dependency_digest
                source_project_path=directory/'input-project.json'
                # 0.2 saves input-project; task input.json is a fallback for old runs.
                if not source_project_path.exists():source_project_path=directory/'input.json'
                source_project=json.loads(source_project_path.read_text())
                if dependency_digest(source_project)!=dependency_digest(project):raise ContractError('机械依赖已改变，不能导出过期结构')
                z.writestr('mechanical_dependency.json',encoded({'geometry_input_revision':manifest['design_revision'],'project_revision':project['revision'],
                    'dependency_digest':dependency_digest(project),'reason':'后续电子选择改变了项目版本，机械输入未改变；不重写历史产物版本'}))
            for asset in manifest['geometry_assets']:
                raw=(directory/asset['file']).read_bytes()
                if hashlib.sha256(raw).hexdigest()!=asset['sha256']:raise ContractError('机械产物摘要校验失败')
            for filename in manifest['artifacts']:
                if filename.endswith('.zip'):continue
                if Path(filename).name!=filename:raise ContractError('产物路径无效')
                z.write(directory/filename,'mechanics/'+filename)
            z.writestr('mechanical_hardware_requests.json',encoded(manifest['hardware_requests']))
        z.writestr('README.txt','设计审阅包，不是可制造/可上电制作包。\n未知尺寸、接头、电流、模块供电、安装及实物验证仍待补齐。\n有机械结果时 mechanics/ 保存同一机械输入版本的实体、装配和检查；跨项目版本复用见 mechanical_dependency.json；STL 单位 mm，GLB 与项目单位 m。\n带机械结果的 project.json 暂不支持单文件重新导入，原工作区可重开；原始外观在 mechanics/source.*，可作为新项目重新生成。\n纯电气 project.json 可导入副本并重新校核，不信任通过状态。\n')
    return stream.getvalue()
