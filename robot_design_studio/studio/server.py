"""Loopback-only Ubuntu application, no runtime third-party dependencies."""
import argparse
import fcntl
import json
import secrets
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from .catalog import Catalog
from .contracts import ContractError, loads, validate, INTEGER
from .electrical import review
from .export import bundle
from .jobs import Jobs
from .project import Store, Conflict
from .providers import live_config, load_ai_env
from . import mechanical_service
from . import articulation_service

APP=Path(__file__).resolve().parents[1]


def make_server(root,port=8765):
    catalog=Catalog();store=Store(root,catalog)
    module_library=loads((APP/'catalog/joint_modules.json').read_text())
    runtime_lock=(store.root/'.server.lock').open('a')
    try:
        fcntl.flock(runtime_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        runtime_lock.close()
        raise ContractError('此项目工作区已有运行中的服务；请复用原服务或选择另一工作区') from None
    jobs=Jobs(store);token=secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def send(self,data,status=200,mime='application/json; charset=utf-8',filename=None):
            body=data if isinstance(data,bytes) else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
            if filename:self.send_header('Content-Disposition','attachment; filename="'+filename+'"')
            self.end_headers();self.wfile.write(body)
        def handle_request(self):
            try:
                allowed={f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'}
                if self.headers.get('Host') not in allowed: return self.send({'error':'不允许的主机'},403)
                path=urlsplit(self.path).path;segments=path.strip('/').split('/')
                data=None
                if self.command=='POST':
                    origin=self.headers.get('Origin')
                    if self.headers.get('X-Studio-Token')!=token or (origin and origin not in {'http://'+h for h in allowed}):
                        return self.send({'error':'请求来源无效；请刷新页面'},403)
                    if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ContractError('请求必须是 JSON')
                    size=int(self.headers.get('Content-Length','0'))
                    maximum=54_000_000 if path.endswith('/appearance') else 1_000_000
                    if size<1 or size>maximum: raise ContractError('请求大小超出允许范围')
                    data=loads(self.rfile.read(size))
                    if not isinstance(data,dict):raise ContractError('需要 JSON 对象')
                if self.command=='GET':
                    if path=='/api/session':return self.send({'token':token,'live_available':all(live_config().values()),'version':'0.9.2',
                        'module_templates':{'revision':module_library['revision'],'catalog_ids':list(module_library['templates'])}})
                    if path=='/api/catalog':return self.send({**catalog.document,'digest':catalog.digest})
                    if path=='/api/projects':return self.send(store.list())
                    if len(segments)==3 and segments[:2]==['api','projects']:return self.send(store.load(segments[2]))
                    if len(segments)==3 and segments[:2]==['api','runs']:return self.send(jobs.get(segments[2]))
                    if len(segments)==5 and segments[:2]==['api','runs'] and segments[3]=='artifacts':
                        run=jobs.get(segments[2]);name=segments[4]
                        if name not in run.get('result',{}).get('artifacts',[]) or Path(name).name!=name:
                            raise ContractError('未登记的任务产物')
                        f=jobs.root/run['id']/name
                        mime={'.json':'application/json','.png':'image/png','.glb':'model/gltf-binary','.zip':'application/zip','.stl':'model/stl'}.get(f.suffix,'application/octet-stream')
                        return self.send(f.read_bytes(),mime=mime,filename=name if f.suffix in {'.zip','.stl','.glb'} else None)
                    if len(segments)==4 and segments[:2]==['api','projects'] and segments[3]=='export':
                        project=store.load(segments[2]);mechanical=project.get('mechanical_result')
                        directory=None
                        if mechanical and mechanical['geometry_valid']:
                            run=jobs.get(mechanical['run_id']);directory=jobs.root/run['id']
                        layout=project.get('layout_result');layout_directory=None
                        if layout and layout.get('layout_valid'):layout_directory=jobs.root/layout['run_id']
                        simulation=project.get('simulation_result');simulation_directory=None
                        if simulation and simulation.get('export_valid'):simulation_directory=jobs.root/simulation['run_id']
                        harness=project.get('harness_result');harness_directory=jobs.root/harness['run_id'] if harness and harness.get('routing_valid') else None
                        return self.send(bundle(project,catalog,directory,layout_directory,simulation_directory,harness_directory),mime='application/zip',filename='robot-design-review.zip')
                    static={'/':'index.html','/app.js':'app.js','/style.css':'style.css','/mechanics.js':'mechanics.js','/viewer.js':'viewer.js','/articulation.js':'articulation.js','/layout.js':'layout.js','/simulation.js':'simulation.js','/harness.js':'harness.js'}
                    if path in static:
                        f=APP/'web'/static[path]
                        mime={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'}[f.suffix]
                        return self.send(f.read_bytes(),mime=mime)
                if self.command=='POST':
                    if path=='/api/projects':return self.send(store.create(data.get('name')),201)
                    if path=='/api/import':return self.send(store.import_design(data),201)
                    if len(segments)==4 and segments[:2]==['api','projects']:
                        identity,action=segments[2:]
                        if action=='selection':return self.send(store.select(identity,data))
                        if action=='appearance':return self.send(mechanical_service.appearance(store,identity,data))
                        if action=='articulation':return self.send(jobs.start(identity,data.get('revision'),data.get('mode'),kind='articulation',configuration=data),202)
                        if action=='clone-at-height':return self.send(jobs.clone_at_height(identity,data),202)
                        if action=='articulation-retry':return self.send(jobs.retry_partition(identity,data),202)
                        if action=='joint-motors':return self.send(articulation_service.bind_motors(store,identity,data))
                        if action=='assembly':return self.send(jobs.start(identity,data.get('revision'),'reference',kind='assembly',configuration=data),202)
                        if action=='size-search':return self.send(jobs.start_size_search(identity,data),202)
                        if action=='layout':return self.send(jobs.start(identity,data.get('revision'),'reference',kind='layout',configuration=data),202)
                        if action=='harness':return self.send(jobs.start(identity,data.get('revision'),'reference',kind='harness',configuration=data),202)
                        if action=='simulation':return self.send(jobs.start(identity,data.get('revision'),'reference',kind='simulation',configuration=data),202)
                        if action=='mechanics':return self.send(jobs.start(identity,data.get('revision'),data.get('mode'),kind='mechanical',configuration=data),202)
                        if action=='generate':
                            validate(data.get('revision'),INTEGER)
                            return self.send(jobs.start(identity,data['revision'],data.get('mode')),202)
                        if action=='review':return self.send(review(store.load(identity),data,catalog))
                    if len(segments)==4 and segments[:2]==['api','runs'] and segments[3]=='cancel':return self.send(jobs.cancel(segments[2]))
                self.send({'error':'未找到'},404)
            except FileNotFoundError:self.send({'error':'项目或任务不存在'},404)
            except Conflict as exc:self.send({'error':str(exc)},409)
            except (ContractError,ValueError,TypeError,KeyError) as exc:self.send({'error':str(exc)},400)
            except Exception:self.send({'error':'内部错误，当前结果未放行'},500)
        do_GET=handle_request
        do_POST=handle_request
    class StudioServer(ThreadingHTTPServer):
        def server_close(self):
            super().server_close()
            runtime_lock.close()
    try:
        server=StudioServer(('127.0.0.1',port),Handler)
    except Exception:
        runtime_lock.close()
        raise
    server.jobs=jobs;server.store=store
    return server


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--workspace',type=Path,default=APP/'workspace')
    parser.add_argument('--ai-env',type=Path,help='显式载入 AI 配置；不会复制或显示凭据')
    args=parser.parse_args()
    if args.ai_env:load_ai_env(args.ai_env)
    server=make_server(args.workspace,args.port)
    print(f'Robot Design Studio: http://127.0.0.1:{server.server_port}',flush=True)
    print(f'Project storage: {args.workspace.resolve()}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.jobs.close();server.server_close()

if __name__=='__main__':main()
