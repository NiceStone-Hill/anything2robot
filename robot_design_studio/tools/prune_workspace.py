"""Keep current project results and their dependencies; dry-run unless --apply."""
import argparse
import json
import shutil
from pathlib import Path

RUN_KEYS={'run_id','mechanical_run_id','layout_run_id','annotation_run_id'}

def references(value):
    if isinstance(value,dict):
        for key,item in value.items():
            if key in RUN_KEYS and isinstance(item,str) and item.startswith('run_'):yield item
            elif key not in {'events','runs'}:yield from references(item)
    elif isinstance(value,list):
        for item in value:yield from references(item)


def plan(root):
    root=Path(root);projects=[];keep=set()
    for path in sorted(root.glob('project_*/project.json')):
        p=json.loads(path.read_text())
        if p.get('active_run_id'):raise ValueError('Project is running: '+p['id'])
        projects.append((path,p));keep.update(references({k:p.get(k) for k in (
            'articulation','mechanical_result','layout_result','harness_result','simulation_result')}))
        for rid in reversed(p.get('runs',[])):
            f=root/'runs'/rid/'run.json'
            if f.exists() and json.loads(f.read_text()).get('kind')=='wiring':keep.add(rid);break
    todo=list(keep)
    while todo:
        rid=todo.pop();folder=root/'runs'/rid
        if not (folder/'run.json').is_file():raise ValueError('Missing required run: '+rid)
        f=folder/'result.json'
        if f.exists():
            for dep in references(json.loads(f.read_text())):
                if dep not in keep:
                    if not (root/'runs'/dep).is_dir():raise ValueError('Missing dependency: '+dep)
                    keep.add(dep);todo.append(dep)
    remove=[p for p in (root/'runs').glob('run_*') if p.name not in keep]
    return projects,keep,remove


def prune(root,apply=False):
    root=Path(root);projects,keep,remove=plan(root)
    report={'projects':len(projects),'retained_runs':sorted(keep),'removed_runs':[p.name for p in remove],
            'removed_bytes':sum(f.stat().st_size for p in remove for f in p.rglob('*') if f.is_file()),'applied':apply}
    if apply:
        # Run only while the server is stopped. Validate all dependencies before
        # mutating anything. Immutable retained run outputs remain unchanged.
        for path,p in projects:
            p['runs']=[r for r in p.get('runs',[]) if r in keep]
            p['events']=[e for e in p.get('events',[]) if not e.get('run_id') or e['run_id'] in keep]
            p['events'].append({'type':'history_compacted','retained_run_count':len(p['runs']),
                                'revision':p['revision'],'at':p['updated_at']})
            path.write_text(json.dumps(p,ensure_ascii=False,indent=2)+'\n')
            history=path.parent/'history'
            if history.exists():shutil.rmtree(history)
            history.mkdir();(history/f"r{p['revision']}_latest.json").write_bytes(path.read_bytes())
        for folder in remove:shutil.rmtree(folder)
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('workspace',type=Path)
    parser.add_argument('--apply',action='store_true');args=parser.parse_args()
    print(json.dumps(prune(args.workspace,args.apply),indent=2))
