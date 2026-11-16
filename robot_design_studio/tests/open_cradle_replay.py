"""Rebuild archived annotations with current modules, without calling an AI service."""
import json,sys,time,traceback
from pathlib import Path
from studio.mechanics.graph_assembly import generate
from studio.project import atomic
source=Path(sys.argv[1]);dest=Path(sys.argv[2]);dest.mkdir(parents=True,exist_ok=True)
p=json.loads((source/'input-project.json').read_text());p['articulation_config']['mode']='reference'
old=json.loads((source/'manifest.json').read_text())
if old.get('analysis',{}).get('access_suggestions'):
 p['surface_plan_reuse']={'source_sha256':p['source_shapes'][0]['sha256'],'graph':p['articulation']['graph'],'articulation_config':p['articulation_config'],'plan':old['analysis']['access_suggestions'],'source_run_id':'archived-surface-regions'}
try:
 r=generate(p,source/'source.raw',dest)
 print(json.dumps({'geometry_valid':r['geometry_valid'],'failed':[c for c in r['checks'] if not c['passed']],'elapsed_s':r['elapsed_s']},ensure_ascii=False,indent=2),flush=True)
except Exception as exc:
 atomic(dest/'replay-failure.json', {'geometry_valid':False,'error':str(exc),'kind':type(exc).__name__})
 traceback.print_exc()
 raise
