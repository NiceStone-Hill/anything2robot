"""Run the repository's solid-with-tenon manufacturing stage on all dog links."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
ROOT = Path(__file__).resolve().parents[2]
if len(sys.argv) == 4 and sys.argv[1] == '--part':
    source, name = Path(sys.argv[2]), sys.argv[3]
    sys.path.insert(0, str(ROOT))
    from metamaterial_filling.script.user_stl_metamaterial_filling_with_tenon import run_metamaterial_filling_for_stl_file
    out = ROOT/'result/custom_dog_manufacturing/solid_tenon'/name
    run_metamaterial_filling_for_stl_file(
        str(source/'urdf'/f'{name}.stl'), 'm', 0.05, None, 1.0, None,
        2.5, f'{name}_solid_tenon.stl', False, str(source/'robot_result.pkl'),
        str(ROOT/'metamaterial_filling/tenon'), False, output_dir=str(out))
    sys.exit(0)
import trimesh
source = Path(sys.argv[1]).resolve()
base = ROOT/'result/custom_dog_manufacturing'
base.mkdir(exist_ok=True)
state = {'source':str(source), 'status':'running', 'jobs':[]}
env = dict(os.environ, OPENSCAD_BIN=str(ROOT/'result/Cactus_manufacturing_20261015/tools/openscad'),
           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MPLCONFIGDIR='/tmp/dog_mpl', QT_QPA_PLATFORM='offscreen')
def save():
    (base/'status.json').write_text(json.dumps(state,indent=2))
for source_file in sorted((source/'urdf').glob('*.stl')):
    name = source_file.stem
    dest = base/'solid_tenon'/name; work = dest/'work'; work.mkdir(parents=True,exist_ok=True)
    job = {'part':name,'status':'running'}; state['jobs'].append(job); save()
    start = time.monotonic()
    with (dest/'run.log').open('w') as log:
        proc = subprocess.run([sys.executable,'-u',str(Path(__file__).resolve()),'--part',str(source),name],
                              env=env,cwd=work,stdout=log,stderr=subprocess.STDOUT)
    job['exit_code'] = proc.returncode; job['seconds'] = round(time.monotonic()-start,2)
    if proc.returncode == 0:
        mesh = trimesh.load(dest/f'{name}_solid_tenon.stl',force='mesh')
        logtext=(dest/'run.log').read_text()
        job.update(volume=bool(mesh.is_volume),watertight=bool(mesh.is_watertight),
                   components=len(mesh.split(only_watertight=False)),faces=len(mesh.faces),
                   concatenation_fallback='Falling back to direct mesh concatenation' in logtext,
                   restored_failed_cut='Restoring the replaced stl file' in logtext)
        job['status']='mesh_checks_passed' if job['volume'] and job['components']==1 and not job['concatenation_fallback'] and not job['restored_failed_cut'] else 'needs_review'
    else:
        job['status']='failed'
    save(); print(name,job,flush=True)
state['status']='passed' if all(j['status']=='mesh_checks_passed' for j in state['jobs']) else 'issues'
save()
