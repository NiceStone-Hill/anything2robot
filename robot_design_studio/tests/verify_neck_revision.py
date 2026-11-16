"""Independent artifact checks for the archived 300/500 mm regeneration."""
import json,hashlib
from pathlib import Path
import trimesh as tm
import numpy as np
base=Path('examples/neck_revision_2026-11-16')
cases=[]
for directory in [base/'formal-300-v5',base/'trial-500']:
    r=json.loads((directory/'manifest.json').read_text())
    if not (directory/'body_shell.stl').exists():
        import zipfile
        # Extract into a separate verification directory; preserve archived ZIP.
        dest=directory/'verification';dest.mkdir(exist_ok=True)
        with zipfile.ZipFile(directory/'mechanical-review.zip') as package:
            for a in r['geometry_assets']:(dest/a['file']).write_bytes(package.read(a['file']))
    else:dest=directory
    for a in r['geometry_assets']:
        f=dest/a['file'];assert hashlib.sha256(f.read_bytes()).hexdigest()==a['sha256']
        mesh=tm.load(f,force='mesh');actual=bool(mesh.is_watertight and mesh.is_volume)
        reported=next(c['passed'] for c in r['checks'] if c['name']=='stl_roundtrip_'+a['id'])
        assert actual==reported,(f,actual,reported)
    cases.append({'directory':str(directory),'elapsed_s':r['elapsed_s'],'geometry_valid':r['geometry_valid'],
        'source_sha256':r['source_sha256'],'seam':r['analysis']['joint_clearance'],
        'bridges_mm':[m['length_mm'] for m in r['mounts']],
        'motor_volume_m3':next(a['volume_m3'] for a in r['geometry_assets'] if a['id'].endswith('_motor')),
        'body_sha256':r['geometry_assets'][0]['sha256'],'failed_checks':[c['name'] for c in r['checks'] if not c['passed']]})
assert cases[0]['source_sha256']==cases[1]['source_sha256']
assert cases[0]['body_sha256']!=cases[1]['body_sha256']
assert np.isclose(cases[0]['motor_volume_m3'],cases[1]['motor_volume_m3'],rtol=1e-5,atol=0)
assert all(c['seam']['hardware_fit_clearance_mm']==.4 for c in cases)
(base/'verification.json').write_text(json.dumps({'checks_passed':True,'cases':cases,'manufacturing_verified':False},ensure_ascii=False,indent=2))
print(json.dumps(cases,ensure_ascii=False))
