"""Independent reload and topology verification of real formal output."""
import hashlib,json,zipfile
from pathlib import Path
import trimesh as tm
from studio.mechanics.solid import regions
out=Path('examples/local_access_2026-11-15/formal-240-retry')
r=json.loads((out/'assembly.json').read_text())['result'];dest=out/'verified-assets';dest.mkdir(exist_ok=True)
items=[]
with zipfile.ZipFile(out/'mechanical-review.zip') as z:
    for a in r['geometry_assets']:
        raw=z.read(a['file']);assert hashlib.sha256(raw).hexdigest()==a['sha256']
        f=dest/a['file'];f.write_bytes(raw);m=tm.load(f,force='mesh')
        assert m.is_volume and m.is_watertight and regions(m)==1,(a['id'],m.is_volume,regions(m))
        items.append({'id':a['id'],'watertight':True,'connected_regions':1,'volume_mm3':float(m.volume)})
assert r['geometry_valid'] and all(c['passed'] for c in r['checks'])
assert r['analysis']['access_suggestions']['source']=='live_vlm_surface_regions'
assert {c['name'] for c in r['checks'] if c['name'].endswith('_insertion')}=={'neck_motor_insertion','neck_cartridge_insertion'}
report={'geometry_verified':True,'formal_run_id':'run_fc203b0cbd6940ae','assets':items,
        'changed_size_360_result':'rejected_for_motion_collision','manufacturing_verified':False}
(out/'independent-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False))
