"""Reopen the full example's guide geometry, CSV, hashes and downloaded package."""
import csv,hashlib,io,json,urllib.request,zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
from studio.mechanics.harness_connections import reference_pad
from studio.catalog import Catalog

APP=Path(__file__).resolve().parents[1]
out=APP/'evidence/pin_routing_2026-11-16';report={}
identity='project_95d1e73d971c4ba1'
for port,workspace in [(8770,APP/'evidence/open_cradle_full_workflow_2026-11-16/trial-2/workspace'),(8768,APP/'workspace/internal_layout_demo_2026_11_15')]:
 base=f'http://127.0.0.1:{port}/api/'
 with urllib.request.urlopen(base+'projects') as stream:assert len(json.load(stream))==3
 with urllib.request.urlopen(base+'projects/'+identity) as stream:p=json.load(stream)
 assert not p['active_run_id']
 old=json.loads((out/('previous-project.json' if port==8770 else 'main-previous-project.json')).read_text())
 for key in ['revision','instances','circuit','motor_bindings','articulation','mechanical_result','layout_result','simulation_result']:assert p[key]==old[key],key
 directory=workspace/'runs'/p['harness_result']['run_id'];r=json.loads((directory/'result.json').read_text())
 assert r['routing_valid'] and not r['full_connection_verified']
 assert len(r['routes'])==11 and len({w['color'] for w in r['routes']})==11
 for name,digest in r['artifact_sha256'].items():assert hashlib.sha256((directory/name).read_bytes()).hexdigest()==digest,name
 for segment in r['bundle_segments']:
  mesh=tm.load(directory/segment['file'],force='mesh');assert mesh.is_volume and mesh.is_watertight
 preview=json.loads((directory/'preview.json').read_text());assert preview['default_hidden_roles']==['bundle_corridor']
 guides=[part for part in preview['parts'] if part.get('role')=='wire_guide'];assert len(guides)==33
 scene=tm.load(directory/'assembly-with-wires.glb',force='scene')
 catalog=Catalog();instances={i['id']:i for i in p['instances']}
 for wire in r['routes']:
  assert abs(wire['diameter_m']-.0012)<1e-12
  for suffix in ['lane','from','to','from_port','to_port']:assert wire['id']+'_'+suffix in scene.graph.nodes_geometry
  for side,ep in wire['endpoints'].items():
   np.testing.assert_allclose(wire['fanout_points_m'][side][0],ep['position_m'],atol=1e-12)
   assert not ep['physical_verified'] and ep['coordinate_frame']=='world_rest'
   if ep['evidence']=='manufacturer_reference_pad':
    asset=next(a for a in r['visual_assets'] if a['instance_id']==ep['instance_id'])
    pin=catalog.resolve(instances[ep['instance_id']])['ports'][ep['port_id']]['physical_pin']
    np.testing.assert_allclose(ep['position_m'],reference_pad(asset,ep['port_id'],pin)['position_m'],atol=1e-12)
  for suffix in ['from','to']:
   guide=next(part for part in guides if part['id']==wire['id']+'_'+suffix)
   assert guide['dashed']==(not wire['endpoints'][suffix]['fanout_clearance_checked'])
 with (directory/'wire-list.csv').open() as stream:rows=list(csv.DictReader(stream))
 assert len(rows)==11 and len({row['display_color'] for row in rows})==11
 assert all(row['physical_connection_verified']=='False' for row in rows)
 with urllib.request.urlopen(base+'projects/'+identity+'/export',timeout=60) as stream:package=stream.read()
 with zipfile.ZipFile(io.BytesIO(package)) as archive:
  assert archive.testzip() is None
  contract=json.loads(archive.read('routing_contract.json'))
  assert contract['ports']==r['endpoint_registration'] and contract['aggregation_exits']==r['terminal_assumptions']
  assert not contract['full_connection_verified'] and contract['fanout_checks']==r['fanout_checks']
  count=len(archive.namelist())
 (out/('review.zip' if port==8770 else 'main-review.zip')).write_bytes(package)
 checked=sum(c['passed'] for c in r['fanout_checks']);failed=[c for c in r['fanout_checks'] if not c['passed']]
 assert checked==17 and len(failed)==5,(checked,failed)
 report[str(port)]={'run_id':p['harness_result']['run_id'],'corridor_checks':len(r['checks']),'wire_count':11,'unique_display_colours':11,
  'terminal_leads_checked':checked,'terminal_leads_unresolved':len(failed),'unresolved':failed,'zip_entries':count,
  'hashes_geometry_pads_csv_zip_verified':True,'upstream_design_unchanged':True,'full_connection_verified':False}
(out/'independent-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
