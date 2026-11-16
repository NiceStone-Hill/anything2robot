"""Generate a narrow-link fixture through the real assembly pipeline."""
import hashlib
import json
import sys
from pathlib import Path
from studio.mechanics.solid import box, union
from studio.mechanics.graph_assembly import generate
from studio.project import atomic


def run(destination,paired=False):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    source=destination/'fixture.stl'
    union(box([-100,-50,-30],[0,50,30]),box([0,-12,-12],[70,12,12])).export(source)
    graph={'root_body_id':'base','bodies':[{'id':'base','name':'Base','seed_mm':[-35.,0.,0.]},
           {'id':'arm','name':'Narrow arm','seed_mm':[50.,0.,0.]}],
           'joints':[{'id':'hinge','name':'Hinge','parent':'base','child':'arm','axis':[0.,0.,1.],
                      'position_mm':[15.,0.,0.],'limits_deg':[-25,25],'dof':1}]}
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    config={'mode':'reference','length_m':.17,'up_axis':'+Z','support':'fixed_base','max_joints':1}
    project={'id':'open_cradle_fixture','revision':1,'source_shapes':[{'format':'stl','unit':'mm','sha256':digest}],
             'articulation_config':config,'articulation':{'graph':graph,'run_id':'fixture'},
             'motor_bindings':[{'joint_id':'hinge','catalog_id':'sg90_digital_towerpro','branch':'text2robot_rectangular','motor_instance_id':'servo'}]}
    project['surface_plan_reuse']={'source_sha256':digest,'graph':graph,'articulation_config':config,'source_run_id':'fixture',
        'plan':{'regions':[{'body_id':'base','center_mm':[-40,0,-20],'normal':[0,0,-1],'reason':'Underside service access'},
                           {'body_id':'arm','center_mm':[50,0,0],'normal':[0,0,1],'reason':'Output-only arm'}]}}
    if paired:
        union(box([-50,-50,-30],[50,50,30]),box([50,-12,-12],[120,12,12]),box([-120,-12,-12],[-50,12,12])).export(source)
        digest=hashlib.sha256(source.read_bytes()).hexdigest()
        project['source_shapes'][0]['sha256']=digest;config['length_m']=.24;config['max_joints']=2
        graph['bodies'][0]['seed_mm']=[0.,0.,0.];graph['bodies'][1]['seed_mm']=[85.,0.,0.]
        graph['bodies'].append({'id':'left','name':'Left arm','seed_mm':[-85.,0.,0.]})
        graph['joints'][0]['position_mm']=[50.,0.,0.]
        graph['joints'].append({'id':'left_hinge','name':'Left hinge','parent':'base','child':'left','axis':[0.,0.,1.],
                                'position_mm':[-50.,0.,0.],'limits_deg':[-25,25],'dof':1})
        project['motor_bindings'].append({'joint_id':'left_hinge','catalog_id':'sg90_digital_towerpro','branch':'text2robot_rectangular','motor_instance_id':'left_servo'})
        reuse=project['surface_plan_reuse'];reuse['source_sha256']=digest
        reuse['plan']['regions'][0]['center_mm']=[0,0,-20];reuse['plan']['regions'][1]['center_mm']=[85,0,0]
        reuse['plan']['regions'].append({'body_id':'left','center_mm':[-85,0,0],'normal':[0,0,1],'reason':'Output-only arm'})
    result=generate(project,source,destination)
    failed=[c for c in result['checks'] if not c['passed']]
    summary={'geometry_valid':result['geometry_valid'],'failed_checks':failed,'elapsed_s':result['elapsed_s'],
             'placement':result['placement_candidates'][0]['selected']}
    atomic(destination/'summary.json',summary)
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    assert result['geometry_valid'],failed
    assert summary['placement']['outside_fraction']>.03
    assert len(result['analysis']['motor_installation'])==2*len(graph['joints'])
    assert next(c for c in result['checks'] if c['name']=='hinge_ear_screw_bores')['passed']
    assert all(i['passed'] for i in result['analysis']['motor_installation'])
    assert not next(d for d in result['analysis']['access_decisions'] if d['owner_body_id']=='arm')['panel_created']
    assert not result['manufacturing_ready']


if __name__=='__main__':run(sys.argv[1],paired='--paired' in sys.argv)
