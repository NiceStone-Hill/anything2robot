"""Configure the existing postprocessor for the dog, with model IDs from optimization."""
import copy
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'robot_interior_postprocess'))
from mechanical import LegacyReader
source = ROOT/'result/custom_dog_test/dog_20261101-204826/result_round1'
with (source/'robot_result.pkl').open('rb') as stream:
    robot = LegacyReader(stream).load()
model_names = ['MG4005V2','DM6006','DM8006','DM10010']
assignments = {}; summary = []
queue = [robot.link_tree]; index = 0
while queue:
    node=queue.pop(0); queue.extend(node.children)
    if np.linalg.norm(node.val.axis[1]) == 0: continue
    values = np.array(robot.motor_results[index]); radius=values[6]; length=np.linalg.norm(values[:3]-values[3:6])
    matches=[i for i,m in enumerate(robot.motor_lib) if np.isclose(m[1],radius) and np.isclose(m[0],length)]
    if len(matches)!=1: raise ValueError('Ambiguous optimized motor envelope')
    model = model_names[matches[0]]; assignments['motor_'+node.val.name]=model
    summary.append({'link':node.val.name,'motor_id':'motor_'+node.val.name,'model':model,
                    'center_mm':((values[:3]+values[3:6])*5).tolist(),
                    'annotated_joint_mm':(np.array(node.val.axis[0])*10).tolist(),
                    'axis':node.val.axis[1].tolist(),'radius_mm':float(radius*10),'envelope_length_mm':float(length*10),
                    'library_torque_Nm':float(robot.motor_lib[matches[0]][2]),
                    'status':'selected_by_original_optimizer; library values and real hardware compatibility unverified'})
    index+=1
(ROOT/'data/custom_dog/motor_selection.json').write_text(json.dumps(summary,indent=2))
base=ROOT/'robot_interior_postprocess/configs'
raw=json.loads((base/'cactus.json').read_text())
raw.update(display_name='Custom dog five joint test', design_result=str(source.relative_to(ROOT)),
           manufacturing_result='result/custom_dog_manufacturing',circuit_directory='data/custom_dog/circuit',
           output='robot_interior_postprocess/outputs/custom_dog_layout',placement_part='BODY',
           motor_mapping=dict(zip(['SERVO2','SERVO1','SERVO3','SERVO4','SERVO5'],
                                  ['motor_HEAD','motor_L_ARM','motor_R_ARM','motor_L_LEG','motor_R_LEG'])),
           motor_model_mapping=assignments)
(base/'custom_dog.json').write_text(json.dumps(raw,indent=2))
config=json.loads((base/'cactus_carrier_assembly.json').read_text())
config['mechanical']['config']='custom_dog.json'
for item in config['components']: item['owner_link']='BODY'
config['openings']={name:copy.deepcopy(config['openings']['BODY']) for name in robot.link_dict}
config['gravity']['evidence']='Test assumption: uploaded dog is upright along source +Z. No balance/walking certification.'
config['search'].update(max_architectures=8,max_seed_candidates=480,max_evaluations=180)
(base/'custom_dog_carrier_assembly.json').write_text(json.dumps(config,indent=2))
print(json.dumps(summary,indent=2));print('CONFIG:',base/'custom_dog_carrier_assembly.json')
