let simulationEpoch=0,simulationRun=null;
const simView=$('simulation-view');simView.append(node('h2','Export'),node('p','Export URDF or MJCF with the internal assembly.'));
simView.append(node('p','Fixed base; assumed mass and motor limits. Contact geometry is not included.','notice'));
const simAdvanced=node('details');simAdvanced.append(node('summary','Export settings (assumed)'));simView.append(simAdvanced);
const simFields={};
for(const [key,title,value,min,max] of [['printed_density_kg_m3','Printed density / kg·m⁻³',1200,100,5000],['hardware_density_kg_m3','Hardware density / kg·m⁻³',1000,100,10000],['assumed_torque_nm','Joint torque limit / N·m',1,.01,20]]){
  const label=node('label',title),input=node('input');input.type='number';input.value=value;input.min=min;input.max=max;input.step='any';input.id='sim-'+key;label.append(input);simAdvanced.append(label);simFields[key]=input;
}
const simGenerate=node('button','Export models','primary'),simCancel=node('button','Cancel');simCancel.hidden=true;
const simState=node('p'),simResults=node('div');simState.setAttribute('role','status');simView.append(simGenerate,simCancel,simState,simResults);
async function simulationRefresh(){
  const p=state.project;if(!p)return;simulationEpoch++;simResults.replaceChildren();simGenerate.disabled=!p.layout_result?.layout_valid;
  simState.textContent=p.layout_result?.layout_valid?'Internal assembly ready to export.':'Complete internal layout first.';
  if(p.simulation_config)for(const [k,input] of Object.entries(simFields))input.value=p.simulation_config[k];
  if(p.simulation_result){const run=await api('runs/'+p.simulation_result.run_id);if(state.project?.id===p.id)simulationRender(run);}
}
function simulationRender(run){
  const r=run.result;if(!r)return;simResults.replaceChildren();if(r.pipeline!=='simulator_export_0.2'&&!r.error){simResults.append(node('p','Regenerate to update this older export.'));return;}simResults.append(node('h3',r.export_valid?'Export ready':'Export failed'),node('p',errorText(r.error)||'URDF / MJCF with local mesh files.'));
  if(r.total_assumed_mass_kg!==undefined)simResults.append(node('p',`Assumed total mass ${r.total_assumed_mass_kg.toFixed(3)} kg`));
  const diagnostics=disclosure('Checks & limitations');diagnostics.open=!r.export_valid;simResults.append(diagnostics);checkSummary(diagnostics,r.checks||[]);
  const names={xml_documents:'XML format',local_mesh_resources:'Mesh resources',body_joint_definitions:'Bodies & joints',joint_coordinate_definitions:'Coordinates & axes'};
  for(const c of r.checks||[]){const d=node('details');d.append(node('summary',(c.passed?'Passed · ':'Failed · ')+(names[c.name]||c.name)),node('pre',englishJSON(c)));diagnostics.append(d);}
  for(const [file,label] of [['simulation-model.zip','Model ZIP'],['robot.xml','MJCF'],['robot.urdf','URDF'],['manifest.json','Export details']])if(r.artifacts?.includes(file)){const a=node('a',label);a.href='/api/runs/'+run.id+'/artifacts/'+file;simResults.append(a,node('br'));}
  for(const text of r.pending||[])diagnostics.append(node('p',text,'muted'));
}
async function simulationPoll(id){
  const epoch=++simulationEpoch;simulationRun=id;simGenerate.disabled=true;simCancel.hidden=false;
  while(epoch===simulationEpoch){const run=await api('runs/'+id);if(epoch!==simulationEpoch)return;
    simState.textContent=run.status==='running'?'Exporting…':run.status==='completed'?'Export complete':statuses[run.status]||run.status;
    if(terminal.has(run.status)){
      state.project=await api('projects/'+run.project_id);simGenerate.disabled=false;simCancel.hidden=true;
      if(run.status!=='stale')simulationRender(run);if(run.error)message(run.error,true);return;
    }await new Promise(resolve=>setTimeout(resolve,300));
  }
}
simGenerate.onclick=safe(async()=>{if(state.dirty)throw Error('Save component changes first.');const parameters=Object.fromEntries(Object.entries(simFields).map(([k,input])=>[k,Number(input.value)]));
  const run=await api('projects/'+state.project.id+'/simulation',{revision:state.project.revision,mode:'reference',...parameters});await simulationPoll(run.id);});
simCancel.onclick=safe(async()=>{if(simulationRun)await api('runs/'+simulationRun+'/cancel',{});});
