let solidViewer=null,mechanicalEpoch=0,mechanicalRun=null,assemblySubmitting=false;
const partitionButton=node('button','Split shells first');partitionButton.id='shell-partition-generate';
$('graph-assembly-generate').before(partitionButton);
partitionButton.onclick=safe(()=>startGraphAssembly(partitionButton,'shell_partition'));
const manualRemovalLabel=node('label');const manualRemoval=document.createElement('input');manualRemoval.type='checkbox';manualRemoval.id='manual-shell-removal';manualRemovalLabel.append(manualRemoval,document.createTextNode(' Allow manual shell removal'));$('graph-assembly-generate').after(manualRemovalLabel);
const mechanicalNext=node('button','Continue to components','primary');mechanicalNext.id='mechanical-next';mechanicalNext.hidden=true;mechanicalNext.onclick=()=>changeView('parts');$('mechanical-result').after(mechanicalNext);
function mechanicalPerformance(){return JSON.stringify(solidViewer?.stats()||{});}
function mechanicalRefresh(){
  const p=state.project;if(!p)return;
  manualRemoval.checked=p.assembly_policy==='manual_extraction_review';
  mechanicalNext.hidden=!p.mechanical_result?.geometry_valid;
  partitionButton.disabled=assemblySubmitting||!!p.active_run_id||!p.articulation||!p.motor_bindings?.length;
  $('graph-assembly-generate').disabled=assemblySubmitting||!!p.active_run_id||!p.articulation||!p.motor_bindings?.length; $('graph-assembly-input').textContent=p.articulation?`${p.articulation.graph.joints.length} joints · ${p.motor_bindings?.length||0} motors`:'Import a model, identify joints and select motors first.';
  $('shape-name').textContent=p.source_shapes?.[0]?.name||'No model imported.';
  $('motor-instance').replaceChildren(node('option','Select SG90'));$('motor-instance').firstChild.value='';
  for(const inst of p.instances.filter(i=>i.catalog_id==='sg90_digital_towerpro')){const option=node('option',instanceName(inst)+' · '+inst.id.slice(-5));option.value=inst.id;$('motor-instance').append(option);}
  const cfg=p.mechanical_config;
  if(cfg){$('shape-length').value=String(cfg.length_m*1000);$('motion-angle').value=String(cfg.amplitude_deg);$('motion-intent').value=cfg.intent;$('motor-instance').value=cfg.motor_instance_id;}
  else if($('motor-instance').options.length===2)$('motor-instance').selectedIndex=1;
  $('mechanical-mode').options[1].disabled=!state.live;
  $('mechanical-mode').value=cfg?.mode||(state.live?'live':'reference');
  $('mechanical-result').hidden=true;
  mechanicalEpoch++;if(solidViewer)solidViewer.play=false;
  if(p.mechanical_result)safe(async()=>{const run=await api('runs/'+p.mechanical_result.run_id);if(state.project?.id===p.id)await mechanicalRender(run);})();
  else if(p.runs?.length)safe(async()=>{const run=await api('runs/'+p.runs.at(-1));if(state.project?.id===p.id&&run.kind==='assembly'&&terminal.has(run.status))await mechanicalRender(run);})();
}
async function mechanicalRender(run){
  const result=run.result;if(!result){
    mechanicalRun=run.id;$('mechanical-result').hidden=false;mechanicalNext.hidden=true;
    $('mechanical-summary').textContent='No validated structure generated';
    $('mechanical-reason').textContent=englishText(run.error||'The structure search did not finish successfully. Review the joint region and assembly constraints before trying again.');
    $('mechanical-canvas').hidden=true;
    for(const id of ['mechanical-fastening','mechanical-checks','mechanical-parts','mechanical-downloads','mechanical-hardware','mechanical-pending','assembly-steps','multiview'])$(id)?.replaceChildren();
    return;
  }mechanicalRun=run.id;$('mechanical-result').hidden=false;$('mechanical-diagnostics').open=!result.geometry_valid&&!result.shell_partition_valid;
  const spatialDraft=!!result.layout_domains?.length&&['assembly_collisions','joint_sweep','structural_paths'].every(name=>result.checks?.some(c=>c.name===name&&c.passed));
  mechanicalNext.hidden=!result.geometry_valid&&!spatialDraft;mechanicalNext.textContent=result.geometry_valid?'Continue to components':'Continue to layout draft';mechanicalNext.onclick=()=>{changeView(result.geometry_valid?'parts':'layout');if(!result.geometry_valid)setTimeout(()=>{layoutDraft.checked=true;},0);};
  $('mechanical-summary').textContent=result.error_kind==='internal_geometry_processing'?'Geometry processing failed':result.geometry_valid&&result.status==='candidate_manual_review'?'Structure candidate — manual assembly review required':result.geometry_valid?(result.warnings?.length?(result.warnings.some(w=>w.code==='module_envelope_outside')?'Structure ready — some parts extend outside the shell':'Structure checks passed'):'Structure checks passed'):'Structure checks failed';
  if(result.shell_partition_valid)$('mechanical-summary').textContent='Shell split ready — mounts and wiring still required';
  $('mechanical-reason').textContent=englishText(result.error||result.plan?.reason||'');
  const failedChecks=(result.checks||[]).filter(c=>!c.passed);
  if(!result.error&&failedChecks.length){
    const reasons=failedChecks.map(c=>c.name.endsWith('_fastener_access')?'Screw or tool access is blocked: '+c.purpose:c.name.endsWith('_final_connections')||c.name==='structural_paths'?'A shell remains disconnected after generating access openings':c.name.endsWith('_intact_leaf_shell')||c.name.startsWith('solid_')&&c.connected_regions>1?'Shell contains disconnected pieces':c.name.endsWith('_module_room')?'Motor envelope outside shell: '+(100*c.outside_fraction).toFixed(2)+'% (limit '+(100*c.maximum_outside_fraction).toFixed(0)+'%）':c.name.endsWith('_mating_surface')?'A required mounting face is missing or disconnected':c.name.endsWith('_output_registration')?'Motor output is displaced from the joint':c.name.endsWith('_retained')?'Mounting surface check failed':c.name.startsWith('export_deviation_')||c.name.startsWith('stl_roundtrip_')?'STL export check failed':c.reason||c.name);
    $('mechanical-reason').textContent='Failed: '+[...new Set(reasons)].join('；')+'。';
  }
  for(const warning of result.warnings||[])$('mechanical-reason').textContent+=' Warning: '+englishText(warning.reason);
  const placements=result.placement_candidates||[];
  if(placements.some(p=>p.mounting_style==='open_cradle'&&p.selected?.output_registration_passed===undefined))$('mechanical-reason').textContent+=' Older result: motor output position and final mating faces have not been checked. Regenerate structure.';
  const access=result.analysis?.access_suggestions;
  if(access){$('mechanical-reason').textContent+=' Cover: '+(access.source==='live_vlm_surface_regions'?'Online VLM surface plan.':'Local surface plan.')+' Generated mounts and insertion paths. '+access.regions.map(r=>bodyName(result.rigid_bodies?.find(b=>b.id===r.body_id))+': '+englishText(r.reason)).join('；');}
  for(const panel of result.analysis?.lid_planes||[])if(panel.method==='local_oval_access_panel')$('mechanical-reason').textContent+=' '+bodyName(result.rigid_bodies?.find(b=>b.id===panel.owner_body_id))+' uses a local access cover.';
  const splitPanels=(result.analysis?.lid_planes||[]).filter(p=>['offset_plane_split','protected_stepped_split'].includes(p.method));
  for(const owner of new Set(splitPanels.map(p=>p.owner_body_id)))$('mechanical-reason').textContent+=' '+bodyName(result.rigid_bodies?.find(b=>b.id===owner))+' has removable shell pieces with intact mounting surfaces.';
  if(splitPanels.length)$('mechanical-reason').textContent+=' Open the shell for actuator installation; no permanent tool ports. Exploded view shows removal directions. Follow the assembly order: some covers require the attached child parts to be removed first.';
  for(const decision of result.analysis?.access_decisions||[])if(!decision.panel_created)$('mechanical-reason').textContent+=' '+bodyName(result.rigid_bodies?.find(b=>b.id===decision.owner_body_id))+' has no service cover.';
  $('mechanical-reason').textContent=englishText($('mechanical-reason').textContent);
  if(result.shell_partition_valid)$('mechanical-reason').textContent='Shell parting stage: empty shell removal and STL exports checked. Mount attachment, installed hardware access, electronics and wiring are still pending. Gold parts are position references. Use Explode to inspect the split; uncheck Hide covers to show every shell piece. This is not a complete robot print package.';
  $('mechanical-checks').replaceChildren();
  const checkNames={assembly_collisions:'Static collisions',joint_sweep:'Joint motion samples',motor_insertion:'Motor insertion',electronics_removal:'Electronics removal',fixed_lid_removal:'Fixed cover removal',moving_lid_removal:'Moving cover removal',external_lid_tools:'Cover screw access',horn_screw_access:'Horn screw access',retainer_fastener_bores:'Motor retainer bores',lid_fastener_seats:'Cover fastener seats',fixed_shell_to_carrier_mm3:'Fixed support',moving_shell_to_bridge_mm3:'Moving support',fixed_shell_to_dock_mm3:'Electronics support',pipeline:'Generation'};
  for(const c of result.checks||[]){const asset=result.geometry_assets?.find(a=>'solid_'+a.id===c.name);const title=asset?assetName(asset)+': solid integrity':c.name.endsWith('_mating_surface')?'Mounting face on '+c.owner_body_id:c.name.endsWith('_output_registration')?'Motor output at joint':c.name.endsWith('_module_room')?'Motor envelope fit':c.name.endsWith('_insertion')?'Motor insertion through cover':c.name.endsWith('_final_connections')?'Support continuity':checkNames[c.name]||c.name;const d=node('details');d.append(node('summary',(c.passed?'Passed · ':'Failed · ')+title));d.append(node('pre',englishJSON(c)));$('mechanical-checks').append(d);}
  $('mechanical-hardware').replaceChildren(...(result.hardware_requests||[]).map(v=>node('li',`${v.specification||v.template_id} × ${v.quantity} · ${v.purpose} · Verify hardware specification`)));
  $('mechanical-pending').replaceChildren(...(result.pending||[]).map(v=>node('li',v)));
  $('assembly-steps').replaceChildren(...(result.assembly_dependencies||[]).map(v=>node('li',v.action)));
  const base='/api/runs/'+run.id+'/artifacts/';$('mechanical-downloads').replaceChildren();
  for(const [file,label] of [['shell-partition-review.zip','Shell split review ZIP'],['mechanical-review.zip','Structure ZIP'],['assembly.glb','Assembly GLB'],['manifest.json','Check details']]){if(result.artifacts?.includes(file)){const a=node('a',label);a.href=base+file;$('mechanical-downloads').append(a);}}
  $('mechanical-parts').replaceChildren();$('multiview').replaceChildren();
  for(const file of ['top.png','front.png','side.png']){if(result.artifacts?.includes(file)){const img=document.createElement('img');img.src=base+file;img.alt=file+' model view';$('multiview').append(img);}}
  $('mechanical-fastening')?.replaceChildren();
  $('mechanical-canvas').hidden=!result.artifacts?.includes('preview.json');
  if(!result.artifacts?.includes('preview.json'))return;
  const response=await fetch(base+'preview.json');const preview=await response.json();
  if(run.id!==mechanicalRun)return;
  try{if(!solidViewer)solidViewer=new SolidViewer($('mechanical-canvas'));solidViewer.actuatorOnly=false;solidViewer.hideBodies=false;$('mechanical-view-mode').value='exterior';solidViewer.hiddenShells=$('hide-lids').checked;solidViewer.explode=$('explode').checked;solidViewer.load(preview);$('viewer-error').textContent='';}
  catch(e){$('viewer-error').textContent=e.message;}
  if(solidViewer){
    let control=$('mechanical-shell-opacity');
    if(!control){control=node('span');control.id='mechanical-shell-opacity';$('mechanical-view-mode').closest('.viewer-tools').append(control);}
    control.replaceChildren(shellOpacityControl(solidViewer,()=>{
      // A shell opacity adjustment should reveal the shell even from actuator view.
      const refit=solidViewer.actuatorOnly||solidViewer.hideBodies;
      $('mechanical-view-mode').value='exterior';solidViewer.actuatorOnly=false;solidViewer.hideBodies=false;solidViewer.hiddenShells=$('hide-lids').checked;
      if(refit){$('part-detail').textContent='';solidViewer.fitVisible();}
    }));
  }
  let fastening=$('mechanical-fastening');
  if(!fastening){fastening=node('details');fastening.id='mechanical-fastening';$('mechanical-downloads').before(fastening);}
  fastening.replaceChildren(node('summary','Screws & assembly access'));
  const accessReports=result.analysis?.fastener_access;
  if(!accessReports){fastening.append(node('p',result.shell_partition_valid?'Shell parting stage only. The gold parts show fixed hardware positions; their shell supports and installed assembly checks are still pending.':'This saved result predates screw/tool access checks. Rebuild structure to check mounting-ear screws, output connections and tool access.','notice'));}
  else{
    fastening.open=true;
    if(!result.geometry_valid&&accessReports.every(r=>r.passed))fastening.append(node('p','Screw access checks passed, but the overall structure is still rejected. Review the failed structure checks before printing.','notice'));
    const seats=result.analysis?.mounting_surfaces;
    if(seats)fastening.append(node('p',`${seats.filter(s=>s.passed).length}/${seats.length} mating faces retained on their connected shells. Output centres are checked against the annotated joints. These geometry checks do not verify physical fit or load capacity.`));
    fastening.append(node('p','Motor screws hold the housing to its mount. For stock-horn servos, the centre screw locks the horn to the shaft; adaptor screws connect the moving shell to the horn. Gold surfaces are integral shell mounts, not extra printed parts. Cyan direction markers indicate checked approaches; red markers indicate a blocked approach. Markers appear at the neutral assembly pose and do not represent the complete clearance shape. Tool sizes are candidates until measured.'));
    const hide=node('button','Hide tool paths');hide.onclick=()=>{solidViewer.showFasteners=false;solidViewer.draw();};fastening.append(hide);
    for(const report of accessReports){
      const row=node('div',undefined,'part-card'),title=node('strong',report.purpose+' · '+(report.passed?'Access checked':'Blocked'));
      row.append(title,node('p','Joint: '+report.joint_id+' · Shell: '+report.owner_body_id+' · '+report.tools.map(t=>t.kind.replaceAll('_',' ')).join(' + ')));
      if(report.deferred_joint_ids?.length)row.append(node('p','Complete this fastening before installing actuators: '+report.deferred_joint_ids.join(', '),'muted'));
      if(report.detach_body_ids.length)row.append(node('p','Fasten before attaching: '+report.detach_body_ids.join(', '),'muted'));
      const show=node('button','Show screw access');show.onclick=()=>{
        solidViewer.showFasteners=true;solidViewer.fastenerFocus=report.id;solidViewer.play=false;solidViewer.angle=0;$('motion-play').textContent='Play';$('motion-slider').value=50;
        const opacity=$('mechanical-shell-opacity').querySelector('input');opacity.value=25;opacity.dispatchEvent(new Event('input'));
        solidViewer.selected='';solidViewer.draw();$('mechanical-canvas').scrollIntoView({block:'center'});
      };row.append(show);fastening.append(row);
    }
    const a=node('a','Download screw access report');a.href=base+'fastener-access.json';fastening.append(a);
  }
  $('motion-play').textContent='Play';
  for(const p of result.geometry_assets){const row=node('div',undefined,'mechanical-part');const b=node('button',assetName(p));b.onclick=()=>{if(solidViewer){solidViewer.selected=p.id;solidViewer.draw();}$('part-detail').textContent=assetName(p)+' · '+bodyName(result.rigid_bodies?.find(b=>b.id===p.owner_body_id))+' · '+p.template_id+' v'+p.template_revision;};
    const a=node('a','STL');a.href=base+p.file;row.append(b,a);$('mechanical-parts').append(row);}
}
async function mechanicalPoll(id){
  mechanicalRun=id;const epoch=++mechanicalEpoch;$('graph-assembly-generate').disabled=true;$('mechanical-generate').disabled=true;$('mechanical-cancel').hidden=false;
  while(epoch===mechanicalEpoch){const run=await api('runs/'+id);if(epoch!==mechanicalEpoch)return;
    $('mechanical-state').textContent=run.status==='running'?'Generating structure…':(run.status==='completed'?'Structure ready':statuses[run.status]||run.status);
    $('mechanical-events').replaceChildren(...[...(run.events||[]),...(run.progress||[])].map(v=>node('li',v.detail||v.type)));
    if(terminal.has(run.status)){const current=await api('projects/'+run.project_id);if(epoch!==mechanicalEpoch)return;
      state.project=current;$('revision').textContent='Version '+current.revision+'';
      renderResult(current.circuit,current.validation,'Saved wiring');
      $('mechanical-generate').disabled=false;$('graph-assembly-generate').disabled=false;$('mechanical-cancel').hidden=true;
      if(run.status!=='stale')await mechanicalRender(run);
      if(run.result?.shell_partition_valid)message('Shell split saved. Mounts and wiring are still required before printing the robot.');
      else if(run.result?.geometry_valid)message(run.result.warnings?.length?'Structure saved. Review warnings below.':'Structure saved.');
      else if(run.status==='rejected')message('Structure checks failed. See details below.',true);
      if(run.error)message(run.error,true);if(typeof layoutRefresh==='function')await layoutRefresh();if(typeof simulationRefresh==='function')await simulationRefresh();await listProjects();return;
    }await new Promise(r=>setTimeout(r,350));
  }
}
$('example-shape').onclick=safe(async()=>{if(!state.project)throw Error('Create or open a project first.');if(state.dirty)throw Error('Save component changes first.');const p=await api('projects/'+state.project.id+'/appearance',{revision:state.project.revision,builtin:'box',dimensions_mm:[240,90,80]});await openProject(p);changeView('mechanics');message('Sample model imported.');});
$('shape-file').onchange=safe(async e=>{const f=e.target.files[0];if(!f||!state.project)return;if(state.dirty)throw Error('Save component changes first.');if(f.size>40000000)throw Error('File limit: 40 MB.');const bytes=new Uint8Array(await f.arrayBuffer());let str='';for(let i=0;i<bytes.length;i+=8192)str+=String.fromCharCode(...bytes.subarray(i,i+8192));
  const p=await api('projects/'+state.project.id+'/appearance',{revision:state.project.revision,filename:f.name,unit:$('shape-unit').value,data_base64:btoa(str)});await openProject(p);changeView('mechanics');});
$('mechanical-generate').onclick=safe(async()=>{if(!state.project)return;if(state.dirty)throw Error('Save component changes first.');const data={revision:state.project.revision,length_m:Number($('shape-length').value)/1000,amplitude_deg:Number($('motion-angle').value),motor_instance_id:$('motor-instance').value,mode:$('mechanical-mode').value,intent:$('motion-intent').value};
  $('mechanical-generate').disabled=true;try{const run=await api('projects/'+state.project.id+'/mechanics',data);await mechanicalPoll(run.id);}finally{$('mechanical-generate').disabled=false;}});
$('mechanical-cancel').onclick=safe(async()=>{if(mechanicalRun)await api('runs/'+mechanicalRun+'/cancel',{});});
$('motion-play').onclick=()=>{if(solidViewer)$('motion-play').textContent=solidViewer.toggle()?'Pause':'Play';};
$('explode').onchange=e=>{if(solidViewer){solidViewer.explode=e.target.checked;solidViewer.fitVisible();}};
$('hide-lids').onchange=e=>{if(solidViewer){solidViewer.hiddenShells=e.target.checked;solidViewer.fitVisible();}};
$('motion-slider').oninput=e=>{if(solidViewer){solidViewer.play=false;const [lo,hi]=solidViewer.joint.limits_rad;solidViewer.angle=lo+(hi-lo)*Number(e.target.value)/100;solidViewer.draw();const j=solidViewer.graph?.joints[0];const degrees=j?(solidViewer.angle>=0?j.limits_deg[1]:-j.limits_deg[0])*solidViewer.angle:solidViewer.angle*180/Math.PI;$('motion-readout').textContent=degrees.toFixed(1)+'° · kinematic preview';$('motion-play').textContent='Play';}};

async function startGraphAssembly(button=$('graph-assembly-generate'),stage='complete'){
  if(!state.project||assemblySubmitting)return;
  if(state.dirty)throw Error('Save component changes first.');
  const projectId=state.project.id;assemblySubmitting=true;button.disabled=true;$('graph-assembly-generate').disabled=true;
  try{
    const current=await api('projects/'+projectId);
    if(state.project?.id!==projectId)return;
    // Synchronize before submitting; never submit a stale revision or duplicate job.
    state.project=current;$('revision').textContent='Version '+current.revision+' · Saved';
    if(current.active_run_id){await openProject(current);message('Task already running. Progress restored.');return;}
    const run=await api('projects/'+projectId+'/assembly',{revision:current.revision,mode:'reference',stage,assembly_policy:manualRemoval.checked?'manual_extraction_review':'strict'});
    changeView('mechanics');message('Generating structure…');await mechanicalPoll(run.id);
  }catch(error){
    if(error.status===409&&state.project?.id===projectId){
      await openProject(await api('projects/'+projectId));message('Project refreshed. Wait for any active task.',false);
    }else throw error;
  }finally{
    assemblySubmitting=false;button.disabled=!!state.project?.active_run_id;
    $('graph-assembly-generate').disabled=!!state.project?.active_run_id||!state.project?.articulation||!state.project?.motor_bindings?.length;
  }
}
$('graph-assembly-generate').onclick=safe(()=>startGraphAssembly());

$('mechanical-view-mode').onchange=e=>{
  if(!solidViewer)return;
  solidViewer.actuatorOnly=e.target.value==='actuator';
  solidViewer.hideBodies=e.target.value!=='exterior';
  solidViewer.hiddenShells=e.target.value!=='exterior'||$('hide-lids').checked;
  solidViewer.selected=solidViewer.actuatorOnly?solidViewer.parts.find(p=>p.id.endsWith('_rotor'))?.id||'':'';
  solidViewer.fitVisible();
  $('part-detail').textContent=solidViewer.actuatorOnly?'Fixed motor housing · highlighted output hub follows the head. Kinematic preview only.':'';
};
