let layoutViewer=null,layoutEpoch=0,layoutRun=null,layoutFields=[];
function layoutPerformance(){return JSON.stringify(layoutViewer?{...layoutViewer.stats(),visible_part_ids:layoutViewer.parts.filter(p=>viewerPartVisible(layoutViewer,p)).map(p=>p.id)}:{});}
const layoutView=$('layout-view');
layoutView.append(node('h2','Internal layout'),node('p','Place component envelopes and generate removable mounts.'));
layoutView.append(node('p','Reference CAD where available; unknown hardware remains an envelope. Physical variants are unverified.','notice'));
const layoutMissing=node('p',undefined,'notice');layoutMissing.setAttribute('role','status');layoutView.append(layoutMissing);
const layoutInputs=node('div',undefined,'catalog');const layoutEditor=disclosure('Component dimensions',layoutInputs);layoutEditor.open=true;layoutView.append(layoutEditor);
function layoutMissingFields(){return layoutFields.flatMap(f=>f.sizes.flatMap((input,index)=>{const value=input.value.trim(),mm=input.valueAsNumber;return !value||!Number.isFinite(mm)||mm<2||mm>300?[{input,label:instanceName(f.instance),axis:['Length','Width','Height'][index],empty:!value}]:[];}));}
function layoutUpdateMissing(){const missing=layoutMissingFields();layoutMissing.hidden=!missing.length;layoutMissing.textContent=missing.length?'Missing or invalid dimensions: '+missing.length+' fields (2–300 mm). '+[...new Set(missing.map(m=>m.label))].join('；')+'. Generate will focus the first field.':'';}
const layoutGenerate=node('button','Generate layout','primary'),layoutCancel=node('button','Cancel');layoutCancel.hidden=true;
layoutGenerate.id='layout-generate';layoutCancel.id='layout-cancel';
const layoutState=node('p');layoutState.setAttribute('role','status');layoutView.append(layoutGenerate,layoutCancel,layoutState);
const layoutDraft=node('input');layoutDraft.type='checkbox';layoutDraft.id='layout-draft';const draftLabel=node('label');draftLabel.append(layoutDraft,document.createTextNode(' Explore layout draft with unresolved structure checks'));layoutView.insertBefore(draftLabel,layoutGenerate);
const layoutResize=node('input');layoutResize.type='checkbox';layoutResize.checked=true;layoutResize.id='layout-auto-resize';
const resizeLabel=node('label');resizeLabel.append(layoutResize,document.createTextNode(' If components do not fit, try larger copies (5% steps, maximum +25%; hardware stays actual size)'));layoutView.insertBefore(resizeLabel,layoutGenerate);
async function layoutSizeSearch(components){
  const source=state.project.id;
  const job=await api('projects/'+source+'/size-search',{revision:state.project.revision,components});layoutRun=job.id;layoutCancel.hidden=false;
  for(;;){
    const run=await api('runs/'+job.id),result=run.result||{},attempt=result.attempts?.at(-1);
    if(state.project?.id!==source)return;
    layoutState.textContent='Size search · '+(attempt?Math.round(attempt.height_m*1000)+' mm · ':'')+(statuses[run.status]||run.status);
    if(terminal.has(run.status)){
      await openProject(await api('projects/'+(result.selected_project_id||attempt?.project_id||source)));
      const descriptions={fit:'Structure and layout passed.',fit_structure_unverified:'Components fit; structure checks remain unresolved.',structure_failed:'Structure checks failed; no evidence that enlargement would solve them.',non_packing_failure:'Stopped for a check unrelated to available space.',limit_reached:'No valid layout found within +25%. All trial copies have been saved.',electrical_failed:'Electrical configuration failed.'};
      message(descriptions[result.reason]||run.error||run.status,run.status!=='completed');return;
    }
    await new Promise(resolve=>setTimeout(resolve,500));
  }
}
const layoutResult=node('div'),layoutCanvas=node('canvas');layoutView.insertBefore(layoutResult,layoutEditor);
const layoutAssumptions=node('button','Use example envelope sizes');layoutAssumptions.id='layout-example-sizes';layoutEditor.insertBefore(layoutAssumptions,layoutInputs);
layoutAssumptions.onclick=()=>{for(const f of layoutFields){const dims=f.instance.catalog_id.startsWith('pico2_')?[55,25,12]:f.instance.catalog_id==='mpu6050_lab'?[25,20,10]:['supply_5_candidate','supply_48_candidate'].includes(f.instance.catalog_id)?[25,18,10]:f.instance.catalog_id==='splitter_candidate'?[20,12,8]:null;if(dims){f.sizes.forEach((input,i)=>{input.value=dims[i];input.removeAttribute('aria-invalid');});f.note.value='Example envelope assumption for software testing only; actual component and connectors are unmeasured.';}}layoutUpdateMissing();message('Example sizes filled. These are assumptions, not measured hardware.');};
layoutCanvas.style.width='100%';layoutCanvas.style.height='480px';layoutCanvas.setAttribute('aria-label','Internal assembly with component envelopes');
async function layoutRefresh(){
  const p=state.project;if(!p)return;layoutEpoch++;layoutInputs.replaceChildren();layoutFields=[];layoutResult.replaceChildren();
  if(layoutViewer)layoutViewer.play=false;
  layoutDraft.checked=!!p.layout_config?.allow_unverified_structure;
  layoutGenerate.disabled=!p.articulation||!p.motor_bindings?.length||!!p.active_run_id;
  layoutState.textContent=layoutDraft.checked?'Layout draft: reuse the saved structure; unresolved structure checks remain visible.':p.mechanical_result?.geometry_valid?'Access covers will be updated if these component sizes change.':p.articulation&&p.motor_bindings?.length?'Enter component sizes. Generate layout will first rebuild and check the access covers.':'Identify joints and save motors first.';
  for(const inst of p.instances.filter(i=>!['actuator','wire','fastener','connector'].includes(part(i.catalog_id).category))){
    const saved=p.layout_config?.components.find(c=>c.instance_id===inst.id)||p.service_opening_requirements?.find(c=>c.instance_id===inst.id);
    const example=inst.catalog_id.startsWith('pico2_')?[55,25,12]:inst.catalog_id==='mpu6050_lab'?[25,20,10]:['','',''];
    const card=node('article',undefined,'part-card');card.append(node('h3',instanceName(inst)));
    const sizes=['Length','Width','Height'].map((label,index)=>{const wrapper=node('label',label+' / mm (envelope)'),input=node('input');input.type='number';input.min=2;input.max=300;input.step=.1;input.value=saved?saved.dimensions_m[index]*1000:example[index];input.setAttribute('aria-label',instanceName(inst)+' '+label);input.placeholder='Enter '+label+'（2–300 mm）';input.required=true;input.oninput=()=>{input.removeAttribute('aria-invalid');layoutUpdateMissing();};wrapper.append(input);card.append(wrapper);return input;});
    const ownerLabel=node('label','Mount on body'),owner=node('select');owner.setAttribute('aria-label',inst.id+' mounting body');
    for(const body of (p.articulation?.graph.bodies||p.rigid_bodies)){const opt=node('option',bodyName(body));opt.value=body.id;owner.append(opt);}
    const automatic=node('option','Automatic');automatic.value='auto';owner.prepend(automatic);owner.value=saved?.owner_body_id||'auto';ownerLabel.append(owner);card.append(ownerLabel);
    const noteLabel=node('label','Dimension notes'),note=node('textarea');note.maxLength=1000;note.value=saved?.note||'Assumed envelope. Measure the complete component and connectors before fabrication.';noteLabel.append(note);card.append(disclosure('Dimension notes',noteLabel));
    card.append(node('p','Registered PCB screw mounts where available; other hardware uses a removable strap mount.','muted'));layoutInputs.append(card);layoutFields.push({instance:inst,sizes,owner,note});
  }
  layoutUpdateMissing();
  if(!layoutFields.length){layoutInputs.append(node('p','Select internal components first.'));layoutGenerate.disabled=true;}
  layoutEditor.open=!p.layout_result?.layout_valid;if(p.layout_result){const run=await api('runs/'+p.layout_result.run_id);if(state.project?.id===p.id)await layoutRender(run);}
}
async function layoutRender(run){
  const result=run.result;if(!result)return;if(layoutViewer)layoutViewer.play=false;layoutResult.replaceChildren();
  layoutResult.append(node('h3',result.layout_valid?(result.structure_verified===false?'Layout draft — structure checks unresolved':result.assembly_access_verified===false?'Layout candidate — manual assembly review required':'Layout ready'):'Layout checks failed'));if(!result.pipeline?.includes('registered-pcb'))layoutResult.append(node('p','Previous mount design. Generate layout to update PCB screw mounts.','notice'));if(result.error)layoutResult.append(node('p',errorText(result.error)));
  const base='/api/runs/'+run.id+'/artifacts/';const modelCount=(result.visual_assets||[]).length;layoutResult.append(node('p',modelCount+' reference models · '+((result.component_placements||[]).length-modelCount)+' envelopes','muted'));
  if(result.artifacts?.includes('preview.json')){
    layoutResult.append(layoutCanvas);
    if(!layoutViewer)layoutViewer=new SolidViewer(layoutCanvas);layoutViewer.hiddenShells=true;layoutViewer.hideBodies=true;layoutViewer.load(await (await fetch(base+'preview.json')).json());
    const tools=node('div',undefined,'viewer-tools');
    const lid=node('button','Toggle cover');lid.onclick=()=>{layoutViewer.hiddenShells=!layoutViewer.hiddenShells;layoutViewer.fitVisible();};
    const inside=node('button','Toggle shell');inside.onclick=()=>{layoutViewer.hideBodies=!layoutViewer.hideBodies;layoutViewer.fitVisible();};
    const access=node('button','Show access openings');access.id='layout-open-covers';access.onclick=()=>{const focus=layoutResult.querySelector('select[aria-label="Focus component"]');if(focus)focus.value='';layoutViewer.focusInstance='';layoutViewer.attachmentFocus=null;layoutViewer.hideBodies=false;layoutViewer.hiddenShells=true;layoutViewer.explode=false;layoutViewer.fitVisible();};
    const play=node('button','Play / pause');play.onclick=()=>layoutViewer.toggle();tools.append(lid,inside,access,play,shellOpacityControl(layoutViewer,()=>{const refit=layoutViewer.hideBodies;layoutViewer.hideBodies=false;if(refit)layoutViewer.fitVisible();}));if(result.visual_assets?.length)tools.append(componentFocusControl(layoutViewer,result.visual_assets));layoutResult.append(tools);
  }
  for(const file of ['layout-review.zip','assembly.glb','manifest.json','pcb-mounts.json','cover-fasteners.json'])if(result.artifacts?.includes(file)){const a=node('a',file==='layout-review.zip'?'Layout ZIP':file==='assembly.glb'?'Assembly GLB':file==='pcb-mounts.json'?'PCB mounting details':file==='cover-fasteners.json'?'Cover fastener details':'Check details');a.href=base+file;layoutResult.append(a,node('br'));}
  const placed=disclosure('Components & carrier attachments ('+(result.component_placements||[]).length+')');placed.open=true;layoutResult.append(placed);for(const p of result.component_placements||[]){
    placed.append(node('p',`${instanceName(state.project.instances.find(i=>i.id===p.instance_id))} → ${p.component_mount?'PCB screws':'strap mount'} → carrier → two mounting screws → ${p.lid_mounted?'removable access cover':'body shell'} (${p.support_asset_id||p.owner_body_id})`));
    placed.append(node('p',`Envelope ${(p.envelope_dimensions_m.map(x=>Math.round(x*1000))).join(' × ')} mm · ${result.visual_assets?.some(a=>a.instance_id===p.instance_id)?'Reference CAD':'Envelope'}`,'muted'));
    if(p.support_asset_id&&result.artifacts?.includes('preview.json')){const show=node('button','Show carrier attachment');show.onclick=()=>{layoutViewer.focusInstance=null;layoutViewer.attachmentFocus={id:'',instances:[p.instance_id],assets:[p.support_asset_id],segments:[]};layoutViewer.hideBodies=false;layoutViewer.hiddenShells=false;layoutViewer.fitVisible();layoutCanvas.scrollIntoView({block:'center'});};placed.append(show);}
  }
  const diagnostics=disclosure('Checks & assembly details');diagnostics.open=!result.layout_valid;layoutResult.append(diagnostics);checkSummary(diagnostics,result.checks||[]);if(result.structure_verified===false){diagnostics.append(node('p','Structure checks remain unresolved. This is a spatial layout draft, not a print-ready assembly.','notice'));const inherited=disclosure('Unresolved structure checks');checkSummary(inherited,result.inherited_structure_checks||[]);diagnostics.append(inherited);}const steps=node('ol');for(const step of result.assembly_dependencies||[])steps.append(node('li',step.action));diagnostics.append(disclosure('Original assembly notes',steps));
  const names={cover_screw_exports:'Exported cover screw channels',cover_screw_access:'Cover screw holes, head access and bearing seats',registered_pcb_mounts:'PCB hole alignment and screw channels',all_selected_components_placed:'All components placed',populated_lid_extraction:'Populated cover removal',component_removal_and_tools:'Mount removal & tool access',component_tool_access:'Mount screw tool access',component_straight_extraction:'Straight removal (advisory)',assembly_collisions:'Static collisions',joint_sweep:'Joint motion samples'};
  for(const c of result.checks||[]){const d=node('details');d.append(node('summary',(c.passed?'Passed · ':c.blocking===false?'Advisory · ':'Failed · ')+(names[c.name]||(c.name.startsWith('stl_')?'Solid export: '+(result.geometry_assets.find(a=>'stl_'+a.id===c.name)?.name||c.name):c.name))),node('pre',englishJSON(c)));diagnostics.append(d);}
  for(const v of result.pending||[])diagnostics.append(node('p',v,'muted'));
  if(result.prototype_print_package){const a=node('a','Download prototype print STLs');a.href='/api/runs/'+run.id+'/artifacts/'+result.prototype_print_package.file;layoutResult.append(a);}
  if(result.layout_valid){const next=node('button','Continue to wire routing','primary');next.id='layout-next';next.onclick=safe(async()=>{await harnessRefresh();changeView('harness');});layoutResult.append(next);}
}
async function layoutPoll(id){
  const epoch=++layoutEpoch;layoutRun=id;layoutGenerate.disabled=true;layoutCancel.hidden=false;
  while(epoch===layoutEpoch){const run=await api('runs/'+id);if(epoch!==layoutEpoch)return;
    layoutState.textContent=statuses[run.status]||run.status;
    if(terminal.has(run.status)){
      const p=await api('projects/'+run.project_id);if(epoch!==layoutEpoch)return;
      state.project=p;state.draft=structuredClone(p.instances);renderSelected();$('revision').textContent='Version '+p.revision;
      layoutGenerate.disabled=false;layoutCancel.hidden=true;layoutState.textContent=run.status==='completed'?'Layout saved':statuses[run.status]||run.status;
      if(run.status!=='stale')await layoutRender(run);if(run.error)message(run.error,true);if(typeof simulationRefresh==='function')await simulationRefresh();if(typeof harnessRefresh==='function')await harnessRefresh();await listProjects();return;
    }await new Promise(resolve=>setTimeout(resolve,400));
  }
}
function openingSignature(components){return JSON.stringify(components.map(c=>[c.instance_id,c.owner_body_id,...c.dimensions_m]).sort((a,b)=>a[0].localeCompare(b[0])));}
async function layoutEnsureAccess(components){
  const identity=state.project.id,current=await api('projects/'+identity);state.project=current;
  if(current.active_run_id)throw Error('A task is already running. Wait for it to finish.');
  if((current.mechanical_result?.geometry_valid||layoutDraft.checked&&current.mechanical_result?.run_id)&&components.every(c=>(current.service_opening_requirements||[]).some(old=>openingSignature([old])===openingSignature([c]))))return;
  layoutGenerate.disabled=true;layoutCancel.hidden=false;layoutState.textContent='Updating access covers for the selected components…';
  const job=await api('projects/'+identity+'/assembly',{revision:current.revision,mode:'reference',service_components:components});layoutRun=job.id;
  for(;;){
    const run=await api('runs/'+job.id);
    if(state.project?.id!==identity)throw Error('Project changed. The access-cover task remains saved in its original project.');
    layoutState.textContent='Updating access covers · '+(statuses[run.status]||run.status);
    if(terminal.has(run.status)){
      state.project=await api('projects/'+identity);$('revision').textContent='Version '+state.project.revision+' · Saved';
      if(typeof mechanicalRefresh==='function')await mechanicalRefresh();
      if(!run.result?.geometry_valid)throw Error('Access-cover update did not pass. Open Structure for the saved checks. '+(run.result?.error||run.error||run.status));
      renderResult(state.project.circuit,state.project.validation,'Revalidated wiring');return;
    }
    await new Promise(resolve=>setTimeout(resolve,500));
  }
}
layoutGenerate.onclick=safe(async()=>{if(state.dirty)throw Error('Save component changes first.');
  const missing=layoutMissingFields();if(missing.length){layoutEditor.open=true;const first=missing[0];first.input.setAttribute('aria-invalid','true');first.input.scrollIntoView({block:'center',behavior:'smooth'});first.input.focus({preventScroll:true});layoutUpdateMissing();throw Error(first.label+'：'+first.axis+(first.empty?' is missing':' must be between 2 and 300 mm')+'. Enter dimensions for every selected component.');}
  const components=layoutFields.map(f=>({instance_id:f.instance.id,dimensions_m:f.sizes.map(i=>Number(i.value)/1000),owner_body_id:f.owner.value,evidence:'engineering_assumption',note:f.note.value}));
  layoutGenerate.disabled=true;
  try{if(layoutResize.checked){await layoutSizeSearch(components);return;}
    await layoutEnsureAccess(components);
    const run=await api('projects/'+state.project.id+'/layout',{revision:state.project.revision,mode:'reference',components,allow_unverified_structure:layoutDraft.checked});await layoutPoll(run.id);
  }finally{layoutGenerate.disabled=!state.project?.articulation||!state.project?.motor_bindings?.length||!!state.project?.active_run_id;layoutCancel.hidden=true;}
});
layoutCancel.onclick=safe(async()=>{if(layoutRun)await api('runs/'+layoutRun+'/cancel',{});});
