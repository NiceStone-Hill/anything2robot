let articulationEpoch=0,articulationRun=null;
const art=$('articulation-view');
art.append(node('h2','Model & joints'),node('p','Import an STL or OBJ, set its size and describe the motion.'));
const artForm=node('div',undefined,'mechanical-form');art.append(artForm);
function artField(parent,title,id,type='text',value=''){
  const label=node('label',title),input=node(type==='textarea'?'textarea':type==='select'?'select':'input');input.id=id;
  if(input.tagName==='INPUT')input.type=type;
  if(type!=='select')input.value=value;label.append(input);parent.append(label);return input;
}
function artOptions(select,choices){for(const [value,label] of choices){const opt=node('option',label);opt.value=value;select.append(opt);}}
const sourceBox=node('div');artForm.append(sourceBox);sourceBox.append(node('h3','1. Model'));
const artExample=node('button','Use sample box');sourceBox.append(artExample);
const artFile=artField(sourceBox,'Model file','art-file','file');artFile.accept='.stl,.obj';
artOptions(artField(sourceBox,'File units','art-unit','select'),[['mm','Millimetres'],['cm','Centimetres'],['m','Metres']]);
sourceBox.append(Object.assign(node('p'),{id:'art-source'}));
const resizeBox=disclosure('Copy at a new height'),resizeName=artField(resizeBox,'New project name','resize-name','text'),resizeHeight=artField(resizeBox,'Height / mm','resize-height','number','200');
resizeHeight.min=50;resizeHeight.max=2000;
const resizeCopy=node('button','Copy and recheck joints');resizeCopy.id='copy-at-height';resizeBox.append(resizeCopy,node('p','Keeps the original project. Resizes the source and joint locations; motor mounts are rebuilt at their real size.','muted'));sourceBox.append(resizeBox);
resizeCopy.onclick=safe(async()=>{
  if(!state.project?.articulation)throw Error('Identify joints before copying at a new height.');
  if(state.dirty)throw Error('Save component changes first.');
  resizeCopy.disabled=true;
  try{const run=await api('projects/'+state.project.id+'/clone-at-height',{revision:state.project.revision,name:resizeName.value||state.project.name+' · '+resizeHeight.value+' mm',height_m:Number(resizeHeight.value)/1000});await openProject(await api('projects/'+run.project_id));await articulationPoll(run.id);}finally{resizeCopy.disabled=false;}
});
const intentBox=node('div');artForm.append(intentBox);intentBox.append(node('h3','2. Motion'));
const artLength=artField(intentBox,'Maximum size / mm','art-length','number','240');artLength.min='50';artLength.max='2000';
artOptions(artField(intentBox,'Up axis','art-up','select'),['+Z','+Y','+X','-Z','-Y','-X'].map(v=>[v,v]));
artOptions(artField(intentBox,'Support','art-support','select'),[['fixed_base','Fixed base'],['tabletop','Tabletop (stability unchecked)']]);
const artIntent=artField(intentBox,'Motion description (optional)','art-intent','textarea');artIntent.maxLength=4000;
const artMax=artField(intentBox,'Maximum joints','art-max','number','3');artMax.min=1;artMax.max=6;
const actionBox=node('div');artForm.append(actionBox);actionBox.append(node('h3','3. Identify joints'));
artOptions(artField(actionBox,'Method','art-mode','select'),[['live','Online VLM'],['reference','Local geometric partition']]);
const artGenerate=node('button','Identify joints','primary'),artCancel=node('button','Cancel');artCancel.hidden=true;actionBox.append(artGenerate,artCancel);
const artState=node('p');artState.setAttribute('role','status');actionBox.append(artState);
actionBox.append(node('p','Use a closed mesh.','muted'));
const artResults=node('div');art.append(artResults);
async function articulationRefresh(){
  const p=state.project;if(!p)return;articulationEpoch++;
  $('art-source').textContent=p.source_shapes?.[0]?.name||'No model imported.';
  $('art-mode').options[0].disabled=!state.live;$('art-mode').value=p.articulation_config?.mode||(state.live?'live':'reference');
  const c=p.articulation_config;
  if(c){artLength.value=c.length_m*1000;$('art-up').value=c.up_axis;$('art-support').value=c.support;artIntent.value=c.intent;artMax.value=c.max_joints;}
  artGenerate.disabled=false;artCancel.hidden=true;artResults.replaceChildren();artState.textContent='';
  const last=p.events?.filter(e=>e.type==='articulation_checked').at(-1);
  const runId=last?.run_id||p.articulation?.run_id;
  if(runId){const id=p.id,run=await api('runs/'+runId);if(state.project?.id===id)articulationRender(run);}
}
function articulationRender(run){
  artResults.replaceChildren();const r=run.result;if(!r)return;
  artResults.append(node('h3',r.annotation_valid?'Joints ready':'Joint checks failed'));
  if(!r.annotation_valid){
    const labels={tree_and_internal_points:'Joint locations',multiview_reprojection:'View consistency',all_occupied_voxels_assigned:'Partition coverage',partition_solids:'Closed partition meshes'};
    const jointOK=r.joint_annotation_valid ?? (r.checks?.find(c=>c.name==='tree_and_internal_points')?.passed && r.checks?.find(c=>c.name==='multiview_reprojection')?.passed);
    artResults.append(node('p',jointOK?'Joint locations passed; partition generation failed. See details.':(errorText(r.error)||'Generation failed. See checks below.')));
    for(const c of r.checks||[])artResults.append(node('p',(c.passed?'Passed: ':'Failed: ')+(labels[c.name]||c.name)+(c.reason?' · '+errorText(c.reason):'')));
    for(const p of r.partitions||[])if(!p.watertight||!p.positive_volume)artResults.append(node('p',p.name+': mesh is not a closed solid.'));
    if(r.error&&jointOK)artResults.append(disclosure('Original error',node('pre',r.error)));
    if(jointOK){const retry=node('button','Retry partition','primary');artResults.append(retry);retry.onclick=safe(async()=>{retry.disabled=true;try{const next=await api('projects/'+state.project.id+'/articulation-retry',{revision:state.project.revision,run_id:run.id});await articulationPoll(next.id);}finally{retry.disabled=false;}});}
    return;
  }
  artResults.append(node('p',(r.source.kind==='reprocessed_annotation'?'Reused joint annotation':r.source.live_model_called?'Online model '+r.source.model:'Local partition') ));
  const views=node('div',undefined,'annotation-views');artResults.append(disclosure('Annotated views',views));
  for(const file of ['top.png','front.png','side.png']){const img=node('img');img.src='/api/runs/'+run.id+'/artifacts/annotated-'+file;img.alt='Joint annotation '+file;views.append(img);}
  artResults.append(node('p',`${r.graph.joints.length} joint(s) · ${r.analysis.source_components} source solid(s)`));
  if(r.analysis.bounds_mm){const size=r.analysis.bounds_mm[1].map((v,i)=>v-r.analysis.bounds_mm[0][i]);artResults.append(node('p','Model width × depth × height: '+size.map(v=>v.toFixed(1)).join(' × ')+' mm','muted'));}
  const holder=node('div',undefined,'catalog'),choices=[];artResults.append(node('h3','4. Motors'),holder);
  for(const joint of r.graph.joints){
    const card=node('article',undefined,'part-card');card.append(node('h3',jointName(joint)),node('p',`${joint.parent} → ${joint.child} · ${joint.limits_deg.join(' to ')}°`));
    const select=node('select');select.setAttribute('aria-label',jointName(joint)+' motor');
    for(const c of r.motor_candidates.find(c=>c.joint_id===joint.id).choices){const o=node('option',(catalogName(c.catalog_id))+(!state.moduleTemplates?.catalog_ids.includes(c.catalog_id)?' (no mount template)':c.template_revision!==state.moduleTemplates?.revision?' (template changed; recheck fit)':c.preliminary_space_fit===false?' (insufficient space)':c.preliminary_space_fit?(c.mounting_style==='open_cradle'?' (open cradle · preliminary fit)':' (preliminary fit)'):' (fit not checked)'));o.value=c.catalog_id;select.append(o);}
    const bound=state.project.motor_bindings?.find(b=>b.joint_id===joint.id);if(bound)select.value=bound.catalog_id;
    const basis=node('p',undefined,'muted');
    const explain=()=>{const c=r.motor_candidates.find(c=>c.joint_id===joint.id).choices.find(c=>c.catalog_id===select.value);basis.textContent=(c?.module_dimensions_mm?'Module '+c.module_dimensions_mm.map(v=>v.toFixed(1)).join(' × ')+' mm. ':'')+'Spatial fit only; load capacity is unverified.';};select.onchange=explain;explain();
    card.append(select,basis);holder.append(card);choices.push({joint_id:joint.id,select});
  }
  const save=node('button','Save motors','primary');artResults.append(save);
  save.onclick=safe(async()=>{if(state.dirty)throw Error('Save component changes first.');const p=await api('projects/'+state.project.id+'/joint-motors',{revision:state.project.revision,choices:choices.map(c=>({joint_id:c.joint_id,catalog_id:c.select.value}))});await openProject(p);message('Motors saved. Generate structure to continue.');});
  const build=node('button','Generate structure');artResults.append(build);build.onclick=safe(()=>startGraphAssembly(build));
  const details=node('details');details.append(node('summary','Partition files & diagnostics'));
  for(const part of r.partitions){const a=node('a',part.id+' · Partition STL');a.href='/api/runs/'+run.id+'/artifacts/'+part.file;details.append(a,node('p',`${part.connected_regions} regions · ${part.voxel_count} voxels`));}
  details.append(node('pre',englishJSON({checks:r.checks,analysis:r.analysis,uncertainties:r.graph.uncertainties})));artResults.append(details);
}
async function articulationPoll(id){
  articulationRun=id;const epoch=++articulationEpoch;artGenerate.disabled=true;artCancel.hidden=false;
  while(epoch===articulationEpoch){
    const run=await api('runs/'+id);if(epoch!==articulationEpoch)return;
    artState.textContent=statuses[run.status]||run.status;
    if(terminal.has(run.status)){
      const p=await api('projects/'+run.project_id);if(epoch!==articulationEpoch)return;
      state.project=p;artGenerate.disabled=false;artCancel.hidden=true;$('revision').textContent='Version '+p.revision;
      if(run.status!=='stale')articulationRender(run);artState.textContent=run.status==='completed'?'Joint checks complete':statuses[run.status];
      if(run.error)message(run.error,true);await listProjects();return;
    }await new Promise(r=>setTimeout(r,400));
  }
}
artExample.onclick=safe(async()=>{if(!state.project)return;if(state.dirty)throw Error('Save component changes first.');await openProject(await api('projects/'+state.project.id+'/appearance',{revision:state.project.revision,builtin:'box',dimensions_mm:[240,90,80]}));});
artFile.onchange=safe(async()=>{const f=artFile.files[0];if(!f||!state.project)return;if(state.dirty)throw Error('Save component changes first.');if(f.size>40000000)throw Error('File limit: 40 MB.');const bytes=new Uint8Array(await f.arrayBuffer());let str='';for(let i=0;i<bytes.length;i+=8192)str+=String.fromCharCode(...bytes.subarray(i,i+8192));await openProject(await api('projects/'+state.project.id+'/appearance',{revision:state.project.revision,filename:f.name,unit:$('art-unit').value,data_base64:btoa(str)}));});
artGenerate.onclick=safe(async()=>{if(!state.project)return;if(state.dirty)throw Error('Save component changes first.');artGenerate.disabled=true;try{const run=await api('projects/'+state.project.id+'/articulation',{revision:state.project.revision,mode:$('art-mode').value,intent:artIntent.value,length_m:Number(artLength.value)/1000,up_axis:$('art-up').value,support:$('art-support').value,max_joints:Number(artMax.value)});await articulationPoll(run.id);}finally{artGenerate.disabled=false;}});
artCancel.onclick=safe(async()=>{if(articulationRun)await api('runs/'+articulationRun+'/cancel',{});});
