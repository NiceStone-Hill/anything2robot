let harnessViewer=null,harnessEpoch=0;
function harnessPerformance(){
  if(!harnessViewer)return '{}';
  const visible=harnessViewer.parts.filter(p=>viewerPartVisible(harnessViewer,p));
  return JSON.stringify({...harnessViewer.stats(),wire_count:state.project?.harnesses?.length,
    colors:[...new Set((state.project?.harnesses||[]).map(w=>w.color))],
    visible_wire_ids:[...new Set(visible.filter(p=>p.wire_id).map(p=>p.wire_id))],
    visible_corridors:visible.filter(p=>p.role==='bundle_corridor').length,
    visible_carriers:visible.filter(p=>p.id.startsWith('electronics_')&&p.id.endsWith('_carrier')).length,
    visible_guides:visible.filter(p=>p.role==='wire_guide').length});
}
const harnessView=$('harness-view');
harnessView.append(node('h2','Wire routing'),node('p','Choose a wire to see where each end connects. Colours identify wires; follow the port labels for polarity.'));
const harnessButton=node('button','Route wires','primary'),harnessStatus=node('p'),harnessResult=node('div');
const harnessSizeLabel=node('label','Insulated wire diameter / mm '),harnessSize=node('input');
harnessSize.type='number';harnessSize.min=.5;harnessSize.max=3;harnessSize.step=.1;harnessSize.value=1.2;harnessSize.id='harness-wire-diameter';harnessSizeLabel.append(harnessSize);
harnessView.append(harnessSizeLabel,harnessButton,harnessStatus,harnessResult);
const harnessScene=node('div',undefined,'wire-scene'),harnessCanvas=node('canvas'),harnessLabels=node('div',undefined,'wire-labels');
harnessCanvas.id='harness-canvas';harnessCanvas.style.width='100%';harnessCanvas.style.height='480px';harnessCanvas.setAttribute('aria-label','Individual wires and labelled endpoints');harnessScene.append(harnessCanvas,harnessLabels);
function harnessEndpointText(ep){
  const inst=state.project.instances.find(i=>i.id===ep.instance_id);
  return instanceName(inst)+' · '+(ep.label||ep.port_id||'Unregistered contact')+(ep.physical_pin?' · pin '+ep.physical_pin:'');
}
function harnessEndpointEvidence(ep){
  if(ep.evidence==='manufacturer_reference_pad')return 'Reference pad · connector unmeasured';
  if(ep.evidence==='reference_pad_assumed_label_order')return 'Reference pad · label order needs confirmation';
  return 'Position unregistered · placeholder';
}
function harnessWireName(route){return route.id.replace('wire_','Wire ');}
function harnessSignalName(route){return ({ground:'Common ground (GND)',power:'Power supply',i2c_sda:'Data (SDA)',i2c_scl:'Clock (SCL)',pwm:'Motor control (PWM)'})[route.kind]||route.kind;}
async function harnessRefresh(){
  const p=state.project;if(!p)return;const epoch=++harnessEpoch;
  harnessButton.disabled=!p.layout_result?.layout_valid||!p.circuit||!!p.active_run_id;
  harnessSize.value=p.harness_config?.wire_diameter_mm||1.2;
  harnessStatus.textContent=harnessButton.disabled?'Complete wiring and internal layout first.':'Diameter is a planning assumption until actual wire is selected.';
  harnessResult.replaceChildren();
  if(p.harness_result){const run=await api('runs/'+p.harness_result.run_id);if(epoch===harnessEpoch&&state.project?.id===p.id)await harnessRender(run,epoch);}
}
async function harnessRender(run,epoch=harnessEpoch){
  const r=run.result;if(!r)return;
  const base='/api/runs/'+run.id+'/artifacts/';
  const preview=r.artifacts?.includes('preview.json')?await(await fetch(base+'preview.json')).json():null;
  if(epoch!==harnessEpoch||run.project_id!==state.project?.id)return;
  harnessResult.replaceChildren();harnessLabels.replaceChildren();
  const modern=!!r.endpoint_registration;
  harnessResult.append(node('h3',r.routing_valid?(r.structure_verified===false?'Routing draft — structure checks unresolved':modern?'Corridors checked · connections need confirmation':'Previous corridor result'):'Routing failed'));
  harnessResult.append(node('p',errorText(r.error)||(r.wire_count+' wires · '+r.shared_segments+' shared segments')));
  if(r.fanout_checks){const passed=r.fanout_checks.filter(c=>c.passed).length;harnessResult.append(node('p',passed+' / '+r.fanout_checks.length+' terminal leads clear · '+(r.fanout_checks.length-passed)+' unresolved. Connector positions and junction transitions still need confirmation.','notice'));}
  if(!modern)harnessResult.append(node('p','This previous result shows bundle space and assumed exits. Route wires to generate individual colours and endpoint labels.','notice'));
  if(preview){
    harnessResult.append(harnessScene);
    if(!harnessViewer)harnessViewer=new SolidViewer(harnessCanvas);
    harnessViewer.hiddenShells=true;harnessViewer.hideBodies=true;harnessViewer.hideCarriers=true;harnessViewer.showCorridors=!modern;harnessViewer.wireFocus=null;harnessViewer.onDraw=null;harnessViewer.load(preview);
    const tools=node('div',undefined,'viewer-tools'),modeLabel=node('label','Show '),mode=node('select');mode.id='harness-display-mode';
    for(const [value,text] of [['wires','Individual wires'],['corridors','Reserved bundle space']]){const option=node('option',text);option.value=value;mode.append(option);}mode.value=modern?'wires':'corridors';mode.options[0].disabled=!modern;modeLabel.append(mode);
    mode.onchange=()=>{harnessViewer.showCorridors=mode.value==='corridors';harnessViewer.fitVisible();};
    const inside=node('button','Toggle shell');inside.onclick=()=>{harnessViewer.hideBodies=!harnessViewer.hideBodies;harnessViewer.fitVisible();};
    const mounts=node('button','Show carriers');mounts.onclick=()=>{harnessViewer.hideCarriers=!harnessViewer.hideCarriers;mounts.textContent=harnessViewer.hideCarriers?'Show carriers':'Hide carriers';harnessViewer.fitVisible();};tools.append(modeLabel,inside,mounts,shellOpacityControl(harnessViewer,()=>{const refit=harnessViewer.hideBodies;harnessViewer.hideBodies=false;if(refit)harnessViewer.fitVisible();}));harnessResult.append(tools);
    harnessResult.append(node('p',modern?`Wires: ${r.visualization.wire_diameter_mm} mm outside diameter, drawn to model scale. Thick bundle space is hidden. Dashed leads are unresolved paths.`:'Thick geometry reserves a complete bundle, not one wire.','muted'));
  }
  if(modern){
    const guide=node('div',undefined,'wire-guide'),heading=node('h3','Connect one wire at a time'),select=node('select');select.id='harness-wire-select';select.setAttribute('aria-label','Select wire');
    const all=node('option','All wires');all.value='';select.append(all);
    for(const route of r.routes){const opt=node('option',harnessWireName(route)+' · '+harnessSignalName(route)+' · '+harnessEndpointText(route.endpoints.from)+' → '+harnessEndpointText(route.endpoints.to));opt.value=route.id;select.append(opt);}
    const controls=node('div',undefined,'viewer-tools'),previous=node('button','Previous wire'),next=node('button','Next wire'),allButton=node('button','Show all wires');controls.append(previous,next,allButton);
    const detail=node('div');detail.id='harness-connection-detail';guide.append(heading,select,controls,detail);harnessResult.append(guide);
    const showWire=()=>{
      const route=r.routes.find(w=>w.id===select.value);detail.replaceChildren();harnessLabels.replaceChildren();
      if(!route){harnessViewer.wireFocus=null;harnessViewer.onDraw=null;detail.append(node('p','Choose a wire. The view will isolate it and its two components.'));harnessViewer.fitVisible();return;}
      const swatch=node('span',undefined,'wire-swatch');swatch.style.backgroundColor=route.color;
      const title=node('h3',harnessWireName(route)+' · '+harnessSignalName(route));title.prepend(swatch);detail.append(title);
      const labels=[];
      for(const [side,letter] of [['from','A'],['to','B']]){
        const ep=route.endpoints[side];detail.append(node('p',letter+' → '+harnessEndpointText(ep)),node('p',harnessEndpointEvidence(ep)+(ep.fanout_clearance_checked?' · lead clearance checked':' · lead path unresolved'),'muted'));
        if(ep.fanout_failure?.blocking_assets?.length){const names=ep.fanout_failure.blocking_assets.map(id=>{const carrier=(r.carrier_attachments||[]).find(p=>p.carrier_asset_id===id);return carrier?instanceName(state.project.instances.find(i=>i.id===carrier.instance_id))+' carrier':id;});detail.append(node('p','Outward lead blocked by: '+names.join(', '),'notice'));}
        const label=node('span',letter+' · '+(ep.label||ep.port_id||'Contact')+(ep.physical_pin?' (#'+ep.physical_pin+')':''),'wire-endpoint-label');label.style.borderColor=route.color;harnessLabels.append(label);labels.push({ep,label});
      }
      detail.append(node('p','Shared corridor: '+(route.centerline_length_m*1000).toFixed(1)+' mm. Terminal leads and installation slack still need confirmation; do not cut to this length.','muted'));
      if(route.kind==='ground')detail.append(node('p','This wire joins the common ground distributor. Each labelled distributor contact must belong to the same GND group.'));
      const ids=[route.from.instance_id,route.to.instance_id],assets=[];
      for(const id of ids){const binding=state.project.motor_bindings.find(b=>b.motor_instance_id===id);if(binding)for(const p of harnessViewer.parts)if(p.id.startsWith(binding.joint_id+'_'))assets.push(p.id);}
      harnessViewer.wireFocus={id:route.id,instances:ids,assets,segments:route.segments.map(s=>s.segment_id)};
      harnessViewer.focusInstance=null;
      harnessViewer.onDraw=()=>{for(const {ep,label} of labels){const [x,y]=viewerProjectPoint(harnessViewer,ep.position_m);label.style.left=Math.max(0,Math.min(harnessCanvas.clientWidth-90,x))+'px';label.style.top=Math.max(0,Math.min(harnessCanvas.clientHeight-30,y-28))+'px';label.hidden=!!harnessViewer.showCorridors;}};
      harnessViewer.fitVisible();
    };
    select.onchange=showWire;
    const advance=step=>{const index=r.routes.findIndex(w=>w.id===select.value);select.value=r.routes[(index+step+r.routes.length)%r.routes.length].id;showWire();};
    next.onclick=()=>advance(1);previous.onclick=()=>advance(-1);allButton.onclick=()=>{select.value='';showWire();};showWire();
    harnessResult.append(node('p','Assembly order: verify labels with power off → connect the common GND group → connect signal wires → confirm the supply voltage and connect power. Actual connectors and current ratings remain to be selected.','notice'));
    const mounts=disclosure('What are the carriers attached to?');
    for(const p of r.carrier_attachments||[]){const name=instanceName(state.project.instances.find(i=>i.id===p.instance_id));mounts.append(node('p',name+' → carrier → two mounting screws → '+(p.lid_mounted?'removable access cover':'body shell')+' ('+p.support_asset_id+')'));}harnessResult.append(mounts);
  }
  for(const [file,label] of [['assembly-with-wires.glb','Assembly guide GLB'],['harness-review.zip','Routing ZIP'],['wire-list.csv','Wire list CSV']])if(r.artifacts?.includes(file)){const a=node('a',label);a.href=base+file;harnessResult.append(a,node('br'));}
  const detail=disclosure('Routing strategy & limitations');harnessResult.append(detail);
  detail.append(node('p','1. Convert the circuit nets into separate wires; multi-endpoint nets branch at the placed distributor.'),node('p','2. Build a grid inside the fixed body, excluding shells, motor mounts and electronics. Reserve the full packed bundle diameter and clearance.'),node('p','3. Grow a shared tree that favours open space, then shorten branches with solid collision checks. Check sampled moving-part poses.'),node('p','4. Show one colour per wire. Reference pads follow the CAD placement; unknown contacts remain placeholders. Terminal leads are checked separately. Junction transitions and real connector access are unverified.'));
  for(const text of r.pending||[])detail.append(node('p',text,'muted'));
  if(r.routing_valid){const next=node('button','Continue to export','primary');next.id='harness-next';next.onclick=safe(async()=>{await simulationRefresh();changeView('simulation');});harnessResult.append(next);}
}
async function harnessPoll(id){
  harnessButton.disabled=true;const epoch=++harnessEpoch;
  for(;;){const r=await api('runs/'+id);if(epoch!==harnessEpoch)return;harnessStatus.textContent=statuses[r.status]||r.status;
    if(terminal.has(r.status)){state.project=await api('projects/'+r.project_id);harnessButton.disabled=false;await harnessRender(r,epoch);await listProjects();return;}await new Promise(resolve=>setTimeout(resolve,700));}
}
harnessButton.onclick=safe(async()=>{const size=harnessSize.valueAsNumber;if(!Number.isFinite(size)||size<.5||size>3)throw Error('Enter a wire diameter between 0.5 and 3 mm.');const r=await api('projects/'+state.project.id+'/harness',{revision:state.project.revision,mode:'reference',wire_diameter_mm:size,grid_mm:4,routing_mode:'bundled'});await harnessPoll(r.id);});
