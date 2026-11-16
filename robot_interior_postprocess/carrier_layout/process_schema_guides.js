// X-ray schema guides only: no synthetic component placement or CAD modification.
let schemaComponent=null,schemaGuidesEnabled=true,schemaPointsEnabled=true,schemaTableRows=[],schemaGuideStageActive=false;
function schemaGuideState(){
 const event=data.events[index];if(event?.stage!=='layout_constraints')return null;
 const policies=event.metrics.schema?.components||[],policy=policies.find(p=>p.instance_id===schemaComponent)||policies[0];
 if(!policy)return null;const domain=data.schema_domains?.owner_cavity_bounds_mm?.[policy.owner_link];if(!domain)return null;
 const bounds=[0,1].map(side=>domain[0].map((lo,k)=>lo+policy.allowed_center_normalized[k][side]*(domain[1][k]-lo)));
 return{policy,domain,bounds,gravity:data.schema_domains?.gravity?.world_m_s2,samples:data.schema_cavity_samples?.[policy.owner_link]?.points_mm||[]};
}
function selectSchemaComponent(id){schemaComponent=id;const state=schemaGuideState();if(!state)return;
 byId('owner').value=state.policy.owner_link;yaw=.65;pitch=.85;zoom=1;renderSchemaGuides(data.events[index]);draw();
}
function renderSchemaGuides(event){const box=byId('schemaGuides');box.replaceChildren();box.hidden=event.stage!=='layout_constraints';byId('owner').disabled=!box.hidden;if(box.hidden){schemaGuideStageActive=false;return}if(!schemaGuideStageActive){yaw=.65;pitch=.85;zoom=1}schemaGuideStageActive=true;
 const policies=event.metrics.schema?.components||[];if(!policies.some(p=>p.instance_id===schemaComponent))schemaComponent=policies[0]?.instance_id;
 const state=schemaGuideState();if(!state){paragraph(box,'Spatial schema guides unavailable: no recorded owner domain.');return}
 byId('owner').value=state.policy.owner_link;
 const select=document.createElement('select');select.id='schemaComponent';for(const p of policies){const option=document.createElement('option');option.value=p.instance_id;option.textContent=p.instance_id+' / '+p.owner_link;select.append(option)}select.value=schemaComponent;select.onchange=()=>selectSchemaComponent(select.value);box.append(select);
 for(const [id,title,checked,callback]of [['schemaOverlay','Schema guides',schemaGuidesEnabled,value=>schemaGuidesEnabled=value],['schemaPoints','Usable-space samples',schemaPointsEnabled,value=>schemaPointsEnabled=value]]){
  const label=document.createElement('label'),input=document.createElement('input');input.id=id;input.type='checkbox';input.checked=checked;input.onchange=()=>{callback(input.checked);draw()};label.append(input,document.createTextNode(title));box.append(label);
 }
 box.append(textElement('p','Purple box: hard bundle-center bounds · Green arrows: allowed mounting normals · Gold arrow / gradient: soft gravity preference.'));
 box.append(textElement('small','X-ray guides in world coordinates. Dots sample usable space; neither the box nor the dots certify that a whole carrier fits. No component pose has been selected yet.'));
 for(const [id,row]of schemaTableRows)row.classList.toggle('activePolicy',id===schemaComponent);
}
function drawSchemaGuides(event,context,center,scale,width,height,ratio){
 const state=schemaGuideState();if(!state||!schemaGuidesEnabled)return;
 const {policy,bounds,domain,gravity,samples}=state,xyz=p=>{const q=project(p,center);return[width/2+q[0]*scale,height/2-q[1]*scale]},line=(a,b,color,thickness=2)=>{context.strokeStyle=color;context.lineWidth=thickness*ratio;context.beginPath();context.moveTo(...xyz(a));context.lineTo(...xyz(b));context.stroke()};
 const tag=(text,p,color)=>{const [x,y]=xyz(p);context.font=11*ratio+'px system-ui';context.fillStyle='#ffffffed';context.fillRect(x-3*ratio,y-13*ratio,context.measureText(text).width+6*ratio,17*ratio);context.fillStyle=color;context.fillText(text,x,y)};
 const arrow=(a,d,length,color,label)=>{const b=a.map((v,k)=>v+d[k]*length),[x,y]=xyz(a),[xx,yy]=xyz(b),angle=Math.atan2(yy-y,xx-x),head=8*ratio;line(a,b,color,2.5);context.beginPath();context.moveTo(xx,yy);context.lineTo(xx-head*Math.cos(angle-.4),yy-head*Math.sin(angle-.4));context.moveTo(xx,yy);context.lineTo(xx-head*Math.cos(angle+.4),yy-head*Math.sin(angle+.4));context.stroke();tag(label,b,color)};
 const corners=Array.from({length:8},(_,mask)=>bounds[0].map((v,k)=>bounds[(mask>>k)&1][k]));
 let up=null,heights=[];if(gravity&&Math.hypot(...gravity)>0){up=gravity.map(v=>-v/Math.hypot(...gravity));heights=Array.from({length:8},(_,mask)=>domain[0].map((v,k)=>domain[(mask>>k)&1][k])).map(p=>p.reduce((s,v,k)=>s+v*up[k],0))}
 if(schemaPointsEnabled){for(const p of samples){if(p.some((v,k)=>v<bounds[0][k]-1e-6||v>bounds[1][k]+1e-6))continue;let preference=0;
  if(up&&policy.preference_weight>0&&policy.vertical_preference!=='neutral'){const h=(p.reduce((s,v,k)=>s+v*up[k],0)-Math.min(...heights))/Math.max(1,Math.max(...heights)-Math.min(...heights));preference=policy.vertical_preference==='low'?1-h:h}
  context.fillStyle=preference>0?'rgb('+Math.round(35+195*preference)+','+Math.round(145+20*preference)+','+Math.round(180-145*preference)+')':'#289bb8';context.globalAlpha=.6;const [x,y]=xyz(p);context.fillRect(x,y,2*ratio,2*ratio)
 }context.globalAlpha=1}
 for(let i=0;i<8;i++)for(let k=0;k<3;k++)if(!(i&(1<<k)))line(corners[i],corners[i|(1<<k)],'#a24fc4',2);
 const midpoint=bounds[0].map((lo,k)=>(lo+bounds[1][k])/2),length=Math.max(20,Math.min(65,Math.hypot(...domain[1].map((v,k)=>v-domain[0][k]))*.15));
 for(const n of policy.allowed_opening_normals)arrow(midpoint,n,length,'#148665','n '+vectorLabel(n));
 tag(policy.instance_id+' · allowed CENTER box',corners[4],'#853ca5');
 if(up&&policy.preference_weight>0&&policy.vertical_preference!=='neutral'){
  const start=midpoint.map((v,k)=>v+(domain[1][k]-domain[0][k])*.24),direction=up.map(v=>v*(policy.vertical_preference==='low'?-1:1));
  arrow(start,direction,length*1.2,'#b47b13',policy.vertical_preference.toUpperCase()+' preferred · w='+policy.preference_weight+' (soft)');
 }
 context.font=11*ratio+'px system-ui';const text='Schema X-ray · '+policy.instance_id+' · rolls '+policy.allowed_rolls_deg.join('/')+'° · clearance ≥ '+policy.min_clearance_mm+' mm';context.fillStyle='#ffffffed';context.fillRect(8*ratio,8*ratio,Math.min(width-16*ratio,context.measureText(text).width+12*ratio),24*ratio);context.fillStyle='#6a358e';context.fillText(text,14*ratio,24*ratio);
}
function renderScreening(event){const box=byId('screeningPanel');box.replaceChildren();box.hidden=event.stage!=='candidate_rejected'||!event.metrics.opening_screening;if(box.hidden)return;
 const rows=event.metrics.opening_screening,fragmented=rows.every(r=>r.reason.includes('disconnected shell fragments'));
 box.append(textElement('b',fragmented?'This step checks the shell cut — electronics placement and EPS have not started.':'Reject candidates that fail local geometry checks.'));
 paragraph(box,fragmented?'Goal: two connected shell sides. Reject cuts that leave extra detached material. Red in the preview marks detached regions.':'Candidate failures are condensed here; the recorded reasons remain visible.');box.append(evidenceTable(['Candidate','Cut plane','Expected regions','Diagnostic regions','Result'],rows.map(r=>{
  const normal=r.plane?.normal;
  return[(r.link||'Link')+' / C'+(r.candidate??'?'),normal?'n '+vectorLabel(normal)+'; offset '+r.plane.offset_mm.toFixed(2)+' mm':'See recorded reason',r.expected_regions??'—',r.actual_regions??'—','REJECTED · '+r.reason];
 })).table);
 paragraph(box,'These are diagnostic clips of rejected cuts, not printable parts. The next step inspects a surviving opening before carrier placement.');
}
