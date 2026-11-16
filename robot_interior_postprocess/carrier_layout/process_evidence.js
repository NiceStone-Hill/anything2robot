// Presentation-only evidence. Plane previews never enter manufacturing geometry.
let openingOverride=null,openingLink=null;
function textElement(tag,text,className){const node=document.createElement(tag);node.textContent=text;if(className)node.className=className;return node}
function evidenceTable(headers,rows){const table=document.createElement('table'),head=document.createElement('thead'),tr=document.createElement('tr');for(const h of headers)tr.append(textElement('th',h));head.append(tr);table.append(head);const body=document.createElement('tbody');for(const cells of rows){const row=document.createElement('tr');for(const cell of cells)row.append(textElement('td',cell));body.append(row)}table.append(body);return{table,body}}
function vectorLabel(n){return '['+n.map(v=>Number(v.toFixed(3))).join(', ')+']'}
function vectorMatches(a,b){return a.length===b.length&&a.every((v,k)=>Math.abs(v-b[k])<1e-6)}
function openingRows(event){
 const rows=[],ranked=Object.fromEntries((event.metrics.ranking_explanations||[]).map(r=>[r.part_id,r.ranked_candidate_ids]));
 const input=data.opening_catalog||{},links=new Set([...Object.keys(input),...Object.keys(event.metrics.openings||{})]);
 const sourceEvent=trace.events.find(e=>e.stage==='hollow_geometry')||event;
 for(const link of links){const source=sourceEvent.instances.find(i=>i.owner===link&&i.role==='shell');if(!source)continue;
  const asset=data.assets[source.asset],bounds=asset.source_bounds_mm,choices=event.metrics.openings?.[link]||[];
  const supplied=(input[link]||[]).map(c=>({...c}));
  for(const [i,c]of choices.entries())if(!supplied.some(r=>vectorMatches(r.normal,c.normal)&&Math.abs(r.fraction-c.fraction)<1e-6)){
   const projections=[0,1].map(sign=>c.normal.reduce((s,n,k)=>s+n*bounds[n>=0?sign:1-sign][k],0));
   supplied.push({...c,id:'configured:'+link+':'+(i+1),offset_mm:projections[0]+c.fraction*(projections[1]-projections[0])});
  }
  for(const c of supplied){const modelIndex=(ranked[link]||[]).indexOf(c.id),solverIndex=choices.findIndex(r=>vectorMatches(r.normal,c.normal)&&Math.abs(r.fraction-c.fraction)<1e-6);
   const tried=Object.values(catalog).find(r=>r.link===link&&(r.source_candidate_id===c.id||(vectorMatches(r.plane.normal,c.normal)&&Math.abs(r.fraction-c.fraction)<1e-6)));
   const axis=c.normal.map(Math.abs).indexOf(Math.max(...c.normal.map(Math.abs))),cardinal=Math.abs(Math.abs(c.normal[axis])-1)<1e-6;
   const result=tried?.selected?'SELECTED':tried?.status==='rejected'?'REJECTED':tried?'EVALUATED':!cardinal?'UNSUPPORTED BY CARRIER EPS':solverIndex<0?'NOT QUEUED':'NOT EVALUATED';
   rows.push({...c,link,source,source_bounds_mm:bounds,reference_asset:source.asset,axis,is_cardinal:cardinal,model_rank:modelIndex>=0?modelIndex+1:null,solver_order:solverIndex>=0?solverIndex+1:null,result,reason:tried?.reason||'',attempt:tried});
  }
 }
 return rows.sort((a,b)=>a.link.localeCompare(b.link)||(a.model_rank??1000)-(b.model_rank??1000)||(a.solver_order??1000)-(b.solver_order??1000)||a.id.localeCompare(b.id));
}
function openingPosition(row){return row.is_cardinal?'XYZ'[row.axis]+' = '+(row.offset_mm/row.normal[row.axis]).toFixed(2)+' mm':'n·x = '+row.offset_mm.toFixed(2)+' mm'}
function openingPreviewInfo(row){const lateral=[0,1,2].filter(k=>k!==row.axis),b=row.source_bounds_mm;const horizontal=lateral.sort((a,c)=>(b[1][c]-b[0][c])-(b[1][a]-b[0][a]))[0],view=lateral.find(k=>k!==horizontal);return{key:'opening-preview:'+row.id,link:row.link,rank:row.model_rank??row.solver_order??'unranked',source_candidate_id:row.id,plane:{id:row.id,normal:row.normal,offset_mm:row.offset_mm},axis:'XYZ'[row.axis],coordinate_mm:row.offset_mm/row.normal[row.axis],source_bounds_mm:b,side_view_axis:view,profile_axes:[horizontal,row.axis],reference_asset:row.reference_asset,is_cardinal:row.is_cardinal}}
function drawOpeningCard(canvas,row){const c=canvas.getContext('2d'),info=openingPreviewInfo(row);canvas.width=396;canvas.height=230;const asset=data.assets[row.reference_asset],b=row.source_bounds_mm,[u,v]=info.profile_axes,w=info.side_view_axis;const scale=Math.min(350/Math.max(b[1][u]-b[0][u],1),170/Math.max(b[1][v]-b[0][v],1)),center=b[0].map((x,k)=>(x+b[1][k])/2);const xy=p=>[198+(p[u]-center[u])*scale,110-(p[v]-center[v])*scale];c.fillStyle='#dbe5ed';for(const face of asset.faces){c.beginPath();face.forEach((j,k)=>c[k?'lineTo':'moveTo'](...xy(asset.vertices[j])));c.closePath();c.fill()}
 // The oblique line is the actual plane trace at the middle viewing depth.
 const fixed=row.offset_mm-row.normal[w]*center[w],ends=[b[0][u]-8,b[1][u]+8].map(x=>{const p=center.slice();p[u]=x;p[v]=(fixed-row.normal[u]*x)/row.normal[v];return xy(p)});
 c.strokeStyle='#00a9c4';c.lineWidth=4;c.beginPath();c.moveTo(...ends[0]);c.lineTo(...ends[1]);c.stroke();c.fillStyle='#007489';c.font='bold 17px system-ui';c.fillText(openingPosition(row),12,205);c.font='13px system-ui';c.fillText(row.is_cardinal?'Profile: '+info.profile_axes.map(k=>'XYZ'[k]).join(' / '):'Plane trace at '+'XYZ'[w]+' = '+center[w].toFixed(1)+' mm',12,225);
}
function previewOpening(row){const info=openingPreviewInfo(row),b=row.source_bounds_mm,n=row.normal,center=b[0].map((x,k)=>(x+b[1][k])/2),dot=(a,c)=>a.reduce((s,v,k)=>s+v*c[k],0),cross=(a,c)=>[a[1]*c[2]-a[2]*c[1],a[2]*c[0]-a[0]*c[2],a[0]*c[1]-a[1]*c[0]];let u=cross(Math.abs(n[2])<.9?[0,0,1]:[0,1,0],n);u=u.map(v=>v/Math.hypot(...u));const v=cross(n,u),point=center.map((x,k)=>x+n[k]*(row.offset_mm-dot(center,n))),half=Math.hypot(...b[1].map((x,k)=>x-b[0][k]))/2,key='opening-plane-preview:'+row.link;
 if(!data.assets[key])data.assets[key]={vertices:[[-half,-half,-.075],[half,-half,-.075],[half,half,-.075],[-half,half,-.075],[-half,-half,.075],[half,-half,.075],[half,half,.075],[-half,half,.075]],faces:[[0,1,2],[0,2,3],[4,6,5],[4,7,6],[0,4,5],[0,5,1],[1,5,6],[1,6,2],[2,6,7],[2,7,3],[3,7,4],[3,4,0]],face_colors:null};
 const plane={id:row.link+':opening-preview',asset:key,role:'plane',owner:row.link,transform:[...point.map((p,k)=>[u[k],v[k],n[k],p]),[0,0,0,1]],side:null,normal:null};info.plane_visual=plane;
 openingOverride={info,instances:[row.source,plane]};byId('owner').value=row.link;orient(info.side_view_axis);byId('openingSelection').textContent=row.id+' · '+openingPosition(row)+' · normal '+vectorLabel(n)+' · '+row.result;draw();
}
function renderOpening(event){const box=byId('openingPanel');box.replaceChildren();box.hidden=event.stage!=='architecture_ranking';if(box.hidden)return;
 const rows=openingRows(event),links=[...new Set(rows.map(r=>r.link))];
 if(!links.includes(openingLink)&&openingLink!=='all'){
  const carriers=trace.events.find(e=>e.stage==='carrier_profiles')?.instances.filter(i=>i.role==='component')||[];
  openingLink=links.reduce((best,link)=>carriers.filter(c=>c.owner===link).length>carriers.filter(c=>c.owner===best).length?link:best,links[0]);
 }
 box.append(textElement('h3','Opening candidates — supplied planes, model rank and local results'));
 paragraph(box,'All '+rows.length+' supplied/configured candidates are listed across '+links.length+' links. VLM rank and EPS queue order are separate; an untested or unsupported plane is not a feasible design. The result column summarizes later checks.');
 const controls=document.createElement('div'),select=document.createElement('select');for(const value of ['all',...links]){const option=document.createElement('option');option.value=value;option.textContent=value==='all'?'All links':value;select.append(option)}select.value=openingLink;select.onchange=()=>{openingLink=select.value;openingOverride=null;renderOpening(event);draw()};controls.append(select,textElement('span','  Click a row or thumbnail to inspect its exact plane.'));box.append(controls);
 const visible=rows.filter(r=>openingLink==='all'||r.link===openingLink),cards=document.createElement('div');cards.className='openingCards';
 for(const row of visible){const card=document.createElement('button');card.className='candidate';card.append(textElement('b',row.link+' · '+(row.model_rank?'VLM #'+row.model_rank:row.solver_order?'EPS #'+row.solver_order:'Unranked')));const thumbnail=document.createElement('canvas');drawOpeningCard(thumbnail,row);card.append(thumbnail,textElement('small',row.id),textElement('small',row.result,'outcome'));card.onclick=()=>{stop();previewOpening(row)};cards.append(card)}box.append(cards);
 const table=evidenceTable(['Link / candidate ID','VLM rank','EPS queue','Normal n','Plane / fraction','Final result'],visible.map(r=>[r.link+' / '+r.id,r.model_rank??'—',r.solver_order??'—',vectorLabel(r.normal),openingPosition(r)+'; f='+r.fraction.toFixed(2),r.result+(r.reason?' · '+r.reason:'')]));
 for(let i=0;i<table.body.children.length;i++){table.body.children[i].className='openingRow';table.body.children[i].onclick=()=>{stop();previewOpening(visible[i])}}
 const scroll=document.createElement('div');scroll.className='tableScroll';scroll.append(table.table);box.append(scroll);const selected=textElement('p','',null);selected.id='openingSelection';box.append(selected);
 const rationale=(event.metrics.ranking_explanations||[]).filter(r=>openingLink==='all'||r.part_id===openingLink);for(const r of rationale){paragraph(box,r.part_id+' — model rationale: '+r.rationale);if(r.assembly_notes?.length)paragraph(box,'Assembly notes: '+r.assembly_notes.join(' '));if(r.unknowns?.length)paragraph(box,'Unknowns: '+r.unknowns.join('; '))}
 const first=visible.find(r=>r.solver_order===1)||visible[0];if(first)previewOpening(first);
}
function renderRestrictionEvidence(event){schemaTableRows=[];const box=byId('schemaSummary');box.replaceChildren();box.hidden=event.stage!=='layout_constraints';if(box.hidden)return;
 const schema=event.metrics.schema;if(!schema){paragraph(box,'No validated restrict schema was recorded.');return}
 box.append(textElement('h3','Restrict schema — explicit hard constraints and soft preferences'));
 paragraph(box,'Schema version '+schema.schema_version+' · units '+schema.units+' · '+schema.components.length+' movable component instances. The same compiled schema gates seed generation, every EPS move and final placement checks.');
 paragraph(box,'HARD: bundle-center domains, allowed mounting normals, atomic rolls and minimum clearance. Normalized center bounds use each owner link’s usable-cavity world AABB; a full [0,1] interval does not bypass cavity or collision checks.');
 const domains=data.schema_domains?.owner_cavity_bounds_mm||{};
 const table=evidenceTable(['Instance / owner','Bundle-center range (XYZ)','Allowed normals / rolls','Min clearance'],schema.components.map(p=>{
  const domain=domains[p.owner_link],region=p.allowed_center_normalized;
  const world=domain?'\nWorld mm: '+region.map((r,k)=>'XYZ'[k]+':['+r.map(x=>(domain[0][k]+x*(domain[1][k]-domain[0][k])).toFixed(2)).join(', ')+']').join(' '):'';
  return[p.instance_id+' / '+p.owner_link,'Normalized: '+region.map((r,k)=>'XYZ'[k]+':['+r.join(', ')+']').join(' ')+world,p.allowed_opening_normals.map(vectorLabel).join(' ')+'\nRolls: '+p.allowed_rolls_deg.join(', ')+'°',p.min_clearance_mm+' mm'];
 }));schemaTableRows=schema.components.map((p,i)=>[p.instance_id,table.body.children[i]]);for(const [id,row]of schemaTableRows){row.className='policyRow';row.onclick=()=>selectSchemaComponent(id)}const scroll=document.createElement('div');scroll.className='tableScroll';scroll.append(table.table);box.append(scroll);
 box.append(textElement('h3','SOFT: gravity preferences and pairwise relations'));
 const gravity=data.schema_domains?.gravity;if(gravity)paragraph(box,'Supplied world gravity: '+vectorLabel(gravity.world_m_s2)+' m/s². '+(gravity.evidence||''));
 for(const p of schema.components){paragraph(box,p.instance_id+': '+p.vertical_preference+' · weight '+p.preference_weight+' (0 = inactive). '+p.evidence);if(p.unknowns.length)paragraph(box,'Unknowns — '+p.instance_id+': '+p.unknowns.join('; '))}
 if(schema.relations.length){box.append(evidenceTable(['Type','Instances','Weight','Evidence'],schema.relations.map(r=>[r.type,r.a+' ↔ '+r.b,r.weight,r.evidence])).table)}else paragraph(box,'Pairwise relations: [] — no additional model-generated pairwise preferences.');
 paragraph(box,'System unknowns: '+(schema.unknowns.join('; ')||'None recorded.'));
 box.append(textElement('h3','Local guardrails'));
 for(const g of event.metrics.guardrails||[])paragraph(box,g.instance_id+': requested clearance '+g.requested_clearance_mm+' mm → effective clearance '+g.effective_clearance_mm+' mm.');
 const raw=data.restrict_schema_raw;box.append(textElement('h3',raw?((event.compute?.kind||'local').startsWith('vlm')?'Complete raw VLM schema':'Complete raw local-rule schema'):'Complete recorded schema'));box.append(textElement('pre',JSON.stringify(raw||schema,null,2),'schemaJson'));
 if(raw&&JSON.stringify(raw)!==JSON.stringify(schema)){box.append(textElement('h3','Complete compiled schema (after guardrails)'));box.append(textElement('pre',JSON.stringify(schema,null,2),'schemaJson'))}
}
