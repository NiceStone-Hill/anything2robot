// Shared right-handed graph kinematics, metres. World-rest meshes rotate once.
const identity4=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
function multiply4(a,b){const out=new Array(16).fill(0);for(let r=0;r<4;r++)for(let c=0;c<4;c++)for(let k=0;k<4;k++)out[r*4+c]+=a[r*4+k]*b[k*4+c];return out;}
function axisPose(axis,origin,angle){const [x,y,z]=axis,c=Math.cos(angle),s=Math.sin(angle),t=1-c;const m=[t*x*x+c,t*x*y-s*z,t*x*z+s*y,0,t*x*y+s*z,t*y*y+c,t*y*z-s*x,0,t*x*z-s*y,t*y*z+s*x,t*z*z+c,0,0,0,0,1];for(let r=0;r<3;r++)m[r*4+3]=origin[r]-m.slice(r*4,r*4+3).reduce((sum,v,i)=>sum+v*origin[i],0);return m;}
function bodyPoses(graph,phase){if(!graph)return {fixed:identity4(),moving:axisPose([0,0,1],[0,0,0],phase)};const result=Object.create(null),pending=[...graph.joints];result[graph.root_body_id]=identity4();while(pending.length){const ready=pending.filter(j=>result[j.parent]);if(!ready.length)throw Error('Joint graph has disconnected bodies.');for(const j of ready){const angle=(phase>=0?j.limits_deg[1]:-j.limits_deg[0])*phase*Math.PI/180;result[j.child]=multiply4(result[j.parent],axisPose(j.axis,j.position_mm.map(v=>v/1000),angle));pending.splice(pending.indexOf(j),1);}}return result;}
function posePoint(m,p){return [0,1,2].map(r=>m[r*4]*p[0]+m[r*4+1]*p[1]+m[r*4+2]*p[2]+m[r*4+3]);}
function viewerExplodedOffset(viewer,part,pose=identity4()){
  if(!viewer.explode)return [0,0,0];
  const local=part.opening_offset_m||(part.id.includes('lid')?[0,0,.065]:(part.id==='motor_retainer'?[0,0,.035]:(part.id.startsWith('electronics')?[0,0,.025]:[0,0,0])));
  return [0,1,2].map(r=>pose[r*4]*local[0]+pose[r*4+1]*local[1]+pose[r*4+2]*local[2]);
}
// Fade enclosing skins only; actuator mounts, retainers and output hubs stay solid.
function viewerPartOpacity(viewer,part){return /_(shell|lid)$/.test(part.id)?(viewer.shellOpacity??1):1;}
function shellOpacityControl(viewer,onChange){
  const label=node('label','Shell opacity '),input=node('input'),value=node('output');
  input.type='range';input.min=10;input.max=100;input.step=5;input.value=Math.round((viewer.shellOpacity??1)*100);input.setAttribute('aria-label','Shell opacity');
  value.textContent=input.value+'%';
  input.oninput=()=>{viewer.shellOpacity=Number(input.value)/100;value.textContent=input.value+'%';onChange?.();viewer.draw();};
  label.append(input,value);return label;
}
// Frame only visible geometry so an internal view can inspect full-size board CAD.
function viewerPartVisible(viewer,part){
  if(part.role==='mount_interface'&&(viewer.shellOpacity??1)>=1)return false;
  if(part.role==='fastener_tool_path'&&(!viewer.showFasteners||viewer.play||Math.abs(viewer.angle)>1e-8||(viewer.fastenerFocus&&part.fastener_access_id!==viewer.fastenerFocus)))return false;
  if(viewer.hideCarriers&&part.id.startsWith('electronics_')&&part.id.endsWith('_carrier'))return false;
  if(part.role==='bundle_corridor'&&!viewer.showCorridors)return false;
  if(viewer.showCorridors&&['wire_guide','wire_endpoint'].includes(part.role))return false;
  const focus=viewer.wireFocus||viewer.attachmentFocus;
  if(focus){
    if(part.wire_id){if(part.wire_id!==focus.id)return false;}
    else if(part.role==='bundle_corridor'){if(!focus.segments.includes(part.id))return false;}
    else if(!focus.instances.includes(part.instance_id)&&!focus.instances.some(id=>part.id.startsWith('electronics_'+id+'_'))&&!focus.assets.includes(part.id))return false;
  }
  return (!viewer.actuatorOnly||/_(motor|rotor)$/.test(part.id))&&(!viewer.focusInstance||part.instance_id===viewer.focusInstance||part.id.startsWith('electronics_'+viewer.focusInstance+'_'))&&!(viewer.hiddenShells&&part.id.includes('lid'))&&!(viewer.hideBodies&&/_(shell|lid)$/.test(part.id));
}
function viewerProjectPoint(viewer,point){
  const [x,y,z]=point.map((v,i)=>v-(viewer.center?.[i]||0)),cy=Math.cos(viewer.yaw),sy=Math.sin(viewer.yaw),cp=Math.cos(viewer.pitch),sp=Math.sin(viewer.pitch);
  const scale=viewer.zoom*viewer.canvas.clientHeight/(2*viewer.radius*(viewer.explode?1.35:1));
  return [viewer.canvas.clientWidth/2+(cy*x-sy*y)*scale,viewer.canvas.clientHeight/2-(-sp*sy*x-sp*cy*y+cp*z)*scale];
}
function fitVisibleGeometry(viewer){
  const lower=[Infinity,Infinity,Infinity],upper=[-Infinity,-Infinity,-Infinity];let count=0;
  const poses=bodyPoses(viewer.graph,viewer.angle||0);
  for(const part of viewer.parts){if(!viewerPartVisible(viewer,part))continue;
    if(part.role==='fastener_tool_path')continue; // Diagnostic paths must not zoom out the robot.
    const pose=poses[part.owner]||identity4(),offset=viewerExplodedOffset(viewer,part,pose);
    const points=part.bounds?Array.from({length:8},(_,i)=>[0,1,2].map(a=>part.bounds[(i>>a)&1][a])):part.vertices;
    for(const source of points){const point=posePoint(pose,source).map((v,i)=>v+offset[i]);for(let axis=0;axis<3;axis++){lower[axis]=Math.min(lower[axis],point[axis]);upper[axis]=Math.max(upper[axis],point[axis]);}count++;}}
  if(!count)return;viewer.center=lower.map((value,axis)=>(value+upper[axis])/2);viewer.radius=Math.max(.001,Math.hypot(...upper.map((value,axis)=>value-lower[axis]))*.65);viewer.zoom=1;
}
function updateJointReadout(viewer){
  if(viewer.canvas.id!=='mechanical-canvas')return;
  const readout=document.getElementById('motion-readout'),slider=document.getElementById('motion-slider');
  const joint=viewer.graph?.joints[0];
  const degrees=joint?(viewer.angle>=0?joint.limits_deg[1]:-joint.limits_deg[0])*viewer.angle:viewer.angle*180/Math.PI;
  if(readout)readout.textContent=degrees.toFixed(1)+'° · kinematic preview';
  if(slider&&viewer.play){const [lo,hi]=viewer.joint.limits_rad;slider.value=100*(viewer.angle-lo)/(hi-lo);}
}
// Local WebGL viewer: generated solids, stable framing, no external library/CDN.
class SolidViewer {
  constructor(canvas) {
    this.canvas=canvas;this.gl=canvas.getContext('webgl',{antialias:true,alpha:false});
    if(!this.gl)return new SoftwareSolidViewer(canvas);
    this.parts=[];this.yaw=-.7;this.pitch=.55;this.zoom=1;this.angle=0;this.play=false;this.explode=false;this.selected='';this.frames=[];
    const gl=this.gl;
    const vertex=`attribute vec3 position;attribute vec3 normal;uniform mat4 camera;uniform mat4 body;uniform vec3 offset;varying vec3 n;void main(){gl_Position=camera*(body*vec4(position,1.)+vec4(offset,0.));n=mat3(body)*normal;}`;
    const fragment=`precision mediump float;varying vec3 n;uniform vec3 color;uniform float opacity;void main(){float light=.55+.45*abs(dot(normalize(n),normalize(vec3(.3,-.5,.8))));gl_FragColor=vec4(color*light,opacity);}`;
    const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error('3D shader compilation failed.');return s;};
    this.program=gl.createProgram();gl.attachShader(this.program,shader(gl.VERTEX_SHADER,vertex));gl.attachShader(this.program,shader(gl.FRAGMENT_SHADER,fragment));gl.linkProgram(this.program);
    if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw Error('3D renderer failed.');
    this.pos=gl.getAttribLocation(this.program,'position');this.normal=gl.getAttribLocation(this.program,'normal');this.uniform={};for(const k of ['camera','body','offset','color','opacity'])this.uniform[k]=gl.getUniformLocation(this.program,k);
    let drag=null;canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
    canvas.onpointermove=e=>{if(!drag)return;this.yaw+=(e.clientX-drag[0])*.008;this.pitch=Math.max(-1.5,Math.min(1.5,this.pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];this.draw();};canvas.onpointerup=()=>drag=null;
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.4,Math.min(3,this.zoom*Math.exp(-e.deltaY*.001)));this.draw();},{passive:false});
    new ResizeObserver(()=>this.draw()).observe(canvas);
  }
  load(data){
    const gl=this.gl;for(const p of this.parts)gl.deleteBuffer(p.buffer);this.parts=[];this.joint=data.joint;this.graph=data.graph;this.focusInstance=null;this.wireFocus=null;this.attachmentFocus=null;this.showFasteners=false;this.fastenerFocus=null;this.angle=0;this.play=false;this.zoom=1;
    let radius=0;
    for(const part of data.parts){const numbers=[];
      for(const f of part.faces){const [a,b,c]=f.map(i=>part.vertices[i]);const u=b.map((v,i)=>v-a[i]),v=c.map((n,i)=>n-a[i]);const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];const len=Math.hypot(...n)||1;
        for(const point of [a,b,c]){numbers.push(...point,...n.map(x=>x/len));radius=Math.max(radius,Math.hypot(...point));}}
      const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(numbers),gl.STATIC_DRAW);
      const lower=[Infinity,Infinity,Infinity],upper=[-Infinity,-Infinity,-Infinity];for(const point of part.vertices)for(let axis=0;axis<3;axis++){lower[axis]=Math.min(lower[axis],point[axis]);upper[axis]=Math.max(upper[axis],point[axis]);}
      this.parts.push({...part,bounds:[lower,upper],vertices:undefined,faces:undefined,buffer,count:numbers.length/6,color:part.color.match(/[a-f0-9]{2}/gi).map(n=>parseInt(n,16)/255)});
    }
    this.radius=radius*1.3;this.fitVisible();
  }
  fitVisible(){fitVisibleGeometry(this);this.draw();}
  draw(){
    if(!this.parts.length)return;updateJointReadout(this);
    const begin=performance.now(),gl=this.gl,c=this.canvas,ratio=Math.min(devicePixelRatio,2),w=Math.max(1,Math.floor(c.clientWidth*ratio)),h=Math.max(1,Math.floor(c.clientHeight*ratio));
    if(c.width!==w||c.height!==h){c.width=w;c.height=h;}gl.viewport(0,0,w,h);gl.clearColor(.94,.96,.94,1);gl.enable(gl.DEPTH_TEST);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);
    const cy=Math.cos(this.yaw),sy=Math.sin(this.yaw),cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),scale=this.zoom/(this.radius*(this.explode?1.35:1)),xs=scale*h/w,ys=scale,zs=-scale*.25;
    const [cx,ccy,cz]=this.center||[0,0,0];gl.uniformMatrix4fv(this.uniform.camera,false,new Float32Array([cy*xs,-sp*sy*ys,cp*sy*zs,0,-sy*xs,-sp*cy*ys,cp*cy*zs,0,0,cp*ys,sp*zs,0,(-cy*cx+sy*ccy)*xs,(sp*sy*cx+sp*cy*ccy-cp*cz)*ys,(-cp*sy*cx-cp*cy*ccy-sp*cz)*zs,1]));
    const matrices=bodyPoses(this.graph,this.angle);
    // Opaque hardware writes depth first; transparent skins cannot occlude it.
    const visible=this.parts.filter(p=>viewerPartVisible(this,p));
    const depth=p=>{const point=posePoint(matrices[p.owner]||identity4(),p.bounds[0].map((v,i)=>(v+p.bounds[1][i])/2));const offset=viewerExplodedOffset(this,p,matrices[p.owner]||identity4());return cp*sy*(point[0]+offset[0])+cp*cy*(point[1]+offset[1])+sp*(point[2]+offset[2]);};
    const transparent=visible.filter(p=>viewerPartOpacity(this,p)<1).sort((a,b)=>depth(a)-depth(b));
    gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);
    for(const p of [...visible.filter(p=>viewerPartOpacity(this,p)===1),...transparent]){
      const opacity=viewerPartOpacity(this,p);gl.uniform1f(this.uniform.opacity,opacity);
      if(opacity<1){gl.enable(gl.BLEND);gl.depthMask(false);gl.enable(gl.CULL_FACE);gl.cullFace(gl.BACK);}else{gl.disable(gl.BLEND);gl.depthMask(true);gl.disable(gl.CULL_FACE);}
      const offset=viewerExplodedOffset(this,p,matrices[p.owner]||identity4());
      const pose=matrices[p.owner]||identity4();gl.uniformMatrix4fv(this.uniform.body,false,new Float32Array(pose.map((_,i)=>pose[(i%4)*4+Math.floor(i/4)])));gl.uniform3fv(this.uniform.offset,offset);gl.uniform3fv(this.uniform.color,p.id===this.selected?[.93,.47,.18]:p.color);
      gl.bindBuffer(gl.ARRAY_BUFFER,p.buffer);gl.enableVertexAttribArray(this.pos);gl.vertexAttribPointer(this.pos,3,gl.FLOAT,false,24,0);gl.enableVertexAttribArray(this.normal);gl.vertexAttribPointer(this.normal,3,gl.FLOAT,false,24,12);gl.drawArrays(gl.TRIANGLES,0,p.count);
    }
    gl.depthMask(true);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);
    this.frames.push(performance.now()-begin);if(this.frames.length>120)this.frames.shift();this.onDraw?.();
  }
  animate(){if(!this.play)return;const [lo,hi]=this.joint.limits_rad;this.angle=(lo+hi)/2+(hi-lo)/2*Math.sin(performance.now()/1100);this.draw();requestAnimationFrame(()=>this.animate());}
  toggle(){this.play=!this.play;if(this.play)this.animate();return this.play;}
  stats(){return {triangles:this.parts.reduce((sum,p)=>sum+p.count/3,0),draw_cpu_ms:this.frames.reduce((a,b)=>a+b,0)/Math.max(1,this.frames.length),frames:this.frames.length,renderer:this.gl.getParameter(this.gl.RENDERER)};}
}

// Ubuntu VMs/headless Firefox may expose no WebGL. Same vertex data and joint
// transforms, painter depth ordering; no placeholder image or remote renderer.
class SoftwareSolidViewer {
  constructor(canvas){
    this.canvas=canvas;this.ctx=canvas.getContext('2d',{alpha:false});if(!this.ctx)throw Error('3D preview unavailable. Model downloads are still available.');
    this.parts=[];this.yaw=-.7;this.pitch=.55;this.zoom=1;this.angle=0;this.play=false;this.explode=false;this.selected='';this.frames=[];this.intervals=[];this.last=0;
    let drag=null;canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};canvas.onpointermove=e=>{if(!drag)return;this.yaw+=(e.clientX-drag[0])*.008;this.pitch=Math.max(-1.5,Math.min(1.5,this.pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];this.draw();};canvas.onpointerup=()=>drag=null;
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.4,Math.min(3,this.zoom*Math.exp(-e.deltaY*.001)));this.draw();},{passive:false});new ResizeObserver(()=>this.draw()).observe(canvas);
  }
  load(data){this.parts=data.parts.map(p=>({...p,color:p.color.match(/[a-f0-9]{2}/gi).map(n=>parseInt(n,16))}));this.joint=data.joint;this.graph=data.graph;this.focusInstance=null;this.wireFocus=null;this.attachmentFocus=null;this.showFasteners=false;this.fastenerFocus=null;this.angle=0;this.play=false;this.zoom=1;this.radius=this.parts.reduce((r,p)=>p.vertices.reduce((r,v)=>Math.max(r,Math.hypot(...v)),r),0)*1.3;this.fitVisible();}
  fitVisible(){fitVisibleGeometry(this);this.draw();}
  draw(){
    if(!this.parts.length)return;updateJointReadout(this);const start=performance.now(),canvas=this.canvas,ctx=this.ctx,w=canvas.clientWidth,h=canvas.clientHeight;
    if(!w||!h)return;const ratio=Math.min(devicePixelRatio,1.5);if(canvas.width!==Math.floor(w*ratio)||canvas.height!==Math.floor(h*ratio)){canvas.width=Math.floor(w*ratio);canvas.height=Math.floor(h*ratio);}ctx.setTransform(ratio,0,0,ratio,0,0);ctx.fillStyle='#f0f5f0';ctx.fillRect(0,0,w,h);
    const cy=Math.cos(this.yaw),sy=Math.sin(this.yaw),cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),scale=this.zoom*h/(2*this.radius*(this.explode?1.35:1));const triangles=[];
    const matrices=bodyPoses(this.graph,this.angle);
    for(const p of this.parts){if(!viewerPartVisible(this,p))continue;
      const pose=matrices[p.owner]||identity4(),offset=viewerExplodedOffset(this,p,pose);
      const vertices=p.vertices.map(v=>{const point=posePoint(pose,v).map((value,i)=>value+offset[i]-(this.center?.[i]||0)),[x,y]=point,z=point[2];return [w/2+(cy*x-sy*y)*scale,h/2-(-sp*sy*x-sp*cy*y+cp*z)*scale,cp*sy*x+cp*cy*y+sp*z];});
      for(const face of p.faces){const [a,b,c]=face.map(i=>vertices[i]);const area=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);if(area>=0)continue;
        const [u,v,t]=face.map(i=>p.vertices[i]);const d=v.map((x,i)=>x-u[i]),e=t.map((x,i)=>x-u[i]);const n=[d[1]*e[2]-d[2]*e[1],d[2]*e[0]-d[0]*e[2],d[0]*e[1]-d[1]*e[0]];const length=Math.hypot(...n)||1,light=.55+.45*Math.abs((n[0]*.3-n[1]*.5+n[2]*.8)/length);const color=p.id===this.selected?[237,120,46]:p.color;
        triangles.push({a,b,c,z:(a[2]+b[2]+c[2])/3,opacity:viewerPartOpacity(this,p),color:'rgb('+color.map(x=>Math.round(x*light)).join(',')+')'});
      }
    }
    triangles.sort((a,b)=>a.z-b.z);for(const t of triangles){ctx.globalAlpha=t.opacity;ctx.fillStyle=t.color;ctx.beginPath();ctx.moveTo(t.a[0],t.a[1]);ctx.lineTo(t.b[0],t.b[1]);ctx.lineTo(t.c[0],t.c[1]);ctx.closePath();ctx.fill();}
    ctx.globalAlpha=1;ctx.fillStyle='#526a5b';ctx.font='12px sans-serif';ctx.fillText('Drag to rotate',14,h-14);
    this.frames.push(performance.now()-start);if(this.frames.length>120)this.frames.shift();this.onDraw?.();
  }
  animate(){if(!this.play)return;const t=performance.now();if(this.last&&t-this.last<1000){this.intervals.push(t-this.last);if(this.intervals.length>120)this.intervals.shift();}this.last=t;const [lo,hi]=this.joint.limits_rad;this.angle=(lo+hi)/2+(hi-lo)/2*Math.sin(t/1100);this.draw();requestAnimationFrame(()=>this.animate());}
  toggle(){this.play=!this.play;if(this.play){this.last=0;this.animate();}return this.play;}
  stats(){return {renderer:'software_triangle_projection',triangles:this.parts.reduce((a,p)=>a+p.faces.length,0),draw_cpu_ms:this.frames.reduce((a,b)=>a+b,0)/Math.max(1,this.frames.length),animation_fps:1000/(this.intervals.reduce((a,b)=>a+b,0)/Math.max(1,this.intervals.length)),frames:this.frames.length};}
}

function componentFocusControl(viewer,assets){
  const label=node('label','Focus '),select=node('select');select.setAttribute('aria-label','Focus component');const all=node('option','All parts');all.value='';select.append(all);
  for(const asset of assets){const option=node('option',asset.name.split(' · ')[0]);option.value=asset.instance_id;select.append(option);}
  select.onchange=()=>{viewer.attachmentFocus=null;viewer.wireFocus=null;viewer.focusInstance=select.value||null;viewer.fitVisible();};label.append(select);return label;
}
