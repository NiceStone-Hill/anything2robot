// Native WebGL depth testing keeps multi-part CAD solid and responsive offline.
function createProcessRenderer(canvas, assets) {
  const gl = canvas.getContext('webgl', {antialias:true, alpha:true});
  if (!gl) return null;
  const vertex = `attribute vec3 position;attribute vec3 normal;attribute vec3 color;
uniform mat4 model;uniform mat4 camera;varying vec3 n;varying vec3 c;
void main(){gl_Position=camera*model*vec4(position,1.0);n=mat3(model)*normal;c=color;}`;
  const fragment = `precision mediump float;varying vec3 n;varying vec3 c;
uniform vec3 tint;uniform float opacity;uniform float unlit;
void main(){float light=mix(.48+.52*abs(dot(normalize(n),normalize(vec3(.3,-.5,.8)))),1.0,unlit);
gl_FragColor=vec4(c*tint*light,opacity);}`;
  function shader(type, source) {
    const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);
    if (!gl.getShaderParameter(s,gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
    return s;
  }
  const program=gl.createProgram();
  gl.attachShader(program,shader(gl.VERTEX_SHADER,vertex));
  gl.attachShader(program,shader(gl.FRAGMENT_SHADER,fragment));gl.linkProgram(program);
  if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
  const uniforms=Object.fromEntries(['model','camera','tint','opacity','unlit'].map(k=>[k,gl.getUniformLocation(program,k)]));
  const attributes=Object.fromEntries(['position','normal','color'].map(k=>[k,gl.getAttribLocation(program,k)]));
  const cache=new Map();
  const columnMajor=m=>new Float32Array([0,1,2,3].flatMap(j=>m.map(row=>row[j])));
  const rgb=hex=>[1,3,5].map(i=>parseInt(hex.slice(i,i+2),16)/255);
  function buffer(key, lines) {
    const id=key+(lines?':lines':':triangles');if(cache.has(id))return cache.get(id);
    const asset=assets[key], vertices=asset.vertices, values=[];
    for(let i=0;i<asset.faces.length;i++) {
      const face=asset.faces[i], [a,b,c]=face.map(j=>vertices[j]);
      const u=b.map((v,j)=>v-a[j]),v=c.map((q,j)=>q-a[j]);
      let n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
      const length=Math.hypot(...n)||1;n=n.map(q=>q/length);
      const color=asset.face_colors?asset.face_colors[i].map(q=>q/255):[1,1,1];
      for(const j of (lines?[0,1,1,2,2,0]:[0,1,2]))values.push(...vertices[face[j]],...n,...color);
    }
    const handle=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,handle);
    gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(values),gl.STATIC_DRAW);
    const result={handle,count:values.length/9};cache.set(id,result);return result;
  }
  return {
    draw(items, options) {
      const {width,height,center,scale,yaw,pitch,wireframe,colors}=options;
      const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
      const rows=[[cy,-sy,0],[cp*sy,cp*cy,-sp],[sp*sy,sp*cy,cp]];
      const factors=[2*scale/width,2*scale/height,-1/Math.max(options.span*2,1)];
      const camera=rows.map((row,i)=>[...row.map(v=>v*factors[i]),-row.reduce((s,v,j)=>s+v*center[j],0)*factors[i]]);
      camera.push([0,0,0,1]);
      gl.viewport(0,0,width,height);gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
      gl.useProgram(program);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);
      gl.uniformMatrix4fv(uniforms.camera,false,columnMajor(camera));
      const line=item=>item.role==='plane'||(item.role==='shell'&&wireframe);
      for(const pass of [false,true]){
        gl.depthMask(!pass);if(pass){gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA)}else gl.disable(gl.BLEND);
        for(const item of items.filter(i=>line(i)===pass)){
          const b=buffer(item.asset,pass);gl.bindBuffer(gl.ARRAY_BUFFER,b.handle);
          for(const [name,offset] of [['position',0],['normal',12],['color',24]]){
            gl.enableVertexAttribArray(attributes[name]);gl.vertexAttribPointer(attributes[name],3,gl.FLOAT,false,36,offset);
          }
          gl.uniformMatrix4fv(uniforms.model,false,columnMajor(item.transform));
          gl.uniform3fv(uniforms.tint,new Float32Array(assets[item.asset].face_colors?[1,1,1]:rgb(colors[item.role]||'#777777')));
          gl.uniform1f(uniforms.opacity,pass?.18:1);gl.uniform1f(uniforms.unlit,pass?1:0);
          gl.drawArrays(pass?gl.LINES:gl.TRIANGLES,0,b.count);
        }
      }
      gl.depthMask(true);
    }
  };
}
