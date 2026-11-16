"""Edited screen recording of actual local UI and its kinematic canvas.
No simulation or fabricated geometry. Run after a case has passed all stages.
Requires Pillow and an ffmpeg binary (ROBOT_DEMO_FFMPEG).
"""
import os,sys,json,time,base64,subprocess,tempfile,io,hashlib
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from tests.browser_smoke import Browser
ROOT=Path(__file__).resolve().parents[1]
OUT=Path(os.environ['ROBOT_DEMO_OUTPUT']);OUT.mkdir(parents=True,exist_ok=True)
PROJECT=os.environ['ROBOT_DEMO_PROJECT'];BASE='http://127.0.0.1:8768/'
FFMPEG=os.environ['ROBOT_DEMO_FFMPEG'];FPS=12;W,H=1280,720
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def ft(n):return ImageFont.truetype(font,n)
manifest={'project_id':PROJECT,'kind':'edited browser screen recording','processing_waits_omitted':True,'kinematic_preview_only':True,'physical_tested':False,'simulation_run':False,'chapters':[],'english_audit':{}}
encoder=subprocess.Popen([FFMPEG,'-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'robot-design-demo.mp4')],stdin=subprocess.PIPE,stderr=(OUT/'encoder.log').open('w'))
frames=0
with tempfile.TemporaryDirectory(prefix='teacher-video-') as tmp:
 profile=Path(tmp);(profile/'user.js').write_text('user_pref("marionette.port",2874);\n')
 log=(OUT/'recording-browser.log').open('w');process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log);b=None
 try:
  for _ in range(60):
   try:b=Browser(2874);break
   except (OSError,RuntimeError):time.sleep(.5)
  assert b
  def js(s):return b.script(s)
  def click(s):js('document.querySelector('+json.dumps(s)+').click()')
  def field(s,v):js('const e=document.querySelector('+json.dumps(s)+');e.value='+json.dumps(v)+';e.dispatchEvent(new Event("input"));e.dispatchEvent(new Event("change"))')
  def wait(s,timeout=40):
   end=time.monotonic()+timeout
   while time.monotonic()<end:
    if js('return '+s):return
    time.sleep(.2)
   raise AssertionError(s)
  def compose(im,title,subtitle):
   frame=Image.new('RGB',(W,H),'#ffffff');im=im.convert('RGB');im.thumbnail((W-40,H-145));frame.paste(im,((W-im.width)//2,90+(H-145-im.height)//2))
   d=ImageDraw.Draw(frame);d.text((24,17),title,font=ft(27),fill='#172b36');d.text((24,55),subtitle,font=ft(15),fill='#52616b');d.line((24,H-39,W-24,H-39),fill='#dce2e6');d.text((24,H-29),'SOFTWARE PROTOTYPE  |  Kinematic preview; no physical assembly or simulation validation.',font=ft(13),fill='#58646d');return frame
  def write(frame,count=1):
   global frames
   raw=frame.tobytes()
   for _ in range(count):encoder.stdin.write(raw);frames+=1
  def screen():
   data=b.call('WebDriver:TakeScreenshot',{'id':None,'full':False,'scroll':False})['value'];return Image.open(io.BytesIO(base64.b64decode(data)))
  def canvas(selector):return Image.open(io.BytesIO(base64.b64decode(js('return document.querySelector('+json.dumps(selector)+').toDataURL("image/png")').split(',',1)[1])))
  def chapter(title,subtitle):manifest['chapters'].append({'start_s':frames/FPS,'title':title,'subtitle':subtitle});print(title,flush=True)
  def page(view,title,subtitle,target=None,seconds=4):
   click('[data-view='+view+']');time.sleep(1.2)
   if target:js('document.querySelector('+json.dumps(target)+').scrollIntoView({block:"center"})');time.sleep(.3)
   else:js('window.scrollTo(0,0)')
   chapter(title,subtitle);im=screen();im.save(OUT/(view+'-screen.png'));write(compose(im,title,subtitle),seconds*FPS)
  def motion(mode,title,subtitle,seconds=7):
   click('[data-view=mechanics]');wait('!document.querySelector("#mechanical-canvas").hidden');time.sleep(.5)
   field('#mechanical-view-mode',mode)
   if js('return document.querySelector("#hide-lids").checked'):click('#hide-lids')
   js('document.querySelector("#mechanical-canvas").scrollIntoView({block:"center"})');time.sleep(.3)
   chapter(title,subtitle)
   import math
   for n in range(seconds*FPS):
    phase=math.sin(n/(FPS*2)*math.pi);field('#motion-slider',50+50*phase)
    im=canvas('#mechanical-canvas');frame=compose(im,title,subtitle+f'  |  Head yaw: {phase*10:+.1f} deg')
    if mode=='actuator':
     d=ImageDraw.Draw(frame);cx,cy=1140,550;theta=phase*10*math.pi/180
     d.ellipse((cx-45,cy-45,cx+45,cy+45),outline='#acb6bc',width=2)
     d.line((cx,cy,cx+42*math.sin(theta),cy-42*math.cos(theta)),fill='#d97917',width=4)
     d.text((1000,608),'Output angle indicator',font=ft(14),fill='#52616b')
    write(frame)
    if n==FPS:(frame).save(OUT/(mode+'-motion.jpg'))
  b.call('WebDriver:SetWindowRect',{'width':1440,'height':960});b.call('WebDriver:Navigate',{'url':BASE});wait('document.querySelectorAll("#project-list option").length>1');field('#project-list',PROJECT);wait('!document.querySelector("#workspace").hidden');time.sleep(2)
  # Inspect every generated text node, including collapsed detail reports. User input values are not translated.
  for view in ['articulation','mechanics','parts','wiring','layout','harness','simulation']:
   click('[data-view='+view+']');time.sleep(.8)
   findings=js('return Array.from(document.querySelectorAll("#'+view+'-view *")).filter(e=>!e.children.length&&/[\\u3400-\\u9fff]/.test(e.textContent)&&!e.matches("script,style,textarea,input")).map(e=>({tag:e.tagName,text:e.textContent.slice(0,300)}))')
   manifest['english_audit'][view]=findings
  assert not any(manifest['english_audit'].values()),manifest['english_audit']
  page('articulation','01  Import model and identify the head joint','User STL · 500 mm maximum dimension · online VLM annotation · one head yaw joint',seconds=4)
  page('mechanics','02  Generate the motor mounting structure','SG90 screw-mounted cartridge · fixed body and moving head · generated service covers','#mechanical-canvas',4)
  motion('exterior','Head motion preview','Actual generated shell meshes · body, legs and tail remain fixed',7)
  motion('actuator','Actuator motion: fixed housing and moving output','SG90 housing stays fixed; highlighted output hub follows the head',8)
  page('parts','03  Select electronics','Pico 2 + MPU6050 reference CAD · two assumed supplies + distribution module',seconds=4)
  page('wiring','04  Generate and check wiring','Online AI proposal checked against component ports and voltage rules','#diagram',4)
  page('layout','05  Place electronics and generate removable mounts','Registered PCB holes, screw channels and removable carriers','#layout-view canvas',4)
  click('#layout-open-covers');time.sleep(.5);chapter('Service openings','Local covers open for access; body, legs and tail stay together')
  write(compose(canvas('#layout-view canvas'),'Service openings','Actual generated openings and mounts · physical tool access and assembly remain unverified'),5*FPS)
  click('[data-view=layout]')
  # Focus the reference boards through the normal UI controls.
  options=js('return Array.from(document.querySelector("#layout-view select[aria-label=\\"Focus component\\"]").options).map(o=>({value:o.value,text:o.textContent}))')
  for needle,title in [('Pico','Pico 2: four registered screw axes'),('MPU','MPU6050: two registered screw axes')]:
   option=next(o for o in options if needle in o['text']);field('#layout-view select[aria-label="Focus component"]',option['value']);time.sleep(.5);chapter(title,'Reference CAD and carrier use the same rigid transform; hardware variant unverified')
   write(compose(canvas('#layout-view canvas'),title,'Reference CAD and generated mounts · physical PCB and fastener variants require measurement'),4*FPS)
  page('harness','06  Route wires as shared bundles','Shared trunks and branches · conservative bundle envelopes · connector fan-out remains assumed','#harness-view canvas',5)
  page('simulation','07  Export the assembly','STL + GLB review files · URDF and MJCF packages · no simulation or controller generation',seconds=4)
  chapter('Completed software case','All registered software stages passed for this case. General model support and manufacturing verification remain incomplete.')
  end=Image.new('RGB',(1120,480),'#f4f6f7');d=ImageDraw.Draw(end)
  for i,line in enumerate(['One head joint / SG90','5 electronics modules / registered PCB mounts','Bundled wiring / export package','Physical parts, power modules and tolerances remain unverified.']):d.text((55,75+i*78),line,font=ft(27 if i<3 else 19),fill='#243846')
  write(compose(end,'Robot design workflow','Edited recording of the local tool; processing waits omitted.'),5*FPS)
  manifest['frames']=frames;manifest['fps']=FPS;manifest['duration_s']=frames/FPS;manifest['passed']=True
 finally:
  if b:
   try:b.call('WebDriver:DeleteSession',{})
   except Exception:pass
   b.sock.close()
  process.terminate()
  try:process.wait(timeout=10)
  except subprocess.TimeoutExpired:process.kill();process.wait()
  encoder.stdin.close();code=encoder.wait(timeout=120);manifest['encoder_exit']=code
  (OUT/'recording-manifest.json').write_text(json.dumps(manifest,indent=2))
  if code:raise RuntimeError('Video encoder failed; see encoder.log')
print('Video:',OUT/'robot-design-demo.mp4',flush=True)
