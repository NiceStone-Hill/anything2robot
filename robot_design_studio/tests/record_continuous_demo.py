"""Continuous real-time browser recording. No added captions, cuts or title cards."""
import os,json,time,subprocess,tempfile,base64,io
from pathlib import Path
from PIL import Image
from tests.browser_smoke import Browser
OUT=Path(os.environ['ROBOT_DEMO_OUTPUT']);OUT.mkdir(parents=True,exist_ok=True)
PROJECT=os.environ['ROBOT_DEMO_PROJECT'];FPS=10;DURATION=45
with tempfile.TemporaryDirectory(prefix='continuous-recording-') as temp:
 profile=Path(temp);(profile/'user.js').write_text('user_pref("marionette.port",2876);\n')
 log=(OUT/'browser.log').open('w');process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log);b=None;encoder=None
 try:
  for _ in range(60):
   try:b=Browser(2876);break
   except (OSError,RuntimeError):time.sleep(.5)
  assert b
  def js(code):return b.script(code)
  def click(selector):js('document.querySelector('+json.dumps(selector)+').click()')
  def field(selector,value):js('const e=document.querySelector('+json.dumps(selector)+');e.value='+json.dumps(value)+';e.dispatchEvent(new Event("input"));e.dispatchEvent(new Event("change"))')
  def wait(condition):
   for _ in range(150):
    if js('return '+condition):return
    time.sleep(.2)
   raise AssertionError(condition)
  b.call('WebDriver:SetWindowRect',{'width':1440,'height':980});b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8768/'});wait('document.querySelectorAll("#project-list option").length>1');field('#project-list',PROJECT);wait('document.querySelector("#mechanical-summary").textContent.includes("passed")');click('[data-view=mechanics]');time.sleep(1)
  if js('return document.querySelector("#hide-lids").checked'):click('#hide-lids')
  field('#mechanical-view-mode','exterior');js('document.querySelector("#mechanical-canvas").dispatchEvent(new WheelEvent("wheel",{deltaY:-450,cancelable:true}));document.querySelector("#mechanical-canvas").scrollIntoView({block:"center"})');click('#motion-play')
  def shot():
   raw=b.call('WebDriver:TakeScreenshot',{'id':None,'full':False,'scroll':False})['value'];im=Image.open(io.BytesIO(base64.b64decode(raw))).convert('RGB');return im
  frame=shot();width,height=frame.size;assert width%2==height%2==0
  encoder=subprocess.Popen([os.environ['ROBOT_DEMO_FFMPEG'],'-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'robot-demo-continuous.mp4')],stdin=subprocess.PIPE,stderr=(OUT/'encoder.log').open('w'))
  started=time.monotonic();written=0;acted=set();actions=[]
  while time.monotonic()-started<DURATION:
   elapsed=time.monotonic()-started
   if elapsed>=14 and 'actuator' not in acted:
    field('#mechanical-view-mode','actuator');js('document.querySelector("#mechanical-canvas").dispatchEvent(new WheelEvent("wheel",{deltaY:-150,cancelable:true}))');acted.add('actuator');actions.append({'at_s':time.monotonic()-started,'action':'Actuator view'})
   if elapsed>=26 and 'layout' not in acted:
    click('[data-view=layout]');js('document.querySelector("#layout-view canvas").scrollIntoView({block:"center"})');acted.add('layout');actions.append({'at_s':time.monotonic()-started,'action':'Internal layout'})
   if elapsed>=32 and 'covers' not in acted:
    click('#layout-open-covers');acted.add('covers');actions.append({'at_s':time.monotonic()-started,'action':'Show service openings'})
   if elapsed>=38 and 'harness' not in acted:
    click('[data-view=harness]');js('document.querySelector("#harness-view canvas").scrollIntoView({block:"center"})');acted.add('harness');actions.append({'at_s':time.monotonic()-started,'action':'Wire routing'})
   frame=shot();target=min(DURATION*FPS,int((time.monotonic()-started)*FPS));raw=frame.tobytes()
   while written<target:encoder.stdin.write(raw);written+=1
   if 'poster' not in acted and elapsed>4:frame.save(OUT/'preview.jpg');acted.add('poster')
   time.sleep(.025)
  while written<DURATION*FPS:encoder.stdin.write(frame.tobytes());written+=1
  encoder.stdin.close();assert encoder.wait(timeout=90)==0
  (OUT/'recording.json').write_text(json.dumps({'project_id':PROJECT,'recording':'continuous real-time browser viewport','duration_s':DURATION,'fps':FPS,'frames':written,'dimensions':[width,height],'cuts':False,'added_captions':False,'title_cards':False,'time_compression':False,'actions':actions},indent=2))
  print('Recorded',OUT/'robot-demo-continuous.mp4',flush=True)
 finally:
  if encoder and encoder.poll() is None:encoder.kill();encoder.wait()
  if b:
   try:b.call('WebDriver:DeleteSession',{})
   except Exception:pass
   b.sock.close()
  process.terminate()
  try:process.wait(timeout=10)
  except subprocess.TimeoutExpired:process.kill();process.wait()
