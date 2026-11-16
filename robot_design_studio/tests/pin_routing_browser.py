"""Actual Firefox check of the retained full-workflow project on port 8770."""
import base64,json,subprocess,tempfile,time
from pathlib import Path
from tests.browser_smoke import Browser
out=Path('robot_design_studio/evidence/pin_routing_2026-11-16');report={}
with tempfile.TemporaryDirectory(prefix='pin-routing-browser-') as temp:
 profile=Path(temp);(profile/'user.js').write_text('user_pref("marionette.port", 2876);\n')
 with (out/'browser.log').open('w') as log:
  process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log);b=None
  try:
   for _ in range(80):
    try:b=Browser(2876);break
    except (OSError,RuntimeError):time.sleep(.5)
   assert b
   b.call('WebDriver:SetWindowRect',{'width':1440,'height':1100});b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8770/'})
   b.wait("document.querySelectorAll('#project-list option').length===4")
   b.script("const s=document.querySelector('#project-list');s.value='project_95d1e73d971c4ba1';s.dispatchEvent(new Event('change'))")
   b.wait("document.querySelector('#harness-wire-select')?.options.length===12")
   b.script("document.querySelector('[data-view=harness]').click();window.scrollTo(0,0)")
   b.screenshot(str(out/'all-wires.png'))
   b.wait("document.querySelector('#harness-canvas').width>100")
   b.wait("JSON.parse(window.wrappedJSObject.harnessPerformance()).frames>1")
   time.sleep(.5)
   b.screenshot(str(out/'all-wires.png'))
   checks=json.loads(b.script("return window.wrappedJSObject.harnessPerformance()"))
   assert len(checks['colors'])==11 and checks['wire_count']==11 and checks['visible_corridors']==0 and checks['visible_carriers']==0,checks
   b.script("const s=document.querySelector('#harness-wire-select');s.value='wire_7';s.dispatchEvent(new Event('change'))")
   b.wait("document.querySelector('#harness-connection-detail').textContent.includes('pin 1')")
   checks['selected']=b.script("return {detail:document.querySelector('#harness-connection-detail').textContent,wireIds:JSON.parse(window.wrappedJSObject.harnessPerformance()).visible_wire_ids,labels:document.querySelectorAll('.wire-endpoint-label').length}")
   assert checks['selected']['wireIds']==['wire_7'] and checks['selected']['labels']==2,checks
   b.script("document.querySelector('#harness-canvas').scrollIntoView({block:'start'})")
   b.screenshot(str(out/'selected-sda.png'))
   element=b.call('WebDriver:FindElement',{'using':'css selector','value':'#harness-canvas'})
   key=element['value']['element-6066-11e4-a52e-4f735466cecf']
   shot=b.call('WebDriver:TakeScreenshot',{'id':key,'full':False,'scroll':True})
   (out/'sda-connection.png').write_bytes(base64.b64decode(shot['value']))
   b.script("const s=document.querySelector('#harness-display-mode');s.value='corridors';s.dispatchEvent(new Event('change'))")
   assert b.script("return JSON.parse(window.wrappedJSObject.harnessPerformance()).visible_corridors>0")
   assert b.script("return JSON.parse(window.wrappedJSObject.harnessPerformance()).visible_guides===0")
   b.script("document.querySelector('[data-view=layout]').click()")
   b.wait("document.querySelector('#layout-view').textContent.includes('removable access cover')")
   b.script("[...document.querySelectorAll('#layout-view button')].find(b=>b.textContent==='Show carrier attachment').click()")
   checks['attachment']=json.loads(b.script("return window.wrappedJSObject.layoutPerformance()"))
   assert 'body_lid' in checks['attachment']['visible_part_ids'] and 'electronics_case_0_carrier' in checks['attachment']['visible_part_ids']
   b.screenshot(str(out/'carrier-attachment.png'))
   b.script("document.querySelector('[data-view=harness]').click();const s=document.querySelector('#harness-wire-select');s.value='wire_8';s.dispatchEvent(new Event('change'))")
   b.wait("document.querySelector('#harness-connection-detail').textContent.includes('Outward lead blocked by: Distribution module carrier')")
   checks['blocked_carrier_visible']=True
   b.call('WebDriver:SetWindowRect',{'width':430,'height':920});b.script("document.querySelector('[data-view=harness]').click()")
   b.screenshot(str(out/'mobile.png'))
   checks['mobile']=b.script("return {width:innerWidth,scroll:document.documentElement.scrollWidth}")
   assert checks['mobile']['scroll']<=checks['mobile']['width']+1,checks['mobile']
   report={'passed':True,**checks}
  finally:
   if b:
    try:b.call('WebDriver:DeleteSession',{})
    except Exception:pass
    b.sock.close()
   process.terminate()
   try:process.wait(timeout=10)
   except subprocess.TimeoutExpired:process.kill();process.wait()
(out/'browser-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
