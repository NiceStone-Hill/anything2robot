"""Opt-in read-only browser verification of generated registered mounting geometry."""
import json,tempfile,subprocess,time,base64
from pathlib import Path
from tests.browser_smoke import Browser
OUT=Path('evidence/registered_mounts_2026-11-16');OUT.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix='registered-mount-browser-') as temp:
 profile=Path(temp);(profile/'user.js').write_text('user_pref("marionette.port", 2870);\n')
 with (OUT/'firefox.log').open('w') as log:
  process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log);b=None
  try:
   for _ in range(60):
    try:b=Browser(2870);break
    except (OSError,RuntimeError):time.sleep(.5)
   assert b;b.call('WebDriver:SetWindowRect',{'width':1440,'height':1000});b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8768/'})
   b.wait('document.querySelectorAll("#project-list option").length>1')
   b.script('const s=document.querySelector("#project-list");s.value="project_95d1e73d971c4ba1";s.dispatchEvent(new Event("change"))')
   b.wait('document.querySelector("#layout-view").textContent.includes("Layout ready")')
   b.script('document.querySelector("[data-view=layout]").click()')
   b.wait('Array.from(document.querySelectorAll("#layout-view a")).some(a=>a.textContent==="PCB mounting details")')
   selector=json.dumps('#layout-view select[aria-label="Focus component"]')
   for identity in ['case_0','case_1']:
    b.script('const s=document.querySelector('+selector+');s.value='+json.dumps(identity)+';s.dispatchEvent(new Event("change"))')
    time.sleep(.5);b.screenshot(str(OUT/(identity+'-browser.png')))
    data=b.script('return document.querySelector("#layout-view canvas").toDataURL("image/png")');(OUT/(identity+'-preview.png')).write_bytes(base64.b64decode(data.split(',',1)[1]))
   b.script('document.querySelector("#layout-open-covers").click()')
   assert b.script('return document.querySelector('+selector+').value')==''
   b.screenshot(str(OUT/'openings-browser.png'))
   b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
   for view in ['layout','harness','simulation']:
    b.script('document.querySelector("[data-view='+view+']").click()');time.sleep(.2)
    assert b.script('return document.documentElement.scrollWidth<=innerWidth'),view
   (OUT/'browser-verification.json').write_text(json.dumps({'passed':True,'project_id':'project_95d1e73d971c4ba1','actions':['reopen saved layout','focus PCB with matching carrier and screw solids','open access covers','inspect PCB and cover report links','430px layout'],'project_modified':False},indent=2))
   print('Registered mount preview and navigation passed')
  finally:
   if b:
    try:b.call('WebDriver:DeleteSession',{})
    except Exception:pass
    b.sock.close()
   process.terminate()
   try:process.wait(timeout=10)
   except subprocess.TimeoutExpired:process.kill();process.wait()
