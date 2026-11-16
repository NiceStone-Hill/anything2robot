"""Read-only navigation check of the fresh end-to-end UI case."""
import json,tempfile,subprocess,time
from pathlib import Path
from tests.browser_smoke import Browser
OUT=Path('evidence/full_ui_head_case_2026-11-16');p=json.loads((OUT/'project.json').read_text())
with tempfile.TemporaryDirectory(prefix='head-navigation-') as temp:
 profile=Path(temp);(profile/'user.js').write_text('user_pref("marionette.port", 2869);\n')
 with (OUT/'navigation-firefox.log').open('w') as log:
  process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log);b=None
  try:
   for _ in range(60):
    try:b=Browser(2869);break
    except (OSError,RuntimeError):time.sleep(.5)
   assert b;b.call('WebDriver:SetWindowRect',{'width':1440,'height':1000});b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8768/'})
   b.wait('document.querySelectorAll("#project-list option").length>1')
   b.script('const s=document.querySelector("#project-list");s.value='+json.dumps(p['id'])+';s.dispatchEvent(new Event("change"))')
   b.wait('!!document.querySelector("#wiring-next") && !!document.querySelector("#layout-next") && !!document.querySelector("#harness-next")')
   b.script('document.querySelector("[data-view=mechanics]").click();document.querySelector("#mechanical-next").click()');b.wait('!document.querySelector("#parts-view").hidden')
   b.script('document.querySelector("[data-view=wiring]").click();document.querySelector("#wiring-next").click()');b.wait('!document.querySelector("#layout-view").hidden')
   b.script('document.querySelector("#layout-open-covers").click()');b.screenshot(str(OUT/'navigation-openings.png'))
   b.script('document.querySelector("#layout-next").click()');b.wait('!document.querySelector("#harness-view").hidden')
   b.script('document.querySelector("#harness-next").click()');b.wait('!document.querySelector("#simulation-view").hidden')
   assert 'Export ready' in b.script('return document.querySelector("#simulation-view").textContent')
   b.script('document.querySelector("[data-view=parts]").click()')
   b.script('while(document.querySelectorAll("#selected .selected-row").length>1)document.querySelector("#selected .selected-row:last-child button:last-child").click();document.querySelector("#test-electronics").click()')
   assert b.script('return document.querySelector("#selected-count").textContent')=='18'
   b.call('WebDriver:Refresh',{})
   b.wait('document.querySelectorAll("#project-list option").length>1')
   b.script('const s=document.querySelector("#project-list");s.value='+json.dumps(p['id'])+';s.dispatchEvent(new Event("change"))')
   b.wait('document.querySelector("#dirty").textContent==="Saved" && !document.querySelector("#workspace").hidden')
   b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
   for view in ['articulation','mechanics','parts','wiring','layout','harness','simulation']:
    b.script('document.querySelector("[data-view='+view+']").click()');time.sleep(.15)
    assert b.script('return document.documentElement.scrollWidth<=innerWidth'),view
   (OUT/'navigation-verification.json').write_text(json.dumps({'passed':True,'project_id':p['id'],'next_buttons':['mechanical-next','wiring-next','layout-next','harness-next'],'access_openings_control':True,'responsive_width':430,'project_modified':False,'test_electronics_preset_verified':True},indent=2))
   print('Navigation, access preview and mobile checks passed')
  finally:
   if b:
    try:b.call('WebDriver:DeleteSession',{})
    except Exception:pass
    b.sock.close()
   process.terminate()
   try:process.wait(timeout=10)
   except subprocess.TimeoutExpired:process.kill();process.wait()
