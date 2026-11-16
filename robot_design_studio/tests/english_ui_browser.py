import json,subprocess,tempfile,time,base64
from pathlib import Path
from tests.browser_smoke import Browser
out=Path('evidence/simple_english_ui');out.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory(prefix='english-ui-browser-') as temp:
 root=Path(temp);(root/'user.js').write_text('user_pref("marionette.port", 2867);\n')
 with (root/'firefox.log').open('w') as log:
  process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(root),'--marionette','about:blank'],stdout=log,stderr=log);b=None
  try:
   for _ in range(60):
    try:b=Browser(2867);break
    except (OSError,RuntimeError):time.sleep(.5)
   assert b
   b.call('WebDriver:SetWindowRect',{'width':1440,'height':1000})
   b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8768/'})
   b.wait("document.querySelectorAll('#project-list option').length>1")
   assert b.script("return document.documentElement.lang")=='en'
   assert '身体工坊' not in b.script('return document.body.innerText')
   b.screenshot(str(out/'welcome.png'))
   b.script("const s=document.querySelector('#project-list');s.value='project_95d1e73d971c4ba1';s.dispatchEvent(new Event('change'))")
   b.wait("document.querySelector('#project-title').textContent.includes('Example')")
   b.wait("document.querySelector('#mechanical-summary').textContent.includes('passed')")
   b.wait("document.querySelector('#layout-view').textContent.includes('Layout ready')")
   report={}
   for view in ['articulation','mechanics','parts','wiring','layout','harness','simulation']:
    b.script("document.querySelector('[data-view="+view+"]').click()")
    b.wait("!document.querySelector('#"+view+"-view').hidden")
    time.sleep(.5)
    text=b.script("return document.querySelector('#"+view+"-view').innerText")
    report[view]={'visible_text':text}
    if view=='layout':assert not b.script("return document.querySelector('#layout-view > details').open")
    if view=='harness':assert 'shared segments' in text or 'Individual wires' in text
    b.screenshot(str(out/(view+'.png')))
   # Verify controls still work, without submitting costly generation tasks.
   b.script("document.querySelector('[data-view=layout]').click();document.querySelector('#layout-view > details').open=true")
   assert b.script("return document.querySelectorAll('#layout-view input[type=number]').length")==15
   b.script("document.querySelector('#layout-view input[type=number]').value='';document.querySelector('#layout-view input[type=number]').dispatchEvent(new Event('input'));Array.from(document.querySelectorAll('#layout-view > button')).find(b=>b.textContent==='Generate layout').click()")
   b.wait("document.querySelector('#message').textContent.includes('missing')")
   assert b.script("return document.querySelector('#layout-view > details').open")
   b.script("document.querySelector('#layout-view input[type=number]').value='55';document.querySelector('#layout-view input[type=number]').dispatchEvent(new Event('input'));document.querySelector('#message').hidden=true")
   b.script("document.querySelector('[data-view=parts]').click()")
   before=int(b.script("return document.querySelector('#selected-count').textContent"))
   b.script("document.querySelector('#catalog .part-card button').click()")
   assert int(b.script("return document.querySelector('#selected-count').textContent"))==before+1
   b.script("document.querySelector('#selected .selected-row:last-child button:last-child').click()")
   b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
   for view in ['articulation','mechanics','parts','wiring','layout','harness','simulation']:
    b.script("document.querySelector('[data-view="+view+"]').click()")
    time.sleep(.15)
    assert b.script('return document.documentElement.scrollWidth<=window.innerWidth'),(view,b.script("return Array.from(document.querySelectorAll('body *')).filter(n=>n.getBoundingClientRect().right>innerWidth+1 && n.getBoundingClientRect().width>0).slice(0,10).map(n=>[n.tagName,n.id,n.className,n.getBoundingClientRect().right])"))
   b.screenshot(str(out/'mobile.png'))
   (out/'browser-verification.json').write_text(json.dumps({'passed':True,'views':report,'responsive_width':430,'model_records_modified':False,'actions':['open saved example','seven views','edit dimension validation','add component to unsaved draft','restore draft','mobile overflow']},ensure_ascii=False,indent=2))
   print('English UI browser checks passed')
  finally:
   if b:
    try:b.call('WebDriver:DeleteSession',{})
    except Exception:pass
    b.sock.close()
   process.terminate()
   try:process.wait(timeout=10)
   except subprocess.TimeoutExpired:process.kill();process.wait()
