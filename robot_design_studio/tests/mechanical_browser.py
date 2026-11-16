"""Real UI workflow and 3D action preview, on dedicated QA ports."""
import json
import base64
import time
from pathlib import Path
from tests.browser_smoke import Browser

b=Browser(2829);b.call('WebDriver:SetWindowRect',{'width':1440,'height':1050})
b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8766/'})
b.wait("document.querySelectorAll('.part-card').length===16")
b.script("document.querySelector('#project-name').value='机械界面验收';document.querySelector('#create-form').requestSubmit();")
b.wait("!document.querySelector('#workspace').hidden")
b.script("Array.from(document.querySelectorAll('.part-card')).find(n=>n.textContent.includes('SG90 Digital')).querySelector('button').click();document.querySelector('#save').click();")
b.wait("!document.querySelector('#wiring-view').hidden")
b.script("document.querySelector('[data-view=mechanics]').click();document.querySelector('#example-shape').click();")
b.wait("document.querySelector('#shape-name').textContent.includes('简单外观')")
b.script("document.querySelector('#mechanical-generate').click()")
b.wait("document.querySelector('#mechanical-summary').textContent.includes('工程试件已生成')")
b.wait("document.querySelectorAll('#mechanical-parts button').length===9")
error=b.script("return document.querySelector('#viewer-error').textContent")
b.screenshot('evidence/mechanical-browser-first.png')
assert not error,error
b.script("document.querySelector('#motion-play').click()")
time.sleep(3)
performance=json.loads(b.script("return window.wrappedJSObject.mechanicalPerformance()"))
assert b.script("return document.querySelector('#motion-play').textContent.includes('暂停')")
b.script("document.querySelector('#motion-play').click();document.querySelector('#explode').click();document.querySelectorAll('#mechanical-parts button')[1].click();")
b.screenshot('evidence/mechanical-browser-exploded.png')
assert b.script("return document.querySelector('#part-detail').textContent.includes('运动侧')")
b.script("document.querySelector('#shape-length').value='280';document.querySelector('#motion-angle').value='15';document.querySelector('#mechanical-generate').click();")
b.wait("document.querySelector('#mechanical-generate').disabled")
b.wait("!document.querySelector('#mechanical-generate').disabled")
assert b.script("return document.querySelector('#mechanical-summary').textContent.includes('工程试件已生成')")
project_id=b.script("return document.querySelector('#project-list').value")
b.call('WebDriver:Refresh',{})
b.wait("document.querySelectorAll('#project-list option').length>1")
b.script("const s=document.querySelector('#project-list');s.value="+json.dumps(project_id)+";s.dispatchEvent(new Event('change'));")
b.wait("!document.querySelector('#workspace').hidden")
b.script("document.querySelector('[data-view=mechanics]').click()")
b.wait("document.querySelectorAll('#mechanical-parts button').length===9")
b.screenshot('evidence/mechanical-browser-restored.png')
element=b.call('WebDriver:FindElement',{'using':'css selector','value':'#mechanical-canvas'})['value']
element_id=next(iter(element.values()))
shot=b.call('WebDriver:TakeScreenshot',{'id':element_id,'full':False,'scroll':True})['value']
Path('evidence/mechanical-preview.png').write_bytes(base64.b64decode(shot))
b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
assert b.script("return document.documentElement.scrollWidth<=window.innerWidth")
b.screenshot('evidence/mechanical-browser-mobile.png')
Path('evidence/mechanical-browser.json').write_text(json.dumps({'status':'passed','project_id':project_id,'performance':performance,'actions':['create','select SG90','source import','generate solids','motion preview','explode','select moving solid','change size and angle','regenerate','reload persisted assembly','mobile overflow'],'physical_tested':False},indent=2))
print('Mechanical browser workflow passed')
b.call('WebDriver:DeleteSession',{})
