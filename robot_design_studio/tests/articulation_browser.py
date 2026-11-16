"""Disposable Firefox UI test against the formal application, no external assets."""
import json
import os
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from studio.server import make_server
from tests.browser_smoke import Browser


def main():
    with tempfile.TemporaryDirectory(prefix='studio-articulation-browser-') as temp:
        root=Path(temp);profile=root/'profile';profile.mkdir()
        (profile/'user.js').write_text('user_pref("marionette.port", 2831);\nuser_pref("browser.shell.checkDefaultBrowser", false);\n')
        server=make_server(root/'workspace',0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        with (root/'firefox.log').open('w') as log:
            process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log)
            b=None
            try:
                for _ in range(60):
                    try:b=Browser(2831);break
                    except (OSError,RuntimeError):time.sleep(.5)
                if b is None:raise RuntimeError('Firefox 未启动')
                b.call('WebDriver:SetWindowRect',{'width':1440,'height':1050})
                b.call('WebDriver:Navigate',{'url':f'http://127.0.0.1:{server.server_port}/'})
                b.wait("document.querySelector('#art-mode')!==null && document.querySelectorAll('#catalog .part-card').length===16")
                b.script("document.querySelector('#create-form').requestSubmit()")
                b.wait("!document.querySelector('#workspace').hidden")
                assert b.script("return !document.querySelector('#articulation-view').hidden")
                assert b.script("return document.querySelector('#selected-count').textContent")=='0'
                b.script("document.querySelector('#articulation-view button').click()")
                b.wait("document.querySelector('#art-source').textContent.includes('简单外观')")
                b.script("document.querySelector('#art-max').value=2;document.querySelector('#art-mode').value='reference';Array.from(document.querySelectorAll('#articulation-view button')).find(b=>b.textContent==='识别关节并检查').click()")
                b.wait("document.querySelectorAll('.annotation-views img').length===3")
                assert b.script("return document.querySelector('#selected-count').textContent")=='0'
                b.screenshot('evidence/articulation-browser-annotation.png')
                b.script("Array.from(document.querySelectorAll('#articulation-view button')).find(b=>b.textContent==='保存关节电机选择').click()")
                b.wait("document.querySelector('#selected-count').textContent==='2'")
                b.script("document.querySelector('[data-view=mechanics]').click();document.querySelector('#graph-assembly-generate').click()")
                for _ in range(300):
                    if b.script("return document.querySelector('#mechanical-result').hidden===false"):break
                    time.sleep(.2)
                assert b.script("return !document.querySelector('#mechanical-result').hidden")
                b.wait("document.querySelectorAll('#mechanical-parts button').length>5")
                assert not b.script("return document.querySelector('#viewer-error').textContent")
                b.script("document.querySelector('#motion-slider').value=80;document.querySelector('#motion-slider').dispatchEvent(new Event('input'))")
                b.screenshot('evidence/articulation-browser-structure.png')
                performance=json.loads(b.script("return window.wrappedJSObject.mechanicalPerformance()"))
                identity=b.script("return document.querySelector('#project-list').value")
                # Evaluate a simple parent/child pose independently in the real JS engine.
                point=b.script("return window.wrappedJSObject.posePoint(window.wrappedJSObject.axisPose([1,0,0],[0,0,0],Math.PI/2),[0,1,0])")
                assert abs(point[1])<1e-8 and abs(point[2]-1)<1e-8,point
                b.call('WebDriver:Refresh',{});b.wait("document.querySelectorAll('#project-list option').length>1")
                b.script("const s=document.querySelector('#project-list');s.value="+json.dumps(identity)+";s.dispatchEvent(new Event('change'))")
                b.wait("document.querySelectorAll('.annotation-views img').length===3")
                b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
                assert b.script("return document.documentElement.scrollWidth<=window.innerWidth")
                b.screenshot('evidence/articulation-browser-mobile.png')
                Path('evidence/articulation-browser.json').write_text(json.dumps({'passed':True,'browser':'Firefox headless','performance':performance,
                    'actions':['empty project','import','reference graph','choose motors afterwards','generate candidate assembly','graph pose slider','reload','430px overflow'],
                    'manufacturing_validated':False},indent=2))
                print('Articulation browser passed',performance)
            finally:
                if b:
                    try:b.call('WebDriver:DeleteSession',{})
                    except Exception:pass
                    b.sock.close()
                process.terminate()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait()
                server.jobs.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':main()
