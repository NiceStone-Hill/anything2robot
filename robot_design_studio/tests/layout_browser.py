"""Read the verified demo in disposable Firefox, including reload and narrow view."""
import json
import subprocess
import tempfile
import time
from pathlib import Path
from tests.browser_smoke import Browser


def main():
    with tempfile.TemporaryDirectory(prefix='studio-layout-browser-') as temp:
        root=Path(temp);(root/'user.js').write_text('user_pref("marionette.port", 2833);\n')
        with (root/'firefox.log').open('w') as log:
            process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(root),'--marionette','about:blank'],stdout=log,stderr=log)
            b=None
            try:
                for _ in range(60):
                    try:b=Browser(2833);break
                    except (OSError,RuntimeError):time.sleep(.5)
                assert b is not None
                b.call('WebDriver:SetWindowRect',{'width':1440,'height':1050})
                b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8768/'})
                b.wait("document.querySelectorAll('#project-list option').length>1")
                b.script("const s=document.querySelector('#project-list');s.selectedIndex=1;s.dispatchEvent(new Event('change'))")
                b.wait("document.querySelector('#layout-view').textContent.includes('内部布局候选已通过')")
                b.script("document.querySelector('[data-view=layout]').click()")
                assert b.script("return document.querySelectorAll('#layout-view input[type=number]').length")==6
                assert b.script("return Array.from(document.querySelectorAll('#layout-view input[type=number]')).map(i=>Number(i.value))")==[60,25,14,25,20,10]
                assert b.script("return document.querySelector('#layout-view canvas').width>1")
                b.screenshot('evidence/model-export-layout-browser.png')
                b.script("Array.from(document.querySelectorAll('#layout-view button')).find(b=>b.textContent==='查看内部件 / 显示壳体').click();document.querySelector('#layout-view canvas').scrollIntoView()")
                b.screenshot('evidence/model-export-layout-browser-interior.png')
                metrics=json.loads(b.script("return window.wrappedJSObject.layoutPerformance()"));assert metrics['triangles']>1000
                assert b.script("return document.querySelectorAll('#layout-view a').length")==3
                b.call('WebDriver:Refresh',{});b.wait("document.querySelectorAll('#project-list option').length>1")
                b.script("const s=document.querySelector('#project-list');s.selectedIndex=1;s.dispatchEvent(new Event('change'))")
                b.wait("document.querySelector('#layout-view').textContent.includes('内部布局候选已通过')")
                b.script("document.querySelector('[data-view=layout]').click()")
                b.script("document.querySelector('[data-view=simulation]').click();Array.from(document.querySelectorAll('#simulation-view button')).find(b=>b.textContent==='导出 URDF / MJCF 模型包').click()")
                b.wait("document.querySelector('#simulation-view').textContent.includes('模型包已生成 · 未运行仿真')")
                assert b.script("return document.querySelectorAll('#simulation-view a').length")==4
                b.screenshot('evidence/model-export-browser.png')
                b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
                assert b.script("return document.documentElement.scrollWidth<=window.innerWidth")
                b.screenshot('evidence/model-export-layout-browser-mobile.png')
                Path('evidence/model-export-layout-browser.json').write_text(json.dumps({'passed':True,'browser':'Firefox headless','performance':metrics,
                    'actions':['open verified project','step 05','editable engineering envelopes','full internal assembly preview','export links','reload','formal model export button without simulation','430px width'],
                    'manufacturing_verified':False},indent=2))
                print('Internal layout browser passed',metrics)
            finally:
                if b:
                    try:b.call('WebDriver:DeleteSession',{})
                    except Exception:pass
                    b.sock.close()
                process.terminate()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
