"""Inspect the completed isolated workflow in a disposable Firefox session."""
import argparse,base64,json,subprocess,tempfile,time
from pathlib import Path
from tests.browser_smoke import Browser


def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--url',default='http://127.0.0.1:8770/');args=parser.parse_args()
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
 with tempfile.TemporaryDirectory(prefix='open-cradle-full-browser-') as temp:
  profile=Path(temp);(profile/'user.js').write_text('user_pref("marionette.port", 2874);\n')
  with (out/'firefox.log').open('w') as log:
   process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log);b=None
   try:
    for _ in range(60):
     try:b=Browser(2874);break
     except (OSError,RuntimeError):time.sleep(.5)
    assert b,'Firefox did not start'
    b.call('WebDriver:SetWindowRect',{'width':1440,'height':1050});b.call('WebDriver:Navigate',{'url':args.url})
    b.wait("document.querySelectorAll('#project-list option').length>1")
    b.script("const s=document.querySelector('#project-list');s.value='project_95d1e73d971c4ba1';s.dispatchEvent(new Event('change'))")
    b.wait("document.querySelector('#project-title').textContent.includes('Full test')")
    b.wait("document.querySelector('#mechanical-summary').textContent.includes('passed')")
    b.wait("document.querySelector('#layout-view').textContent.includes('Layout ready')")
    report={'url':args.url,'passed':False,'views':{},'project_modified':False}
    for view in ['articulation','mechanics','parts','wiring','layout','harness','simulation']:
     b.script("document.querySelector('[data-view="+view+"]').click();window.scrollTo(0,0)")
     b.wait("!document.querySelector('#"+view+"-view').hidden");time.sleep(.5)
     visible=b.script("return document.querySelector('#"+view+"-view').innerText")
     report['views'][view]={'visible_text':visible}
     if view=='mechanics':
      assert 'passed' in visible
      b.script("document.querySelector('#mechanical-view-mode').value='actuator';document.querySelector('#mechanical-view-mode').dispatchEvent(new Event('change'))")
     if view=='wiring':assert 'Wiring checks passed' in visible
     if view=='layout':assert '2 reference models' in visible
     if view=='harness':assert 'Routes ready' in visible and 'shared segments' in visible
     if view=='simulation':assert 'Export ready' in visible
     b.screenshot(str(out/(view+'.png')))
     if view in ['layout','harness']:
      data=b.script("return document.querySelector('#"+view+"-view canvas').toDataURL('image/png')")
      (out/(view+'-preview.png')).write_bytes(base64.b64decode(data.split(',',1)[1]))
    b.script("document.querySelector('[data-view=mechanics]').click();document.querySelector('#mechanical-view-mode').value='exterior';document.querySelector('#mechanical-view-mode').dispatchEvent(new Event('change'));document.querySelector('#motion-play').click()")
    time.sleep(1.5);b.screenshot(str(out/'motion.png'))
    b.script("document.querySelector('#motion-play').click()")
    b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
    for view in ['mechanics','layout','harness','simulation']:
     b.script("document.querySelector('[data-view="+view+"]').click()");time.sleep(.2)
     assert b.script('return document.documentElement.scrollWidth<=window.innerWidth'),view
    report.update(passed=True,responsive_width=430,motion_preview_started=True)
    (out/'browser-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('PASS seven views, motor preview, layout, bundles, model exports and mobile layout',flush=True)
   except Exception:
    if b:
     try:
      (out/'failure-dom.txt').write_text(b.script('return document.body.innerText'))
      b.screenshot(str(out/'failure.png'))
     except Exception:pass
    raise
   finally:
    if b:
     try:b.call('WebDriver:DeleteSession',{})
     except Exception:pass
     b.sock.close()
    process.terminate()
    try:process.wait(timeout=10)
    except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
