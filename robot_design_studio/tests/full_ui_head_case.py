"""Opt-in real UI test. Sends rendered model views to configured VLM and parts to AI.
Run only with authorization for those online calls. Creates a new project; no old
project is modified. Every generation action is a page button, not a direct POST.
"""
import json,time,tempfile,subprocess,urllib.request,base64,os
from pathlib import Path
from tests.browser_smoke import Browser
ROOT=Path(__file__).resolve().parents[1]
OUT=Path(os.environ.get('ROBOT_DEMO_OUTPUT',str(ROOT/'evidence/full_ui_head_case_2026-11-16')));OUT.mkdir(parents=True,exist_ok=True)
BASE='http://127.0.0.1:8768/'
SOURCE=Path(os.environ.get('ROBOT_DEMO_SOURCE','/home/qinxuan/.codex/attachments/9009dad3-b1d5-4866-a4fa-b868ba74e3dc/tripo_convert_3ddc0a25-403d-4ab7-9861-1d640434ba37.stl')).resolve()
def get(path):
 with urllib.request.urlopen(BASE+'api/'+path,timeout=40) as response:return json.load(response)
def save(name,data):(OUT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2))
recording=None
record_mode=os.environ.get('ROBOT_DEMO_RECORD')=='1'
expected_joints=int(os.environ.get('ROBOT_DEMO_JOINTS','1'))
report={'generation_via':'actual DOM button clicks','online_calls':True,'actions':[]}
with tempfile.TemporaryDirectory(prefix='full-ui-head-') as temp:
 profile=Path(temp);(profile/'user.js').write_text('user_pref("marionette.port", 2868);\n')
 with (OUT/'firefox.log').open('w') as log:
  process=subprocess.Popen(['/usr/bin/firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=log,stderr=log);b=None
  try:
   for _ in range(60):
    try:b=Browser(2868);break
    except (OSError,RuntimeError):time.sleep(.5)
   assert b
   def js(code):return b.script(code)
   def pause(seconds=1.2):
    if record_mode:time.sleep(seconds)
   def click(selector):
    if record_mode:
     js('document.querySelector('+json.dumps(selector)+').scrollIntoView({block:"center"})');pause(.6)
    js('document.querySelector('+json.dumps(selector)+').click()')
    if selector.startswith('[data-view='):js('window.scrollTo(0,0)')
    pause()

   def button(view,text):
    js('const e=Array.from(document.querySelectorAll('+json.dumps(view+' button')+')).find(n=>n.textContent==='+json.dumps(text)+');e.scrollIntoView({block:"center"})');pause(.6)
    js('Array.from(document.querySelectorAll('+json.dumps(view+' button')+')).find(n=>n.textContent==='+json.dumps(text)+').click()');pause()

   def field(selector,value):
    js('const e=document.querySelector('+json.dumps(selector)+');e.scrollIntoView({block:"center"});e.value='+json.dumps(value)+';e.dispatchEvent(new Event("input"));e.dispatchEvent(new Event("change"))');pause(.6)

   def wait(condition,timeout=240):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
     if js('return '+condition):return
     if report.get('project_id'):
      current=get('projects/'+report['project_id'])
      if current.get('runs') and not current.get('active_run_id'):
       last=get('runs/'+current['runs'][-1])
       if last['status'] in ['rejected','failed','timed_out']:
        raise AssertionError('Task '+last['id']+' '+last['status']+': '+str(last.get('error') or (last.get('result') or {}).get('error') or 'Check details'))
     time.sleep(.5)
    raise AssertionError('UI timed out: '+condition+' | '+str(js('return document.querySelector("#message").textContent')))
   def stage(label,condition,timeout=240):
    print(label,flush=True)
    if recording:recording.mark(label+' started')
    wait(condition,timeout)
    if recording:recording.mark(label+' complete')
    pause(2)
    p=get('projects/'+report['project_id']);save(label+'-project.json',p)
    if p.get('active_run_id'):raise AssertionError('Still active '+p['active_run_id'])
    report['actions'].append(label);save('progress.json',report);b.screenshot(str(OUT/(label+'.png')))
    return p
   b.call('WebDriver:SetWindowRect',{'width':1440,'height':1050});b.call('WebDriver:Navigate',{'url':BASE})
   wait('document.querySelectorAll("#project-list option").length>1')
   if record_mode:
    from tests.live_workflow_recording import WorkflowRecording
    recording=WorkflowRecording(b,OUT);recording.start();pause(2)
   resume=os.environ.get('ROBOT_DEMO_PROJECT')
   if resume:
    report['project_id']=resume
    field('#project-list',resume)
    wait('document.querySelector("#articulation-view").textContent.includes("Joints ready")')
    p=get('projects/'+resume);save('01-annotation-project.json',p)
    report['actions'].append('01-annotation retained from this same fresh case; initial failed structure preserved')
   else:
    field('#project-name',os.environ.get('ROBOT_DEMO_NAME','UI test · head only · 500 mm'));js('document.querySelector("#create-form").requestSubmit()')
    wait('!document.querySelector("#workspace").hidden');report['project_id']=js('return document.querySelector("#project-list").value');save('progress.json',report)
    pause(2)
    if recording:recording.mark('Upload STL')
    element=b.call('WebDriver:FindElement',{'using':'css selector','value':'#art-file'})['value']
    eid=element['element-6066-11e4-a52e-4f735466cecf']
    b.call('WebDriver:ElementSendKeys',{'id':eid,'text':str(SOURCE),'value':list(str(SOURCE))})
    wait('document.querySelector("#art-source").textContent.includes('+json.dumps(SOURCE.name)+')')
    field('#art-length',os.environ.get('ROBOT_DEMO_LENGTH','500'));field('#art-max',str(expected_joints));field('#art-intent',Path(os.environ['ROBOT_DEMO_MOTION_FILE']).read_text() if os.environ.get('ROBOT_DEMO_MOTION_FILE') else os.environ.get('ROBOT_DEMO_MOTION','Only the entire head turns gently left and right around a vertical neck axis, +/-10 degrees. Keep the body, all four legs, tail and both ears fixed to their corresponding body/head. No other joints.'));field('#art-mode','live')
    button('#articulation-view','Identify joints');p=stage('01-annotation','document.querySelector("#articulation-view").textContent.includes("Joints ready")')
    assert len(p['articulation']['graph']['joints'])==expected_joints, 'The requested joint count was not generated'
   motor_count=js('return document.querySelectorAll("#articulation-view article select").length')
   assert motor_count==expected_joints
   for index in range(motor_count):
    js('const e=document.querySelectorAll("#articulation-view article select")['+str(index)+'];e.scrollIntoView({block:"center"});e.value="sg90_digital_towerpro";e.dispatchEvent(new Event("change"))');pause()
   button('#articulation-view','Save motors');wait('document.querySelector("#message").textContent.includes("Motors saved")')
   button('#articulation-view','Generate structure');p=stage('02-initial-structure','document.querySelector("#mechanical-summary").textContent.includes("passed")',240+90*(expected_joints-1))
   assert p['mechanical_result']['geometry_valid']
   report['initial_mechanical_run_id']=p['mechanical_result']['run_id']
   if os.environ.get('ROBOT_DEMO_STOP_AFTER_STRUCTURE')=='1':
    report.update(passed=True,scope='upload_to_validated_structure_motion_preview',full_workflow_passed=False,revision=p['revision'])
    save('project.json',p)
    click('[data-view=mechanics]');wait('document.querySelector("#mechanical-summary").textContent.includes("passed")')
    field('#mechanical-view-mode','exterior')
    if js('return document.querySelector("#hide-lids").checked'):click('#hide-lids')
    js('document.querySelector("#mechanical-canvas").scrollIntoView({block:"center"});document.querySelector("#mechanical-canvas").dispatchEvent(new WheelEvent("wheel",{deltaY:-350,cancelable:true}))')
    if recording:recording.mark('Three-joint structural motion preview; electronics workflow not completed')
    click('#motion-play');pause(15)
    b.screenshot(str(OUT/'structure-motion-preview.png'))
    field('#mechanical-view-mode','actuator');js('document.querySelector("#mechanical-canvas").scrollIntoView({block:"center"})');pause(10)
    field('#mechanical-view-mode','internal');pause(6)
    field('#mechanical-view-mode','exterior');pause(8)
    save('verification.json',report)
    print('STRUCTURE_PREVIEW_PASSED',p['id'],flush=True)
    raise SystemExit(0)
   click('[data-view=parts]');click('#test-electronics') if record_mode else click('#starter')
   for title in ([] if record_mode else ['5 V supply','4.8 V supply','Distribution module']):
    js('Array.from(document.querySelectorAll("#catalog article")).find(n=>n.querySelector("h3").textContent==='+json.dumps(title)+').querySelector("button").click()')
   for _ in range(0 if record_mode else 12):js('Array.from(document.querySelectorAll("#catalog article")).find(n=>n.querySelector("h3").textContent==="Wire").querySelector("button").click()')
   if expected_joints>1:
    for _ in range(20):
     js('const card=Array.from(document.querySelectorAll("#catalog article")).find(n=>n.querySelector("h3").textContent==="Wire");card.scrollIntoView({block:"center"});card.querySelector("button").click()')
    pause(2)
   field('#intent',os.environ.get('ROBOT_DEMO_REQUIREMENTS','A fixed-base robot with '+str(expected_joints)+' joints and internal sensing electronics.'));click('#save');wait('!document.querySelector("#wiring-view").hidden');field('#mode','live');click('#generate')
   p=stage('03-online-wiring','document.querySelector("#result-title").textContent==="Wiring checks passed"')
   oldnets=p['circuit']['nets'];report['wiring_run_id']=next((rid for rid in reversed(p['runs']) if get('runs/'+rid)['kind']=='wiring'),None)
   click('[data-view=layout]');wait('document.querySelectorAll("#layout-view input[type=number]").length===15');click('#layout-example-sizes');click('#layout-generate')
   p=stage('04-access-and-layout','document.querySelector("#layout-view").textContent.includes("Layout ready")',360+180*(expected_joints-1))
   assert p['layout_result']['layout_valid'];assert p['circuit']['nets']==oldnets
   assert p['mechanical_result']['run_id']!=report['initial_mechanical_run_id']
   assert any(e['type']=='electrical_revalidated_for_access' for e in p['events'])
   report['feedback_mechanical_run_id']=p['mechanical_result']['run_id']
   click('#layout-open-covers');time.sleep(1);b.screenshot(str(OUT/'04-access-openings.png'))
   canvas=js('return document.querySelector("#layout-view canvas").toDataURL("image/png")');(OUT/'access-openings-preview.png').write_bytes(base64.b64decode(canvas.split(',',1)[1]))
   click('[data-view=harness]')
   # Refresh newly enabled downstream controls through the ordinary saved-project selector.
   js('const s=document.querySelector("#project-list");s.dispatchEvent(new Event("change"))')
   wait('!document.querySelector("#harness-view > button").disabled');button('#harness-view','Route bundles')
   p=stage('05-bundles','document.querySelector("#harness-view").textContent.includes("Routes ready")')
   assert p['harness_result']['routing_valid']
   click('[data-view=simulation]');button('#simulation-view','Export models');p=stage('06-export','document.querySelector("#simulation-view").textContent.includes("Export ready")')
   assert p['simulation_result']['export_valid']
   report['revision']=p['revision'];report['passed']=True
   report['checks']={key:get('runs/'+p[key]['run_id']) for key in ['mechanical_result','layout_result','harness_result','simulation_result']}
   save('project.json',p);save('verification.json',report)
   with urllib.request.urlopen(BASE+'api/projects/'+p['id']+'/export',timeout=60) as response:(OUT/'ui-head-case-review.zip').write_bytes(response.read())
   if recording:
    recording.mark('Final proposal preview')
    click('[data-view=mechanics]');wait('document.querySelector("#mechanical-summary").textContent.includes("passed")');pause(2)
    field('#mechanical-view-mode','exterior')
    if js('return document.querySelector("#hide-lids").checked'):click('#hide-lids')
    js('document.querySelector("#mechanical-canvas").scrollIntoView({block:"center"});document.querySelector("#mechanical-canvas").dispatchEvent(new WheelEvent("wheel",{deltaY:-450,cancelable:true}))')
    click('#motion-play');pause(10)
    field('#mechanical-view-mode','actuator');js('document.querySelector("#mechanical-canvas").scrollIntoView({block:"center"})');pause(8)
    click('[data-view=layout]');wait('document.querySelector("#layout-view canvas")!==null');js('document.querySelector("#layout-view canvas").scrollIntoView({block:"center"})');pause(5)
    click('#layout-open-covers');pause(4)
    click('[data-view=harness]');wait('document.querySelector("#harness-view canvas")!==null');js('document.querySelector("#harness-view canvas").scrollIntoView({block:"center"})');pause(5)
    click('[data-view=mechanics]');field('#mechanical-view-mode','exterior');js('document.querySelector("#mechanical-canvas").scrollIntoView({block:"center"})');pause(6)
   print('PASSED',p['id'],'revision',p['revision'],flush=True)
  except Exception as error:
   report['passed']=False;report['error']=str(error);save('verification.json',report)
   if b:
    try:
     if recording:
      recording.mark('Workflow stopped: '+str(error))
      current=get('projects/'+report['project_id']);save('failed-project.json',current)
      if current.get('runs'):
       failed=get('runs/'+current['runs'][-1]);save('failed-task.json',failed)
      js('window.scrollTo(0,0)');pause(5)
      if current.get('articulation'):
       click('[data-view=articulation]');js('for(const e of document.querySelectorAll("#articulation-view details"))e.open=true;document.querySelector("#articulation-view h3").scrollIntoView({block:"start"})');pause(8)
     b.screenshot(str(OUT/'failure.png'));save('failure-dom.json',{'text':js('return document.body.innerText')})
    except Exception:pass
   raise
  finally:
   recording_error=None
   if recording:
    try:recording.stop(report.get('passed',False))
    except Exception as error:recording_error=error
   if b:
    try:b.call('WebDriver:DeleteSession',{})
    except Exception:pass
    b.sock.close()
   process.terminate()
   try:process.wait(timeout=10)
   except subprocess.TimeoutExpired:process.kill();process.wait()
   if recording_error:raise recording_error
