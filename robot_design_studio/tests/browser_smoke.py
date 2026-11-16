"""Opt-in Firefox Marionette smoke test; needs disposable profile and QA server."""
import base64
import json
import socket
import time
from pathlib import Path

class Browser:
    def __init__(self,port=2828):
        self.sock=socket.create_connection(('127.0.0.1',port),timeout=10);self.index=0
        self.read()
        try:self.call('WebDriver:DeleteSession',{})
        except RuntimeError:pass
        self.call('WebDriver:NewSession',{'capabilities':{'alwaysMatch':{'acceptInsecureCerts':True}}})
    def read(self):
        length=b''
        while not length.endswith(b':'):length+=self.sock.recv(1)
        left=int(length[:-1]);data=b''
        while left:
            block=self.sock.recv(left);data+=block;left-=len(block)
        return json.loads(data)
    def call(self,name,params):
        self.index+=1;raw=json.dumps([0,self.index,name,params]).encode();self.sock.sendall(str(len(raw)).encode()+b':'+raw)
        response=self.read()
        if response[2]:raise RuntimeError(response[2])
        return response[3]
    def script(self,script):return self.call('WebDriver:ExecuteScript',{'script':script,'args':[],'newSandbox':False,'sandbox':'default','line':1,'filename':'browser_smoke.py'})['value']
    def wait(self,condition):
        for _ in range(100):
            if self.script('return '+condition):return
            time.sleep(.1)
        raise AssertionError('Browser condition timed out: '+condition)
    def screenshot(self,path):
        r=self.call('WebDriver:TakeScreenshot',{'id':None,'full':True,'scroll':True})
        Path(path).write_bytes(base64.b64decode(r['value']))

if __name__=='__main__':
    b=Browser();b.call('WebDriver:SetWindowRect',{'width':1440,'height':1000})
    b.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8765/'})
    b.wait("document.querySelectorAll('.part-card').length === 16")
    b.script("document.querySelector('#project-name').value='浏览器验证 中文 项目';document.querySelector('#create-form').requestSubmit();")
    b.wait("!document.querySelector('#workspace').hidden")
    b.script("document.querySelector('#starter').click()")
    b.wait("document.querySelector('#selected-count').textContent === '2'")
    b.script("Array.from(document.querySelectorAll('.part-card')).find(n=>n.textContent.includes('SG90 Digital')).querySelector('button').click()")
    b.wait("document.querySelector('#selected-count').textContent === '3'")
    b.screenshot('evidence/browser-parts.png')
    b.script("document.querySelector('#save').click()")
    b.wait("!document.querySelector('#wiring-view').hidden")
    b.script("document.querySelector('#generate').click()")
    b.wait("document.querySelector('#run-state').textContent.includes('Wiring saved')")
    assert b.script("return document.querySelectorAll('#net-table tr').length") == 5
    assert b.script("return document.querySelector('#source-badge').textContent") == 'Local rules'
    assert b.script("return document.querySelector('#requests').textContent.includes('4.8 V')")
    b.screenshot('evidence/browser-wiring.png')
    b.script("document.querySelector('#requests button').click()")
    b.wait("!document.querySelector('#parts-view').hidden")
    assert b.script("return document.querySelector('#selected-count').textContent")=='4'
    b.script("document.querySelector('#save').click()")
    b.wait("document.querySelector('#wiring-result').hidden")
    b.script("document.querySelector('#generate').click()")
    b.wait("document.querySelector('#run-state').textContent.includes('Wiring saved')")
    project_id=b.script("return document.querySelector('#project-list').value")
    b.call('WebDriver:Refresh',{})
    b.wait("document.querySelectorAll('#project-list option').length>1")
    b.script("const list=document.querySelector('#project-list');list.value="+json.dumps(project_id)+";list.dispatchEvent(new Event('change'));")
    b.wait("document.querySelector('#selected-count').textContent === '4'")
    b.script("document.querySelector('[data-view=wiring]').click()")
    assert b.script("return !document.querySelector('#wiring-result').hidden")
    b.call('WebDriver:SetWindowRect',{'width':430,'height':900})
    b.screenshot('evidence/browser-mobile.png')
    assert b.script("return document.documentElement.scrollWidth<=window.innerWidth")
    Path('evidence/browser-smoke.json').write_text(json.dumps({'status':'passed','browser':'Firefox 136.0 headless','actions':['create Chinese project','select Pico+MPU6050+SG90','save','generate reference circuit','show requests and checks','add request explicitly','invalidate on change','regenerate','reload persisted project','430px overflow check'],'live_ai_tested':False,'physical_tested':False},indent=2))
    print('Browser smoke passed: two-layer workflow, requests, invalidation, reload, responsive layout')
    b.call('WebDriver:DeleteSession',{})
