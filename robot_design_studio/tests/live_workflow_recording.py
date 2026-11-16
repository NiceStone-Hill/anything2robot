"""Record the same browser that operates the full workflow, at real elapsed time."""
import base64,io,json,os,subprocess,threading,time
from pathlib import Path
from PIL import Image
class WorkflowRecording:
 def __init__(self,browser,directory):
  self.browser=browser;self.directory=Path(directory);self.stop_event=threading.Event();self.error=None;self.frames=0;self.fps=8;self.events=[]
  lock=threading.RLock();original=browser.call
  def serial(name,params):
   with lock:return original(name,params)
  browser.call=serial
 def screenshot(self):
  raw=self.browser.call('WebDriver:TakeScreenshot',{'id':None,'full':False,'scroll':False})['value']
  return Image.open(io.BytesIO(base64.b64decode(raw))).convert('RGB')
 def start(self):
  image=self.screenshot();self.size=image.size;assert all(n%2==0 for n in self.size)
  self.log=(self.directory/'video-encoder.log').open('w')
  self.encoder=subprocess.Popen([os.environ['ROBOT_DEMO_FFMPEG'],'-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{self.size[0]}x{self.size[1]}','-r',str(self.fps),'-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(self.directory/'upload-to-preview.mp4')],stdin=subprocess.PIPE,stderr=self.log)
  self.started=time.monotonic();self.thread=threading.Thread(target=self.capture,daemon=True);self.thread.start()
 def capture(self):
  try:
   while not self.stop_event.is_set():
    image=self.screenshot();assert image.size==self.size
    target=max(1,round((time.monotonic()-self.started)*self.fps));data=image.tobytes()
    while self.frames<target:self.encoder.stdin.write(data);self.frames+=1
    self.stop_event.wait(.3)
  except Exception as error:self.error=str(error)
 def mark(self,name):self.events.append({'at_s':round(time.monotonic()-self.started,3),'action':name})
 def stop(self,passed):
  self.stop_event.set();self.thread.join(timeout=30)
  if self.thread.is_alive():raise RuntimeError('Recorder did not stop')
  self.encoder.stdin.close();code=self.encoder.wait(timeout=90);self.log.close()
  (self.directory/'video-recording.json').write_text(json.dumps({'recording':'same live browser as the full workflow','duration_s':self.frames/self.fps,'dimensions':self.size,'fps':self.fps,'frames':self.frames,'cuts':False,'time_compression':False,'added_captions':False,'title_cards':False,'actions':self.events,'workflow_passed':passed,'encoder_exit':code,'error':self.error},indent=2))
  if code or self.error:raise RuntimeError(self.error or 'Video encoding failed')
