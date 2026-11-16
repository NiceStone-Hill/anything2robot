"""Host-only checks of the separate MicroPython physical bench helper."""
import contextlib,io,runpy,sys,types,unittest
from itertools import islice
from pathlib import Path
from unittest.mock import patch
SCRIPT=Path(__file__).resolve().parents[1]/'exports/friday_head_prototype/bench_head.py'

class BenchTests(unittest.TestCase):
    def test_sweep_starts_at_centre_and_never_exceeds_narrow_limits(self):
        code=runpy.run_path(str(SCRIPT));samples=list(islice(code['sweep_values'](),1000))
        self.assertEqual(samples[0],1500);self.assertEqual(min(samples),1420);self.assertEqual(max(samples),1580)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(samples,samples[1:])),2)
    def test_invalid_range_and_step_are_rejected(self):
        sweep=runpy.run_path(str(SCRIPT))['sweep_values']
        for args in [(1500,200,2),(1500,80,0),(1500,80,9),(1500,81,2)]:
            with self.assertRaises(ValueError):next(sweep(*args))
    def exercise(self,enabled):
        code=runpy.run_path(str(SCRIPT));code['main'].__globals__['SWEEP_ENABLED']=enabled
        events=[];sleeps=[]
        class Pin:
            OUT=1
            def __init__(self,pin):events.append(('gpio',pin))
            def init(self,*args,**kwargs):events.append(('low',kwargs['value']))
        class PWM:
            def __init__(self,pin,**kwargs):events.append(('start',kwargs))
            def duty_ns(self,value):events.append(('pulse',value))
            def deinit(self):events.append(('deinit',))
        def sleep(ms):
            sleeps.append(ms)
            if len(sleeps)>90:raise KeyboardInterrupt()
        with patch.dict(sys.modules,{'machine':types.SimpleNamespace(Pin=Pin,PWM=PWM)}),patch('time.sleep_ms',sleep,create=True),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(KeyboardInterrupt):code['main']()
        self.assertIn(('gpio',2),events);self.assertIn(('start',{'freq':50,'duty_ns':1500000}),events)
        self.assertEqual(events[-2:],[('deinit',),('low',0)])
        return events
    def test_default_mode_only_centres_then_releases_on_interrupt(self):
        events=self.exercise(False);self.assertFalse(any(e[0]=='pulse' for e in events))
    def test_sweep_mode_limits_every_sent_pulse_and_releases_on_interrupt(self):
        events=self.exercise(True);pulses=[e[1] for e in events if e[0]=='pulse']
        self.assertGreater(len(pulses),80);self.assertTrue(all(1420000<=p<=1580000 for p in pulses))

if __name__=='__main__':unittest.main()
