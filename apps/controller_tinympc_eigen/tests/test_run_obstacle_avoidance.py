"""No-radio lifecycle regression checks for the explicit flight runner."""
import importlib.util
import math
from pathlib import Path
import threading
import tempfile
from types import SimpleNamespace
import unittest

SPEC = importlib.util.spec_from_file_location('obstacle_runner', Path(__file__).parents[1]/'tools/run_obstacle_avoidance.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

class Clock:
    def __init__(self): self.now=0
    def monotonic(self): return self.now
    def sleep(self,n): self.now+=n

class Rig:
    def __init__(self, failure=None):
        self.clock=Clock(); self.calls=[]; self.values={'system.selftestPassed':'1'}
        self.failure=failure; self.running_since=None; self.takeoff_at=None; self.handoff=0
        self.ground=.17; self.z=self.ground; self.lost=threading.Event(); self.lock=threading.RLock()
        self.latest={}; self.configured=False
        toc=SimpleNamespace(get_element_by_complete_name=lambda n: object())
        def call(name):
            return lambda *args:self.calls.append((name,args,self.clock.now))
        def setvalue(name,value):
            self.calls.append(('param',(name,value),self.clock.now)); self.values[name]=value
            if name=='stabilizer.controller' and value=='6': self.handoff+=1
            if name=='espTest.run' and value=='1': self.running_since=self.clock.now
        def takeoff(z,duration):
            call('takeoff')(z,duration);self.z=z;self.takeoff_at=self.clock.now
        self.cf=SimpleNamespace(param=SimpleNamespace(toc=toc,set_value=setvalue,get_value=lambda n:self.values.get(n,'0')),
          platform=SimpleNamespace(send_arming_request=call('arm')),
          commander=SimpleNamespace(send_position_setpoint=call('position'),send_notify_setpoint_stop=call('release'),send_stop_setpoint=call('stop')),
          high_level_commander=SimpleNamespace(takeoff=takeoff,land=call('land'),stop=call('hlstop')))
    def event(self,message): self.calls.append(('event',(message,),self.clock.now))
    def snapshot(self):
        running=self.running_since is not None and self.values.get('espTest.run')=='1'
        if running and self.failure in ('stale','interrupt'):
            raise (KeyboardInterrupt() if self.failure=='interrupt' else RuntimeError('Required telemetry is missing or older than 600 ms'))
        elapsed=self.clock.now-(self.running_since or self.clock.now)
        distance=float(self.values.get('dgAvoid.distance','.5'))
        travel=distance if running and elapsed>.1 else 0
        state={f'stateEstimate.{n}':0. for n in ('x','y','z','vx','vy','vz','roll','pitch','yaw')}
        state.update({'stateEstimate.x':travel,'stateEstimate.z':self.z,'dgRx.ageMs':0,'dg.inferUs':1452800,
            'supervisor.info':3,'espTest.handoff':self.handoff,'dgAvoid.enabled':1,'dgAvoid.active':1,'dgAvoid.fresh':1,
            'dgAvoid.fault':1 if running and self.failure=='fault' else (9 if travel else 0),'dgAvoid.travel':travel})
        state.update({f'motor.m{i}':0 for i in range(1,5)})
        if self.failure=='startup':state['dgRx.ageMs']=1801
        self.latest=state
        return state

class Lifecycle(unittest.TestCase):
    def run_rig(self,failure=None):
        rig=Rig(failure); result={'completed':False};args=m.parse_args(['--fly','--speed','.12','--distance','.4'])
        if failure in ('fault','stale','startup','interrupt'):
            with self.assertRaises(KeyboardInterrupt if failure=='interrupt' else RuntimeError):
                m.fly(rig.cf,rig,args,result,rig.clock)
        else:m.fly(rig.cf,rig,args,result,rig.clock)
        return rig,result
    def test_complete_flight(self):
        rig,result=self.run_rig();self.assertTrue(result['completed']);self.assertEqual(result['stop_reason'],'distance')
        calls=rig.calls;takeoff=next(c for c in calls if c[0]=='takeoff')
        self.assertEqual(takeoff[1],(.67,1.));first=next(c for c in calls if c[0]=='position')
        self.assertGreaterEqual(first[2]-takeoff[2],1.5)
        self.assertEqual(rig.values['dgAvoid.speed'],'0.12');self.assertEqual(rig.values['dgAvoid.distance'],'0.4')
        names=[c[0] for c in calls];self.assertLess(names.index('release'),names.index('land'))
        self.assertEqual(calls[names.index('land')][1][0],.17)
        self.assertEqual([c[1] for c in calls if c[0]=='arm'],[(True,),(False,)])
        release=names.index('release');self.assertFalse(any(c[0]=='position' for c in calls[release:]))
        stopped=next(c for c in calls if c[0]=='event' and c[1][0].startswith('DISTANCE'))
        self.assertGreaterEqual(calls[names.index('land')][2]-stopped[2],.5)
    def test_startup_failure_never_arms(self):
        rig,result=self.run_rig('startup');self.assertFalse(result['completed'])
        self.assertFalse(any(c[0] in ('arm','takeoff','position','land','param') for c in rig.calls))
    def test_fault_stale_and_interrupt_land(self):
        for failure in ('fault','stale','interrupt'):
            with self.subTest(failure=failure):
                rig,result=self.run_rig(failure);self.assertFalse(result['completed'])
                names=[c[0] for c in rig.calls];self.assertIn('land',names)
                self.assertLess(names.index('release'),names.index('land'))
                self.assertEqual([c[1] for c in rig.calls if c[0]=='arm'][-1],(False,))
    def test_real_fault_wins_over_distance_threshold(self):
        rig=Rig();original=rig.snapshot
        def snapshot():
            state=original()
            if rig.values.get('espTest.run')=='1':
                state['stateEstimate.x']=1.
                state['dgAvoid.fault']=3
            return state
        rig.snapshot=snapshot;result={'completed':False}
        with self.assertRaisesRegex(RuntimeError,'inside clearance'):
            m.fly(rig.cf,rig,m.parse_args(['--fly']),result,rig.clock)
        self.assertFalse(result['completed'])
        self.assertNotEqual(result.get('stop_reason'),'distance')
        self.assertTrue(any(c[0]=='land' for c in rig.calls))

    def test_landing_ack_error_still_attempts_descent(self):
        rig=Rig();original=rig.cf.param.set_value
        def setvalue(name,value):
            original(name,value)
            landing=any(c[0]=='event' and c[1][0].startswith('LAND:') for c in rig.calls)
            if landing and name in ('espTest.run','stabilizer.controller'):
                raise RuntimeError('simulated acknowledgement failure')
        rig.cf.param.set_value=setvalue;result={'completed':False}
        m.fly(rig.cf,rig,m.parse_args(['--fly']),result,rig.clock)
        self.assertFalse(result['completed'])
        self.assertEqual(len(result['landing_errors']),2)
        names=[c[0] for c in rig.calls]
        self.assertLess(names.index('release'),names.index('land'))
        self.assertLess(names.index('land'),names.index('hlstop'))
        self.assertLess(names.index('hlstop'),names.index('stop'))
        land=next(c for c in rig.calls if c[0]=='land')
        stop=next(c for c in rig.calls if c[0]=='stop')
        self.assertGreaterEqual(stop[2]-land[2],land[1][1]+.5)
        self.assertEqual([c[1] for c in rig.calls if c[0]=='arm'][-1],(False,))

    def test_log_packet_budget_and_schema(self):
        sizes={'float':4,'uint32_t':4,'uint16_t':2,'uint8_t':1}
        fields=[]
        for group,entries in m.GROUPS.items():
            self.assertLessEqual(sum(sizes[kind] for _,kind in entries),26,group)
            fields.extend(name for name,_ in entries)
        self.assertEqual(len(fields),len(set(fields)))
        self.assertTrue({'dgPlane0.sample','dgPlane1.sample','dgAvoid.sample','dgAvoid.travel','dgRx.ageMs','dg.inferUs'}<=set(fields))
    def test_real_telemetry_missing_stale_and_nonfinite(self):
        clock=Clock()
        with tempfile.TemporaryDirectory() as directory:
            log=m.Telemetry(Path(directory),clock)
            try:
                with self.assertRaises(RuntimeError):log.snapshot()
                for group in ('position','attitude','avoid','mission','depth','health'):
                    log.data(0,{'stateEstimate.x':0.},SimpleNamespace(name=group))
                self.assertEqual(log.snapshot()['stateEstimate.x'],0)
                clock.sleep(.601)
                with self.assertRaises(RuntimeError):log.snapshot()
                for group in ('position','attitude','avoid','mission','depth','health'):
                    log.data(1,{'stateEstimate.x':float('nan')},SimpleNamespace(name=group))
                with self.assertRaises(RuntimeError):log.snapshot()
            finally:log.close(SimpleNamespace(close_link=lambda:None))

    def test_defaults_and_projected_distance(self):
        args=m.parse_args([]);self.assertFalse(args.fly);self.assertFalse(args.record)
        self.assertEqual(args.height,.5)
        self.assertAlmostEqual(m.forward_distance({'stateEstimate.x':1,'stateEstimate.y':3},(1,2),math.pi/2),1)

if __name__=='__main__':unittest.main()
