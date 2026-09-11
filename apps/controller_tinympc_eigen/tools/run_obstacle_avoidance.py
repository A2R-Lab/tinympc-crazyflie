#!/usr/bin/env python3
"""Log and run the DDND straight-line obstacle test. No connection without --record or --fly."""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time

CFCLIENT_SOURCE = ('https://github.com/bitcraze/crazyflie-clients-python/blob/'
                   '8396d2854575d9c679fabb1f936e24679d553ce2/src/cfclient/ui/tabs/FlightTab.py')
GROUPS = {
    'position': [(f'stateEstimate.{n}', 'float') for n in ('x','y','z','vx','vy','vz')],
    'attitude': [(f'stateEstimate.{n}', 'float') for n in ('roll','pitch','yaw')],
    'avoid': [('dgAvoid.sample','uint32_t')] + [(f'dgAvoid.{n}','uint8_t') for n in
              ('count','mode','fresh','fault','enabled','active')] + [('dgAvoid.seq','uint16_t'),
              ('dgAvoid.ageMs','uint32_t'),('dgAvoid.cmdSpeed','float'),('dgAvoid.violation','float')],
    'plane0': [(f'dgPlane0.{n}','float') for n in ('nx','ny','b')] + [('dgPlane0.sample','uint32_t')],
    'plane1': [(f'dgPlane1.{n}','float') for n in ('nx','ny','b')] + [('dgPlane1.sample','uint32_t')],
    'depth': [(f'dgAvoid.{n}','float') for n in ('left','center','right')] +
             [('dg.inferUs','uint32_t'),('dgRx.ageMs','uint32_t'),('dg.seq','uint16_t')],
    'mission': [('dgAvoid.travel','float'),('espTest.handoff','uint16_t'),('espTest.run','uint8_t')],
    'health': [(f'motor.m{i}','uint16_t') for i in range(1,5)] + [('supervisor.info','uint16_t')],
}
FAULTS = {1:'vision/configuration unavailable',2:'attitude limit',3:'inside clearance',
          4:'firmware timeout',5:'reference projection failed',6:'solver constraint failure',
          7:'RUN asserted during mode change',8:'speed/state limit',9:'distance reached'}
PARAMETERS = ('stabilizer.controller','commander.enHighLevel','espTest.run','espTest.external',
              'dgAvoid.enable','dgAvoid.speed','dgAvoid.distance','dgAvoid.timeout')


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--fly',action='store_true',help='Take off, run the test, hold, and land')
    mode.add_argument('--record',action='store_true',help='Record telemetry only; send no flight/parameter commands')
    p.add_argument('--speed',type=float,default=.1,help='Forward target speed, 0 <= speed <= 2 m/s; zero holds for timeout')
    p.add_argument('--distance',type=float,default=.5,help='Forward displacement at which to stop, meters')
    p.add_argument('--height',type=float,default=.5,help='Takeoff climb above current Z (default .5 m)')
    p.add_argument('--timeout',type=float,help='Mission limit, default distance/speed + 10 seconds, max 120')
    p.add_argument('--seconds',type=float,default=30,help='Duration for --record')
    p.add_argument('--uri',default='radio://0/80/2M/E7E7E7E7E7')
    p.add_argument('--output',type=Path,default=Path('obstacle-recordings')/time.strftime('%Y%m%d-%H%M%S'))
    a = p.parse_args(argv)
    for key,lo,hi in [('distance',0,20),('height',0,1),('seconds',0,600)]:
        if not math.isfinite(getattr(a,key)) or not lo < getattr(a,key) <= hi:
            p.error(f'--{key} must be finite and in ({lo}, {hi}]')
    if not math.isfinite(a.speed) or not 0 <= a.speed <= 2:
        p.error('--speed must be finite and in [0, 2]')
    if a.height < .2: p.error('--height must be at least 0.2 m')
    if a.timeout is None: a.timeout = a.distance/a.speed+10 if a.speed > 0 else 10
    if not math.isfinite(a.timeout) or not 1 <= a.timeout <= 120:
        p.error('--timeout (including its default) must be 1–120 seconds; shorten distance or specify a timeout')
    if a.speed > 0 and a.timeout <= a.distance/a.speed:
        p.error('--timeout must exceed distance/speed to allow acceleration and avoidance')
    return a


def forward_distance(state, origin, heading):
    return ((state['stateEstimate.x']-origin[0])*math.cos(heading) +
            (state['stateEstimate.y']-origin[1])*math.sin(heading))


def cfclient_takeoff(cf, current_z, climb):
    """Same HLC target/duration as FlightTab: +0.5 m at 0.5 m/s by default.

    The caller confirms commander.enHighLevel=1 first. No low-level setpoints
    may be sent during the takeoff, since they override the HLC trajectory.
    """
    cf.high_level_commander.takeoff(current_z+climb,climb/.5)


class Telemetry:
    def __init__(self, directory, clock=time):
        self.clock, self.start = clock, clock.monotonic()
        self.lock = threading.RLock()
        self.latest, self.received, self.configs = {}, {}, []
        self.lost = threading.Event()
        self.log_error = None
        self.raw = (directory/'telemetry.csv').open('w',newline='',buffering=1)
        self.writer = csv.writer(self.raw)
        self.writer.writerow(['host_s','firmware_ms','group','values_json'])
        self.events = (directory/'events.csv').open('w',newline='',buffering=1)
        self.event_writer = csv.writer(self.events); self.event_writer.writerow(['host_s','event'])
        self.console = (directory/'console.txt').open('w',buffering=1)

    def event(self, message):
        with self.lock: self.event_writer.writerow([self.clock.monotonic()-self.start,message])
        print(message,flush=True)

    def data(self, timestamp, values, config):
        with self.lock:
            now = self.clock.monotonic()
            self.writer.writerow([now-self.start,timestamp,config.name,json.dumps(values)])
            self.latest.update(values); self.received[config.name] = now

    def snapshot(self, required=('position','attitude','avoid','mission','depth','health')):
        with self.lock:
            if self.lost.is_set(): raise RuntimeError('Radio link lost')
            if self.log_error: raise RuntimeError(self.log_error)
            now = self.clock.monotonic()
            if any(now-self.received.get(g,-math.inf)>.6 for g in required):
                raise RuntimeError('Required telemetry is missing or older than 600 ms')
            state = dict(self.latest)
        if any(not math.isfinite(v) for k,v in state.items() if k.startswith('stateEstimate.')):
            raise RuntimeError('Nonfinite state estimate')
        return state

    def attach(self, cf):
        from cflib.crazyflie.log import LogConfig
        # Validate the complete schema before starting any blocks or motors.
        for fields in GROUPS.values():
            for name,_ in fields:
                if cf.log.toc.get_element_by_complete_name(name) is None:
                    raise RuntimeError(f'Missing {name}; flash the matching obstacle-test firmware')
        for name,fields in GROUPS.items():
            config = LogConfig(name,100 if name in ('depth','health') else 50)
            for field,kind in fields: config.add_variable(field,kind)
            config.data_received_cb.add_callback(self.data)
            def error(_, message):
                self.log_error = 'Log error: '+message
                self.event(self.log_error)
            config.error_cb.add_callback(error)
            cf.log.add_config(config); self.configs.append(config); config.start()

    def close(self, cf):
        for config in self.configs:
            try: config.stop()
            except Exception: pass
        cf.close_link()
        # Closing the link terminates callbacks before closing their files.
        with self.lock:
            self.raw.close(); self.events.close(); self.console.close()


def set_parameter(cf, name, value, clock=time):
    cf.param.set_value(name,str(value))
    deadline = clock.monotonic()+2
    while clock.monotonic()<deadline:
        if math.isclose(float(cf.param.get_value(name)),float(value),rel_tol=1e-5,abs_tol=1e-6): return
        clock.sleep(.02)
    raise RuntimeError(f'Parameter acknowledgement timed out: {name}')


def wait_for(log, seconds, predicate, label, clock=time, command=None):
    end = clock.monotonic()+seconds
    while clock.monotonic()<end:
        state = log.snapshot()
        if command: command()
        if predicate(state): return state
        clock.sleep(.02)
    raise RuntimeError('Timed out waiting for '+label)


def fly(cf, log, a, result, clock=time):
    """Run one mission; called only with explicit --fly. Always attempt landing after arming."""
    for name in PARAMETERS:
        if cf.param.toc.get_element_by_complete_name(name) is None:
            raise RuntimeError('Missing firmware parameter '+name)
    initial = log.snapshot()
    if any(initial[f'motor.m{i}'] for i in range(1,5)):
        raise RuntimeError('Motors must be stopped before this test')
    if cf.param.get_value('system.selftestPassed') != '1': raise RuntimeError('Firmware self-test failed')
    if initial['dgRx.ageMs']>1800 or not 0<initial['dg.inferUs']<=2000000:
        raise RuntimeError('Fresh compact DepthGate packets required before takeoff')
    if not int(initial['supervisor.info']) & 3: raise RuntimeError('Supervisor is not ready to arm')
    arm = getattr(cf,'supervisor',None) or cf.platform
    airborne = False
    try:
        for name,value in [('espTest.run',0),('stabilizer.controller',1),('espTest.external',0),
                           ('dgAvoid.enable',1),('dgAvoid.speed',a.speed),('dgAvoid.distance',a.distance),
                           ('dgAvoid.timeout',a.timeout),('commander.enHighLevel',1)]:
            set_parameter(cf,name,value,clock)
        airborne = True  # Also clean up if arming request raises after being sent.
        arm.send_arming_request(True)
        wait_for(log,2,lambda s:int(s['supervisor.info'])&2,'arming',clock)
        ground = log.snapshot()['stateEstimate.z']
        target = ground+a.height
        result.update(ground_z=ground,takeoff_target_z=target,takeoff_duration_s=a.height/.5)
        log.event('TAKEOFF: cfclient high-level command')
        cfclient_takeoff(cf,ground,a.height)
        started, stable = clock.monotonic(), None
        def settled(s):
            nonlocal stable
            speed = math.sqrt(sum(s[f'stateEstimate.v{axis}']**2 for axis in 'xyz'))
            good = (clock.monotonic()-started >= a.height/.5 and abs(s['stateEstimate.z']-target)<.08
                    and speed<.12 and abs(s['stateEstimate.roll'])<10 and abs(s['stateEstimate.pitch'])<10)
            stable = (clock.monotonic() if stable is None else stable) if good else None
            return stable is not None and clock.monotonic()-stable>=.5
        state = wait_for(log,10,settled,'stable PID hover',clock)
        hold = tuple(state[f'stateEstimate.{n}'] for n in ('x','y','z','yaw'))
        def heartbeat(): cf.commander.send_position_setpoint(*hold)
        handoff = state['espTest.handoff']
        # Start the low-level heartbeat only AFTER HLC takeoff has completed.
        heartbeat()
        set_parameter(cf,'stabilizer.controller',6,clock)
        log.event('HANDOFF: TinyMPC selected; waiting for fresh capture history')
        wait_for(log,2,lambda s:s['espTest.handoff']!=handoff,'TinyMPC handoff',clock,heartbeat)
        state = wait_for(log,8,lambda s:s['dgAvoid.active'] and s['dgAvoid.fresh'] and
                         not s['dgAvoid.fault'],'camera-time pose history',clock,heartbeat)
        origin = (state['stateEstimate.x'],state['stateEstimate.y'])
        heading = math.radians(hold[3])
        result.update(mission_origin=origin,heading_rad=heading)
        log.event(f'RUN: speed={a.speed:g} m/s distance={a.distance:g} m')
        set_parameter(cf,'espTest.run',1,clock)
        start = clock.monotonic()
        while True:
            state = log.snapshot(); heartbeat()
            travel = forward_distance(state,origin,heading)
            result.update(forward_distance_m=travel,firmware_travel_m=state['dgAvoid.travel'])
            fault = int(state['dgAvoid.fault'])
            if a.speed == 0 and fault in (0,4) and (fault == 4 or clock.monotonic()-start >= a.timeout-.1):
                result['stop_reason']='hold duration'; break
            if fault and fault!=9: raise RuntimeError('Firmware stopped: '+FAULTS.get(fault,str(fault)))
            if fault==9 or travel>=a.distance:
                result['stop_reason']='distance'; break
            if clock.monotonic()-start>a.timeout+1: raise RuntimeError('Host mission timeout')
            clock.sleep(.02)
        set_parameter(cf,'espTest.run',0,clock)
        log.event(f"STOP: {result['stop_reason']}; TinyMPC hold")
        stable = None
        def stopped(s):
            nonlocal stable
            good = math.hypot(s['stateEstimate.vx'],s['stateEstimate.vy'])<.08 and abs(s['stateEstimate.vz'])<.1
            stable = (clock.monotonic() if stable is None else stable) if good else None
            return stable is not None and clock.monotonic()-stable>=.5
        wait_for(log,5,stopped,'stationary hold before landing',clock,heartbeat)
        result['completed']=True
    finally:
        if airborne and not log.lost.is_set():
            errors = []
            def attempt(label, action):
                try: action()
                except Exception as exc:
                    errors.append(label+': '+str(exc))
                    log.event('LAND ERROR: '+errors[-1])
            log.event('LAND: release low-level priority; PID high-level descent')
            # An ACK error may follow a successfully transmitted command. Try
            # each landing step independently instead of skipping the descent.
            attempt('release RUN',lambda:set_parameter(cf,'espTest.run',0,clock))
            attempt('select PID',lambda:set_parameter(cf,'stabilizer.controller',1,clock))
            with log.lock: z=log.latest.get('stateEstimate.z',initial['stateEstimate.z']+a.height)
            ground = result.get('ground_z',initial['stateEstimate.z'])
            duration = max(1,min(5,(z-ground)/.5)) if math.isfinite(z) else 3
            attempt('release priority',cf.commander.send_notify_setpoint_stop)
            attempt('descent',lambda:cf.high_level_commander.land(ground,duration))
            try: clock.sleep(duration+.5)
            finally:
                attempt('stop HLC',cf.high_level_commander.stop)
                attempt('stop setpoints',cf.commander.send_stop_setpoint)
                attempt('disarm',lambda:arm.send_arming_request(False))
            if errors:
                result['landing_errors']=errors
                result['completed']=False



def main(argv=None):
    a = parse_args(argv)
    print(f'Straight obstacle test: {a.speed:g} m/s, {a.distance:g} m, timeout {a.timeout:g} s')
    if not (a.fly or a.record):
        print('Preview only. Use --record for telemetry or --fly for takeoff, test, and landing.')
        return
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'metadata.json').write_text(json.dumps(dict(vars(a),schema_version=1,
        cfclient_takeoff_source=CFCLIENT_SOURCE),default=str,indent=2)+'\n')
    log = Telemetry(a.output)
    cf = Crazyflie(rw_cache=str(a.output/'cache'))
    ready = threading.Event()
    cf.fully_connected.add_callback(lambda *_:ready.set())
    cf.connection_failed.add_callback(lambda *_:log.lost.set())
    cf.connection_lost.add_callback(lambda *_:log.lost.set())
    cf.disconnected.add_callback(lambda *_:log.lost.set())
    def console(message):
        with log.lock: log.console.write(message)
    cf.console.receivedChar.add_callback(console)
    result={'completed':False,'mode':'flight' if a.fly else 'record'}
    try:
        cflib.crtp.init_drivers(); cf.open_link(a.uri)
        deadline=time.monotonic()+25
        while not ready.is_set():
            if log.lost.is_set() or time.monotonic()>deadline: raise RuntimeError('Connection failed/timed out')
            time.sleep(.05)
        log.attach(cf)
        time.sleep(1)
        if a.fly:
            fly(cf,log,a,result)
            if result.get('landing_errors'): raise RuntimeError('Landing cleanup reported errors; see summary.json')
        else:
            log.event('RECORD ONLY: no parameter or flight commands')
            deadline=time.monotonic()+a.seconds
            while time.monotonic()<deadline: log.snapshot(); time.sleep(.1)
            result['completed']=True
    except BaseException as exc:
        result['error']=str(exc) or type(exc).__name__
        log.event('ERROR: '+result['error'])
        raise
    finally:
        (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        log.close(cf)
        print('Recording: '+str(a.output.resolve()))


if __name__=='__main__': main()
