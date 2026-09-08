#!/usr/bin/env python3
"""Record telemetry while disarmed by default; --fly runs a bounded DepthGate trial.
Adapted from apps/controller_tinympc_eigen/tools/run_brake_csv.py.
"""
import argparse
import csv
import json
import math
import signal
from pathlib import Path
import threading
import time


FLOAT = 'float'
GROUPS = {
    'dg': [('dg.seq','uint16_t'),('dg.status','uint16_t'),('dg.sourceMs','uint32_t'),('dg.inferUs','uint32_t')]+[(f'dg.inv{n}',FLOAT) for n in ('Left','Center','Right')],
    'dg_rx': [(f'dgRx.{n}','uint32_t') for n in ('ok','crcErr','invalid','stale','shortRx','ageMs')],
    'dg_top': [(f'dg{c}.{n}',FLOAT) for c in ('LT','RT') for n in ('x','y','logit')],
    'dg_bottom': [(f'dg{c}.{n}',FLOAT) for c in ('LB','RB') for n in ('x','y','logit')],
    'dg_state': [(f'dgAvoid.{n}','uint8_t') for n in ('enabled','fresh','count','mode','fault')]+[('dgAvoid.seq','uint16_t'),('dgAvoid.ageMs','uint32_t')]+[(f'dgAvoid.{n}',FLOAT) for n in ('left','center','right')],
    'dg_planes': [(f'dgAvoid.{n}',FLOAT) for n in ('bound0','bound1','violation','cmdSpeed')],
    'constraints': [(f'mpcLimit.{n}',FLOAT) for n in ('tiltErr','rateErr','primal','solveUs')],
    'position': [(f'stateEstimate.{n}',FLOAT) for n in ('x','y','z','vx','vy','vz')],
    'attitude': [(f'stateEstimate.{n}',FLOAT) for n in ('roll','pitch','yaw')]+[(f'gyro.{n}',FLOAT) for n in ('x','y','z')],
    'motors': [(f'motor.m{i}','uint16_t') for i in range(1,5)]+[('supervisor.info','uint16_t')],
    'health': [('pm.vbat',FLOAT)]+[(f'kalman.varP{n}',FLOAT) for n in ('X','Y','Z')],
    'test': [('espTest.handoff','uint16_t'),('espTest.phase','uint8_t'),('espTest.run','uint8_t')],
}

def start_cfclient_takeoff(cf, current_z, climb):
    """Issue the same high-level takeoff sequence as cfclient's Flight tab."""
    target = current_z + climb
    cf.param.set_value('commander.enHighLevel', '1')
    cf.high_level_commander.takeoff(target, climb / .5)
    return target


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--uri', default='radio://0/80/2M/E7E7E7E7E7')
    p.add_argument('--fly', action='store_true')
    p.add_argument('--height', type=float, default=.5)
    p.add_argument('--speed', type=float, default=.5, help='Forward speed in m/s; >0 through0.5 tested firmware limit')
    p.add_argument('--distance', type=float, default=1., help='Measured forward travel along initial heading, meters')
    p.add_argument('--timeout', type=float, default=9., help='Maximum RUN seconds, <=9; firmware also has10s cap')
    p.add_argument('--output', type=Path, default=Path('flight-recordings') / time.strftime('%Y%m%d-%H%M%S'))
    a = p.parse_args()
    flight_height = a.height
    for name,lo,hi in [('height',.3,.5),('speed',.01,.5),('distance',.05,3.),('timeout',.5,9.)]:
        value=getattr(a,name)
        if not math.isfinite(value) or not lo<=value<=hi: p.error(f'--{name} must be {lo} through {hi}')
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    def interrupt(*_):
        raise KeyboardInterrupt('Flight interrupted; attempting PID landing')
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'metadata.json').write_text(json.dumps(vars(a), default=str, indent=2))
    ready, lost = threading.Event(), threading.Event()
    cache = a.output / "cache"
    cache.mkdir()
    cf = Crazyflie(rw_cache=str(cache))
    arming = getattr(cf, "supervisor", cf.platform)
    # Same TOC-only connection workaround used by the existing flight probe.
    cf._log_toc_updated_cb = lambda: cf.param.refresh_toc(cf._param_toc_updated_cb, cf._toc_cache)
    cf.fully_connected.add_callback(lambda *_: ready.set())
    cf.disconnected.add_callback(lambda *_: lost.set())
    cf.connection_failed.add_callback(lambda *_: lost.set())
    lock = threading.RLock()
    latest, received, files, configs = {}, {}, [], []
    origin = time.monotonic()
    console = (a.output/'console.txt').open('w', buffering=1)
    events = (a.output/'events.csv').open('w', newline='', buffering=1)
    ew = csv.writer(events); ew.writerow(['host_s','event'])
    def event(message):
        with lock:
            ew.writerow([time.monotonic()-origin, message])
        print(message, flush=True)
    def console_data(message):
        with lock:
            console.write(message)
    cf.console.receivedChar.add_callback(console_data)
    def start_log(name, fields):
        for field, _ in fields:
            if cf.log.toc.get_element_by_complete_name(field) is None:
                raise RuntimeError(f'Missing firmware log variable: {field}')
        f = (a.output/f'{name}.csv').open('w', newline='', buffering=1); files.append(f)
        writer = csv.writer(f)
        writer.writerow(['host_s','firmware_ms']+[n for n,_ in fields])
        config = LogConfig(name, 50 if name in ('position','attitude','dg_state') else 100)
        for field, kind in fields:
            config.add_variable(field, kind)
        def data(ts, values, _):
            with lock:
                now = time.monotonic()
                writer.writerow([now-origin,ts]+[values[n] for n,_ in fields])
                latest.update(values); received[name] = now
        config.data_received_cb.add_callback(data)
        config.error_cb.add_callback(lambda _, msg: event('LOG ERROR: '+msg))
        cf.log.add_config(config); config.start(); configs.append(config)
    def snapshot():
        with lock:
            if lost.is_set() or any(time.monotonic()-received.get(g,0) > .5 for g in GROUPS):
                raise RuntimeError('Disconnected or stale telemetry')
            return dict(latest)
    def hover(seconds, height, predicate=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            state = snapshot()
            if flying: check_flight(state)
            cf.commander.send_hover_setpoint(0,0,0,height)
            if predicate and predicate(state):
                return
            time.sleep(.02)
        if predicate:
            raise RuntimeError('Timed out waiting for flight state')
    def require_depth(state):
        values=[state[f'dg.inv{n}'] for n in ('Left','Center','Right')]
        if int(state['dg.status']) != 3 or state['dgRx.ageMs'] > 200 or not 0 < state['dg.inferUs'] <= 150000 or not all(math.isfinite(v) and v>0 for v in values):
            raise RuntimeError('DepthGate invalid/stale')
    def check_flight(state):
        vals=[state[f'stateEstimate.{n}'] for n in ('x','y','z','vx','vy','vz','roll','pitch','yaw')]
        if not all(math.isfinite(v) for v in vals): raise RuntimeError('Invalid pose')
        if abs(vals[6])>20 or abs(vals[7])>20: raise RuntimeError('Attitude envelope exceeded')
        if vals[2]<-.1 or vals[2]>flight_height+.3: raise RuntimeError('Altitude envelope exceeded')
        if math.hypot(vals[0]-launch_xy[0],vals[1]-launch_xy[1])>a.distance+.5: raise RuntimeError('Travel envelope exceeded')
        if math.hypot(vals[3],vals[4])>.85: raise RuntimeError('Speed envelope exceeded')
        # Battery voltage is logged; firmware retains its battery protections.
    launch_xy=(0.,0.)
    flying = False
    try:
        cflib.crtp.init_drivers(); cf.open_link(a.uri)
        if not ready.wait(25):
            raise RuntimeError('Connection timeout')
        for name, fields in GROUPS.items():
            start_log(name, fields)
        time.sleep(1)
        if not a.fly:
            event('Disarmed recording initialized; no parameters, arm or flight commands sent')
            return
        for name,value in [('stabilizer.controller',1),('espTest.run',0),('espTest.external',0),('dgAvoid.speed',a.speed),('dgAvoid.clearance',.5),('dgAvoid.enable',1)]:
            cf.param.set_value(name,value)
        with lock:
            takeoff_state = dict(latest)
        current_z = takeoff_state.get('stateEstimate.z', 0.0)
        launch_xy=(takeoff_state.get('stateEstimate.x', 0.0),
                   takeoff_state.get('stateEstimate.y', 0.0))
        flying=True; arming.send_arming_request(True)
        flight_height = start_cfclient_takeoff(cf, current_z, a.height)
        event(f'PID high-level takeoff: target {flight_height:.3f}m over {a.height/.5:.2f}s')
        # Low-level hover packets would override the high-level trajectory.
        # Observe its ascent and position hold without sending those packets.
        end=time.monotonic()+a.height/.5
        while time.monotonic()<end:
            s=snapshot(); check_flight(s); require_depth(s)
            time.sleep(.02)
        # No stationary/settling dwell. Do not start forward motion on the floor.
        if snapshot()['stateEstimate.z'] < flight_height-.2:
            raise RuntimeError('Takeoff did not lift to the handoff height')
        previous_handoff=snapshot()['espTest.handoff']
        # Keep commander alive while async controller selection is acknowledged.
        cf.param.set_value('stabilizer.controller',6);event('OOT selected')
        hover(1,flight_height,lambda s:s['espTest.handoff'] != previous_handoff)
        hover(.25,flight_height)
        s=snapshot(); require_depth(s)
        if not s['dgAvoid.fresh'] or s['dgAvoid.fault']: raise RuntimeError('DG controller not ready')
        run_x,run_y=s['stateEstimate.x'],s['stateEstimate.y']
        heading=math.radians(s['stateEstimate.yaw']); forward=(math.cos(heading),math.sin(heading))
        cf.param.set_value('espTest.run',1);event(f'DG RUN requested:{a.speed}m/s, target{a.distance}m,0.5m clearance')
        start=time.monotonic(); ack=False; reached=False
        while time.monotonic()-start<a.timeout:
            s=snapshot();check_flight(s);require_depth(s)
            cf.commander.send_hover_setpoint(0,0,0,flight_height)
            travel=(s['stateEstimate.x']-run_x)*forward[0]+(s['stateEstimate.y']-run_y)*forward[1]
            if travel>=a.distance:
                reached=True; event(f'Target reached:{travel:.3f}m');break
            ack=ack or bool(s['espTest.run'])
            if s['dgAvoid.fault']: raise RuntimeError('DG fault '+str(s['dgAvoid.fault']))
            if not ack and time.monotonic()-start>.5: raise RuntimeError('RUN not acknowledged')
            time.sleep(.02)
        if not reached: event('Distance not reached before timeout; stopping test')
        cf.param.set_value('espTest.run',0);event('RUN released; OOT hold')
        hover(2,flight_height,lambda s: not s['espTest.run'] and math.hypot(s['stateEstimate.vx'],s['stateEstimate.vy'])<.15)
        event('Hold settled; PID landing')
    finally:
        # On completion/error, cancel the test and attempt a PID descent if linked.
        if flying and not lost.is_set():
            try:
                with lock:
                    landing_state = dict(latest)
                    position_fresh = time.monotonic() - received.get('position', 0) <= .5
                    attitude_fresh = time.monotonic() - received.get('attitude', 0) <= .5
                hold_values = [landing_state.get(f'stateEstimate.{axis}', math.nan)
                               for axis in ('x', 'y', 'z', 'yaw')]
                hold_position = position_fresh and attitude_fresh and all(math.isfinite(v) for v in hold_values)
                hold_x, hold_y, measured_z, hold_yaw = hold_values
                z = max(.05, min(1, measured_z)) if math.isfinite(measured_z) else flight_height
                cf.param.set_value('espTest.run',0)
                cf.param.set_value('stabilizer.controller',1)
                ack_start=time.monotonic(); retry=ack_start+.25
                while (cf.param.get_value('stabilizer.controller') != '1' or
                       cf.param.get_value('espTest.run') != '0'):
                    now=time.monotonic()
                    cf.commander.send_hover_setpoint(0,0,0,z)
                    if now-ack_start>1:
                        raise RuntimeError('PID landing selection not acknowledged')
                    if now>=retry:
                        cf.param.set_value('espTest.run',0)
                        cf.param.set_value('stabilizer.controller',1)
                        retry=now+.25
                    time.sleep(.02)
                event('PID landing')
                if hold_position:
                    event(f'Landing position hold: x={hold_x:.3f} y={hold_y:.3f} yaw={hold_yaw:.1f}')
                else:
                    event('Landing fallback: stale/invalid pose; zero horizontal velocity')
                start=time.monotonic()
                while time.monotonic()-start < 3 and not lost.is_set():
                    landing_z = max(.03, z*(1-(time.monotonic()-start)/3))
                    if hold_position:
                        cf.commander.send_position_setpoint(hold_x, hold_y, landing_z, hold_yaw)
                    else:
                        cf.commander.send_hover_setpoint(0,0,0,landing_z)
                    time.sleep(.02)
            finally:
                cf.commander.send_stop_setpoint(); arming.send_arming_request(False)
        for config in configs:
            config.stop()
        cf.close_link()
        with lock:
            for f in files: f.close()
            console.close(); events.close()
        print(f'Recording: {a.output.resolve()}')

if __name__ == '__main__':
    main()
