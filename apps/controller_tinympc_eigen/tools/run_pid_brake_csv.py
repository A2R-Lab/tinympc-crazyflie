#!/usr/bin/env python3
"""PID-only straight-line braking test; --fly explicitly enables flight.

Command the requested forward velocity until measured travel reaches --distance,
then command zero XY velocity, capture a hold after settling, and land.
Heading is captured after takeoff. No vision or TinyMPC reference is used.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time


def forward_components(state, origin, heading):
    c, s = math.cos(heading), math.sin(heading)
    return ((state['x']-origin[0])*c + (state['y']-origin[1])*s,
            state['vx']*c + state['vy']*s)


def hover_command(target, heading, yaw_degrees):
    error = math.atan2(math.sin(heading-math.radians(yaw_degrees)),
                       math.cos(heading-math.radians(yaw_degrees)))
    return target*math.cos(error), target*math.sin(error), max(-60,min(60,2*math.degrees(error)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fly', action='store_true')
    p.add_argument('--speed', type=float, default=3, help='Requested forward speed in m/s')
    p.add_argument('--distance', type=float, default=1.5, help='Measured distance to trigger braking, meters')
    p.add_argument('--height', type=float, default=.5, help='Height above initial estimated Z, meters')
    p.add_argument('--uri', default='radio://0/80/2M/E7E7E7E7E7')
    p.add_argument('--output', type=Path,
                   default=Path('pid-brake-recordings')/time.strftime('%Y%m%d-%H%M%S'))
    a = p.parse_args()
    for key, upper in [('speed',12),('distance',20),('height',1.5)]:
        if not math.isfinite(getattr(a,key)) or not 0 < getattr(a,key) <= upper:
            p.error(f'--{key} must be greater than zero and at most {upper}')
    print(f'PID test: target {a.speed:g} m/s; brake at {a.distance:g} m; height {a.height:g} m')
    if not a.fly:
        print('Preview only. Add --fly to take off, test, and land.')
        return

    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'metadata.json').write_text(json.dumps(vars(a), default=str, indent=2))
    print(f'Recording: {a.output.resolve()}')
    cflib.crtp.init_drivers()
    lock, lost = threading.Lock(), threading.Event()
    latest, received = {}, {}
    result = {}
    with (a.output/'telemetry.csv').open('w',newline='') as telemetry, \
         (a.output/'console.txt').open('w',buffering=1) as console, \
         (a.output/'events.txt').open('w',buffering=1) as events:
        writer = csv.writer(telemetry)
        writer.writerow(['host_s','firmware_ms','group','values_json'])
        t0 = time.monotonic()
        def event(message):
            events.write(f'{time.monotonic()-t0:.6f} {message}\n')
            print(message)
        cf = Crazyflie(rw_cache=str(a.output/'cache'))
        cf.console.receivedChar.add_callback(console.write)
        def disconnected(uri, reason=''):
            lost.set(); event(f'LINK CLOSED: {reason}')
        cf.connection_lost.add_callback(disconnected)
        cf.disconnected.add_callback(disconnected)
        with SyncCrazyflie(a.uri, cf=cf):
            configs = []
            def record(timestamp, data, config):
                with lock:
                    latest.update(data); received[config.name] = time.monotonic()
                    writer.writerow([time.monotonic()-t0,timestamp,config.name,json.dumps(data)])
            groups = {
                'position': [('stateEstimate.'+n,'float') for n in ('x','y','z','vx','vy','vz')],
                'attitude': [('stateEstimate.'+n,'float') for n in ('roll','pitch','yaw')],
                'motors': [('motor.m'+str(i),'uint16_t') for i in range(1,5)],
                'health': [('pm.vbat','float'),('supervisor.info','uint16_t')],
            }
            for name, fields in groups.items():
                config = LogConfig(name,50 if name != 'health' else 100)
                for field,dtype in fields: config.add_variable(field,dtype)
                cf.log.add_config(config); config.data_received_cb.add_callback(record)
                config.start(); configs.append(config)
            def state():
                with lock:
                    if lost.is_set() or any(time.monotonic()-received.get(g,0)>.5 for g in groups):
                        raise RuntimeError('Link lost or telemetry stale')
                    s = {n:latest['stateEstimate.'+n] for n in ('x','y','z','vx','vy','vz','roll','pitch','yaw')}
                if not all(math.isfinite(v) for v in s.values()): raise RuntimeError('Invalid state estimate')
                return s
            def connected(): return not lost.is_set() and cf.is_connected()
            def cleanup(label, action):
                if not connected(): return
                try: action()
                except Exception as exc: event(f'{label} failed: {exc}')
            flying = False
            arming = getattr(cf,'supervisor',cf.platform)
            try:
                time.sleep(1)
                initial = state()
                with lock:
                    if any(latest['motor.m'+str(i)] for i in range(1,5)):
                        raise RuntimeError('Require stopped motors before starting')
                if cf.param.get_value('system.selftestPassed') != '1':
                    raise RuntimeError('Firmware self-test has not passed')
                cf.param.set_value('stabilizer.controller',1)
                time.sleep(.2)
                if cf.param.get_value('stabilizer.controller') != '1':
                    raise RuntimeError('PID selection was not confirmed')
                height = initial['z']+a.height
                cf.commander.send_setpoint(0,0,0,0)
                flying = True; arming.send_arming_request(True)
                event('PID TAKEOFF')
                start, stable = time.monotonic(), None
                while time.monotonic()-start < 8:
                    s = state(); elapsed = time.monotonic()-start
                    cf.commander.send_position_setpoint(initial['x'],initial['y'],
                        initial['z']+a.height*min(1,elapsed/2),initial['yaw'])
                    good = (elapsed>=2 and abs(s['z']-height)<.08 and
                            math.sqrt(s['vx']**2+s['vy']**2+s['vz']**2)<.15 and
                            abs(s['roll'])<10 and abs(s['pitch'])<10)
                    stable = (stable or time.monotonic()) if good else None
                    if stable is not None and time.monotonic()-stable>.4: break
                    time.sleep(.02)
                else: raise RuntimeError('Takeoff did not reach a stable hover; test not started')
                s = state(); origin=(s['x'],s['y']); heading=math.radians(s['yaw'])
                event('PID RUN')
                start=time.monotonic()
                while True:
                    s=state(); distance,speed=forward_components(s,origin,heading)
                    if abs(distance)>=a.distance or time.monotonic()-start>=15: break
                    cf.commander.send_hover_setpoint(*hover_command(a.speed,heading,s['yaw']),height)
                    time.sleep(.02)
                # Timestamp the command itself; the triggering speed comes from
                # the latest 50 ms estimator sample, not an interpolated peak.
                cf.commander.send_hover_setpoint(*hover_command(0,heading,s['yaw']),height)
                brake_time=time.monotonic(); brake_distance=distance
                result.update(brake_speed_m_s=speed,brake_distance_m=distance,
                              trigger='distance' if abs(distance)>=a.distance else 'timeout')
                event(f'PID BRAKE: speed={speed:.3f} m/s; distance={distance:.3f} m; target=0')
                max_distance, min_speed, min_z, stable = distance,speed,s['z'],None
                while time.monotonic()-brake_time<6:
                    s=state(); distance,speed=forward_components(s,origin,heading)
                    max_distance=max(max_distance,distance); min_speed=min(min_speed,speed); min_z=min(min_z,s['z'])
                    cf.commander.send_hover_setpoint(*hover_command(0,heading,s['yaw']),height)
                    good=math.hypot(s['vx'],s['vy'])<.15 and abs(s['vz'])<.2 and abs(s['roll'])<10 and abs(s['pitch'])<10
                    stable=(stable or time.monotonic()) if good else None
                    if stable is not None and time.monotonic()-stable>.3: break
                    time.sleep(.02)
                result.update(settled=stable is not None and time.monotonic()-stable>.3,
                              brake_duration_s=time.monotonic()-brake_time,
                              max_forward_travel_m=max_distance-brake_distance,
                              peak_backward_speed_m_s=max(0,-min_speed),minimum_brake_height_m=min_z)
                event('PID BRAKE RESULT '+json.dumps(result))
            except KeyboardInterrupt:
                event('CONTROL-C: ending test')
            except Exception as exc:
                event(f'ERROR: {exc}'); raise
            finally:
                try:
                    if flying and connected():
                        s=state(); event('PID LANDING AT CURRENT XY')
                        start=time.monotonic()
                        while time.monotonic()-start<3 and connected():
                            z=max(initial['z']+.03,s['z']-(time.monotonic()-start)*max(.1,(s['z']-initial['z'])/3))
                            cf.commander.send_position_setpoint(s['x'],s['y'],z,s['yaw'])
                            time.sleep(.02)
                except Exception as exc: event(f'LANDING FAILED: {exc}')
                finally:
                    cleanup('stop',cf.commander.send_stop_setpoint)
                    if flying: cleanup('disarm',lambda:arming.send_arming_request(False))
                    for config in configs: cleanup('log stop',config.stop)
                    (a.output/'summary.json').write_text(json.dumps(result,indent=2))


if __name__ == '__main__': main()
