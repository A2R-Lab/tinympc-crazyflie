#!/usr/bin/env python3
"""Configurable ellipse laps using pure MPC and tangent yaw (default 3 m x 2 m).

Default: generate reference.csv only. --fly connects, takes off, flies and lands.
Start at the near minor-axis endpoint, facing along the major axis; the ellipse
center is minor/2 meters to the left. Height is relative to the initial estimated z.
Requires firmware exposing espTest.external. External mode bypasses vision.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time

from ellipse_reference import sample, semiaxes

CRUISE_SPEED = 2.0
ACCELERATION = 1.0


def trajectory(dt=.02, target_speed=CRUISE_SPEED, major=3.0, minor=2.0, laps=1):
    if not isinstance(laps, int) or isinstance(laps, bool) or laps < 1:
        raise ValueError('laps must be a positive integer')
    a, b = semiaxes(major, minor)
    if not math.isfinite(target_speed) or target_speed <= 0:
        raise ValueError('target_speed must be finite and greater than zero')
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be finite and greater than zero')
    # Arc-length lookup allows smooth acceleration/deceleration without changing
    # the ellipse dimensions or using a constant angular speed.
    angles = [i * 2 * math.pi / 20000 for i in range(20001)]
    lengths = [0.0]
    for lo, hi in zip(angles, angles[1:]):
        mid = (lo + hi) / 2
        lengths.append(lengths[-1] + (hi-lo)*math.hypot(a*math.cos(mid), b*math.sin(mid)))
    import bisect
    distance, elapsed = 0.0, 0.0
    lap_length = lengths[-1]
    length = lap_length * laps
    # Preserve 1 m/s^2 acceleration and deceleration with consistent arc length.
    # Ramp once at the start and once at the end of the complete mission.
    # If the mission is too short to reach the target, use a triangular profile.
    peak_speed = min(target_speed, math.sqrt(length * ACCELERATION))
    ramp_time = peak_speed / ACCELERATION
    duration = length / peak_speed + ramp_time
    while elapsed < duration:
        speed = min(ACCELERATION*elapsed, peak_speed, ACCELERATION*(duration-elapsed))
        distance = (.5*ACCELERATION*elapsed**2 if elapsed < ramp_time else
                    length-.5*ACCELERATION*(duration-elapsed)**2 if elapsed > duration-ramp_time
                    else peak_speed*(elapsed-.5*ramp_time))
        completed_laps, local_distance = divmod(distance, lap_length)
        i = max(1, min(len(lengths)-1, bisect.bisect_left(lengths, local_distance)))
        theta = completed_laps*2*math.pi + angles[i-1] + (angles[i]-angles[i-1])*(local_distance-lengths[i-1])/(lengths[i]-lengths[i-1])
        yield elapsed, theta, speed
        elapsed += dt
    yield duration, laps*2*math.pi, 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true')
    parser.add_argument('--record-seconds', type=float, default=0,
                        help='Connect and record diagnostics without flight commands')
    parser.add_argument('--laps', type=int, default=1,
                        help='Number of continuous laps before landing (default: 1)')
    parser.add_argument('--major', type=float, default=3.0,
                        help='Full major-axis length in meters (default: 3)')
    parser.add_argument('--minor', type=float, default=2.0,
                        help='Full minor-axis length in meters (default: 2)')
    parser.add_argument('--speed', type=float, default=CRUISE_SPEED,
                        help='Target speed in m/s (default: 2); limited by lap length and acceleration')
    parser.add_argument('--height', type=float, default=.5)
    parser.add_argument('--uri', default='radio://0/80/2M/E7E7E7E7E7')
    parser.add_argument('--output', type=Path,
                        default=Path('ellipse-recordings') / time.strftime('%Y%m%d-%H%M%S'))
    args = parser.parse_args()
    if not math.isfinite(args.record_seconds) or args.record_seconds < 0 or (args.fly and args.record_seconds):
        parser.error('--record-seconds must be nonnegative and cannot be combined with --fly')
    if args.laps < 1:
        parser.error('--laps must be a positive integer')
    try:
        semiaxes(args.major, args.minor)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error('--speed must be finite and greater than zero')
    if not math.isfinite(args.height) or not .2 <= args.height <= 1.5:
        parser.error('--height must be 0.2–1.5 meters')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'metadata.json').write_text(json.dumps(vars(args), default=str, indent=2))
    plan = list(trajectory(target_speed=args.speed, major=args.major, minor=args.minor, laps=args.laps))
    ramp_time = min(args.speed / ACCELERATION, plan[-1][0] / 2)
    with (args.output/'reference.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s','theta_rad','speed_m_s','x','y','vx','vy','yaw_rad','yaw_rate_rad_s'])
        for t, theta, speed in plan:
            writer.writerow([t, theta, speed, *sample(theta, speed, major=args.major, minor=args.minor)])
    print(f'{args.major:g} m x {args.minor:g} m ellipse; target {args.speed:g} m/s; planned peak {max(r[2] for r in plan):.3f} m/s; tangent yaw; {args.laps} lap(s) in {plan[-1][0]:.2f}s')
    print(f'Recording: {args.output.resolve()}')
    if not args.fly and args.record_seconds == 0:
        print('Preview only. Add --fly to execute with the external-reference MPC firmware.')
        return

    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    cflib.crtp.init_drivers()
    lock = threading.Lock()
    link_lost = threading.Event()
    latest, received = {}, {}
    groups = {'position': ['x','y','z','vx','vy','vz'], 'attitude': ['yaw','roll','pitch'],
              'motors': ['m1','m2','m3','m4']}
    with (args.output/'telemetry.csv').open('w', newline='') as f, \
         (args.output/'diagnostics.csv').open('w', newline='') as diagnostic_file, \
         (args.output/'console.txt').open('w', buffering=1) as console_file, \
         (args.output/'events.txt').open('w', buffering=1) as event_file:
        def event(message):
            event_file.write(f'{time.monotonic():.6f} {message}\n')
        diagnostic_writer = csv.writer(diagnostic_file)
        diagnostic_writer.writerow(['host_time','device_ms','group','values_json'])
        writer = csv.writer(f)
        state_fields = ('x','y','z','vx','vy','vz','yaw','roll','pitch')
        writer.writerow(['host_time','device_ms','group',*state_fields,'m1','m2','m3','m4'])
        cf = Crazyflie(rw_cache=str(args.output/'cache'))
        cf.console.receivedChar.add_callback(lambda text: console_file.write(text))
        def disconnected(uri, message=''):
            link_lost.set()
            event(f'LINK CLOSED {uri}: {message}')
        cf.connection_lost.add_callback(disconnected)
        cf.disconnected.add_callback(disconnected)
        with SyncCrazyflie(args.uri, cf=cf) as scf:
            cf = scf.cf
            if 'external' not in cf.param.toc.toc.get('espTest', {}):
                raise RuntimeError('Flash the updated firmware first: espTest.external is absent')
            configs = []
            def record(timestamp, data, config):
                with lock:
                    latest.update(data)
                    received[config.name] = time.monotonic()
                    writer.writerow([time.monotonic(), timestamp, config.name,
                                     *[latest.get('stateEstimate.'+n, '') for n in state_fields],
                                     *[latest.get('motor.m'+str(i), '') for i in range(1,5)]])
                    diagnostic_writer.writerow([time.monotonic(),timestamp,config.name,json.dumps(data)])
            for name, fields in groups.items():
                config = LogConfig(name, 50)
                for field in fields:
                    config.add_variable(('motor.' if name == 'motors' else 'stateEstimate.')+field,
                                        'uint16_t' if name == 'motors' else 'float')
                cf.log.add_config(config)
                config.data_received_cb.add_callback(record)
                config.start()
                configs.append(config)
            # Optional groups preserve compatibility with older firmware. Raw
            # per-packet timestamps distinguish telemetry gaps from resets.
            diagnostic_groups = {
                'solver': [('mpcLimit.'+n,'float') for n in ('tiltErr','rateErr','primal','solveUs')],
                'gyro': [('gyro.'+n,'float') for n in ('x','y','z')],
                'health': [('pm.vbat','float'),('supervisor.info','uint16_t'),('stabilizer.intToOut','uint32_t')],
            }
            for name, fields in diagnostic_groups.items():
                config = LogConfig(name, 100)
                for variable, dtype in fields:
                    group, field = variable.split('.')
                    if field in cf.log.toc.toc.get(group, {}):
                        config.add_variable(variable, dtype)
                    else:
                        event(f'LOG UNAVAILABLE {variable}')
                if config.variables:
                    cf.log.add_config(config)
                    config.data_received_cb.add_callback(record)
                    config.start()
                    configs.append(config)
            def pose():
                if link_lost.is_set():
                    raise RuntimeError('Connection lost; ending mission')
                with lock:
                    if any(time.monotonic()-received.get(g, 0) > .5 for g in groups):
                        raise RuntimeError('Pose telemetry stale; ending mission')
                    values = [latest['stateEstimate.'+n] for n in ('x','y','z','vx','vy','vz','yaw')]
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError('Invalid pose')
                return values
            time.sleep(1)
            if not args.fly:
                event('RECORD ONLY; NO FLIGHT COMMANDS')
                try:
                    time.sleep(args.record_seconds)
                finally:
                    for config in configs:
                        config.stop()
                return
            initial = pose()
            if cf.param.get_value('system.selftestPassed') != '1':
                raise RuntimeError('Firmware self-test has not passed')
            with lock:
                if any(latest['motor.m'+str(i)] != 0 for i in range(1,5)):
                    raise RuntimeError('Require stopped motors before starting the script')
            origin, heading = initial[:2], math.radians(initial[6])
            height = initial[2] + args.height
            arming = getattr(cf, 'supervisor', cf.platform)
            flying = False
            try:
                cf.param.set_value('stabilizer.controller', 1)
                cf.param.set_value('espTest.run', 0)
                cf.param.set_value('espTest.external', 1)
                time.sleep(.3)
                cf.commander.send_setpoint(0, 0, 0, 0)
                flying = True
                arming.send_arming_request(True)
                start = time.monotonic()
                while time.monotonic()-start < 3:
                    pose()
                    z = initial[2] + args.height*min(1, (time.monotonic()-start)/2)
                    cf.commander.send_position_setpoint(*origin, z, initial[6])
                    time.sleep(.02)
                def send(theta, speed, tangential_acc=0):
                    x,y,vx,vy,yaw,rate = sample(theta, speed, origin, heading, args.major, args.minor)
                    ax = -rate*vy + tangential_acc*math.cos(yaw)
                    ay = rate*vx + tangential_acc*math.sin(yaw)
                    cf.commander.send_full_state_setpoint(
                        [x,y,height], [vx,vy,0], [ax,ay,0],
                        # The installed encoder packs milliradians/s, matching
                        # firmware fullStateDecoder (despite its docstring).
                        [0,0,math.sin(yaw/2),math.cos(yaw/2)], 0,0,rate)
                # Supply a valid full-state hover before selecting MPC.
                send(0,0)
                cf.param.set_value('stabilizer.controller', 6)
                time.sleep(.02)
                start = time.monotonic()
                print('MPC ellipse started (vision bypassed)')
                event('MPC TRAJECTORY START')
                for t,theta,speed in plan:
                    delay = start+t-time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    if time.monotonic()-start-t > .1:
                        raise RuntimeError('Reference streaming fell behind schedule')
                    pose()
                    tangential_acc = (ACCELERATION if t < ramp_time else
                                      -ACCELERATION if t > plan[-1][0]-ramp_time else 0)
                    if speed == 0:
                        tangential_acc = 0
                    send(theta, speed, tangential_acc)
                print(f'All {args.laps} lap(s) complete; landing')
                event('TRAJECTORY COMPLETE')
            except KeyboardInterrupt:
                print('Interrupted; landing at current XY')
                event('CONTROL-C')
            except Exception as exc:
                event(f'ERROR {type(exc).__name__}: {exc}')
                raise
            finally:
                def connected():
                    return not link_lost.is_set() and cf.is_connected()
                def cleanup_action(label, action):
                    if not connected():
                        event(f'SKIPPED {label}: link unavailable')
                        return
                    try:
                        action()
                        event(label)
                    except Exception as exc:
                        event(f'CLEANUP FAILED {label}: {type(exc).__name__}: {exc}')
                try:
                    if flying and connected():
                        # Capture current XY, never the original takeoff point.
                        current = pose()
                        cf.param.set_value('stabilizer.controller', 1)
                        event('PID LANDING')
                        start = time.monotonic()
                        while time.monotonic()-start < 3:
                            if not connected():
                                break
                            z = max(initial[2]+.03, current[2]-(time.monotonic()-start)*max(.1,(current[2]-initial[2])/3))
                            cf.commander.send_position_setpoint(current[0],current[1],z,current[6])
                            time.sleep(.02)
                except Exception as exc:
                    event(f'LANDING FAILED: {type(exc).__name__}: {exc}')
                finally:
                    cleanup_action('STOP SENT', cf.commander.send_stop_setpoint)
                    if flying:
                        cleanup_action('DISARM REQUEST SENT', lambda: arming.send_arming_request(False))
                    cleanup_action('EXTERNAL MODE CLEARED', lambda: cf.param.set_value('espTest.external', 0))
                    if connected() and 'taskDump' in cf.param.toc.toc.get('system', {}):
                        cleanup_action('TASK DUMP REQUESTED', lambda: cf.param.set_value('system.taskDump', 1))
                        time.sleep(2) # Dump stack high-water marks only after disarm.
                    for config in configs:
                        cleanup_action(f'LOG STOP {config.name}', config.stop)


if __name__ == '__main__':
    main()
