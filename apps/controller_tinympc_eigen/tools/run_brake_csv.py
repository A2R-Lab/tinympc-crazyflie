#!/usr/bin/env python3
"""Record disarmed by default; --fly runs a PID/OOT vision mission."""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

FLOAT = 'float'
GROUPS = {
    'vision': [(f'collision.{n}', FLOAT) for n in ('pLeft','pCenter','pRight')] + [('collision.valid','uint8_t'),('collision.ageMs','uint32_t'),('collision.seq','uint16_t')],
    'constraints': [(f'mpcLimit.{n}', FLOAT) for n in ('tiltErr','rateErr','primal','solveUs')],
    'position': [(f'stateEstimate.{n}', FLOAT) for n in ('x','y','z','vx','vy','vz')],
    'attitude': [(f'stateEstimate.{n}', FLOAT) for n in ('roll','pitch','yaw')] + [(f'gyro.{n}', FLOAT) for n in ('x','y','z')],
    'reference': [(f'mpcDir.{n}', FLOAT) for n in ('refX','refVx','vx','pitchR','wy')],
    'forces': [(f'mpcDir.u{i}', FLOAT) for i in range(4)],
    'motors': [(f'motor.m{i}', 'uint16_t') for i in range(1,5)] + [('supervisor.info','uint16_t')],
    'test': [('espTest.handoff','uint16_t'),('espTest.phase','uint8_t'),('espTest.reason','uint8_t'),('espTest.run','uint8_t'),('espTest.brakeFrames','uint8_t'),('espTest.clearFrames','uint8_t'),('espTest.holdLocked','uint8_t')] + [(f'espTest.{n}', FLOAT) for n in ('distance','speed')],
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--uri', default='radio://0/80/2M/E7E7E7E7E7')
    p.add_argument('--fly', action='store_true')
    p.add_argument('--gate-logs', action='store_true', help='Record gate observations and visual-servo state (requires gate firmware)')
    p.add_argument('--ellipse', action='store_true', help='Run an ellipse with gate alignment and vision braking; requires --major and --minor')
    p.add_argument('--major', type=float, help='Full ellipse major-axis length in meters (not radius)')
    p.add_argument('--minor', type=float, help='Full ellipse minor-axis length in meters (not radius)')
    p.add_argument('--speed', type=float, default=2, help='Distance-test target or ellipse speed in m/s (default: 2)')
    p.add_argument('--distance', type=float, help='Brake after this measured straight-line distance in meters; disables vision/gate maneuvers')
    p.add_argument('--laps', type=int, default=1, help='Ellipse lap count, 1–10 (default: 1)')
    p.add_argument('--mission-timeout', type=float, default=120, help='Ellipse mission timeout in seconds, 5–600 (default: 120)')
    p.add_argument('--height', type=float, default=.5)
    p.add_argument('--seconds', type=float, default=25)
    p.add_argument('--output', type=Path, default=Path('brake-recordings') / time.strftime('%Y%m%d-%H%M%S'))
    a = p.parse_args()
    if a.distance is not None:
        if a.ellipse or a.gate_logs:
            p.error('--distance cannot be combined with --ellipse or --gate-logs')
        if not math.isfinite(a.distance) or not 0 < a.distance <= 20:
            p.error('--distance must be greater than 0 and at most 20 m')
        if not math.isfinite(a.speed) or not 0 < a.speed <= 12:
            p.error('--speed must be greater than 0 and at most 12 m/s')
    if a.ellipse:
        if (a.major is None or a.minor is None or not math.isfinite(a.major)
                or not math.isfinite(a.minor) or not .5 <= a.minor <= a.major <= 20):
            p.error('--ellipse requires full axis lengths: 0.5 <= --minor <= --major <= 20 meters')
        if not math.isfinite(a.speed) or not 0 < a.speed <= 2:
            p.error('ellipse --speed must be greater than 0 and at most 2 m/s')
        if not 1 <= a.laps <= 10:
            p.error('ellipse --laps must be 1–10')
        if not math.isfinite(a.mission_timeout) or not 5 <= a.mission_timeout <= 600:
            p.error('ellipse --mission-timeout must be 5–600 seconds')
        a.gate_logs = True
        GROUPS['ellipse'] = [('ellipse.phase', 'uint8_t')] + [
            (f'ellipse.{n}', FLOAT) for n in ('theta','yawRef','refX','refY','vRef')]
    elif a.major is not None or a.minor is not None:
        p.error('--major and --minor require --ellipse')
    if a.gate_logs:
        GROUPS.update({
            'gate_nav': [(f'gateNav.{n}', 'uint8_t') for n in ('phase','rails','center','reject')] + [(f'gateNav.{n}', FLOAT) for n in ('errX','errY','distance','vLeft','vUp')],
            'gate_seen': [(f'gate.{n}', FLOAT) for n in ('pLeft','pRight')] + [('gate.valid','uint8_t'),('gate.edgeMask','uint8_t'),('gate.ageMs','uint32_t'),('gate.seq','uint16_t')],
            'gate_top': [(f'gate.{c}{n}', FLOAT) for c in ('lt','rt') for n in ('X','Y','Conf')],
            'gate_bottom': [(f'gate.{c}{n}', FLOAT) for c in ('lb','rb') for n in ('X','Y','Conf')],
        })
    if not .2 <= a.height <= 1 or not 1 <= a.seconds <= 120:
        p.error('height must be 0.2–1 m; seconds must be 1–120')
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
        config = LogConfig(name, 100 if name.startswith('gate_') or name == 'ellipse' else 50)
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
            cf.commander.send_hover_setpoint(0,0,0,height)
            if predicate and predicate(state):
                return
            time.sleep(.02)
        if predicate:
            raise RuntimeError('Timed out waiting for flight state')
    def require_clear_vision(state):
        if a.distance is not None:
            return
        scores = [state[f'collision.{n}'] for n in ('pLeft','pCenter','pRight')]
        if not state['collision.valid'] or state['collision.ageMs'] > 400 or not all(math.isfinite(v) and 0 <= v <= 1 for v in scores):
            raise RuntimeError('Vision unavailable/stale; require fresh AI-deck packets')
        if scores[1] > .95:
            raise RuntimeError('Vision already blocked: center > 0.95')
    flying = False
    try:
        cflib.crtp.init_drivers(); cf.open_link(a.uri)
        if not ready.wait(25):
            raise RuntimeError('Connection timeout')
        for name, fields in GROUPS.items():
            start_log(name, fields)
        time.sleep(1)
        s = snapshot()
        if not a.fly:
            event('Recording only; no flight commands')
            end=time.monotonic()+a.seconds
            while time.monotonic()<end:
                snapshot(); time.sleep(.1)
            return
        if cf.param.get_value('system.selftestPassed') != '1' or any(s[f'motor.m{i}'] != 0 for i in range(1,5)):
            raise RuntimeError('Require passed self-test and stopped motors')
        require_clear_vision(s)
        if a.gate_logs and (not s['gate.valid'] or s['gate.ageMs'] > 400):
            raise RuntimeError('Gate telemetry unavailable/stale; update the AI-deck sender first')
        cf.param.set_value('stabilizer.controller',1)
        cf.param.set_value('espTest.run',0)
        params = cf.param.toc.toc.get('espTest', {})
        if a.distance is not None:
            if not all(name in params for name in ('speed','distance')):
                raise RuntimeError('Flash distance-test firmware first: espTest.speed/distance absent')
            cf.param.set_value('espTest.speed', a.speed)
            cf.param.set_value('espTest.distance', a.distance)
        elif 'distance' in params:
            cf.param.set_value('espTest.distance', 0)
        if 'external' in cf.param.toc.toc.get('espTest', {}):
            cf.param.set_value('espTest.external', 0)
        has_ellipse = 'ellipse' in cf.param.toc.toc.get('espTest', {})
        if a.ellipse and not has_ellipse:
            raise RuntimeError('This firmware supports the straight mission only; omit --ellipse')
        if a.ellipse:
            for field, value in (('major', a.major), ('minor', a.minor),
                                 ('speed', a.speed), ('laps', a.laps),
                                 ('timeout', a.mission_timeout)):
                cf.param.set_value(f'ellCfg.{field}', value)
        if has_ellipse:
            cf.param.set_value('espTest.ellipse', int(a.ellipse))
        event(f'Ellipse configured: axes={a.major} x {a.minor} m; speed ceiling={a.speed} m/s; laps={a.laps}'
              if a.ellipse else 'Straight mission selected')
        time.sleep(.3)
        cf.commander.send_setpoint(0,0,0,0)
        flying=True
        arming.send_arming_request(True)
        event('PID takeoff/hover')
        hover(3,a.height)
        stable=[None]
        def stationary(s):
            speed=math.sqrt(sum(s[f'stateEstimate.v{n}']**2 for n in 'xyz'))
            good=speed<.12 and abs(s['stateEstimate.z']-a.height)<.1
            if not good: stable[0]=None
            elif stable[0] is None: stable[0]=time.monotonic()
            return stable[0] is not None and time.monotonic()-stable[0]>.5
        hover(8,a.height,stationary)
        previous_handoff = snapshot()['espTest.handoff']
        cf.param.set_value('stabilizer.controller',6); event('OOT selected')
        # Wait only for initialization acknowledgement, not another hover dwell.
        hover(1,a.height,lambda s:s['espTest.handoff'] != previous_handoff)
        s = snapshot()
        require_clear_vision(s)
        if a.ellipse and (not s['gate.valid'] or s['gate.ageMs'] > 400):
            raise RuntimeError('Gate telemetry became unavailable/stale before mission start')
        cf.param.set_value('espTest.run',1)
        event(f'Distance brake test requested: target={a.speed} m/s; measured distance={a.distance} m; vision OFF'
              if a.distance is not None else
              ('Ellipse' if a.ellipse else 'Straight') + ' mission requested (center > 0.95 brakes; three clear frames resume)')
        hover(2,a.height,lambda s:s['espTest.phase'] in (1, 2))
        deadline = time.monotonic() + (a.mission_timeout + 5 if a.ellipse else 18)
        previous_phase = snapshot()['espTest.phase']
        saw_brake = previous_phase == 2
        while time.monotonic() < deadline:
            s = snapshot()
            cf.commander.send_hover_setpoint(0,0,0,a.height)
            phase, reason = s['espTest.phase'], s['espTest.reason']
            if phase != previous_phase:
                if phase == 2 and reason == 1:
                    saw_brake = True
                    event('Vision braking detected; waiting for three clear frames')
                elif phase == 1 and previous_phase == 2:
                    event('Vision clear; firmware resumed path travel')
                previous_phase = phase
            if phase == 2 and reason != 1:
                break
            time.sleep(.02)
        else:
            raise RuntimeError('Timed out waiting for terminal mission stop')
        event('Terminal stop detected')
        handoff_stable = [None]
        def ready_for_pid(s):
            horizontal = math.hypot(s['stateEstimate.vx'], s['stateEstimate.vy'])
            good = (horizontal < .3 and abs(s['stateEstimate.vz']) < .2
                    and abs(s['stateEstimate.roll']) < 15
                    and abs(s['stateEstimate.pitch']) < 15
                    and max(abs(s[f'gyro.{axis}']) for axis in 'xyz') < 60)
            if not good:
                handoff_stable[0] = None
            elif handoff_stable[0] is None:
                handoff_stable[0] = time.monotonic()
            return handoff_stable[0] is not None and time.monotonic() - handoff_stable[0] >= .2
        hover(5, a.height, ready_for_pid)
        event('Braking handoff ready; landing immediately')
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
                z = max(.05, min(1, measured_z)) if math.isfinite(measured_z) else a.height
                cf.param.set_value('espTest.run',0)
                cf.param.set_value('stabilizer.controller',1)
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
