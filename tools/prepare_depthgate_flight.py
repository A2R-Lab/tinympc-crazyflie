"""Configure avoidance while disarmed and verify live data; never arm or take off."""
import json
import signal
import threading
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(TimeoutError()))
signal.alarm(60)
cflib.crtp.init_drivers()
with SyncCrazyflie('radio://0/80/2M/E7E7E7E7E7', cf=Crazyflie()) as scf:
    scf.wait_for_params()
    cf = scf.cf
    assert 'dgAvoid' in cf.param.values and 'dgBench' not in cf.log.toc.toc
    assert cf.param.values['stabilizer']['controller'] == '1'
    assert cf.param.values['espTest']['run'] == '0'
    safety = []
    config = LogConfig('dgSafety', 100)
    config.add_variable('supervisor.info', 'uint16_t')
    for i in range(1, 5):
        config.add_variable(f'motor.m{i}', 'uint16_t')
    config.data_received_cb.add_callback(lambda t, d, c: safety.append(dict(d)))
    cf.log.add_config(config)
    config.start()
    time.sleep(1)
    def safe():
        return len(safety) >= 5 and all(
            not int(d['supervisor.info']) & 18 and
            all(d[f'motor.m{i}'] == 0 for i in range(1, 5)) for d in safety)
    assert safe(), 'Must be disarmed with all motors zero'
    requested = {'dgAvoid.speed': .5, 'dgAvoid.clearance': .5,
                 'dgAvoid.enable': 1, 'espTest.external': 0}
    for name, value in requested.items():
        assert safe()
        cf.param.set_value(name, str(value))
        group, key = name.split('.')
        deadline = time.monotonic() + 3
        while abs(float(cf.param.values[group][key]) - value) > 1e-5:
            if time.monotonic() > deadline:
                raise TimeoutError(f'Parameter not confirmed: {name}')
            time.sleep(.02)
    packets = []
    lc = LogConfig('dgLive', 100)
    for name, kind in {'dg.seq': 'uint16_t', 'dg.status': 'uint16_t',
                       'dg.inferUs': 'uint32_t', 'dg.invLeft': 'float',
                       'dg.invCenter': 'float', 'dg.invRight': 'float',
                       'dgRx.ok': 'uint32_t'}.items():
        lc.add_variable(name, kind)
    lc.data_received_cb.add_callback(lambda t, d, c: packets.append(dict(d)))
    cf.log.add_config(lc)
    lc.start()
    time.sleep(3)
    lc.stop()
    config.stop()
    result = {'params': {g: cf.param.values[g] for g in ('dgAvoid', 'espTest', 'stabilizer')},
              'safety_samples': len(safety), 'safety_ok': safe(),
              'first_packet': packets[0] if packets else None,
              'last_packet': packets[-1] if packets else None,
              'live_packets': bool(packets and packets[-1]['dgRx.ok'] > packets[0]['dgRx.ok']),
              'log_groups': list(cf.log.toc.toc)}
    print(json.dumps(result, indent=2))
    assert safe()
    assert result['live_packets'], 'No fresh GAP8 predictions; do not start the test'
