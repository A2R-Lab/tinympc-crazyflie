"""Read motor-inhibited firmware timing counters; sends no control commands."""
import json
import signal
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(TimeoutError()))
signal.alarm(30)
cflib.crtp.init_drivers()
with SyncCrazyflie('radio://0/80/2M/E7E7E7E7E7', cf=Crazyflie()) as scf:
    scf.wait_for_params()
    groups = []
    for i in range(1, 5):
        groups.append({f'dgBench.{key}{i}': 'uint32_t' for key in ('min', 'max', 'mean')})
    groups.append({'dgBench.done': 'uint8_t', 'dgBench.stage': 'uint8_t',
                   'dgBench.stack': 'uint32_t', 'supervisor.info': 'uint16_t',
                   **{f'motor.m{i}': 'uint16_t' for i in range(1, 5)}})
    result = {}
    for i, variables in enumerate(groups):
        event = threading.Event()
        config = LogConfig(f'bench{i}', 100)
        for name, kind in variables.items():
            config.add_variable(name, kind)
        def callback(timestamp, data, logconf):
            result.update(data)
            event.set()
        config.data_received_cb.add_callback(callback)
        scf.cf.log.add_config(config)
        config.start()
        if not event.wait(3):
            raise TimeoutError('Missing benchmark telemetry')
        config.stop()
    print(json.dumps(result, indent=2))
    assert result['dgBench.done'] == 1
    assert not int(result['supervisor.info']) & 18
    assert all(result[f'motor.m{i}'] == 0 for i in range(1, 5))
