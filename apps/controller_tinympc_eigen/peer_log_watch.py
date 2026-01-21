#!/usr/bin/env python3
"""
Simple log watcher for peer localization data.

Prints peer.id0/id1 + positions to verify obstacle reception.
"""
import os
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


CF_URI = os.getenv("CF_URI", "radio://0/80/2M/E7E7E7E7E7")
LOG_PERIOD_MS = int(os.getenv("LOG_PERIOD_MS", "200"))
LOG_TWO_PEERS = os.getenv("LOG_TWO_PEERS", "0") == "1"


def main():
    cflib.crtp.init_drivers()
    lg = LogConfig(name="peer", period_in_ms=LOG_PERIOD_MS)
    vars_one = ("peer.id0", "peer.x0", "peer.y0", "peer.z0", "peer.t0")
    vars_two = ("peer.id1", "peer.x1", "peer.y1", "peer.z1", "peer.t1")
    for name in vars_one + (vars_two if LOG_TWO_PEERS else ()):
        lg.add_variable(name)

    with SyncCrazyflie(CF_URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        try:
            scf.cf.log.add_config(lg)
            lg.start()
        except Exception as exc:
            print(f"Log setup failed: {exc}")
            print("Make sure firmware includes the peer log group and is flashed.")
            return

        last = 0.0

        def cb(timestamp, data, _):
            nonlocal last
            now = time.time()
            if now - last < 0.2:
                return
            last = now
            msg = (f"id0={data['peer.id0']} "
                   f"p0=({data['peer.x0']:.2f},{data['peer.y0']:.2f},{data['peer.z0']:.2f})")
            if LOG_TWO_PEERS:
                msg += (f" id1={data['peer.id1']} "
                        f"p1=({data['peer.x1']:.2f},{data['peer.y1']:.2f},{data['peer.z1']:.2f})")
            print(msg)

        lg.data_received_cb.add_callback(cb)
        print("Logging peer positions... Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            lg.stop()


if __name__ == "__main__":
    sys.exit(main())
