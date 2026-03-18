#!/usr/bin/env python3

import argparse
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.utils import uri_helper


def parse_args():
    parser = argparse.ArgumentParser(
        description="Takeoff with PID, then switch to OOT controller 6."
    )
    parser.add_argument(
        "--uri",
        default=uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7"),
        help="Crazyflie URI (default from CFLIB_URI or radio://0/80/2M/E7E7E7E7E7)",
    )
    parser.add_argument("--takeoff-height", type=float, default=1.0)
    parser.add_argument("--takeoff-time", type=float, default=2.0)
    parser.add_argument("--hover-time", type=float, default=2.0)
    parser.add_argument("--hold-after-switch", type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache="./cf_cache")
    with SyncCrazyflie(args.uri, cf=cf) as scf:
        # Enable high-level commander
        scf.cf.param.set_value("commander/enHighLevel", "1")

        # Start on PID (controller 1) for safe takeoff
        scf.cf.param.set_value("stabilizer/controller", "1")

        hlc = HighLevelCommander(scf.cf)
        hlc.takeoff(args.takeoff_height, args.takeoff_time)
        time.sleep(args.takeoff_time + 0.5)

        # Hover at current position/height
        hlc.go_to(0.0, 0.0, args.takeoff_height, 0.0, args.hover_time)
        time.sleep(args.hover_time + 0.5)

        input("Press Enter to switch to controller 6...")
        scf.cf.param.set_value("stabilizer/controller", "6")

        # Hold for a moment after switching
        time.sleep(args.hold_after_switch)

        input("Press Enter to land...")
        hlc.land(0.02, 2.5)
        time.sleep(2.5)
        hlc.stop()


if __name__ == "__main__":
    main()
