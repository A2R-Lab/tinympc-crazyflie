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
        description="Takeoff with PID, switch to OOT controller 6, land on keypress."
    )
    parser.add_argument(
        "--uri",
        default=uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7"),
        help="Crazyflie URI (default from CFLIB_URI or radio://0/80/2M/E7E7E7E7E7)",
    )
    parser.add_argument("--takeoff-height", type=float, default=1.0)
    parser.add_argument("--takeoff-time", type=float, default=2.0)
    parser.add_argument("--hover-time", type=float, default=2.0)
    parser.add_argument("--post-switch-hold", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache="./cf_cache")
    with SyncCrazyflie(args.uri, cf=cf) as scf:
        scf.cf.param.set_value("commander/enHighLevel", "1")

        # PID for takeoff
        scf.cf.param.set_value("stabilizer/controller", "1")

        hlc = HighLevelCommander(scf.cf)
        hlc.takeoff(args.takeoff_height, args.takeoff_time)
        time.sleep(args.takeoff_time + 0.5)

        hlc.go_to(0.0, 0.0, args.takeoff_height, 0.0, args.hover_time)
        time.sleep(args.hover_time + 0.5)

        print("Switching to OOT controller 6...")
        scf.cf.param.set_value("stabilizer/controller", "6")
        time.sleep(args.post_switch_hold)

        input("Press Enter to land immediately...")
        hlc.land(0.02, 2.0)
        time.sleep(2.1)
        hlc.stop()


if __name__ == "__main__":
    main()
