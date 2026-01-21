#!/usr/bin/env python3
"""
Test PSD - use high-level commander like cfclient does.
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/2M/E7E7E7E7E7'

def run_test():
    print("Initializing drivers...")
    cflib.crtp.init_drivers()
    
    print(f"Connecting to {URI}...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        print("Connected!")
        
        # Set OOT controller
        print("Setting OOT controller (6)...")
        cf.param.set_value('stabilizer.controller', '6')
        time.sleep(0.5)
        
        # Enable high-level commander
        cf.param.set_value('commander.enHighLevel', '1')
        time.sleep(0.5)
        
        # Set initial position
        cf.param.set_value('kalman.initialX', 0.0)
        cf.param.set_value('kalman.initialY', 0.0)
        cf.param.set_value('kalman.initialZ', 0.0)
        cf.param.set_value('kalman.initialYaw', 0.0)
        
        # Reset estimator
        print("Resetting estimator...")
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        
        # ARM
        print("ARMING...")
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        
        print("\n=== TAKEOFF (high-level commander) ===")
        cf.high_level_commander.takeoff(0.4, 2.0)
        time.sleep(3.0)
        
        print("\n=== HOVER - PSD DEMO ===")
        print("Hovering at (0,0,0.4) - PSD demo active!")
        print("Watch cfclient console for 'PSD: ACTIVE' messages...")
        
        # Send go_to commands to maintain position
        for i in range(20):
            cf.high_level_commander.go_to(0, 0, 0.4, 0, 0.5)
            time.sleep(1)
            print(f"Time: {i+1}s")
        
        print("\nLanding...")
        cf.high_level_commander.land(0.0, 2.0)
        time.sleep(3)
        
        cf.high_level_commander.stop()
        print("Done!")

if __name__ == '__main__':
    run_test()
