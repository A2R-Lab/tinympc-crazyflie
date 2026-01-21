#!/usr/bin/env python3
"""
Simple test script to verify position control works.
Sends the drone left/right to test if MPC+PID actually moves it.
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

# URI of the Crazyflie
URI = 'radio://0/80/2M/E7E7E7E7E7'

def simple_position_test():
    """Test basic position control by moving left and right"""
    
    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        # Wait for position estimate to stabilize
        print("Waiting for position estimate...")
        time.sleep(2)
        
        # Use high-level commander for position control
        cf.param.set_value('commander.enHighLevel', '1')
        cf.param.set_value('stabilizer.controller', '6')  # OOT controller
        
        print("Taking off...")
        cf.high_level_commander.takeoff(0.5, 2.0)  # 50cm height, 2s duration
        time.sleep(3)
        
        print("Hover at origin for 3s...")
        cf.high_level_commander.go_to(0.0, 0.0, 0.5, 0.0, 2.0)
        time.sleep(3)
        
        print("Move LEFT to x=-0.3...")
        cf.high_level_commander.go_to(-0.3, 0.0, 0.5, 0.0, 2.0)
        time.sleep(3)
        
        print("Move RIGHT to x=0.3...")
        cf.high_level_commander.go_to(0.3, 0.0, 0.5, 0.0, 2.0)
        time.sleep(3)
        
        print("Return to origin...")
        cf.high_level_commander.go_to(0.0, 0.0, 0.5, 0.0, 2.0)
        time.sleep(3)
        
        print("Landing...")
        cf.high_level_commander.land(0.0, 2.0)
        time.sleep(3)
        
        print("Done!")

if __name__ == '__main__':
    simple_position_test()
