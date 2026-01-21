#!/usr/bin/env python3
"""
Test PSD - takeoff with PID, then switch to OOT/MPC-PSD mid-flight.
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import json

URI = 'radio://0/80/2M/E7E7E7E7E7'
DEBUG_LOG_PATH = "/Users/ishaanmahajan/A2R_Research/workspace/tinympc-sdp/.cursor/debug.log"

# #region agent log
def debug_log(hypothesis_id, location, message, data):
    payload = {
        "sessionId": "debug-session",
        "runId": "pre-fix",
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data or {},
        "timestamp": int(time.time() * 1000),
    }
    try:
        with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as fp:
            fp.write(json.dumps(payload) + "\n")
    except Exception:
        pass
# #endregion

def run_test():
    print("Initializing drivers...")
    cflib.crtp.init_drivers()
    
    print(f"Connecting to {URI}...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        print("Connected!")
        
        # Start with PID controller (1) - the safe default
        print("Setting PID controller (1)...")
        cf.param.set_value('stabilizer.controller', '1')
        time.sleep(0.5)
        try:
            print(f"stabilizer.controller now {cf.param.get_value('stabilizer.controller')}")
        except Exception as exc:
            print(f"Param read failed: {exc}")
        
        # Enable high-level commander
        cf.param.set_value('commander.enHighLevel', '1')
        time.sleep(0.5)

        # PSD + target logs to confirm OOT is actually running
        psd_log = LogConfig(name="psdMPC", period_in_ms=500)
        psd_log.add_variable("psdMPC.planMode", "uint8_t")
        psd_log.add_variable("psdMPC.planStatus", "uint8_t")
        psd_log.add_variable("psdMPC.planAge", "int32_t")
        psd_log.add_variable("psdMPC.diskCount", "int32_t")
        psd_log.add_variable("psdMPC.traceGap", "float")
        psd_log.add_variable("psdMPC.etaMin", "float")

        target_log = LogConfig(name="ctrltarget", period_in_ms=500)
        target_log.add_variable("ctrltarget.vx", "float")
        target_log.add_variable("ctrltarget.vy", "float")
        target_log.add_variable("ctrltarget.x", "float")
        target_log.add_variable("ctrltarget.y", "float")

        psd_dbg = LogConfig(name="psdDbg", period_in_ms=500)
        psd_dbg.add_variable("psdDbg.cmdX", "float")
        psd_dbg.add_variable("psdDbg.cmdY", "float")
        psd_dbg.add_variable("psdDbg.cmdVx", "float")
        psd_dbg.add_variable("psdDbg.cmdVy", "float")
        psd_dbg.add_variable("psdDbg.goalX", "float")
        psd_dbg.add_variable("psdDbg.goalY", "float")

        state_log = LogConfig(name="state", period_in_ms=200)
        state_log.add_variable("stateEstimate.x", "float")
        state_log.add_variable("stateEstimate.y", "float")
        state_log.add_variable("stateEstimate.z", "float")

        last_state = {"x": 0.0, "y": 0.0, "z": 0.0}

        def psd_log_callback(timestamp, data, logconf):
            print(
                f"[psd] mode={data['psdMPC.planMode']} status={data['psdMPC.planStatus']} "
                f"age={data['psdMPC.planAge']} disks={data['psdMPC.diskCount']} "
                f"gap={data['psdMPC.traceGap']:.3f} eta={data['psdMPC.etaMin']:.3f}"
            )
            # #region agent log
            debug_log(
                "H2",
                "test_psd_demo.py:psd_log_callback",
                "psd_log",
                {
                    "mode": int(data["psdMPC.planMode"]),
                    "status": int(data["psdMPC.planStatus"]),
                    "age": int(data["psdMPC.planAge"]),
                    "disks": int(data["psdMPC.diskCount"]),
                    "gap": float(data["psdMPC.traceGap"]),
                    "eta": float(data["psdMPC.etaMin"]),
                },
            )
            # #endregion

        def target_log_callback(timestamp, data, logconf):
            print(
                f"[target] vx={data['ctrltarget.vx']:.3f} vy={data['ctrltarget.vy']:.3f} "
                f"x={data['ctrltarget.x']:.3f} y={data['ctrltarget.y']:.3f}"
            )
            # #region agent log
            debug_log(
                "H3",
                "test_psd_demo.py:target_log_callback",
                "target_log",
                {
                    "vx": float(data["ctrltarget.vx"]),
                    "vy": float(data["ctrltarget.vy"]),
                    "x": float(data["ctrltarget.x"]),
                    "y": float(data["ctrltarget.y"]),
                },
            )
            # #endregion

        def state_log_callback(timestamp, data, logconf):
            last_state["x"] = data["stateEstimate.x"]
            last_state["y"] = data["stateEstimate.y"]
            last_state["z"] = data["stateEstimate.z"]
            print(
                f"[state] x={data['stateEstimate.x']:.3f} "
                f"y={data['stateEstimate.y']:.3f} z={data['stateEstimate.z']:.3f}"
            )
            # #region agent log
            debug_log(
                "H1",
                "test_psd_demo.py:state_log_callback",
                "state_log",
                {
                    "x": float(data["stateEstimate.x"]),
                    "y": float(data["stateEstimate.y"]),
                    "z": float(data["stateEstimate.z"]),
                },
            )
            # #endregion

        def psd_dbg_callback(timestamp, data, logconf):
            print(
                f"[mpc] cmd=({data['psdDbg.cmdX']:.3f},{data['psdDbg.cmdY']:.3f}) "
                f"v=({data['psdDbg.cmdVx']:.3f},{data['psdDbg.cmdVy']:.3f}) "
                f"goal=({data['psdDbg.goalX']:.3f},{data['psdDbg.goalY']:.3f})"
            )
            # #region agent log
            debug_log(
                "H2",
                "test_psd_demo.py:psd_dbg_callback",
                "psd_dbg",
                {
                    "cmdX": float(data["psdDbg.cmdX"]),
                    "cmdY": float(data["psdDbg.cmdY"]),
                    "cmdVx": float(data["psdDbg.cmdVx"]),
                    "cmdVy": float(data["psdDbg.cmdVy"]),
                    "goalX": float(data["psdDbg.goalX"]),
                    "goalY": float(data["psdDbg.goalY"]),
                },
            )
            # #endregion

        try:
            cf.log.add_config(psd_log)
            if psd_log.valid:
                psd_log.data_received_cb.add_callback(psd_log_callback)
                psd_log.start()
                debug_log("H4", "test_psd_demo.py:psd_log", "psd_log_started", {"valid": True})
            else:
                print("psdMPC log not valid (OOT firmware/log group missing?)")
                debug_log("H4", "test_psd_demo.py:psd_log", "psd_log_invalid", {"valid": False})
        except Exception as exc:
            print(f"psdMPC log setup failed: {exc}")
            debug_log("H4", "test_psd_demo.py:psd_log", "psd_log_failed", {"error": str(exc)})

        try:
            cf.log.add_config(target_log)
            if target_log.valid:
                target_log.data_received_cb.add_callback(target_log_callback)
                target_log.start()
                debug_log("H4", "test_psd_demo.py:target_log", "target_log_started", {"valid": True})
            else:
                print("ctrltarget log not valid")
                debug_log("H4", "test_psd_demo.py:target_log", "target_log_invalid", {"valid": False})
        except Exception as exc:
            print(f"ctrltarget log setup failed: {exc}")
            debug_log("H4", "test_psd_demo.py:target_log", "target_log_failed", {"error": str(exc)})

        try:
            cf.log.add_config(psd_dbg)
            if psd_dbg.valid:
                psd_dbg.data_received_cb.add_callback(psd_dbg_callback)
                psd_dbg.start()
                debug_log("H4", "test_psd_demo.py:psd_dbg", "psd_dbg_started", {"valid": True})
            else:
                print("psdDbg log not valid")
                debug_log("H4", "test_psd_demo.py:psd_dbg", "psd_dbg_invalid", {"valid": False})
        except Exception as exc:
            print(f"psdDbg log setup failed: {exc}")
            debug_log("H4", "test_psd_demo.py:psd_dbg", "psd_dbg_failed", {"error": str(exc)})

        try:
            cf.log.add_config(state_log)
            if state_log.valid:
                state_log.data_received_cb.add_callback(state_log_callback)
                state_log.start()
                debug_log("H4", "test_psd_demo.py:state_log", "state_log_started", {"valid": True})
            else:
                print("stateEstimate log not valid")
                debug_log("H4", "test_psd_demo.py:state_log", "state_log_invalid", {"valid": False})
        except Exception as exc:
            print(f"stateEstimate log setup failed: {exc}")
            debug_log("H4", "test_psd_demo.py:state_log", "state_log_failed", {"error": str(exc)})
        
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
        
        print("\n=== TAKEOFF with PID ===")
        cf.high_level_commander.takeoff(0.4, 2.0)
        time.sleep(3.0)
        print("\n=== STABILIZE with PID at (0,0) for 2 seconds ===")
        for i in range(2):
            cf.high_level_commander.go_to(0.0, 0.0, 0.4, 0, 1.2)
            time.sleep(1)
            print(f"PID hover: {i+1}s")
        
        # Switch to OOT/MPC-PSD
        print("\n=== SWITCHING TO OOT (6) / MPC-PSD ===")
        cf.param.set_value('stabilizer.controller', '6')
        # #region agent log
        debug_log(
            "H4",
            "test_psd_demo.py:controller_switch",
            "switch_to_oot",
            {"controller": 6},
        )
        # #endregion
        
        goal_x = 0.0
        goal_y = 0.0
        print("\n=== HOVER with MPC-PSD for 10 seconds ===")
        print("Obstacle comes from RIGHT (-Y) toward origin; drone should move LEFT (+Y)")
        for i in range(10):
            cf.high_level_commander.go_to(goal_x, goal_y, 0.4, 0, 1.2)
            time.sleep(1)
            print(f"MPC-PSD: {i+1}s")
            # #region agent log
            debug_log(
                "H1",
                "test_psd_demo.py:mpc_loop",
                "mpc_goal",
                {"goal_x": goal_x, "goal_y": goal_y, "tick": i + 1},
            )
            # #endregion
        
        # Land immediately without intermediate holds
        print("\n=== LANDING IN PLACE (OOT) ===")
        print("\nLanding...")
        cf.high_level_commander.land(0.0, 1.5)
        time.sleep(2.0)
        
        cf.high_level_commander.stop()
        try:
            psd_log.stop()
            target_log.stop()
            state_log.stop()
            psd_dbg.stop()
        except Exception:
            pass
        print("Done!")

if __name__ == '__main__':
    run_test()
