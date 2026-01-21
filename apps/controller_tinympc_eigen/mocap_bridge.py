#!/usr/bin/env python3
"""
OptiTrack (NatNet) -> Crazyflie bridge

Sends external position for the CF and optional obstacle position (arm)
as EXT_POSITION_PACKED packets over CRTP.

Place NatNetClient.py from OptiTrack NatNet SDK in the same folder.
"""

import os
import math
import re
import socket
import struct
import sys
import time
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crtp.crtpstack import CRTPPacket

# Suppress noisy NatNet prints by redirecting its prints
class NatNetPrintFilter:
    def __init__(self, orig_stdout):
        self.orig = orig_stdout
        self.suppress_patterns = [
            "MoCap Frame:",
            " MoCap Frame Begin",
            "Frame #:",
            "Markerset Count:",
            "MarkerData:",
            "Model Name :",
            "Marker Count :",
            "Marker ",
            "Unlabeled Marker",
            "Rigid Body Count:",
            "Rigid Body ",
            "ID ",
            "Position ",
            "Orientation ",
            "Marker Error",
            "Tracking Valid",
            "Skeleton Count:",
            "Asset Count:",
            "Labeled Marker",
            "size ",
            "err ",
            "occluded",
            "point_cloud_solved",
            "model_solved",
            "Force Plate",
            "Device Count:",
            "Timecode:",
            "Timestamp",
            "exposure",
            "Camera data",
            "Transmit timestamp",
            "MoCap Frame End",
            "-----------------",
            "[x=",
            "pos ",
        ]
    
    def write(self, text):
        # Only suppress if it matches NatNet noise patterns
        stripped = text.strip()
        if stripped and any(p in stripped for p in self.suppress_patterns):
            return  # suppress
        self.orig.write(text)
    
    def flush(self):
        self.orig.flush()

# Apply filter
_original_stdout = sys.stdout
# Only filter NatNet spam if explicitly requested
if os.getenv("NATNET_LOG_FRAMES", "0") == "1":
    sys.stdout = NatNetPrintFilter(_original_stdout)

try:
    from NatNetClient import NatNetClient
except ImportError:
    sdk_path = "/Users/ishaanmahajan/Downloads/NatNetSDK/Samples/PythonClient"
    if os.path.isdir(sdk_path):
        sys.path.append(sdk_path)
        from NatNetClient import NatNetClient
    else:
        raise SystemExit(
            "NatNetClient.py not found. Install NatNet SDK or copy it "
            "into this folder."
        )

# Keep filtered stdout to suppress NatNet spam; our logs still print.

# -------------------- User config --------------------
WINDOWS_MOTIVE_IP = os.getenv("MOTIVE_IP", "172.27.164.103")
MAC_IP = os.getenv("MAC_IP", "172.27.168.101")
CF_URI = os.getenv("CF_URI", "radio://0/80/2M/E7E7E7E7E7")

# Rigid body naming (Motive)
CF_RB_NAME = os.getenv("CF_RB_NAME", "cf1")
ARM_RB_NAME = os.getenv("ARM_RB_NAME", "arm")

# If NatNet client does not provide names, use IDs instead
CF_RB_ID = int(os.getenv("CF_RB_ID", "1"))
ARM_RB_ID = int(os.getenv("ARM_RB_ID", "-1"))
ARM_PEER_ID = int(os.getenv("ARM_PEER_ID", "2"))

SEND_ARM = os.getenv("SEND_ARM", "0") == "1"
SEND_RATE_HZ = float(os.getenv("SEND_RATE_HZ", "100"))
SEND_CF = os.getenv("SEND_CF", "1") == "1"
SEND_CF_POS = os.getenv("SEND_CF_POS", "1") == "1"
FORCE_CONTROLLER = os.getenv("FORCE_CONTROLLER", "")
FORCE_CONTROLLER_VAL = os.getenv("FORCE_CONTROLLER_VAL", "")
ENABLE_PSD_LOG = os.getenv("ENABLE_PSD_LOG", "0") == "1"
NATNET_USE_MULTICAST = os.getenv("NATNET_MULTICAST", "1") == "1"
NATNET_DATA_PORT = int(os.getenv("NATNET_DATA_PORT", "1511"))
NATNET_CMD_PORT = int(os.getenv("NATNET_CMD_PORT", "1510"))
NATNET_MULTICAST_ADDR = os.getenv("NATNET_MULTICAST_ADDR", "239.255.42.99")
NATNET_SNIFF_SECS = float(os.getenv("NATNET_SNIFF_SECS", "0"))
NATNET_BIND_IP = os.getenv("NATNET_BIND_IP", "")
NATNET_LOG_FRAMES = os.getenv("NATNET_LOG_FRAMES", "0") == "1"  # Default OFF to reduce noise
NO_MOCAP = os.getenv("NO_MOCAP", "0") == "1"
FAKE_CF_POS = os.getenv("FAKE_CF_POS", "0.0,0.0,0.0")
FAKE_ARM_POS = os.getenv("FAKE_ARM_POS", "0.5,0.0,0.0")
FAKE_ARM_MODE = os.getenv("FAKE_ARM_MODE", "static")  # static | circle
FAKE_ARM_CENTER = os.getenv("FAKE_ARM_CENTER", "0.5,0.0,0.0")
FAKE_ARM_RADIUS = float(os.getenv("FAKE_ARM_RADIUS", "0.3"))
FAKE_ARM_PERIOD = float(os.getenv("FAKE_ARM_PERIOD", "6.0"))

# -----------------------------------------------

CRTP_PORT_LOCALIZATION = 0x06
EXT_POSITION_PACKED = 2

latest = {
    "cf": {"x": 0.0, "y": 0.0, "z": 0.0, "ok": False},
    "arm": {"x": 0.0, "y": 0.0, "z": 0.0, "ok": False},
}
last_frame_time = 0.0
last_frame_num = -1
last_frame_log = 0.0


def parse_cf_id(uri: str) -> int:
    match = re.search(r"/([0-9A-Fa-f]{10})$", uri)
    if not match:
        return 0xE7  # default
    addr = match.group(1)
    return int(addr[-2:], 16)


def mm16(val_m: float) -> int:
    val = int(round(val_m * 1000.0))
    if val < -32768:
        return -32768
    if val > 32767:
        return 32767
    return val


def on_rigid_body(rigid_body_id, pos, rot, rigid_body_name=None):
    x, y, z = pos  # Motive usually meters, Z-up

    if rigid_body_name:
        if rigid_body_name == CF_RB_NAME:
            latest["cf"].update({"x": x, "y": y, "z": z, "ok": True})
        elif rigid_body_name == ARM_RB_NAME:
            latest["arm"].update({"x": x, "y": y, "z": z, "ok": True})
        return

    # Name not provided by this NatNet client; fall back to IDs.
    global CF_RB_ID, ARM_RB_ID
    if CF_RB_ID < 0:
        CF_RB_ID = rigid_body_id
        print(f"Auto-selected CF rigid body ID: {CF_RB_ID}")

    if SEND_ARM and ARM_RB_ID < 0 and rigid_body_id != CF_RB_ID:
        ARM_RB_ID = rigid_body_id
        print(f"Auto-selected ARM rigid body ID: {ARM_RB_ID}")

    if rigid_body_id == CF_RB_ID:
        latest["cf"].update({"x": x, "y": y, "z": z, "ok": True})
    elif rigid_body_id == ARM_RB_ID:
        latest["arm"].update({"x": x, "y": y, "z": z, "ok": True})


def on_frame(info):
    global last_frame_time, last_frame_num, last_frame_log
    last_frame_time = time.time()
    last_frame_num = info.get("frame_number", last_frame_num)
    if NATNET_LOG_FRAMES:
        if last_frame_time - last_frame_log > 1.0:
            rb_count = info.get("rigid_body_count", 0)
            print(f"NatNet frame {last_frame_num} (rigid bodies: {rb_count})")
            last_frame_log = last_frame_time


def on_frame_with_data(info):
    mocap_data = info.get("mocap_data")
    if mocap_data is None:
        return
    rb_data = getattr(mocap_data, "rigid_body_data", None)
    if rb_data is None:
        return
    rb_list = getattr(rb_data, "rigid_body_list", [])
    if NATNET_LOG_FRAMES and rb_list:
        ids = [getattr(rb, "id_num", None) for rb in rb_list]
        print(f"RigidBody IDs: {ids}")
    for rb in rb_list:
        rb_id = getattr(rb, "id_num", None)
        pos = getattr(rb, "pos", None)
        rot = getattr(rb, "rot", None)
        valid = getattr(rb, "tracking_valid", getattr(rb, "trackingValid", True))
        if rb_id is None or pos is None or rot is None:
            continue
        if valid is False:
            if rb_id == CF_RB_ID:
                latest["cf"]["ok"] = False
            elif rb_id == ARM_RB_ID:
                latest["arm"]["ok"] = False
            continue
        on_rigid_body(rb_id, pos, rot)


def build_extpos_packed(cf_id, cf_pos=None, arm_pos=None, send_cf_pos=True):
    items = []
    if send_cf_pos and cf_pos is not None:
        items.append((cf_id, cf_pos["x"], cf_pos["y"], cf_pos["z"]))
    if arm_pos is not None:
        items.append((ARM_PEER_ID, arm_pos["x"], arm_pos["y"], arm_pos["z"]))

    payload = bytearray()
    for item_id, x, y, z in items:
        payload += struct.pack("<Bhhh", item_id, mm16(x), mm16(y), mm16(z))
    return payload


def sniff_natnet_packets():
    if NATNET_SNIFF_SECS <= 0:
        return
    print(f"Sniffing UDP on {NATNET_DATA_PORT} for {NATNET_SNIFF_SECS:.1f}s...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.2)
    try:
        sock.bind(("", NATNET_DATA_PORT))
        if NATNET_USE_MULTICAST:
            mreq = struct.pack("4s4s",
                               socket.inet_aton(NATNET_MULTICAST_ADDR),
                               socket.inet_aton("0.0.0.0"))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        start = time.time()
        got = 0
        id_counts = {}
        while time.time() - start < NATNET_SNIFF_SECS:
            try:
                data, addr = sock.recvfrom(4096)
                got += 1
                msg_id = int.from_bytes(data[0:2], byteorder="little", signed=True)
                id_counts[msg_id] = id_counts.get(msg_id, 0) + 1
                if got <= 3:
                    print(f"UDP packet from {addr[0]}:{addr[1]} "
                          f"({len(data)} bytes, msg_id={msg_id})")
            except socket.timeout:
                pass
        if got == 0:
            print("No UDP packets seen on data port.")
        else:
            top = sorted(id_counts.items(), key=lambda kv: kv[1], reverse=True)[:5]
            print("Top NatNet msg_ids:", ", ".join(f"{mid}:{cnt}" for mid, cnt in top))
    finally:
        sock.close()


def main():
    cf_id = parse_cf_id(CF_URI)
    print(f"Crazyflie ID (from URI): 0x{cf_id:02X}")
    print(f"NatNet server: {WINDOWS_MOTIVE_IP}  local: {MAC_IP}")
    print(f"Rigid body (CF): {CF_RB_NAME} / ID {CF_RB_ID}")
    if SEND_ARM:
        print(f"Rigid body (arm): {ARM_RB_NAME} / ID {ARM_RB_ID}")

    cflib.crtp.init_drivers()

    def parse_xyz(csv_text, default):
        try:
            parts = [float(p.strip()) for p in csv_text.split(",")]
            if len(parts) != 3:
                return default
            return parts
        except Exception:
            return default

    def fake_arm_position(now_s):
        if FAKE_ARM_MODE == "circle":
            center = parse_xyz(FAKE_ARM_CENTER, [0.5, 0.0, 0.0])
            if FAKE_ARM_PERIOD <= 0:
                phase = 0.0
            else:
                phase = 2.0 * 3.141592653589793 * (now_s / FAKE_ARM_PERIOD)
            return {
                "x": center[0] + FAKE_ARM_RADIUS * math.cos(phase),
                "y": center[1] + FAKE_ARM_RADIUS * math.sin(phase),
                "z": center[2],
                "ok": True,
            }
        # static default
        arm_xyz = parse_xyz(FAKE_ARM_POS, [0.5, 0.0, 0.0])
        return {"x": arm_xyz[0], "y": arm_xyz[1], "z": arm_xyz[2], "ok": True}

    if NO_MOCAP:
        cf_xyz = parse_xyz(FAKE_CF_POS, [0.0, 0.0, 0.0])
        arm_xyz = parse_xyz(FAKE_ARM_POS, [0.5, 0.0, 0.0])
        if SEND_CF_POS:
            latest["cf"].update({"x": cf_xyz[0], "y": cf_xyz[1], "z": cf_xyz[2], "ok": True})
        if SEND_ARM:
            latest["arm"].update({"x": arm_xyz[0], "y": arm_xyz[1], "z": arm_xyz[2], "ok": True})
        print("NO_MOCAP=1: using fixed positions")
        if SEND_CF_POS:
            print(f"  CF  = ({latest['cf']['x']:.2f}, {latest['cf']['y']:.2f}, {latest['cf']['z']:.2f})")
            print("  WARNING: sending fake CF position will override estimator")
        if SEND_ARM:
            print(f"  ARM = ({latest['arm']['x']:.2f}, {latest['arm']['y']:.2f}, {latest['arm']['z']:.2f})")
    else:
        sniff_natnet_packets()

    if not NO_MOCAP:
        # Start NatNet client
        natnet = NatNetClient()
        bind_ip = NATNET_BIND_IP if NATNET_BIND_IP else MAC_IP
        if hasattr(natnet, "set_server_address"):
            natnet.set_server_address(WINDOWS_MOTIVE_IP)
        if hasattr(natnet, "set_client_address"):
            natnet.set_client_address(bind_ip)
        if hasattr(natnet, "server_ip_address"):
            natnet.server_ip_address = WINDOWS_MOTIVE_IP
        if hasattr(natnet, "local_ip_address"):
            natnet.local_ip_address = bind_ip
        if hasattr(natnet, "set_use_multicast"):
            natnet.set_use_multicast(NATNET_USE_MULTICAST)
        if hasattr(natnet, "use_multicast"):
            natnet.use_multicast = NATNET_USE_MULTICAST
        if hasattr(natnet, "multicast_address"):
            natnet.multicast_address = NATNET_MULTICAST_ADDR
        if hasattr(natnet, "multicastAddress"):
            natnet.multicastAddress = NATNET_MULTICAST_ADDR
        if hasattr(natnet, "data_port"):
            natnet.data_port = NATNET_DATA_PORT
        if hasattr(natnet, "dataPort"):
            natnet.dataPort = NATNET_DATA_PORT
        if hasattr(natnet, "command_port"):
            natnet.command_port = NATNET_CMD_PORT
        if hasattr(natnet, "commandPort"):
            natnet.commandPort = NATNET_CMD_PORT
        if hasattr(natnet, "rigid_body_listener"):
            natnet.rigid_body_listener = on_rigid_body
        if hasattr(natnet, "rigidBodyListener"):
            natnet.rigidBodyListener = on_rigid_body
        if hasattr(natnet, "new_frame_listener"):
            natnet.new_frame_listener = on_frame
        if hasattr(natnet, "newFrameListener"):
            natnet.newFrameListener = on_frame
        if hasattr(natnet, "new_frame_with_data_listener"):
            natnet.new_frame_with_data_listener = on_frame_with_data
        if hasattr(natnet, "newFrameWithDataListener"):
            natnet.newFrameWithDataListener = on_frame_with_data
        
        print("Starting NatNet client...")
        if not natnet.run("d"):
            raise RuntimeError("NatNetClient failed to start")

        # Wait for MoCap data to start coming in
        print("Waiting for MoCap data...")
        for _ in range(50):  # 5 seconds max
            if latest["cf"]["ok"]:
                print(f"Got CF position: ({latest['cf']['x']:.2f}, {latest['cf']['y']:.2f}, {latest['cf']['z']:.2f})")
                break
            time.sleep(0.1)
        else:
            print("Warning: No CF rigid body data yet (will keep trying)")
        
        if SEND_ARM and latest["arm"]["ok"]:
            print(f"Got arm position: ({latest['arm']['x']:.2f}, {latest['arm']['y']:.2f}, {latest['arm']['z']:.2f})")
        elif SEND_ARM:
            print("Warning: No arm rigid body data yet")

    # Connection state tracking
    connection_state = {"connected": False, "failed": False, "error": None}
    
    def on_connected(link_uri):
        print(f"\n=== CONNECTED to {link_uri} ===")
        connection_state["connected"] = True
        connection_state["failed"] = False
    
    def on_disconnected(link_uri):
        print(f"\n=== DISCONNECTED from {link_uri} ===")
        connection_state["connected"] = False
    
    def on_connection_failed(link_uri, msg):
        print(f"\n=== CONNECTION FAILED: {msg} ===")
        connection_state["failed"] = True
        connection_state["error"] = msg
    
    def on_link_quality(percentage):
        pass  # Suppress link quality spam
    
    def console_callback(text):
        # Print firmware console output (includes "psd disks=X" messages)
        print(f"[CF] {text}", end="")

    def set_param_with_timeout(param, value, timeout_s=2.0):
        result = {"ok": False, "err": None}
        def _worker():
            try:
                cf.param.set_value(param, str(value))
                result["ok"] = True
            except Exception as e:
                result["err"] = str(e)
        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        t.join(timeout=timeout_s)
        if t.is_alive():
            print(f"[WARN] param set timeout: {param}={value}")
            return False
        if not result["ok"]:
            print(f"[WARN] param set failed: {param}={value} ({result['err']})")
            return False
        print(f"[OK] param set: {param}={value}")
        return True

    if not SEND_CF:
        print("SEND_CF=0, just printing MoCap data without sending to CF")
        last_print = 0.0
        while True:
            now = time.time()
            if now - last_print > 1.0:
                cf = latest["cf"]
                arm = latest["arm"]
                if cf["ok"]:
                    msg = f"cf1: ({cf['x']:.3f}, {cf['y']:.3f}, {cf['z']:.3f})"
                    if SEND_ARM and arm["ok"]:
                        msg += f" | arm: ({arm['x']:.3f}, {arm['y']:.3f}, {arm['z']:.3f})"
                    print(msg)
                else:
                    print("Waiting for CF rigid body...")
                last_print = now
            time.sleep(0.1)
        return

    # Connect to Crazyflie with retry logic
    MAX_RETRIES = 3
    cf = None
    
    for attempt in range(MAX_RETRIES):
        print(f"\nConnecting to Crazyflie at {CF_URI} (attempt {attempt + 1}/{MAX_RETRIES})...")
        
        cf = Crazyflie(rw_cache="./cache")
        cf.connected.add_callback(on_connected)
        cf.disconnected.add_callback(on_disconnected)
        cf.connection_failed.add_callback(on_connection_failed)
        cf.link_quality_updated.add_callback(on_link_quality)
        cf.console.receivedChar.add_callback(console_callback)
        
        connection_state["connected"] = False
        connection_state["failed"] = False
        connection_state["error"] = None
        
        cf.open_link(CF_URI)
        
        # Wait for connection (up to 5 seconds)
        for _ in range(50):
            if connection_state["connected"]:
                break
            if connection_state["failed"]:
                break
            time.sleep(0.1)
        
        if connection_state["connected"]:
            print("Connection established!")
            break
        else:
            err = connection_state["error"] or "Timeout"
            print(f"Connection attempt failed: {err}")
            try:
                cf.close_link()
            except:
                pass
            if attempt < MAX_RETRIES - 1:
                print("Retrying in 2 seconds...")
                time.sleep(2.0)
    
    if not connection_state["connected"]:
        print("\n" + "="*50)
        print("FAILED to connect to Crazyflie after all retries.")
        print("Check that:")
        print("  1. Crazyflie is powered on")
        print("  2. Crazyradio USB dongle is plugged in")
        print("  3. No other process is using the radio (close cfclient!)")
        print("  4. Try unplugging and replugging the Crazyradio")
        print("="*50)
        return

    # Optional: force controller param (e.g., stabilizer.controller=6 for TinyMPC)
    if FORCE_CONTROLLER and FORCE_CONTROLLER_VAL:
        print(f"Setting controller param: {FORCE_CONTROLLER}={FORCE_CONTROLLER_VAL}")
        set_param_with_timeout(FORCE_CONTROLLER, FORCE_CONTROLLER_VAL, timeout_s=3.0)

    # Optional: PSD log (diskCount/certified)
    if ENABLE_PSD_LOG:
        try:
            psd_log = LogConfig(name="PSD", period_in_ms=500)
            psd_log.add_variable("psdMPC.diskCount", "int32_t")
            psd_log.add_variable("psdMPC.certified", "uint8_t")
            def psd_log_callback(timestamp, data, logconf):
                disk_count = data.get("psdMPC.diskCount", 0)
                certified = data.get("psdMPC.certified", 0)
                print(f"[PSD] diskCount={disk_count} certified={certified}")
            cf.log.add_config(psd_log)
            psd_log.data_received_cb.add_callback(psd_log_callback)
            psd_log.start()
            print("[OK] PSD log started")
        except Exception as e:
            print(f"[WARN] PSD log failed: {e}")

    # Start streaming thread
    stop_streaming = threading.Event()
    packets_sent = {"count": 0, "last_log": 0.0}
    
    def stream_thread():
        dt = 1.0 / SEND_RATE_HZ
        last_status = 0.0
        while not stop_streaming.is_set():
            if not connection_state["connected"]:
                time.sleep(0.1)
                continue
            
            now = time.time()
            arm_pos = None
            if SEND_ARM:
                if NO_MOCAP:
                    arm_pos = fake_arm_position(now)
                elif latest["arm"]["ok"]:
                    arm_pos = latest["arm"]

            cf_pos = None
            if SEND_CF_POS:
                if NO_MOCAP and latest["cf"]["ok"]:
                    cf_pos = latest["cf"]
                elif latest["cf"]["ok"]:
                    cf_pos = latest["cf"]

            if not SEND_CF_POS and arm_pos is None:
                time.sleep(dt)
                continue
            if SEND_CF_POS and cf_pos is None and arm_pos is None:
                time.sleep(dt)
                continue

            payload = build_extpos_packed(cf_id, cf_pos, arm_pos, send_cf_pos=SEND_CF_POS)
            pk = CRTPPacket()
            pk.port = CRTP_PORT_LOCALIZATION
            pk.channel = EXT_POSITION_PACKED
            pk.data = payload
            try:
                cf.send_packet(pk)
                packets_sent["count"] += 1
            except Exception:
                pass  # Ignore send errors, connection callbacks will handle state
            
            # Print status every 2 seconds
            if now - last_status > 2.0:
                n_items = (1 if (SEND_CF_POS and cf_pos is not None) else 0) + (1 if arm_pos else 0)
                msg = f"[STREAMING] pkts={packets_sent['count']} items={n_items}"
                if cf_pos is not None:
                    msg += f" | cf=({cf_pos['x']:.2f},{cf_pos['y']:.2f},{cf_pos['z']:.2f})"
                if arm_pos:
                    msg += f" arm=({arm_pos['x']:.2f},{arm_pos['y']:.2f},{arm_pos['z']:.2f})"
                print(msg)
                last_status = now
            
            time.sleep(dt)
    
    streamer = threading.Thread(target=stream_thread, daemon=True)
    streamer.start()
    print("Started position streaming thread")
    
    # Interactive flight control
    try:
        print("\n" + "="*50)
        print("READY FOR FLIGHT TEST")
        print("Position data is being streamed to the Crazyflie.")
        print("="*50 + "\n")
        
        input(">>> Press ENTER to TAKEOFF (0.5m height) <<<")
        
        if not connection_state["connected"]:
            print("Lost connection! Cannot takeoff.")
            return
        
        print("Enabling high-level commander and taking off...")
        cf.param.set_value("commander.enHighLevel", "1")
        time.sleep(0.2)
        cf.high_level_commander.takeoff(0.5, 2.0)  # 0.5m height, 2s duration
        print("Takeoff command sent. Waiting 3 seconds...")
        time.sleep(3.0)
        
        print("\n" + "="*50)
        print("FLYING! Move the arm near the drone to test obstacle avoidance.")
        print("Watch for [CF] prints showing 'psd disks=X' where X > 0")
        print("="*50 + "\n")
        
        input(">>> Press ENTER to LAND <<<")
        
        print("Landing...")
        cf.high_level_commander.land(0.0, 2.0)
        time.sleep(3.0)
        print("Landed.")
        
    except KeyboardInterrupt:
        print("\nInterrupted! Landing...")
        try:
            cf.high_level_commander.land(0.0, 2.0)
            time.sleep(2.0)
        except:
            pass
    finally:
        stop_streaming.set()
        time.sleep(0.2)
        cf.close_link()
        print("Connection closed.")


if __name__ == "__main__":
    main()
