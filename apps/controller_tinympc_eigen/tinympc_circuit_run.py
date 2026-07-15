#!/usr/bin/env python3
"""tinympc_circuit_run.py -- fly the two-gate circle and dump a CSV.

Sets up the params, takes off, pre-positions on the loop's join point so the engage
is smooth, flies N laps of circuit.en=1, lands, and writes every relevant LOG group
to CSV so an --inject 0 run and an --inject 1 run diff directly.

  # bench, PROPS OFF -- no arming, no takeoff. Just watches the weight schedule sweep.
  ./tinympc_circuit_run.py --dryrun

  # baseline: odometry only, vision computed+logged but NOT fed to the EKF
  ./tinympc_circuit_run.py --laps 5 --inject 0 --out baseline.csv

  # the same laps with the vision correcting the estimator
  ./tinympc_circuit_run.py --laps 5 --inject 1 --out fused.csv

GEOMETRY. The world frame is defined by where the drone sits and which way it points
when kalman.resetEstimation fires: that spot is the origin and its heading is +x.
Crazyflie axes are +x forward, +y LEFT, +z up, so gate 2 being to the RIGHT means -y.
Measure --gate1 (origin to gate 1, along the drone's forward axis) and --spacing (gate
1 to gate 2) in inches; --gatez is the gate CENTRE height in metres and is NOT
something this script can infer, so set it if your gates aren't hung at 0.5 m.

PRECONDITIONS
  - Firmware from apps/controller_tinympc_eigen flashed (needs the circuit + visFuse
    params; a build without them will fail at the param-set step, loudly).
  - Drone placed at the origin, FACING THE GATES, before you start.
  - PROPS ON, NETTED, fresh battery -- except with --dryrun.
"""
import argparse
import csv
import json
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

IN2M = 0.0254

# --- Camera calibration (from --calib against gate A; see the run that put us into the
# gate). These do NOT persist across a Crazyflie reboot/reflash, so the script re-pushes
# them EVERY run -- from these calibrated values, never the firmware defaults (gateW 19in,
# trims 0), which is uncalibrated and unsafe to inject. Re-run --calib and update these
# three numbers whenever the mount or the gate changes. ---
CAL_GATEW_IN = 23.5     # effective aperture that makes vg_range match the true distance
CAL_MNTYAW   = 0.074    # + = camera aimed left  [rad] (gate reads left of true)
CAL_MNTPIT   = -0.095   # + = camera aimed down  [rad]
GATE_OPENING_IN = 19.0  # the PHYSICAL opening -- only the starting point for --calib

ap = argparse.ArgumentParser()
ap.add_argument("--uri", default="radio://0/80/2M/E7E7E7E7E8")
ap.add_argument("--gate1", type=float, default=43.0, help="origin -> gate 1, inches")
ap.add_argument("--spacing", type=float, default=58.0, help="gate 1 -> gate 2, inches")
ap.add_argument("--gatez", type=float, default=13.0,
                help="gate CENTRE height above the floor, inches (NOT the aperture size -- "
                     "that lives in visGate.gateW/gateH and must match the physical gate "
                     "or every range is scaled wrong)")
ap.add_argument("--speed", type=float, default=0.3, help="circuit speed, m/s")
ap.add_argument("--rampto", type=float, default=None,
                help="if set, linearly ramp circuit.speed from --speed up to this over the "
                     "run (m/s), so one flight sweeps the speed and you can read off in the "
                     "CSV where the vision drops out. leg column carries the live speed.")
ap.add_argument("--laps", type=float, default=3.0)
ap.add_argument("--hover", type=float, default=6.0, help="hover-at-origin dwell, s")
ap.add_argument("--inject", type=int, default=0, choices=(0, 1),
                help="1 = vision actually corrects the EKF; 0 = computed+logged only")
ap.add_argument("--std0", type=float, default=0.05, help="vision fix noise, constant term")
ap.add_argument("--join", type=float, default=150.0,
                help="loop phase to enter at, deg. 180 = gate A itself; smaller = further "
                     "back on its approach arc. 'auto' (=0) uses the phase nearest the "
                     "origin, which is well off to the right -- see below.")
ap.add_argument("--dryrun", action="store_true",
                help="PROPS OFF: no arm, no takeoff. Only sweeps the schedule.")
ap.add_argument("--calib", action="store_true",
                help="PROPS OFF: park the drone on a box at the takeoff mark, at gate "
                     "height, facing gate A. Measures the camera's range scale and "
                     "boresight bias and prints the visGate trims. Do this BEFORE any "
                     "--inject 1 flight -- an uncalibrated fix corrupts the EKF.")
ap.add_argument("--gatew", type=float, default=None,
                help="EFFECTIVE gate aperture, inches -- the size that makes range come "
                     "out right (range = fx*W/width_px). Default = the calibrated value "
                     "below (NOT the physical 19 in; the detector box is larger and fx "
                     "absorbs the rest). Pass a value to override.")
ap.add_argument("--mntyaw", type=float, default=None,
                help="lateral camera boresight trim, rad (+ = camera aimed left). "
                     "Default = calibrated value below.")
ap.add_argument("--mntpit", type=float, default=None,
                help="vertical camera boresight trim, rad (+ = camera aimed down). "
                     "Default = calibrated value below.")
ap.add_argument("--minvbat", type=float, default=3.6)
ap.add_argument("--out", default="tinympc_circuit_run.csv")
a = ap.parse_args()

# ---------------------------------------------------------------- geometry
gAx, gAy, gAz = a.gate1 * IN2M, 0.0, a.gatez * IN2M
gBx, gBy, gBz = gAx, -a.spacing * IN2M, a.gatez * IN2M   # gate 2 is to the RIGHT -> -y
loopW = 0.5 * abs(gBy - gAy)      # = semi-major -> the loop is a true CIRCLE
CIRC_DIR = 1                      # +1: join heading forward, near gate (A) first

# EVERY leg flies at the gate height -- takeoff, hover, goto, settle, land. The circuit
# itself is pinned to the gate centres (circuitPoint interpolates gAz..gBz), so hovering
# at some other altitude would (a) put a step into the engage and (b) tilt the camera off
# the gate during the hover check, which is precisely when we want the geometry to be
# textbook. Keep them the same and the gate sits dead on the boresight.
Z_FLY = 0.5 * (gAz + gBz)


def circuit_point(th):
    """Mirror of circuitPoint() in controller_tinympc.cpp. Returns (P, T)."""
    ux, uy = gBx - gAx, gBy - gAy
    L = math.hypot(ux, uy) or 1e-3
    ux, uy = ux / L, uy / L
    semi = 0.5 * L
    nx, ny = CIRC_DIR * uy, -CIRC_DIR * ux
    mx, my = 0.5 * (gAx + gBx), 0.5 * (gAy + gBy)
    c, s = math.cos(th), math.sin(th)
    P = (mx + semi * c * ux + loopW * s * nx,
         my + semi * c * uy + loopW * s * ny,
         0.5 * (gAz + gBz) - 0.5 * (gAz - gBz) * c)
    T = (-semi * s * ux + loopW * c * nx,
         -semi * s * uy + loopW * c * ny,
         0.5 * (gAz - gBz) * s)
    return P, T


# The firmware joins at the nearest of 48 sampled phases (see circ_armed), so snap our
# chosen entry to that same grid -- otherwise we'd pre-position between two samples and
# still eat a step on engage.
def snap(th):
    k = round(th / (2.0 * math.pi / 48.0)) % 48
    return 2.0 * math.pi * k / 48.0


def nearest_phase(x, y):
    best, bth = 1e30, 0.0
    for k in range(48):
        th = 2.0 * math.pi * k / 48.0
        P, _ = circuit_point(th)
        d = (P[0] - x) ** 2 + (P[1] - y) ** 2
        if d < best:
            best, bth = d, th
    return bth


# Where to enter the loop. Letting the firmware pick (the phase nearest the origin) is a
# trap: the circle's near side is offset to the RIGHT of the takeoff spot and its tangent
# there points ~53 deg LEFT, so the drone crabs sideways-right while yawing left before
# curving back in. It looks wrong because it is a silly place to join.
#
# The origin actually sits exactly on the circle's TANGENT LINE at gate A -- the centre is
# abeam gate A, so the tangent there runs along +x, which is the line the drone is already
# on and already pointing down. So entering further along gate A's approach arc turns the
# pre-position into a mostly-straight-forward move. 150 deg is the sweet spot: a short hop
# forward and slightly left, and still inside gate A's vision window (110-170 deg).
TH0 = nearest_phase(0.0, 0.0) if a.join == 0 else snap(math.radians(a.join))
JP, JT = circuit_point(TH0)
JOIN_YAW = math.degrees(math.atan2(JT[1], JT[0]))

# NB: even from a good join, the nose LEADS the gate by half the remaining arc (tangent-
# following yaw on a circle -- the same inscribed-angle relation that sets the vision
# window). So the drone points left of gate A on the approach and converges to dead-on
# exactly at the gate. That part is inherent, not a bug.

# Path length by numeric integration, so the lap time stays right even if loopW is
# changed away from a circle. The firmware advances the phase at a constant tangential
# speed, so lap time is just perimeter / speed.
def perimeter(n=720):
    total = 0.0
    prev, _ = circuit_point(0.0)
    for k in range(1, n + 1):
        P, _ = circuit_point(2.0 * math.pi * k / n)
        total += math.dist(P, prev)
        prev = P
    return total


PERIM = perimeter()
LAP_S = PERIM / a.speed
# When ramping, the lap time shrinks as it speeds up, so budget the wall-clock at the
# MEAN speed. Otherwise the run is just laps x lap-time.
if a.rampto:
    CIRC_S = a.laps * PERIM / (0.5 * (a.speed + a.rampto))
else:
    CIRC_S = LAP_S * a.laps
# Bank angle (v^2/r) is the first thing to bite on this tight circle -- warn if the top
# speed banks the camera far enough to lose the gate.
def bank_deg(v):
    return math.degrees(math.atan2(v * v / max(loopW, 1e-3), 9.81))

print("=" * 68)
print(f"  gate A (near)  ({gAx:+.3f}, {gAy:+.3f}, {gAz:.2f}) m   "
      f"[{a.gate1:.0f} in ahead]")
print(f"  gate B (right) ({gBx:+.3f}, {gBy:+.3f}, {gBz:.2f}) m   "
      f"[{a.spacing:.0f} in from A]")
print(f"  loop: circle r={loopW:.3f} m about "
      f"({0.5*(gAx+gBx):+.3f}, {0.5*(gAy+gBy):+.3f}), CLOCKWISE from above")
print(f"  join point ({JP[0]:+.3f}, {JP[1]:+.3f}) yaw {JOIN_YAW:+.0f} deg "
      f"@ phase {math.degrees(TH0):.0f} deg")
if a.rampto:
    print(f"  RAMP {a.speed}->{a.rampto} m/s over {a.laps:g} laps ~= {CIRC_S:.0f} s "
          f"(bank {bank_deg(a.speed):.0f}->{bank_deg(a.rampto):.0f} deg)")
    if bank_deg(a.rampto) > 15:
        print(f"  !! top speed banks the camera {bank_deg(a.rampto):.0f} deg -- expect the "
              f"gate to leave frame and vision to drop out up there")
else:
    print(f"  lap {LAP_S:.1f} s  x {a.laps:g} laps = {CIRC_S:.0f} s  @ {a.speed} m/s "
          f"(bank {bank_deg(a.speed):.0f} deg)")
print(f"  flying the WHOLE run at z={Z_FLY:.3f} m (= gate centre height, {a.gatez:.0f} in)")
if Z_FLY < 0.35:
    print(f"  !! z={Z_FLY:.2f} m is a low cruise -- expect more ground effect and a "
          f"noisier Flow deck")
print(f"  vision: en=1  inject={a.inject}"
      f"{'   <-- EKF IS BEING CORRECTED' if a.inject else '   (logged only)'}")
if a.dryrun:
    print("  *** DRYRUN: props off, no arming, no takeoff ***")
else:
    print("  *** PROPS ON, NETTED. Ctrl-C aborts (lands). ***")
print("=" * 68)

# ---------------------------------------------------------------- logging
d = {}
rows = []
t0 = [None]
sp = [0.0, 0.0, 0.0, 0.0]     # last commanded setpoint x,y,z,yaw
leg = ["init"]                 # host-side phase label, so the CSV slices cleanly
reset_host_t = [None]          # host time of kalman.resetEstimation (defines the reset frame)
console = []

# The two columns you'll actually diff:
#   x / y / z            -- ESTIMATOR position (fused IMU + Flow deck; + vision if inj=1).
#                           "where the drone thinks it is."
#   imp_x / imp_y / imp_z -- VISION-implied drone position = surveyed gate - measured
#                           offset to it. "where the camera + the known gate say it is."
#                           Blank when no gate is in view this row.
# and the gate itself:
#   vg_gx/gy/gz          -- gate position the VISION computes (from the drifting estimate)
#   survX/survY/survZ    -- the SURVEYED gate truth it should match
# vf_dx/dy/dz = imp - estimator is the headline disagreement (== the EKF correction).
COLS = ["t", "leg",
        "x", "y", "z", "imp_x", "imp_y", "imp_z",     # estimator vs vision-implied drone pos
        "yaw", "vx", "vy",
        "vf_w", "vf_gate", "vf_std", "vf_dx", "vf_dy", "vf_dz",
        "vf_n", "vf_rej", "vf_expR", "vf_expA",
        "vg_valid", "vg_reason", "vg_range", "vg_phase",
        "vg_gx", "vg_gy", "vg_gz", "survX", "survY", "survZ",  # vision gate vs surveyed gate
        "g8_rxOk", "vbat", "spX", "spY", "spZ", "spYaw",
        # host wall-clock (time.time(), epoch s) stamped when this row is built. The other
        # `t` is the DRONE clock; this one shares the clock the mocap logger stamps with, so
        # compare_vision_truth.py can join the two machines' streams by time (NTP-synced).
        "host_t"]


def cb(ts, data, lc):
    d.update(data)


def cb_row(ts, data, lc):
    """The estimator block drives one CSV row; everything else is last-known."""
    d.update(data)
    if t0[0] is None:
        t0[0] = ts
    ex, ey, ez = (d.get("stateEstimate.x"), d.get("stateEstimate.y"),
                  d.get("stateEstimate.z"))
    dxv, dyv, dzv = d.get("visFuse.dx"), d.get("visFuse.dy"), d.get("visFuse.dz")
    gate = d.get("visFuse.gate")
    fresh = d.get("visGate.valid") == 1 and gate in (1, 2)
    # Vision-implied drone position: only meaningful when a gate is actually in view this
    # row. vf_d* holds its last value between camera frames, so gating on validity keeps a
    # stale fix from masquerading as a live one. (With --inject 1 this sits close to the
    # estimator -- the EKF has already moved onto the vision; the raw disagreement is the
    # --inject 0 run.)
    if fresh and None not in (ex, ey, ez, dxv, dyv, dzv):
        imp = (ex + dxv, ey + dyv, ez + dzv)
    else:
        imp = (None, None, None)
    surv = {1: (gAx, gAy, gAz), 2: (gBx, gBy, gBz)}.get(gate, (None, None, None))
    rows.append([(ts - t0[0]) / 1000.0, leg[0],
                 ex, ey, ez, imp[0], imp[1], imp[2],
                 d.get("stateEstimate.yaw"),
                 d.get("stateEstimate.vx"), d.get("stateEstimate.vy"),
                 d.get("visFuse.w"), gate, d.get("visFuse.std"),
                 dxv, dyv, dzv,
                 d.get("visFuse.n"), d.get("visFuse.rej"),
                 d.get("visFuse.expR"), d.get("visFuse.expA"),
                 d.get("visGate.valid"), d.get("visGate.reason"),
                 d.get("visGate.range"), d.get("visGate.phase"),
                 d.get("visGate.gx"), d.get("visGate.gy"), d.get("visGate.gz"),
                 surv[0], surv[1], surv[2],
                 d.get("gate8.rxOk"), d.get("pm.vbat"),
                 sp[0], sp[1], sp[2], sp[3],
                 time.time()])


BLOCKS = [
    # (name, period_ms, [(var, type)...], drives_row)
    ("est", 50, [("stateEstimate.x", "float"), ("stateEstimate.y", "float"),
                 ("stateEstimate.z", "float"), ("stateEstimate.yaw", "float"),
                 ("stateEstimate.vx", "float"), ("stateEstimate.vy", "float")], True),
    ("fuse", 50, [("visFuse.w", "float"), ("visFuse.std", "float"),
                  ("visFuse.dx", "float"), ("visFuse.dy", "float"),
                  ("visFuse.dz", "float"), ("visFuse.gate", "uint8_t")], False),
    ("fuse2", 100, [("visFuse.n", "uint32_t"), ("visFuse.rej", "uint32_t"),
                    ("visFuse.expR", "float"), ("visFuse.expA", "float"),
                    ("visGate.valid", "uint8_t"), ("visGate.reason", "uint8_t")], False),
    ("gate", 100, [("visGate.range", "float"), ("visGate.phase", "float"),
                   ("visGate.gx", "float"), ("visGate.gy", "float"),
                   ("visGate.gz", "float"), ("gate8.rxOk", "uint32_t")], False),
    ("sys", 500, [("pm.vbat", "float")], False),
]


def reset_logging(cf):
    """Purge stale drone-side log blocks; a leftover block with a different layout
    decodes as bit-frozen garbage."""
    from cflib.crtp.crtpstack import CRTPPacket
    pk = CRTPPacket()
    pk.set_header(5, 1)          # port LOGGING, channel SETTINGS
    pk.data = (5,)               # CMD_RESET_LOGGING
    cf.send_packet(pk)
    time.sleep(0.5)


# ---------------------------------------------------------------- flight
cflib.crtp.init_drivers()
cfgs = []
try:
    with SyncCrazyflie(a.uri, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        cf.console.receivedChar.add_callback(lambda t: console.append(t))
        reset_logging(cf)

        def P(name, val):
            cf.param.set_value(name, val)
            time.sleep(0.03)

        def Pcal(name, val):
            """Push a calibration param, but don't die if the firmware predates it --
            visGate.mntYaw is new, so an un-reflashed drone lacks it. Warn instead."""
            try:
                cf.param.set_value(name, val)
                time.sleep(0.03)
            except (KeyError, AttributeError):
                print(f"  !! {name} not on the drone -- REFLASH the firmware "
                      f"(it has the boresight trim). Continuing UNCALIBRATED in {name}.")

        P("stabilizer.controller", 6)            # out-of-tree = TinyMPC
        P("gateNav.navEn", 0)                    # single-gate nav off; circuit owns the ref
        # --- surveyed gates + loop ---
        P("circuit.gAx", gAx); P("circuit.gAy", gAy); P("circuit.gAz", gAz)
        P("circuit.gBx", gBx); P("circuit.gBy", gBy); P("circuit.gBz", gBz)
        P("circuit.loopW", loopW)
        P("circuit.speed", a.speed)
        P("circuit.dir", CIRC_DIR)
        P("circuit.en", 0)
        # --- vision -> EKF ---
        P("visFuse.en", 1)                       # compute + log the correction
        P("visFuse.inj", 0 if a.calib else a.inject)   # calib NEVER injects
        P("visFuse.sched", 1)                    # weight from the PLANNED pose
        P("visFuse.std0", a.std0)
        # Calibration. --calib MEASURES it, so it starts from the raw physical opening and
        # zero trims; every other mode FLIES it, so it applies the calibrated values (or a
        # flag override). Pushed every run because Crazyflie params don't survive a reboot.
        if a.calib:
            gatew_in = a.gatew if a.gatew is not None else GATE_OPENING_IN
            mntyaw = a.mntyaw if a.mntyaw is not None else 0.0
            mntpit = a.mntpit if a.mntpit is not None else 0.0
        else:
            gatew_in = a.gatew if a.gatew is not None else CAL_GATEW_IN
            mntyaw = a.mntyaw if a.mntyaw is not None else CAL_MNTYAW
            mntpit = a.mntpit if a.mntpit is not None else CAL_MNTPIT
        P("visGate.gateW", gatew_in * IN2M)
        P("visGate.gateH", gatew_in * IN2M)
        Pcal("visGate.mntYaw", mntyaw)
        Pcal("visGate.mntPit", mntpit)
        print(f"calibration: gateW={gatew_in:.1f}in  mntYaw={mntyaw:+.3f}  "
              f"mntPit={mntpit:+.3f}" + ("   [--calib: RAW, measuring]" if a.calib else ""))

        for name, per, vars_, drives in BLOCKS:
            lc = LogConfig(name=name, period_in_ms=per)
            for v, ty in vars_:
                lc.add_variable(v, ty)
            cf.log.add_config(lc)
            lc.data_received_cb.add_callback(cb_row if drives else cb)
            lc.start()
            cfgs.append(lc)
        time.sleep(0.5)
        if not rows:
            raise SystemExit("no telemetry -- is the OOT controller flashed and selected?")

        vb = d.get("pm.vbat")
        print(f"vbat {vb}")
        if not a.dryrun and vb is not None and vb < a.minvbat:
            raise SystemExit(f"ABORT: battery {vb:.2f}V < {a.minvbat}V")

        # Origin + heading are defined HERE. The drone must be on its mark, facing the gates.
        # Stamp the host time of the reset: this instant defines the reset frame (drone at
        # (0,0,0), nose = +x), so the mocap drone pose AT this time is the mocapWorld->reset
        # transform that compare_vision_truth.py uses to put the truth in the same frame.
        reset_host_t[0] = time.time()
        P("kalman.resetEstimation", 1)
        time.sleep(0.1)
        P("kalman.resetEstimation", 0)
        time.sleep(2.0)
        print("estimator reset -- origin is here, +x is where the nose points")

        def send(x, y, z, yaw):
            sp[0], sp[1], sp[2], sp[3] = x, y, z, yaw
            cf.commander.send_position_setpoint(x, y, z, yaw)

        def stream(secs, fn, label):
            """Hold a setpoint at 50 Hz for `secs`. The commander MUST keep being fed
            even while circuit.en=1 overrides the position -- the controller cuts the
            motors on setpoint.mode.z == modeDisable, so a gap here is a mid-air stop."""
            leg[0] = label
            t = time.time()
            while time.time() - t < secs:
                fn(time.time() - t)
                time.sleep(0.02)

        def est():
            return (d.get("stateEstimate.x", 0.0), d.get("stateEstimate.y", 0.0),
                    d.get("stateEstimate.z", 0.0), d.get("stateEstimate.yaw", 0.0))

        def show(tag):
            x, y, z, yw = est()
            print(f"  {tag} est=({x:+.2f},{y:+.2f},{z:+.2f}) yaw={yw:+6.1f} "
                  f"| w={d.get('visFuse.w', 0):.2f} gate={d.get('visFuse.gate')} "
                  f"std={d.get('visFuse.std', 0):.2f} n={d.get('visFuse.n')} "
                  f"rej={d.get('visFuse.rej')} vgValid={d.get('visGate.valid')} "
                  f"rng={d.get('visGate.range', 0):.2f}   ", end="\r")

        try:
            if a.calib:
                # Props off, drone parked at the takeoff mark at gate height facing gate A.
                # The estimator was just reset, so its position IS the origin and the true
                # drone->gate vector is known exactly from the survey. Anything the camera
                # disagrees about is therefore camera error, not drift -- which is what
                # makes this a calibration and not a chicken-and-egg.
                print(f"\nCALIB: hold still. Expect the drone parked at the takeoff mark, "
                      f"at z={Z_FLY:.2f} m, facing gate A.")
                stream(10.0, lambda t: show("calib"), "calib")
                s = [r for r in rows if r[1] == "calib"
                     and r[COLS.index("vg_valid")] == 1
                     and r[COLS.index("vg_range")]]

                def avg(k):
                    v = [r[COLS.index(k)] for r in s if r[COLS.index(k)] is not None]
                    return sum(v) / len(v) if v else float("nan")

                print()
                if len(s) < 20:
                    raise SystemExit(f"CALIB FAILED: only {len(s)} valid gate detections. "
                                     f"Is gate A in view and lit?")
                ex, ey, ez = avg("x"), avg("y"), avg("z")
                gx, gy, gz = avg("vg_gx"), avg("vg_gy"), avg("vg_gz")
                rng = avg("vg_range")
                # truth: surveyed gate minus where the estimator says we are
                tx, ty, tz = gAx - ex, gAy - ey, gAz - ez
                R = math.sqrt(tx * tx + ty * ty + tz * tz)
                # measured drone->gate vector (world == body here: we are facing +x)
                mx, my, mz = gx - ex, gy - ey, gz - ez
                mh = math.hypot(mx, my)
                th = math.hypot(tx, ty)

                # 1. range scale -> the aperture is the only linear knob in range=fx*W/px
                W_new = (a.gatew * IN2M) * R / rng
                # 2. lateral boresight: rotate the measured bearing onto the true one
                yaw_fix = math.atan2(ty, tx) - math.atan2(my, mx)
                # 3. vertical boresight: +mntPit tilts the measured vector DOWN
                pit_fix = math.atan2(mz, mh) - math.atan2(tz, th)

                print("=" * 68)
                print(f"  parked at ({ex:+.3f},{ey:+.3f},{ez:+.3f})   {len(s)} detections")
                print(f"  range   : measured {rng:.3f} m   true {R:.3f} m   "
                      f"({100*(rng/R-1):+.0f}%)")
                print(f"  bearing : lateral {math.degrees(-yaw_fix):+.1f} deg off, "
                      f"vertical {math.degrees(pit_fix):+.1f} deg off")
                print(f"  correction it would inject: "
                      f"({avg('vf_dx'):+.3f},{avg('vf_dy'):+.3f},{avg('vf_dz'):+.3f}) m "
                      f"-- must be ~0 before you fly --inject 1")
                print("-" * 68)
                print("  SET THESE (cfclient PARAM tab, or the flags below):")
                print(f"    visGate.gateW  = {W_new:.4f}   ({W_new/IN2M:.1f} in)")
                print(f"    visGate.gateH  = {W_new:.4f}")
                print(f"    visGate.mntYaw = {yaw_fix:+.4f}   rad "
                      f"({math.degrees(yaw_fix):+.1f} deg)")
                print(f"    visGate.mntPit = {pit_fix:+.4f}   rad "
                      f"({math.degrees(pit_fix):+.1f} deg)")
                print()
                print(f"  gateW is a CROSS-CHECK, not a measurement: go tape the hoop's")
                print(f"  inner opening. If it is not ~{W_new/IN2M:.0f} in, the range error is")
                print(f"  something else (fx, or the corner scaling) and trimming gateW")
                print(f"  would just hide it.")
                print(f"  Then re-run --calib to confirm vf_d ~ (0,0,0), and only then fly.")
                print("=" * 68)
            elif a.dryrun:
                # Motors never spin: the loop phase advances regardless of arming, so the
                # whole schedule (w, gate A->none->B->none) sweeps with the props off.
                print(f"\nDRYRUN: sweeping the schedule for {CIRC_S:.0f}s "
                      f"(expect gate 1 -> 0 -> 2 -> 0 each lap)")
                P("circuit.en", 1)
                stream(CIRC_S, lambda t: show("dry"), "dryrun")
                P("circuit.en", 0)
            else:
                cf.platform.send_arming_request(True)
                time.sleep(1.2)

                stream(1.5, lambda t: send(0, 0, Z_FLY * min(1.0, t / 1.5), 0.0), "takeoff")

                # Hover on the origin AT GATE HEIGHT. Gate A is then dead ahead, level with
                # the camera and square-on -- every term of the visibility weight is 1, so
                # this is the free check: visFuse.w should read ~1.0 with gate=1. If it
                # doesn't, the survey or the mount extrinsics are wrong and you should NOT
                # be running with --inject 1.
                print(f"\nhover on origin {a.hover:.0f}s at z={Z_FLY:.2f} "
                      f"-- expect visFuse.w ~1.0, gate=1")
                stream(a.hover, lambda t: (send(0, 0, Z_FLY, 0.0), show("hover")), "hover")
                print(f"\n  -> w={d.get('visFuse.w')} gate={d.get('visFuse.gate')} "
                      f"drift=({d.get('visFuse.dx')}, {d.get('visFuse.dy')}, "
                      f"{d.get('visFuse.dz')})")

                # Ease onto the join point instead of engaging from the origin: the origin
                # is ~0.5 m OFF the circle, so engaging there makes the target snap to the
                # join point and the drone dash for it. Get there first, at the loop's own
                # heading, and circuit.en=1 becomes a no-op.
                x0, y0, _, _ = est()
                print(f"\ngoto join ({JP[0]:+.2f},{JP[1]:+.2f}) yaw {JOIN_YAW:+.0f}")
                GOTO = 4.0
                stream(GOTO, lambda t: (send(
                    x0 + (JP[0] - x0) * min(1.0, t / GOTO),
                    y0 + (JP[1] - y0) * min(1.0, t / GOTO),
                    Z_FLY, JOIN_YAW * min(1.0, t / GOTO)), show("goto")), "goto")
                stream(2.5, lambda t: (send(JP[0], JP[1], Z_FLY, JOIN_YAW),
                                       show("settle")), "settle")
                _, _, _, yw = est()
                if abs(((yw - JOIN_YAW + 180) % 360) - 180) > 15:
                    print(f"\n  !! yaw is {yw:+.0f}, wanted {JOIN_YAW:+.0f} -- engage "
                          f"will be lively")

                print(f"\nCIRCUIT: {a.laps:g} laps, {CIRC_S:.0f}s"
                      + (f", RAMP {a.speed}->{a.rampto} m/s" if a.rampto else ""))
                P("circuit.en", 1)
                spd = [a.speed]   # currently-commanded circuit speed (for the ramp)

                def circ_fn(t):
                    # Keep streaming a setpoint that TRACKS the estimate. circuit.en
                    # overrides the position, so this is ignored in flight -- but it feeds
                    # the commander watchdog, and it means disengaging drops us into a
                    # hover right here instead of lunging back across the ring.
                    if a.rampto:
                        v = a.speed + (a.rampto - a.speed) * min(1.0, t / CIRC_S)
                        if abs(v - spd[0]) > 0.02:
                            spd[0] = v
                            cf.param.set_value("circuit.speed", v)   # live speed change
                        leg[0] = f"circ v{v:.2f}"   # stamp live speed into the CSV
                    else:
                        leg[0] = f"lap{int(t / LAP_S) + 1}/{int(a.laps)}"
                    send(*est())
                    show(leg[0])

                stream(CIRC_S, circ_fn, "circuit")
                P("circuit.en", 0)
                if a.rampto:
                    P("circuit.speed", a.speed)   # restore for the next run

                hx, hy, hz, hyw = est()
                stream(2.0, lambda t: (send(hx, hy, Z_FLY, hyw), show("hold")), "hold")
                print("\nlanding")
                stream(2.5, lambda t: send(hx, hy, max(0.05, Z_FLY * (1 - t / 2.5)), hyw),
                       "land")
        except KeyboardInterrupt:
            print("\n*** ABORT -- landing ***")
            try:
                cf.param.set_value("circuit.en", 0)
                hx, hy, _, hyw = est()
                stream(2.0, lambda t: send(hx, hy, max(0.05, Z_FLY * (1 - t / 2.0)), hyw),
                       "abort")
            except Exception:
                pass
        finally:
            for fn in ([lambda: cf.param.set_value("circuit.en", 0),
                        lambda: cf.commander.send_stop_setpoint(),
                        lambda: cf.platform.send_arming_request(False)]
                       + [c.stop for c in cfgs]):
                try:
                    fn()
                except Exception:
                    pass
except Exception as exc:
    print(f"\nsession ended: {exc!r}")

# ---------------------------------------------------------------- dump
with open(a.out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(COLS)
    w.writerows(rows)
print(f"\nwrote {a.out}  ({len(rows)} rows)")

# Sidecar for compare_vision_truth.py: the reset instant (frame origin) + the surveyed
# geometry that `imp_*` is anchored on. Written even if the flight aborted early.
meta_path = a.out.rsplit(".", 1)[0] + ".meta.json"
with open(meta_path, "w") as f:
    json.dump({
        "reset_host_t": reset_host_t[0],   # epoch s; None if we never reset (dryrun/calib abort)
        "gates_survey_m": {"A": [gAx, gAy, gAz], "B": [gBx, gBy, gBz]},
        "z_fly_m": Z_FLY,
        "calib": {"gateW_in": (a.gatew if a.gatew is not None else CAL_GATEW_IN),
                  "mntYaw": (a.mntyaw if a.mntyaw is not None else CAL_MNTYAW),
                  "mntPit": (a.mntpit if a.mntpit is not None else CAL_MNTPIT)},
        "inject": a.inject, "uri": a.uri,
    }, f, indent=2)
print(f"wrote {meta_path}")

# NB: test for None, not truthiness -- on an --inject 0 baseline vf_n is legitimately 0
# the whole run, and that run is the one whose drift summary matters most.
seen = [r for r in rows if r[COLS.index("vf_n")] is not None]
if seen:
    n_end = seen[-1][COLS.index("vf_n")]
    rej_end = seen[-1][COLS.index("vf_rej")]
    print(f"vision: {n_end} fixes injected, {rej_end} rejected"
          f"{'  (inject was OFF -- n should be 0)' if not a.inject else ''}")
    # circuit rows are now labelled "lap.../..." or "circ v..." (ramp) -- match the prefix
    def on_circuit(r):
        return r[1].startswith("lap") or r[1].startswith("circ v")
    circ = [r for r in rows if on_circuit(r) and r[COLS.index("vf_gate")]]
    if circ:
        drift = [math.sqrt(sum((r[COLS.index(k)] or 0.0) ** 2
                               for k in ("vf_dx", "vf_dy", "vf_dz"))) for r in circ]
        print(f"|correction| on gate approaches: first {drift[0]:.3f} m, "
              f"last {drift[-1]:.3f} m, max {max(drift):.3f} m")
    # Ramp run: report the speed at which valid vision fell off, per gate. This is the
    # headline number of a --rampto flight -- how fast you can go and still see the gates.
    if a.rampto:
        vi, vg, vv = (COLS.index("leg"), COLS.index("vg_valid"), COLS.index("vf_gate"))
        def speed_of(r):
            try: return float(r[vi].split("v")[1])
            except Exception: return None
        good = [speed_of(r) for r in rows if r[1].startswith("circ v")
                and r[vg] == 1 and r[vv] in (1, 2)]
        good = [v for v in good if v is not None]
        if good:
            print(f"vision held up to {max(good):.2f} m/s "
                  f"(valid detections stop above that)")
if console:
    with open("tinympc_console.txt", "w") as f:
        f.write("".join(console))
    print("wrote tinympc_console.txt")
