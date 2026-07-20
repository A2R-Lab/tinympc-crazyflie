# Camera accuracy vs. mocap — runbook

**Goal:** produce a CSV of *where the camera says the drone is* vs *where the drone
really is in the world*, so we can put a number on the camera + its calibration.

## Why this works

The firmware already turns a sighting of a **surveyed** gate into an **absolute** drone
position: `imp = surveyed_gate − measured_offset` (`controller_tinympc.cpp:464`). It does
**not** use the drifting Flow-deck estimate — only the camera measurement and the known
gate location — so it is a genuine camera-based localization. `tinympc_circuit_run.py`
logs it per row as `imp_x/imp_y/imp_z`. That is *"where the camera says the drone is."*

What was missing is independent **truth**. The estimator's own `x/y/z` is Flow-deck
odometry (it drifts — it is *not* truth). Mocap is truth. So we log mocap concurrently,
frame-align it to the drone's reset origin, and diff it against `imp_*`.

```
 camera error  =  imp_*  (surveyed_gate − measured_offset)   −   mocap_drone (reset frame)
                  └──────────── logged onboard ─────────────┘      └──── ground truth ────┘
```

The residual is the accuracy of the whole camera chain: intrinsics (`camera_calibration.yaml`),
the range/aperture scale (`visGate.gateW`), and the mount boresight trims
(`visGate.mntYaw/mntPit`). With `--gate-truth` you can also remove tape-survey error and
isolate *pure* camera error.

---

## Full command sequence (copy-paste)

Two machines: **tiger** (`ssh lab@10.10.30.2`, mocap) and the **radio host / Mac** (the
flight). Confirm your rigid-body names first (`--drone-topic`/`--gate-topic`) and the
drone's radio `--uri` (the script defaults to `...E8`; drones flash to `...E7`).

### 0. Sync the clocks (once per session)
```bash
# tiger:
sudo timedatectl set-ntp true
# Mac:
sudo sntp -sS time.apple.com
```

### 1. tiger — bridge, then the logger (START BEFORE THE FLIGHT)
```bash
# bridge (in the ROS2 container — see README_mocap_labeling.md "restart the bridge"):
cd ~/ros2-crazyflie-mocap && sudo make run
#   inside the container:
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/root/ros2_ws/src/optitrack_ros2/src/lib:$LD_LIBRARY_PATH
ros2 run optitrack_ros2 optitrack_node --ros-args \
    -p server_address:=10.10.30.3 -p local_address:=10.10.30.2 -p tracking_mode:=rigid_body

# second container shell (sudo make attach) — the logger:
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
python3 /root/ros2_ws/mocap_ros2_logger.py \
    --drone-topic /optitrack/drone/pose \
    --gate-topic /optitrack/gate_1/pose /optitrack/gate_2/pose \
    --out /root/ros2_ws/cam_accuracy_01.csv
```

### 2. Mac — fly and log the vision run
```bash
cd /Users/char_chen/School/TinyMPC/tinympc-crazyflie/apps/controller_tinympc_eigen

# static calibration check (props off, cleanest — do this first):
./tinympc_circuit_run.py --calib --uri radio://0/80/2M/E7E7E7E7E7 --out cam_accuracy_01.csv

# OR a full circuit (props on, netted; --inject 0 so vision is logged, not fused):
./tinympc_circuit_run.py --laps 5 --inject 0 --uri radio://0/80/2M/E7E7E7E7E7 --out cam_accuracy_01.csv
```
→ writes `cam_accuracy_01.csv` **and** `cam_accuracy_01.meta.json`.

### 3. Stop the logger cleanly, pull its CSV to the Mac
```bash
# tiger: Ctrl-C the logger, wait for "wrote N rows", then on the Mac:
scp lab@10.10.30.2:~/ros2-crazyflie-mocap/ws/cam_accuracy_01.csv ./mocap_01.csv
```

### 4. Mac — analyze
```bash
# find the camera latency first (writes the best-offset result):
./compare_vision_truth.py --run cam_accuracy_01.csv --mocap mocap_01.csv --sweep -0.12 0.02 0.01

# the comparison CSV (plug in the best offset from the sweep, e.g. -0.04):
./compare_vision_truth.py --run cam_accuracy_01.csv --mocap mocap_01.csv \
    --offset -0.04 --out vision_vs_truth_01.csv

# pure camera error, tape-survey removed (anchors on the mocap gate):
./compare_vision_truth.py --run cam_accuracy_01.csv --mocap mocap_01.csv \
    --offset -0.04 --gate-truth
```

The sections below explain each step, the two run modes, and how to read the output.

## Two ways to run it

**A. Static / hover facing a gate — the cleanest calibration check (do this first).**
The estimator is fresh (no drift), the drone barely moves (no motion-blur or sync error),
and the geometry is textbook. This most directly answers *"is my calibration right?"*
Use the existing `--calib`/hover leg of `tinympc_circuit_run.py`, or just hover.

**B. Full circuit — localization accuracy across the real operating envelope.**
Fly `tinympc_circuit_run.py` as usual; the comparison covers every gate approach in the
lap and breaks the error down by range.

Either way the capture + analysis steps are identical.

## Capture (paired: run + mocap, same wall clock)

The vision run is logged on the machine with the CrazyRadio (`tinympc_circuit_run.py`,
cflib over CRTP). Mocap is logged on **tiger** by `esp_color_object/tools/mocap_ros2_logger.py`
inside the ROS2 bridge container. Both stamp host `time.time()`; NTP keeps them within a
few ms. (See `esp_color_object/tools/README_mocap_labeling.md` for the full mocap/Motive
setup, network map, and the drone/gate rigid-body alignment — the same bridge and logger.)

1. **NTP both machines** (once per session):
   - tiger: `sudo timedatectl set-ntp true`
   - the run host (Mac): `sudo sntp -sS time.apple.com`

2. **Bring up the mocap bridge on tiger** and confirm `/optitrack/drone/pose` (+ gate
   topics) — per the labeling README's "restart the bridge" block.

3. **Start the mocap logger on tiger, BEFORE the flight** (it stamps host time and logs
   the drone + every gate):
   ```bash
   python3 /root/ros2_ws/mocap_ros2_logger.py \
       --drone-topic /optitrack/drone/pose \
       --gate-topic /optitrack/gate_1/pose /optitrack/gate_2/pose \
       --out /root/ros2_ws/cam_accuracy_01.csv
   ```

4. **Run the flight** on the radio host. The script now stamps `host_t` per row and writes
   a `*.meta.json` sidecar recording the `kalman.resetEstimation` instant (that instant
   defines the reset frame the comparison aligns to):
   ```bash
   # static/hover check (props off is fine for the calib leg):
   ./tinympc_circuit_run.py --calib --out cam_accuracy_01.csv
   # or a full circuit:
   ./tinympc_circuit_run.py --laps 5 --inject 0 --out cam_accuracy_01.csv
   ```
   → writes `cam_accuracy_01.csv` **and** `cam_accuracy_01.meta.json`.

   > Use `--inject 0`. With `--inject 1` the vision is folded into the EKF, so `imp_*`
   > and the estimate converge and the comparison no longer measures the raw camera.

5. **Stop the mocap logger cleanly** (Ctrl-C, wait for `wrote N rows`) and pull its CSV to
   the radio host:
   ```bash
   scp lab@10.10.30.2:~/ros2-crazyflie-mocap/ws/cam_accuracy_01.csv ./mocap_01.csv
   ```

## Analyze

```bash
# frame-aligns via the reset instant, joins by host time, writes the comparison CSV:
./compare_vision_truth.py --run cam_accuracy_01.csv --mocap mocap_01.csv \
    --out vision_vs_truth_01.csv

# find the camera latency (positive = camera lags truth); writes the best-offset result:
./compare_vision_truth.py --run cam_accuracy_01.csv --mocap mocap_01.csv \
    --sweep -0.12 0.02 0.01

# isolate PURE camera error (remove tape-survey error by anchoring on the mocap gate):
./compare_vision_truth.py --run cam_accuracy_01.csv --mocap mocap_01.csv \
    --offset <best> --gate-truth
```

Prints, and writes per-frame to the output CSV:
- `cam_*` (camera-predicted), `true_*` (mocap, reset frame), `est_*` (odometry, reference)
- `err_*` / `err_norm` (camera error), `est_err_norm` (odometry error, for comparison)
- summary: mean/median/RMS/p95 `|error|`, per-axis **bias** (the calibration tell — a
  consistent x/y/z offset points at a range-scale or boresight-trim error, not noise), and
  a **per-range** breakdown (does accuracy fall off with distance, as expected?).

### Reading it
- **Bias ≈ 0, small spread** → calibration is good; residual is detection noise.
- **Systematic bias** → a calibration error. A radial (along-range) bias ⇒ `visGate.gateW`
  range scale; a lateral/vertical bias ⇒ `visGate.mntYaw`/`mntPit` boresight. This mirrors
  what `tinympc_circuit_run.py --calib` estimates, but against mocap truth instead of the
  tape survey — so it also catches survey error (cross-check with `--gate-truth`).
- **Error grows with range** → expected (apparent-size range error is ~quadratic); the CSV
  quantifies where it crosses your tolerance.
- **Camera vs odometry line** → whether a vision fix is actually better than dead-reckoning
  at that point in the flight (the whole reason for fusing it).

## Bonus: let mocap set the gates (skip the tape survey)

The drone already builds the whole loop **onboard** from two gate centres (`circuitPoint()`
in `controller_tinympc.cpp`, params `circuit.gAx..gBz` + `circuit.loopW`). Normally those
come from tape (`--gate1/--spacing/--gatez`). `--gates-from-mocap` sources them from mocap
instead — so if you move or **raise** the gates, you don't re-measure, and the height is
exact:

```bash
# on tiger, inside the ROS2 container (rclpy + bridge up), drone parked on its mark:
./tinympc_circuit_run.py --laps 5 --inject 0 --gates-from-mocap \
    --drone-topic /optitrack/drone/pose \
    --gate-topics /optitrack/gate_1/pose /optitrack/gate_2/pose \
    --out cam_accuracy_01.csv
```

At the estimator reset it snapshots the mocap drone + gate poses, transforms the gate
centres into the reset frame (`gate_reset = Rz(−yaw0)·(gate_mocap − p0)`, the same
transform the accuracy comparison uses), pushes `circuit.gAx..gBz` + `loopW`, and reprints
the plan. `--gate-topics` order = gate **A** then **B**. The gate rigid-body pivots are
already centered on the opening (from the labeling setup), so the mocap position *is* the
gate centre the trajectory wants.

Preview what mocap sees without flying (drone parked = the reset pose):
```bash
./mocap_gates.py --drone-topic /optitrack/drone/pose \
    --gate-topics /optitrack/gate_1/pose /optitrack/gate_2/pose
```
Prints each gate in the reset frame and the exact `circuit.g*` it would push.

> This replaces only the *survey*; the trajectory is still generated onboard. It does not
> feed mocap into the estimator — flight positioning stays vision/odometry, so a
> `--gates-from-mocap` run is still a valid camera-accuracy capture (the meta records
> `gates_source: mocap`).

## Notes / gotchas
- The comparison needs the `host_t` column and the `*.meta.json` sidecar — both are
  produced by the updated `tinympc_circuit_run.py`. Older CSVs (drone-clock only) can't be
  time-joined; re-fly them.
- No reset time in the meta (e.g. a `--dryrun`)? Pass `--align fit` to recover the frame by
  least-squares against early (low-drift) odometry instead — slightly less accurate.
- `--max-dt` (default 30 ms) drops rows with no closely-synced mocap sample; the summary
  reports the achieved sync `|dt|`.
- Log the **gate** rigid bodies in mocap (step 3 does) even for the static test — it's what
  makes `--gate-truth` possible.
