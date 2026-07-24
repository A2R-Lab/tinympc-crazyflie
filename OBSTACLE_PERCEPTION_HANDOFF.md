# Obstacle Perception Handoff

Date: 2026-07-23

Workspace used: `/home/charchen/TinyMPC`

Hardware status: the Crazyflie and AI-deck are not available, so the changes in
this handoff have build validation but no new hardware or flight validation.

## Repositories and branches

Two sibling repositories must be checked out on the replacement machine:

| Repository | Branch | Remote |
| --- | --- | --- |
| `tinympc-crazyflie` | `vision` | `git@github.com:A2R-Lab/tinympc-crazyflie.git` |
| `tinympc-nanocockpit` | `gate8-dory-package` | `git@github.com:cookacola/tinympc-nanocockpit.git` |

The Crazyflie repository contains a `crazyflie-firmware` submodule on branch
`agent/flow-obstacle-uart`. Its required UART hardening commit is
`93abf2c859a5ebc02713854f1217c824dcfa95aa`, already pushed to
`https://github.com/cookacola/crazyflie-firmware.git`.

After cloning, run:

```sh
git -C tinympc-crazyflie submodule update --init --recursive
git -C tinympc-nanocockpit submodule update --init --recursive
```

Use `git log -1 --oneline` in each sibling repository to identify the handoff
commits. Both branches were pushed at the end of this session; no PR was opened.

## Goal and current architectural decision

The goal is reliable close-range monocular obstacle detection and avoidance
using the AI-deck camera, GAP8, Crazyflie STM32, and TinyMPC.

The flight experiments preceding this handoff showed:

- With little or no lateral peering, a roughly 0.5 m obstacle was estimated at
  about 0.23 m and confidence was about 0.15.
- With approximately +/-8 cm peering, the same setup estimated about 0.45 m,
  confidence rose to about 0.22, and the vehicle successfully avoided it.
- UART delivery was generally clean at approximately 15 Hz. Observability,
  calibration, temporal alignment, and premature spatial aggregation are more
  important limitations than raw transport reliability.

Do not make the nine image sectors the primary perception representation.
Retain them as a compatibility fallback and as the final compact planner
interface. The intended pipeline is:

```text
calibrated feature detection + forward/backward LK on GAP8
    -> timestamped, quantized per-feature tracks over UART
    -> capture-time state interpolation on STM32
    -> per-track depth and uncertainty
    -> robust spatial/temporal clustering
    -> sectors/cylinder for TinyMPC
```

Looming/time-to-contact should remain an independent emergency braking cue.

## Task completed in this session

The capture-time state synchronization and camera calibration/extrinsics task
is complete at source and build level.

### STM32 / `tinympc-crazyflie`

- Sends position, velocity, compressed attitude, and body rates to GAP8 at
  100 Hz using the existing `!STA` packet ABI.
- State construction in the controller is non-blocking: a one-element queue
  retains the newest state and a low-priority UART1 DMA task transmits it.
- Enables UART1 DMA explicitly in the out-of-tree build.
- Records a 64-sample local state history and interpolates velocity, pose, and
  yaw at the camera exposure tick echoed by GAP8.
- Falls back to current state when no valid echo exists, with `syncOk`,
  `syncErr`, and `syncMiss` logs for diagnosis.
- Supports runtime horizontal camera extrinsics through:
  `flowCal.camYaw`, `flowCal.camFwd`, and `flowCal.camLeft`.
- Applies camera lever-arm velocity and the camera-to-body planar transform.
- Propagates flow, velocity, and gyro uncertainty to range uncertainty and
  rejects estimates with excessive absolute or relative uncertainty.

Relevant files:

- `apps/controller_tinympc_eigen/src/controller_tinympc.cpp`
- `apps/controller_tinympc_eigen/src/gate8_link.[ch]`
- `apps/controller_tinympc_eigen/src/flowdeck_obstacle_link.[ch]`
- `apps/controller_tinympc_eigen/Makefile`

### GAP8 / `tinympc-nanocockpit`

- Receives `!STA` packets concurrently with camera processing.
- Maps each GAP8 camera timestamp into the STM32 millisecond tick domain and
  echoes that tick in the outgoing flow packet.
- Uses measured camera intrinsics and Brown-Conrady distortion coefficients:
  `fx=89.15584`, `fy=89.46082`, `cx=81.10381`, `cy=73.34730`,
  distortion `[-0.01764488, 0.09941325, 0.00544322, -0.00604001,
  -0.19001899]`.
- Converts tracked pixels to undistorted normalized rays before computing flow.
- Adds a flow-dispersion estimate to every sector. The flow packet ABI is now
  204 bytes on both processors; update both sides together.

The same GAP8 worktree also contains the next frontend improvements because
they were developed while auditing synchronization:

- two coarse LK iterations plus one full-resolution refinement;
- forward/backward consistency rejection at 0.75 pixels;
- confidence based on support and flow signal-to-noise instead of a fixed
  track-count divisor;
- optional one-frame-pair diagnostic capture controlled by
  `FLOW_DIAGNOSTIC_CAPTURE`.

Relevant files:

- `src/gap/examples/pulp-frontnet/main.c`
- `src/gap/examples/pulp-frontnet/flow_obstacle_uart.[ch]`
- `src/gap/examples/pulp-frontnet/Makefile`

## Validation completed

The following commands passed on 2026-07-23:

```sh
cd tinympc-crazyflie/apps/controller_tinympc_eigen
make -j4
```

This produced a CF21BL firmware build. Reported memory usage was approximately
340,656/1,032,192 bytes flash, 110,264/131,072 bytes RAM, and
62,392/65,536 bytes CCM.

```sh
cd tinympc-nanocockpit/src/gap
./gap8.sh examples/pulp-frontnet FLOW_FEATURE_COUNT=27 clean build
```

This produced the GAP8 image. Reported memory usage was approximately
152,408/524,288 bytes L2 and 7,740/16,380 bytes FC TCDM.

`git diff --check` passed in both repositories before committing. Hardware
timing, UART duplex behavior, estimator accuracy, and flight safety remain
unvalidated because the drone is unavailable.

Generated build directories, caches, raw hardware logs, derived timing CSVs,
and other untracked artifacts were intentionally not committed.

## Remaining implementation tasks, in recommended order

### 1. Replace early sectors with a per-feature UART packet

Add a second packet type while keeping the 204-byte sector packet as a fallback.
A reasonable bounded design is 32 tracks at 15 Hz:

- payload header: GAP8 timestamp, echoed STM32 tick, frame `dt`, sequence,
  count, and flags;
- each track: quantized starting `u,v`, `du,dv`, LK residual, and
  forward/backward residual;
- approximately 408 bytes total for 32 twelve-byte tracks, or about 6.1 kB/s
  at 15 Hz, which fits 115200 baud alongside the other traffic.

Select tracks across the image rather than merely taking the first 32. Preserve
the existing CRC and resynchronizing header parser. Add compile-time size
assertions on both processors and host serialization tests.

### 2. Perform per-track depth estimation on STM32

For each accepted track:

- reconstruct calibrated undistorted rays;
- use the echoed capture tick to interpolate velocity, pose, and angular rate;
- remove rotational flow;
- compute parallax and/or looming inverse depth;
- propagate LK/FB, velocity, gyro, calibration, and timestamp uncertainty;
- reject unobservable or high-variance tracks.

Cluster candidate 2-D obstacle points using a bounded robust method such as
union-find with range/position gates or a small fixed-grid vote. Weight accepted
points by inverse variance, apply N-of-M temporal persistence, and only then
generate planner sectors and the obstacle cylinder.

### 3. Add peering observability and planner safety gates

- Accumulate actual lateral baseline rather than assuming the commanded peer
  motion was achieved.
- Do not freeze an obstacle until minimum baseline, support, temporal
  persistence, and range uncertainty requirements are satisfied.
- Log baseline, per-track rejection reasons, accepted track count, component
  size, and cylinder covariance.
- Have the flight script abort or continue safe peering when covariance is too
  large, instead of accepting confidence alone.
- Preserve conservative emergency stop behavior when looming indicates danger.

### 4. Finish diagnostic capture and deterministic replay

The GAP8 `FLOWCAP_*` text dump exists but has no checked-in parser yet.

- Add a host tool that converts the dump to two PGM images plus JSON/CSV track
  metadata.
- Add an embedded-equivalent replay path that runs the exact frontend or a
  carefully tested portable extraction of it.
- Store only small, curated fixtures in Git. Do not commit long serial logs or
  build artifacts.
- Add regression cases for subpixel motion, low texture, edge tracks, repeated
  patterns, blur, exposure change, and distortion near image borders.

### 5. Upgrade simulation to image level

The existing primary PyBullet and estimator simulations bypass the camera:
they use ground-truth rays or analytic ideal flow. Add a calibrated grayscale
renderer that exercises the actual feature frontend, including:

- measured intrinsics and distortion;
- textured planes and cylinders/boxes;
- motion blur, read noise, exposure variation, and low texture;
- frame timing jitter, dropped frames, UART latency, and clock offset;
- state/gyro/velocity noise and camera-extrinsic perturbations.

Test expected degradation and safety behavior, not just ideal accuracy:
no-motion unobservability, insufficient peering, timestamp bias, mixed
foreground/background depth, outliers, and false looming.

### 6. Hardware validation when the drone is available

Bench first, with props removed:

1. Flash matching GAP8 and CF21BL builds.
2. Confirm `gate8.stateTx` increases near 100 Hz with no state drops.
3. Confirm GAP8 receives fresh state and flow packets echo nonzero STM32 ticks.
4. Confirm `flowObsRx.syncOk=1`, low `syncErr`, stable 15 Hz flow reception,
   zero or negligible CRC errors, and no UART/reset regressions.
5. Measure GAP8 frontend time after full-resolution and FB LK; ensure the
   camera callback and 15 Hz transmission remain sustainable.
6. Check camera extrinsics against the physical AI-deck mount and set the
   `flowCal` parameters.
7. Use a measured obstacle at several ranges with manual lateral translation;
   compare estimated range, sigma, and timestamp-aligned state.

Then use restrained flight tests:

1. Hover facing an obstacle at a known safe distance.
2. Peer laterally while holding range.
3. Require the new observability/uncertainty gates before freezing.
4. Verify the mapped obstacle and cylinder position before enabling avoidance.
5. Enable PID detour first, then TinyMPC only after repeatable detections.
6. Keep a pilot abort path and conservative minimum separation throughout.

## Known risks and details to revisit

- The new state-channel clock mapping assumes UART latency is small and
  approximately constant. Measure residual clock error on hardware; a fitted
  offset/skew estimator may be needed.
- UART1 DMA uses a 64-byte static driver buffer; the 40-byte state packet fits.
  Future packet types sent STM32-to-GAP8 must respect that buffer or revise the
  driver.
- STM32 RAM is already at roughly 84% and CCM at roughly 95%. Keep future
  buffers fixed and small, and inspect memory reports after every change.
- Full-resolution refinement plus backward LK built successfully but has not
  been timed on GAP8 hardware. It may require fewer tracks, conditional
  refinement, or moving work to the cluster.
- The current extrinsic model is planar (`yaw`, forward, left). Add vertical
  offset and full camera rotation only if the 3-D estimator begins using them.
- The sector ABI change is intentionally synchronized between the two sibling
  repositories. Mixing older and newer images will fail CRC/frame parsing until
  a versioned packet strategy is introduced.
- The current confidence/range-uncertainty formulas are engineering priors, not
  calibrated noise models. Tune from replay and measured hardware data.

## Previous successful flight command context

The working obstacle flight path used:

```sh
/home/charchen/cf-venv/bin/python tinympc_flow_obstacle_flight.py \
  --height 0.3 \
  --takeoff-s 5 \
  --settle-s 5 \
  --freeze-after-s 10 \
  --approach-s 12 \
  --approach-x 0.8 \
  --approach-peer-amp 0.15 \
  --run-s 14 \
  --goal-x 3.2 \
  --goal-y 0 \
  --obs-safety 0.15 \
  --detour-margin 0.05 \
  --pid-detour
```

The earlier confidence floor of 0.25 rejected a valid-looking frozen map with
confidence 0.117; lowering thresholds alone is not the desired final fix.
Restore decisions should be based on calibrated uncertainty, support,
observability, and persistence.

## Copy-ready prompt for the next agent

```text
Continue the Crazyflie monocular obstacle-perception work in
/home/charchen/TinyMPC. There are two sibling Git repositories:

- tinympc-crazyflie, branch vision, remote A2R-Lab/tinympc-crazyflie
- tinympc-nanocockpit, branch gate8-dory-package, remote
  cookacola/tinympc-nanocockpit

First read tinympc-crazyflie/OBSTACLE_PERCEPTION_HANDOFF.md completely and
inspect both repositories' current status and latest commits. Initialize
submodules if necessary. The Crazyflie and AI-deck are not available, so do
not claim hardware or flight validation.

Capture-time STM32/GAP8 state synchronization, calibrated undistortion, planar
camera extrinsics, flow/range uncertainty, forward/backward LK validation, and
diagnostic capture scaffolding are already implemented and cross-compiled.
Preserve those changes.

The next task is to stop using nine sectors as the primary perception
representation. Implement a bounded, versioned, CRC-protected per-feature UART
packet from GAP8 to STM32 (target about 32 spatially distributed quantized
tracks at 15 Hz), while retaining the existing sector packet as a compatibility
fallback. Add matching compile-time ABI assertions and host serialization/parser
tests on both sides. Then implement capture-time synchronized per-track depth
and uncertainty on STM32, robust clustering/persistence, and only late
sector/cylinder formation for TinyMPC. Work incrementally and keep fixed memory
bounds because STM32 RAM/CCM are tight.

After that, continue the remaining tasks in the handoff: peering observability
and planner safety gates; diagnostic dump parsing and deterministic replay;
image-level simulation with calibrated sensor/timing/state randomization; host
tests and clean builds; and finally the documented hardware checklist when a
drone becomes available.

Do not stage caches, raw logs, derived timing CSVs, or build artifacts. At each
safe milestone, run the relevant clean builds/tests. If asked to publish, commit
and push the two existing branches but do not open a PR.
```
