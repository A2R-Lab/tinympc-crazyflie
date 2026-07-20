# Firmware Obstacle-Avoidance Implementation Plan

This is the concrete firmware handoff for moving the validated simulation path onto the
Crazyflie. It assumes the code is split across two repositories:

- `tinympc-crazyflie/apps/controller_tinympc_eigen`: STM32 flight controller, TinyMPC,
  state estimate, logging, and UART receive path.
- `tinympc-nanocockpit`: AI-deck/GAP8 camera frontend, optical flow, depth-sector
  aggregation, and UART transmit path.

The first firmware version should implement **one active position half-space**. The simulator
can carry `MAX_HS=3`, but the batch result was worse for naive multi-plane selection. Firmware
v1 should therefore use the best validated behavior: choose one most-threatening sector, hold
it briefly, synthesize one `n^T p <= b` constraint, and feed that to TinyMPC.

## Target Data Flow

```text
AI-deck camera
  -> GAP8 sparse LK optical flow
  -> GAP8 de-rotation + inverse depth + sector aggregation
  -> UART flow-sector frame
  -> STM32 seqlock receiver
  -> STM32 sector gating / hold / half-space synthesis
  -> TinyMPC ADMM state projection
  -> existing cascade PID output
```

Pixels and per-feature tracks should stay on the GAP8 in the production path. The STM32 should
receive only compact sector summaries.

## Stage 1: STM32 TinyMPC Half-Space Support

Repository: `tinympc-crazyflie/apps/controller_tinympc_eigen`

### Solver storage

For firmware v1, use a single half-space slot per horizon step. There are two acceptable
layouts:

1. Keep the current sim-compatible layout:

```cpp
Eigen::Vector3f a_hs[NHORIZON][MAX_HS];
float b_hs[NHORIZON][MAX_HS];
int en_hs[NHORIZON][MAX_HS];
```

and only write index `[k][0]` on firmware.

2. If flash/RAM pressure matters, reduce firmware to:

```cpp
Eigen::Vector3f a_hs[NHORIZON];
float b_hs[NHORIZON];
int en_hs[NHORIZON];
```

The first option keeps the host/sim and firmware code closer. The second option is smaller.
Given the current state of the project, prefer option 1 until memory measurements say
otherwise.

### Solver projection

The ADMM projection belongs in `TinyMPC-ADMM/src/tinympc/admm.cpp`, inside
`UpdateSlackDual()`, replacing the state-box projection when a half-space is enabled:

```cpp
if (work->data->en_hs[k][0]) {
  work->ZX_new[k] = work->soln->YX[k];
  Eigen::Vector3f z_pos = work->ZX_new[k].head(3);
  const Eigen::Vector3f a = work->data->a_hs[k][0];
  const float b = work->data->b_hs[k][0];
  const float dist = a.dot(z_pos) - b;
  if (dist > 0.0f) {
    z_pos = z_pos - dist * a;
  }
  work->ZX_new[k].head(3) = z_pos;
} else {
  work->ZX_new[k] =
      work->soln->YX[k].cwiseMin(*(work->data->ucx)).cwiseMax(*(work->data->lcx));
}
```

`a` must be unit length. Normalize before writing the constraint, not inside the hot ADMM loop.

### Solver setter

Add a small setter to `TinyMPC-ADMM/src/tinympc/constraint_linear.cpp` and declare it in
`constraint_linear.h`:

```cpp
enum tiny_ErrorCode tiny_ClearPositionHalfspaces(tiny_AdmmWorkspace* work) {
  for (int k = 0; k < NHORIZON; ++k) {
    work->data->a_hs[k][0].setZero();
    work->data->b_hs[k][0] = 0.0f;
    work->data->en_hs[k][0] = 0;
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetPositionHalfspace(tiny_AdmmWorkspace* work,
                                              int k,
                                              const Eigen::Vector3f& a,
                                              float b,
                                              int enable) {
  if (k < 0 || k >= NHORIZON) {
    return TINY_ERROR;
  }
  if (!enable) {
    work->data->a_hs[k][0].setZero();
    work->data->b_hs[k][0] = 0.0f;
    work->data->en_hs[k][0] = 0;
    return TINY_NO_ERROR;
  }
  const float norm = a.norm();
  if (norm < 1e-6f) {
    work->data->a_hs[k][0].setZero();
    work->data->b_hs[k][0] = 0.0f;
    work->data->en_hs[k][0] = 0;
    return TINY_NO_ERROR;
  }
  work->data->a_hs[k][0] = a / norm;
  work->data->b_hs[k][0] = b / norm;
  work->data->en_hs[k][0] = 1;
  return TINY_NO_ERROR;
}
```

If the firmware uses the single-plane storage layout, remove the `[0]` index.

### Controller integration

In `src/controller_tinympc.cpp`:

1. Initialize state constraints:

```cpp
stgs.en_cstr_states = 1;
tiny_ClearPositionHalfspaces(&work);
```

2. Before each MPC solve, update the constraint from the latest flow-sector sample:

```cpp
updateObstacleHalfspace(state, sensors, tick);
tiny_UpdateLinearCost(&work);
tiny_SolveAdmm(&work);
```

3. Start with a safe sub-horizon, not the full horizon. A good first setting is:

```cpp
static constexpr int OBS_K_START = 3;
static constexpr int OBS_K_END = NHORIZON;
```

Write `en_hs[k][0]=1` only for `k >= OBS_K_START`. This avoids asking the solver to satisfy a
new hard constraint at the current position when the constraint appears late or noisy.

## Stage 2: STM32 Flow-Sector Receiver

Repository: `tinympc-crazyflie/apps/controller_tinympc_eigen`

Add a new receiver pair, separate from gate corners:

- `src/flowdeck_obstacle_link.h`
- `src/flowdeck_obstacle_link.c`

Use the same pattern as `src/gate8_link.c`: sync header, packed payload, CRC32, seqlock
publish, and LOG counters.

Suggested wire format:

```c
#define FLOW_SECT_MAX 9
#define FLOW_SECT_MSG_HEADER "\x90\x19\x8\x34"

typedef struct __attribute__((packed)) {
  float azimuth_rad;   /* camera-frame bearing, x-right */
  float inv_depth;     /* 1/m, 0 if invalid */
  float ttc_s;         /* seconds, <0 if invalid/diverging */
  float confidence;    /* [0, 1] */
} flow_sector_t;

typedef struct __attribute__((packed)) {
  uint32_t gap8_ts_us;
  uint32_t stm32_ts_echo;
  float dt_s;
  uint8_t n_sectors;
  uint8_t flags;
  uint16_t reserved;
  flow_sector_t sector[FLOW_SECT_MAX];
} flow_sectors_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t header[4];
  flow_sectors_payload_t p;
  uint32_t checksum;
} flow_sectors_msg_t;
```

Expose:

```c
void flowObstacleLinkInit(void);
bool flowObstacleLinkGetLatest(flow_sectors_payload_t *out,
                               uint32_t *out_age_ms,
                               uint32_t *out_sample);
```

Add LOG values for:

- `flowObs.rxOk`
- `flowObs.crcErr`
- `flowObs.badRx`
- `flowObs.ageMs`
- `flowObs.n`
- `flowObs.bestDepth`
- `flowObs.bestConf`
- `flowObs.active`
- `flowObs.a0`, `flowObs.a1`, `flowObs.a2`, `flowObs.b`

Do not reuse the existing `gate8` message header. Gate corners and obstacle sectors should be
distinct wire messages.

## Stage 3: Half-Space Synthesis On STM32

Repository: `tinympc-crazyflie/apps/controller_tinympc_eigen`

Add a small C++ helper near the controller code or in a dedicated file such as
`src/obstacle_halfspace.cpp`.

### Input

Use the latest `flow_sectors_payload_t`. Reject it if:

- sample age exceeds `OBS_MAX_AGE_MS`, initially `150 ms`;
- `n_sectors == 0`;
- `confidence < OBS_MIN_CONF`, initially `0.35`;
- `inv_depth <= 0`;
- depth is outside `[OBS_MIN_DEPTH_M, OBS_MAX_DEPTH_M]`, initially `[0.25, 4.0]`;
- low-excitation or rotation-dominated flags are set.

### Sector selection

Pick the lowest score:

```text
score = depth_m / max(confidence, 0.05)
```

Then apply switch hysteresis:

```text
switch only if new_depth < OBS_SWITCH_RATIO * held_depth
```

Use `OBS_SWITCH_RATIO = 0.75` as a first value. Hold a valid sector for
`OBS_HOLD_MS = 100-150 ms` to avoid flicker.

### Geometry

For the selected sector:

```text
depth_m = 1 / inv_depth
d_cam   = normalize([sin(azimuth_rad), 0, cos(azimuth_rad)])
d_body  = R_BC * d_cam
d_world = R_WB * d_body
p_obst  = p_drone + depth_m * d_world
n       = -d_world
b       = dot(n, p_obst) - margin
```

Start with no vertical bearing in the sector message. The sector represents a horizontal
bearing band through the center of the camera image. This matches the sim’s first-order
closed-loop test and avoids sending more payload before the simpler path works.

Use:

```text
margin = OBS_MARGIN_MIN + (1 - confidence) * OBS_MARGIN_SLACK
```

Initial values:

```text
OBS_MARGIN_MIN   = 0.30 m
OBS_MARGIN_SLACK = 0.20 m
```

Clamp `margin <= 0.8 * depth_m` so a close obstacle does not create a plane behind the drone.

### Horizon write

Clear all half-spaces each MPC tick. If a held sector is active:

```cpp
for (int k = OBS_K_START; k < NHORIZON; ++k) {
  tiny_SetPositionHalfspace(&work, k, n, b, 1);
}
```

Otherwise leave all half-spaces disabled.

## Stage 4: STM32 Back-Channel To GAP8

Repository: `tinympc-crazyflie/apps/controller_tinympc_eigen`

The GAP8 needs gyro and body-frame velocity to de-rotate flow and estimate metric inverse
depth. Reuse the NanoCockpit `aideck_protocol` transmit path, or add a new message beside
`state_msg_t`.

Recommended payload:

```c
#define FLOW_STATE_MSG_HEADER "!FST"

typedef struct __attribute__((packed)) {
  uint8_t header[4];
  uint32_t stm32_tick;
  float wx;
  float wy;
  float wz;
  float vbx;
  float vby;
  float vbz;
  uint32_t checksum;
} flow_state_msg_t;
```

Send at 100-200 Hz. Units:

- `w*`: rad/s, body frame;
- `vb*`: m/s, body frame;
- `stm32_tick`: FreeRTOS/stabilizer tick.

The GAP8 should echo the most recent `stm32_tick` in every flow-sector frame so the STM32 can
log latency and eventually compensate it.

## Stage 5: GAP8 Flow Frontend

Repository: `tinympc-nanocockpit`

Implement this in obstacle mode on the AI-deck. Do not try to run the gate CNN and the
obstacle flow frontend at full rate until the flow path works by itself.

Required pieces:

1. Capture grayscale frames at a fixed resolution, initially `160x160` or similar.
2. Detect grid-bucketed Shi-Tomasi features.
3. Track with pyramidal LK between adjacent frames.
4. Run forward-backward rejection.
5. Convert pixel coordinates to normalized camera coordinates using calibrated intrinsics.
6. Subtract rotational flow using the latest STM32 gyro.
7. Estimate inverse depth per feature using body-frame velocity.
8. Reject bad estimates:
   - low translational observability;
   - negative or out-of-range inverse depth;
   - excessive forward-backward error;
   - sector depth outliers.
9. Aggregate to 5 or 9 sectors.
10. Send `flow_sectors_payload_t` over UART.

For bring-up, also add an optional reduced per-feature debug message, but keep it off by
default because it is much larger than the sector message.

## Stage 6: Bring-Up Order

### 1. STM32 log-only receiver

Flash STM32 with `flowObstacleLinkInit()` enabled, but leave:

```cpp
stgs.en_cstr_states = 0;
```

Verify in logs:

- flow-sector messages arrive;
- CRC errors stay low;
- age stays bounded;
- best depth roughly matches front multiranger or mocap;
- selected sector does not flicker.

### 2. Constraint synthesis log-only

Still keep ADMM state constraints disabled. Log the half-space that would be used:

- `active`
- `n`
- `b`
- selected depth/confidence
- sample age
- current `n^T p - b`

Expected behavior:

- no active constraint when the scene is empty;
- active constraint points away from the obstacle;
- `n^T p - b <= 0` when safely outside;
- value approaches zero as the drone approaches the keep-out plane.

### 3. Host replay parity

Take logged STM32 sector frames and state estimates, replay them through the Python/sim
half-space code, and verify the generated `n,b,en` match firmware logs within tolerance.

### 4. Closed-loop, low speed

Enable:

```cpp
stgs.en_cstr_states = 1;
```

Start with:

- low commanded speed;
- large margin;
- `OBS_K_START = 3` or higher;
- multiranger/emergency stop still armed;
- one wall, then one box, then clutter.

## Initial Parameters

These should be firmware PARAMs, not compile-time constants, once the path is working:

| Parameter | Initial value | Meaning |
|---|---:|---|
| `obs.enable` | `0` | master enable for applying constraints |
| `obs.logOnly` | `1` | compute/log but do not enable ADMM projection |
| `obs.minConf` | `0.35` | minimum sector confidence |
| `obs.minDepth` | `0.25 m` | reject too-close/noisy estimates |
| `obs.maxDepth` | `4.0 m` | reject far estimates for v1 |
| `obs.maxAgeMs` | `150 ms` | stale sample cutoff |
| `obs.holdMs` | `120 ms` | hold last good sector |
| `obs.switchRatio` | `0.75` | sector switch hysteresis |
| `obs.marginMin` | `0.30 m` | base keep-out margin |
| `obs.marginSlack` | `0.20 m` | added margin at low confidence |
| `obs.kStart` | `3` | first constrained horizon index |
| `obs.maxIterActive` | `2` | start with current MPC iteration count |

Only increase `maxIterActive` after measuring STM32 timing. The current controller is already
budget-sensitive.

## Files To Modify

### STM32 repo

Required:

- `TinyMPC-ADMM/src/tinympc/types.h`: add half-space arrays if not already present.
- `TinyMPC-ADMM/src/tinympc/constants.h`: add `MAX_HS` only if using sim-compatible layout.
- `TinyMPC-ADMM/src/tinympc/admm.cpp`: add position half-space projection.
- `TinyMPC-ADMM/src/tinympc/constraint_linear.h`: declare half-space setters.
- `TinyMPC-ADMM/src/tinympc/constraint_linear.cpp`: implement half-space setters.
- `src/flowdeck_obstacle_link.h`: new sector RX API.
- `src/flowdeck_obstacle_link.c`: new UART receiver, CRC, seqlock, logs.
- `src/controller_tinympc.cpp`: initialize receiver, synthesize constraints, enable/log ADMM
  constraints.
- `src/Kbuild`: include the new receiver/source files.

Likely:

- `src/aideck_protocol.h/.c`: add or reuse a STM32-to-GAP8 flow-state message.
- a new C params/log file, for example `src/obstacle_params.c`, because Crazyflie PARAM/LOG
  macros are easier from C than the C++ controller translation unit.

### AI-deck repo

Required in `tinympc-nanocockpit`:

- add obstacle-mode camera pipeline;
- add LK frontend;
- add STM32 back-channel receiver for gyro/velocity;
- add sector aggregation;
- add flow-sector UART transmitter matching `flow_sectors_payload_t`;
- add optional debug per-feature streamer for bring-up.

## Definition Of Done

Do not consider the firmware path ready for untethered obstacle avoidance until all of these
are true:

- GAP8 sector depth tracks independent range/mocap truth in log-only flight.
- STM32 generated half-spaces match host replay for the same logs.
- Constraint activation does not cause large terminal target jumps.
- With `obs.logOnly=0`, slow wall approaches bend the path before the multiranger failsafe
  would need to intervene.
- The one-half-space firmware behavior matches or improves the one-half-space sim behavior.

The multi-half-space path should remain a simulation experiment until its plane-selection and
projection policy outperform the single-plane path.
