/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * controller_tinympc.c - App layer application of TinyMPC.
 */

/** 
 * Single lap
 */

#include "Eigen.h"
using namespace Eigen;

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM
#include "estimator.h"         // estimatorEnqueuePosition (vision landmark fusion)

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (3 * configMINIMAL_STACK_SIZE)

// Rodriguez parameters conversion function (needed for old firmware compatibility)
static inline struct vec quat2rp(struct quat q) {
  struct vec v;
  float w_abs = fabsf(q.w);
  if (w_abs > 1e-6f) { // Avoid division by near-zero
    v.x = q.x / q.w;
    v.y = q.y / q.w;
    v.z = q.z / q.w;
  } else {
    // Handle singular case
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
  }
  return v;
}

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"
#include "gate8_link.h"   // GAP8 vision link, UART1/USART3 corner RX
#include "gate_tinympc_core.h"  // gate fallback state machine + reference selection
#include "gate_pnp.h"           // 2-DOF corner -> metric gate projection

// gateEnable is defined further down (inside the extern "C" block, C linkage).
// Forward-declare it so appMain can gate the UART receiver on it.
extern "C" uint8_t gateEnable;
extern "C" uint8_t fuseEnable;

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  // Defer the AI-deck UART receiver until the gate path is enabled. uart1Init()
  // reconfigures USART3 (the deck UART) and conflicts with the AI-deck/CPX deck
  // driver; doing it unconditionally at boot wedges the system before the radio
  // task comes up (no CRTP link). Mirror the existing rule of guarding deck I/O
  // behind an enable param: start the link once, lazily, when visMpc.enable=1.
  bool gate8_link_started = false;

  while(1) {
    if (!gate8_link_started && (gateEnable || fuseEnable)) {
      gate8LinkInit();    // start the AI-deck UART corner receiver (servo or fusion)
      gate8_link_started = true;
    }
    vTaskDelay(M2T(2000));
  }
}

// Macro variables - define locally to avoid dependency issues
#define DT 0.002f       // dt
#define NHORIZON 25     // horizon steps (must match constants.h if used)
#define MPC_RATE RATE_100_HZ  // control frequency
#define LQR_RATE RATE_500_HZ  // control frequency

/* Include trajectory to track */
#include "traj_fig8_12.h"
#include "traj_circuit.h"   // two-gate racetrack (world pos/vel + heading) for the chart controller
// #include "traj_circle_500hz.h"  // Large circle (1m radius)
// #include "traj_circle_small.h"  // Small circle (0.5m radius)
// #include "traj_perching.h"
//#include "traj_straight_line.h"  // Straight line (0,0,0.5) to (1,0,0.5)

// Precomputed data and cache, in params_*.h
static MatrixNf A;
static MatrixNMf B;
static MatrixMNf Kinf;
static MatrixMNf Klqr;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static MatrixNf Q;
static MatrixMf R;

/* Allocate global variables for MPC */

static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON-1]; 
static VectorMf Ulqr;
static VectorMf d[NHORIZON-1];
static VectorNf p[NHORIZON];
static VectorMf YU[NHORIZON];

static VectorNf q[NHORIZON-1];
static VectorMf r[NHORIZON-1];
static VectorMf r_tilde[NHORIZON-1];

static VectorNf Xref[NHORIZON];
static VectorMf Uref[NHORIZON-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;

static VectorMf Qu;
static VectorMf ZU[NHORIZON-1]; 
static VectorMf ZU_new[NHORIZON-1];

static VectorNf x0;
static VectorNf xg;
static VectorMf ug;

// Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

// Helper variables
static uint64_t startTimestamp;
// static bool isInit = false;  // fix for tracking problem - UNUSED, commented out
// static uint32_t mpcTime = 0;  // UNUSED (was for logging), commented out
// cf21bl Brushless: per-motor hover thrust fraction = m*g/(4*THRUST_MAX)
//                  = 0.0393*9.81/(4*0.1625) = 0.5931 (normalized, 1.0 = THRUST_MAX).
static float u_hover[4] = {0.5931f, 0.5931f, 0.5931f, 0.5931f};  // cf21bl
// static float u_hover[4] = {0.7f, 0.663f, 0.7373f, 0.633f};    // cf1 (brushed)
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;  // Default to commander/setpoint control on main
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
//static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 1;       // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

// --- Gate-vision MPC (visMpc PARAM group, exposed from gate_vis_params.c) ---
// Non-static so the C params/log file can extern them (C linkage via extern "C").
uint8_t gateEnable = 0;            // PARAM: 0 = off (default, bench-safe)
float   gate_target_speed = 0.45f; // PARAM: forward approach speed [m/s]
uint8_t gate_engaged = 0;          // LOG: 1 when the gate path drove the last solve
uint8_t gate_source = 0;           // LOG: GateControlSource of the last gate step
// Terminal-commit params: once close/centered and the gate overflows the frame,
// fly through on the last good estimate instead of stalling (PARAM group visMpc).
float   gate_tc_speed = 0.45f;      // forward speed during commit [m/s]
float   gate_tc_distance = 0.9f;    // max range to allow commit [m]
float   gate_tc_center_tol = 0.15f; // max lateral/vertical error to commit [m]
float   gate_tc_min_margin = 0.10f; // min aperture margin to commit [m]
float   gate_tc_max_age_s = 1.0f;   // max age of last estimate to commit [s]
// Live flight-tuning knobs (PARAM group visMpc), no reflash needed:
float   ctrlGainScale = 1.0f;       // PARAM: scale on the MPC input correction (1=full, <1 softer/safer)
float   ctrlUHover    = 0.5931f;    // PARAM: per-motor hover thrust fraction trim (1.0 = THRUST_MAX)
float   ctrlGateLookahead = 0.5f;   // PARAM: gate-approach lookahead [m]; bigger = stronger forward/lateral pull
// --- Vision landmark fusion (visMpc PARAM group): use the gate as a KNOWN
// landmark to correct Flow-deck drift via the EKF. The trajectory/MPC are
// untouched; only the estimator gets the correction. ---
uint8_t fuseEnable = 0;             // PARAM: run the fusion read + compute the live drift (LOG)
uint8_t fuseInject = 0;            // PARAM: actually estimatorEnqueuePosition() the correction
float   fuseStd    = 0.10f;         // PARAM: injected position measurement std-dev [m]
float   fuseMaxInnov = 0.75f;       // PARAM: reject corrections larger than this [m] (outlier gate)
float   fuseGateWX = 0.0f;          // PARAM: surveyed gate-A center world pose (takeoff-origin frame)
float   fuseGateWY = 0.0f;
float   fuseGateWZ = 0.0f;
// Two-gate circuit: gate B world pose + heading-association window. When circuitEnable,
// fusion picks which gate a detection belongs to by the chart heading (frameYaw, which is
// drift-immune): heading ~+x => gate A, ~-x (|h|~pi) => gate B, mid-turn => NO gate framed
// (reject, avoids associating a phantom/partial view). When !circuitEnable, single-gate
// (gate A) as before. This is the phantom-gate safeguard: wrong-heading or far-from-expected
// detections never reach the EKF.
float   fuseGateBX = 0.0f;          // PARAM: surveyed gate-B center world pose
float   fuseGateBY = 0.0f;
float   fuseGateBZ = 0.0f;
float   fuseYawWin = 0.60f;         // PARAM: heading half-window [rad] for gate A/B association (~34 deg)
float   fuseYawRateMax = 0.10f;     // PARAM: only inject fusion when scheduled |yaw rate| < this [rad/s] (settled on a straight)
float   g_fuse_dx = 0.0f, g_fuse_dy = 0.0f, g_fuse_dz = 0.0f;  // LOG: live drift = gate_world - g_gate_center
uint32_t g_fuse_n = 0;              // LOG: number of injected fixes
uint8_t  g_fuse_gate = 0;           // LOG: which gate the last detection associated to (0=none/rejected,1=A,2=B)

// --- Yaw-compensated "chart" circuit mode (visMpc.circuit). The controller solves
// in a frame rotated by -frameYaw (the desired heading), so the fixed-gain linear
// MPC always sees a ~yaw-0 problem while the drone physically yaws around the track.
// Ported from tinympc-vision render_two_gate_circle_demo (_controller_state). ---
uint8_t circuitEnable = 0;          // PARAM: 1 = fly traj_circuit.h via the chart
float frameYaw = 0.0f;              // LOG: current chart heading [rad] (CIRC_YAW[idx])
uint32_t circ_idx = 0;             // LOG: index into CIRC_* arrays (lap progress)
// CIRC_* is sampled at dt=0.02 (50 Hz) but the MPC runs at MPC_RATE=100 Hz, so
// circ_idx must advance only once per `circStride` ticks to play the path at its
// designed speed (stride 2 = 50 Hz = 1x). Larger stride => slower lap (more yaw
// margin); e.g. stride 4 ~= 0.5x ~0.2 m/s. PARAM: visMpc.circStr.
uint8_t circStride = 2;
static GateControllerConfig gate_cfg;
static GateVisionPacket     gate_vis;
static DroneState           gate_state;
static uint8_t              gate_have_fix = 0; // set once a valid gate is seen
// Latched gate world pose: captured on first engage (estimate is cleanest when
// far/stable), flown to as a FIXED target so close-range estimate jitter (the
// gate filling the frame) can't throw the approach off. Cleared when disengaged.
static bool  gate_latch_valid = false;
static float gate_lx = 0.0f, gate_ly = 0.0f, gate_lz = 0.0f;

// Basic mode - no obstacle avoidance constraints

void updateInitialState(const sensorData_t *sensors, const state_t *state) {
  // Angular rate (body frame; chart-invariant)
  x0(9)  = radians(sensors->gyro.x);
  x0(10) = radians(sensors->gyro.y);
  x0(11) = radians(sensors->gyro.z);
  attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current world attitude
  if (circuitEnable) {
    // Chart: rotate the WORLD state by -frameYaw so the controller sees a ~yaw-0
    // problem. pos/vel xy rotated; attitude = R_z(-frameYaw) * q (yaw removed).
    const float c = cosf(-frameYaw), s = sinf(-frameYaw);
    x0(0) = c * state->position.x - s * state->position.y;
    x0(1) = s * state->position.x + c * state->position.y;
    x0(2) = state->position.z;
    x0(6) = c * state->velocity.x - s * state->velocity.y;
    x0(7) = s * state->velocity.x + c * state->velocity.y;
    x0(8) = state->velocity.z;
    struct quat qframe = rpy2quat(mkvec(0.0f, 0.0f, -frameYaw));
    phi = quat2rp(qnormalize(qqmul(qframe, attitude)));  // chart (yaw-removed) attitude
  } else {
    x0(0) = state->position.x;
    x0(1) = state->position.y;
    x0(2) = state->position.z;
    x0(6) = state->velocity.x;
    x0(7) = state->velocity.y;
    x0(8) = state->velocity.z;
    phi = quat2rp(qnormalize(attitude));  // world attitude -> Rodrigues
  }
  x0(3) = phi.x;
  x0(4) = phi.y;
  x0(5) = phi.z;
}

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < NSTATES; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        if (i < NHORIZON - 1) {
          for (int j = 0; j < NINPUTS; ++j) {
            Uref[i](j) = U_ref_data[traj_idx][j];
          }          
        }
      }
    }
  }
  else {
    xg(0)  = setpoint->position.x;
    xg(1)  = setpoint->position.y;
    xg(2)  = setpoint->position.z;
    xg(6)  = setpoint->velocity.x;
    xg(7)  = setpoint->velocity.y;
    xg(8)  = setpoint->velocity.z;
    xg(9)  = radians(setpoint->attitudeRate.roll);
    xg(10) = radians(setpoint->attitudeRate.pitch);
    xg(11) = radians(setpoint->attitudeRate.yaw);
    desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                        radians(setpoint->attitude.pitch), 
                        radians(setpoint->attitude.yaw));
    attitude = rpy2quat(desired_rpy);
    phi = quat2rp(qnormalize(attitude));  
    xg(3) = phi.x;
    xg(4) = phi.y;
    xg(5) = phi.z;
    tiny_SetGoalState(&work, Xref, &xg);
    tiny_SetGoalInput(&work, Uref, &ug);
    // // xg(1) = 1.0;
    // // xg(2) = 2.0;
  }
  // DEBUG_PRINT("z_ref = %.2f\n", (double)(Xref[0](2)));

  // Trajectory progression
  if (en_traj) {
    if (traj_idx >= traj_length - 1 - NHORIZON + 1) { 
      // Reached end of trajectory - hold at final position
      // Don't reset step, just stay at the end
    } 
    else {
      step += 1;
    }
  }
}

// Reference time step for the gate horizon; matches the params_100hz.h model
// discretization (A is discretized at dt=0.02 s), as validated in tinympc-vision.
#define GATE_REF_DT 0.02f
// Wall-clock control period (MPC_RATE = 100 Hz), used for the core's time
// accounting (distinct from the prediction horizon step above).
#define GATE_CTRL_DT 0.01f

// Gate solver callback: build a gate-centered approach reference into Xref and
// run the existing firmware ADMM. Registered with the core via
// gate_tinympc_set_solver(); invoked from inside gate_tinympc_step().
// NOTE: hard gate (half-space) state constraints are intentionally NOT used here
// — the shipped gain cache only regularizes inputs, so enabling en_cstr_states
// diverges the solve (see gate-solve fix in tinympc-vision). The gate enters via
// the reference only.
static bool firmwareGateSolve(const DroneState *st, const GateTinyMpcReference *ref,
                              const GateControllerConfig *cfg, MotorCommand *cmd,
                              GateControllerDebug *dbg) {
  (void)dbg;
  const float speed = ref->target_speed_mps;
  // Latch the gate world pose on first engage (cleanest estimate, far/stable).
  if (!gate_latch_valid) {
    gate_lx = cfg->gate_x; gate_ly = ref->gate_pose_m[1]; gate_lz = ref->gate_pose_m[2];
    gate_latch_valid = true;
  }
  // Straight-line approach to a through-point 0.5 m past the LATCHED gate, at
  // `speed`. A minimum lookahead (independent of speed) gives the short horizon
  // decisive authority to BOTH advance and center (x, y, z) on the fixed target,
  // instead of chasing the jittery close-range live estimate.
  const float tx = gate_lx + 0.5f, ty = gate_ly, tz = gate_lz;
  float dx = tx - st->x, dy = ty - st->y, dz = tz - st->z;
  float dist = sqrtf(dx*dx + dy*dy + dz*dz);
  if (dist < 1e-3f) dist = 1e-3f;
  const float ux = dx/dist, uy = dy/dist, uz = dz/dist;
  for (int i = 0; i < NHORIZON; ++i) {
    float la = speed * GATE_REF_DT * (float)i;   // distance along the line at step i
    if (la < ctrlGateLookahead) la = ctrlGateLookahead;  // min lookahead -> decisive pull (live param)
    if (la > dist) la = dist;
    for (int j = 0; j < NSTATES; ++j) Xref[i](j) = 0.0f;
    Xref[i](0) = st->x + ux*la;
    Xref[i](1) = st->y + uy*la;
    Xref[i](2) = st->z + uz*la;
    Xref[i](6) = ux*speed;
    Xref[i](7) = uy*speed;
    Xref[i](8) = uz*speed;
    if (i < NHORIZON - 1) {
      for (int j = 0; j < NINPUTS; ++j) Uref[i](j) = 0.0f;
    }
  }
  // x0 is already populated by updateInitialState() and referenced by work.
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  tiny_UpdateLinearCost(&work);
  tiny_SolveAdmm(&work);
  cmd->motor_delta[0] = ZU_new[0](0);
  cmd->motor_delta[1] = ZU_new[0](1);
  cmd->motor_delta[2] = ZU_new[0](2);
  cmd->motor_delta[3] = ZU_new[0](3);
  cmd->solver_iterations = info.iter;
  cmd->solver_success = info.status_val >= 0;
  return true;
}

// Drive one gate-vision MPC step: read corners, project to a metric gate, run the
// core (which invokes firmwareGateSolve). Returns true iff a valid gate engaged
// the solve (so the caller skips the normal reference + solve). On no/stale/invalid
// detection, returns false and the controller falls back to the commander path.
static bool runGateMpc(const sensorData_t *sensors, const state_t *state) {
  gate_engaged = 0;
  gate_state.x = state->position.x;
  gate_state.y = state->position.y;
  gate_state.z = state->position.z;
  gate_state.vx = state->velocity.x;
  gate_state.vy = state->velocity.y;
  gate_state.vz = state->velocity.z;
  gate_state.qx = state->attitudeQuaternion.x;
  gate_state.qy = state->attitudeQuaternion.y;
  gate_state.qz = state->attitudeQuaternion.z;
  gate_state.qw = state->attitudeQuaternion.w;
  gate_state.wx = radians(sensors->gyro.x);
  gate_state.wy = radians(sensors->gyro.y);
  gate_state.wz = radians(sensors->gyro.z);

  float corners[GATE8_N_CORNERS];
  uint32_t age_ms = 0;
  // No corner data at all -> let the commander hold (don't engage gate path).
  if (!gate8LinkGetLatest(corners, &age_ms)) return false;
  // Project; gate_vis is a well-formed invalid packet on failure. We still run
  // the core on invalid packets so terminal-commit can fly through, but only
  // once we have ever had a valid fix (else the gate center is uninitialized).
  if (gate_pnp_project(corners, age_ms, &gate_state, &gate_vis)) {
    gate_have_fix = 1;
  }
  if (!gate_have_fix) return false;

  memset(&gate_cfg, 0, sizeof(gate_cfg));
  gate_cfg.mode = GATE_CONTROL_TERMINAL_COMMIT;
  gate_cfg.gate_x = g_gate_center_x;
  gate_cfg.gate_y = g_gate_center_y;
  gate_cfg.gate_z = g_gate_center_z;
  gate_cfg.safe_half_width = 0.5f * g_gate_width_m;
  gate_cfg.safe_half_height = 0.5f * g_gate_height_m;
  gate_cfg.target_speed = gate_target_speed;
  gate_cfg.invalid_packet_speed = 0.0f;  // stall if gate lost without a valid commit
  gate_cfg.terminal_commit_speed = gate_tc_speed;
  gate_cfg.terminal_commit_max_age_s = gate_tc_max_age_s;
  gate_cfg.terminal_commit_distance_m = gate_tc_distance;
  gate_cfg.terminal_commit_min_margin_m = gate_tc_min_margin;
  gate_cfg.terminal_commit_center_tolerance_m = gate_tc_center_tol;
  gate_cfg.terminal_commit_bound_shrink_m = 0.0f;

  MotorCommand mc = gate_tinympc_step(&gate_state, &gate_vis, &gate_cfg, GATE_CTRL_DT);
  gate_source = (uint8_t)mc.control_source;
  result = info.status_val * info.iter;
  gate_engaged = 1;
  return true;
}

// Vision landmark fusion: when the gate (a KNOWN surveyed landmark) is seen,
// gate_pnp computes its world position FROM the drifted estimate. The mismatch
// (surveyed - computed) is exactly the estimate's drift, so corrected drone pos
// = estimate + drift; we feed that to the EKF. Control/trajectory are untouched.
// Throttled to ~10 Hz; only injects when fuseInject and the innovation is sane.
static void runGateFusion(const sensorData_t *sensors, const state_t *state) {
  gate_state.x = state->position.x;  gate_state.y = state->position.y;  gate_state.z = state->position.z;
  gate_state.vx = state->velocity.x; gate_state.vy = state->velocity.y; gate_state.vz = state->velocity.z;
  gate_state.qx = state->attitudeQuaternion.x; gate_state.qy = state->attitudeQuaternion.y;
  gate_state.qz = state->attitudeQuaternion.z; gate_state.qw = state->attitudeQuaternion.w;
  gate_state.wx = radians(sensors->gyro.x); gate_state.wy = radians(sensors->gyro.y); gate_state.wz = radians(sensors->gyro.z);

  float corners[GATE8_N_CORNERS]; uint32_t age_ms = 0;
  if (!gate8LinkGetLatest(corners, &age_ms)) return;
  if (!gate_pnp_project(corners, age_ms, &gate_state, &gate_vis)) return;  // need a clean valid fix

  // --- Data association: which surveyed gate does this detection belong to? ---
  // Use the chart heading (frameYaw) when flying the circuit: it is the scheduled
  // heading, so it is immune to the very estimator drift we are trying to correct.
  float gx_known = fuseGateWX, gy_known = fuseGateWY, gz_known = fuseGateWZ;
  if (circuitEnable) {
    float h = frameYaw;                                   // wrap to [-pi, pi]
    while (h >  M_PI_F) h -= 2.0f * M_PI_F;
    while (h < -M_PI_F) h += 2.0f * M_PI_F;
    const float ah = fabsf(h);
    if (ah < fuseYawWin) {                                // heading ~ +x  => gate A ahead
      g_fuse_gate = 1; gx_known = fuseGateWX; gy_known = fuseGateWY; gz_known = fuseGateWZ;
    } else if (ah > M_PI_F - fuseYawWin) {                // heading ~ -x  => gate B ahead
      g_fuse_gate = 2; gx_known = fuseGateBX; gy_known = fuseGateBY; gz_known = fuseGateBZ;
    } else {                                              // mid-turn: no gate framed -> reject
      g_fuse_gate = 0; g_fuse_dx = g_fuse_dy = g_fuse_dz = 0.0f; return;
    }
  } else {
    g_fuse_gate = 1;                                      // single-gate (legacy) path
  }

  // (surveyed gate world) - (gate world computed from the drifted estimate) = drift.
  const float dx = gx_known - g_gate_center_x;
  const float dy = gy_known - g_gate_center_y;
  const float dz = gz_known - g_gate_center_z;
  g_fuse_dx = dx; g_fuse_dy = dy; g_fuse_dz = dz;   // logged even when not injecting (bring-up)

  if (!fuseInject) return;
  // Only inject when SETTLED ON A STRAIGHT (scheduled yaw rate ~0). During a turn the chart
  // rotates every position error by the heading (~150 deg), so even a small correction gets
  // amplified into a lunge. The heading window alone lets a fix in at ~145 deg (still turning);
  // this also requires the yaw to have stopped. On the straights CIRC_YAW is constant -> rate 0.
  if (circuitEnable) {
    const uint32_t kk = (circ_idx + 1 < CIRC_N) ? (circ_idx + 1) : circ_idx;
    const float sched_yawrate = fabsf(CIRC_YAW[kk] - CIRC_YAW[circ_idx]) / 0.02f;
    if (sched_yawrate > fuseYawRateMax) return;   // mid-turn / not yet settled -> don't inject
  }
  // Innovation gate doubles as the association sanity check: a real detection of the
  // expected gate sits within fuseMaxInnov of its surveyed pose; a phantom or a
  // mis-associated/other gate lands far away and is rejected here.
  if (fabsf(dx) > fuseMaxInnov || fabsf(dy) > fuseMaxInnov || fabsf(dz) > fuseMaxInnov) return;  // outlier gate
  positionMeasurement_t pos;
  pos.x = state->position.x + dx;
  pos.y = state->position.y + dy;
  pos.z = state->position.z + dz;
  pos.stdDev = fuseStd;
  pos.source = MeasurementSourceLocationService;
  estimatorEnqueuePosition(&pos);
  g_fuse_n++;
}

// Half-space constraint function removed for basic functionality test

void controllerOutOfTreeInit(void) { 
  /* Start MPC initialization*/

  // Precompute/Cache
  // #include "params_500hz.h"
  // #include "params_100hz.h"        // brushed cf2 gains (original)
  #include "params_brushless.h"       // cf21bl Brushless gains (regenerated, rho=250)
  // #include "params_constrained.h"

  // End of Precompute/Cache

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0;  // Original stable rho
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct (pass 0 for state constraints - not used)
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, 0, 0);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, 0, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  // tiny_SetGoalState(&work, Xref, &xg);
  // tiny_SetGoalInput(&work, Uref, &ug);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);
  // R = R + stgs.rho_init * MatrixMf::Identity();
  // /* Set up constraints */
  ucu << 1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;  // No state constraints for basic test
  stgs.max_iter = 2;        // Original working value
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  Klqr << 
  -0.123589f,0.123635f,0.285625f,-0.394876f,-0.419547f,-0.474536f,-0.073759f,0.072612f,0.186504f,-0.031569f,-0.038547f,-0.187738f,
  0.120236f,0.119379f,0.285625f,-0.346222f,0.403763f,0.475821f,0.071330f,0.068348f,0.186504f,-0.020972f,0.037152f,0.187009f,
  0.121600f,-0.122839f,0.285625f,0.362241f,0.337953f,-0.478858f,0.069310f,-0.070833f,0.186504f,0.022379f,0.015573f,-0.185212f,
  -0.118248f,-0.120176f,0.285625f,0.378857f,-0.322169f,0.477573f,-0.066881f,-0.070128f,0.186504f,0.030162f,-0.014177f,0.185941f;

  /* Register the gate-vision MPC solver bridge and reset its state machine. */
  gate_tinympc_reset();
  gate_tinympc_set_solver(firmwareGateSolve);
  gate_have_fix = 0;

  /* End of MPC initialization */
  step = 0;
  traj_iter = 0;
  
  if (en_traj) {
    DEBUG_PRINT("Stored trajectory enabled\n");
  } else {
    DEBUG_PRINT("Commander/setpoint mode enabled\n");
  }
}

// Build the horizon reference for the racetrack IN THE CHART (rotated by -frameYaw):
// world pos/vel rotated into the heading-aligned frame, chart-yaw = (heading-frameYaw).
// Advances circ_idx one trajectory point per MPC step; holds at the end of the lap.
static void buildCircuitReference(void) {
  const float c = cosf(-frameYaw), s = sinf(-frameYaw);
  for (int i = 0; i < NHORIZON; ++i) {
    uint32_t k = circ_idx + (uint32_t)i;
    if (k >= CIRC_N) k = CIRC_N - 1;                 // hold at the end of the lap
    const float wx = CIRC_POS[k][0], wy = CIRC_POS[k][1], wz = CIRC_POS[k][2];
    const float vx = CIRC_VEL[k][0], vy = CIRC_VEL[k][1], vz = CIRC_VEL[k][2];
    for (int j = 0; j < NSTATES; ++j) Xref[i](j) = 0.0f;
    Xref[i](0) = c * wx - s * wy;                    // chart position
    Xref[i](1) = s * wx + c * wy;
    Xref[i](2) = wz;
    Xref[i](5) = 0.5f * (CIRC_YAW[k] - frameYaw);    // chart yaw (half-angle); tilt ff = 0
    Xref[i](6) = c * vx - s * vy;                    // chart velocity
    Xref[i](7) = s * vx + c * vy;
    Xref[i](8) = vz;
    // Yaw-RATE feedforward (omega_z): command the schedule's yaw rate so the drone turns
    // AT the right rate, not just chasing yaw-position error -> kills the ramp lag (which
    // grew chart_yaw to ~44 deg through the turn). Re-added now that the engage-yaw crash
    // is fixed by the short hover. Body yaw rate is frame-invariant; dt = 0.02 s.
    const uint32_t k1 = (k + 1 < CIRC_N) ? (k + 1) : k;
    Xref[i](11) = (CIRC_YAW[k1] - CIRC_YAW[k]) / 0.02f;
    if (i < NHORIZON - 1) {
      for (int j = 0; j < NINPUTS; ++j) Uref[i](j) = 0.0f;
    }
  }
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  // Advance the base index at 50 Hz / circStride (MPC ticks at 100 Hz), so the
  // 0.02 s-sampled path plays at its designed wall-clock speed instead of 2x.
  static uint8_t stride_count = 0;
  if (++stride_count >= (circStride ? circStride : 1)) {
    stride_count = 0;
    if (circ_idx + 1 + NHORIZON < CIRC_N) circ_idx++;  // advance, hold near the end
  }
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Get current time
  startTimestamp = usecTimestamp();

  /* Get current state (initial state for MPC) */
  // delta_x = x - x_bar; x_bar = 0
  // Positon error, [m]
  // Set the chart heading for THIS step before building x0 (circuit mode).
  if (circuitEnable) {
    frameYaw = CIRC_YAW[circ_idx < CIRC_N ? circ_idx : (CIRC_N - 1)];
  } else {
    frameYaw = 0.0f; circ_idx = 0;   // re-start the lap fresh on next enable
  }
  updateInitialState(sensors, state);

  /* Vision landmark fusion (~10 Hz): correct estimator drift from the known gate.
   * Runs independently of the control path (trajectory/commander/servo). */
  if (fuseEnable && RATE_DO_EXECUTE(10, tick)) {
    runGateFusion(sensors, state);
  }

  /* Controller rate */
  if (RATE_DO_EXECUTE(MPC_RATE, tick)) {
    // Gate-vision MPC: when enabled and a valid gate is seen, runGateMpc()
    // builds the gate reference and runs the solve itself. Otherwise fall back
    // to the normal commander/trajectory reference + solve.
    bool gate_ran = false;
    if (circuitEnable) {
      // Yaw-compensated racetrack: build the chart reference and solve.
      buildCircuitReference();
      tiny_UpdateLinearCost(&work);
      tiny_SolveAdmm(&work);
      result = info.status_val * info.iter;
    } else {
      if (gateEnable) {
        gate_ran = runGateMpc(sensors, state);
      } else {
        gate_latch_valid = false;   // clear latch when off -> re-latch fresh on next engage
      }
      if (!gate_ran) {
        // Get command reference
        updateHorizonReference(setpoint);

        /* MPC solve */
        // Solve optimization problem using ADMM
        tiny_UpdateLinearCost(&work);
        tiny_SolveAdmm(&work);

        result =  info.status_val * info.iter;
      }
    }
    
    // Detailed logging every 0.5 seconds
    static uint32_t mpc_log_counter = 0;
    if (mpc_log_counter % 50 == 0) {  // 100Hz / 50 = every 0.5s
      DEBUG_PRINT("MPC: pos=(%.2f,%.2f,%.2f) ref=(%.2f,%.2f,%.2f)\n", 
                  (double)x0(0), (double)x0(1), (double)x0(2),
                  (double)Xref[0](0), (double)Xref[0](1), (double)Xref[0](2));
      DEBUG_PRINT("MPC: u=(%.2f,%.2f,%.2f,%.2f) iter=%d\n",
                  (double)(Uhrz[0](0) + u_hover[0]), (double)(Uhrz[0](1) + u_hover[1]),
                  (double)(Uhrz[0](2) + u_hover[2]), (double)(Uhrz[0](3) + u_hover[3]),
                  info.iter);
    }
    mpc_log_counter++;
    
    // Position logging disabled
    // static uint32_t pos_log_counter = 0;
    // if (pos_log_counter % 50 == 0) {
    //   DEBUG_PRINT("POS: x=%.2f y=%.2f z=%.2f cstr=%d\n", 
    //               (double)x0(0), (double)x0(1), (double)x0(2), obs_constraint_active);
    // }
    // pos_log_counter++;
  }

  /* Output control. Convert the MPC per-motor thrust solution to SI thrust+torque
   * and let powerDistributionForceTorque map it to the brushless motors -- the
   * stock, hardware-verified brushless path (NOT raw PWM, which is brushed-cf2).
   * Per-motor force f_i = (ZU_i + u_hover_i) * THRUST_MAX [N]; u is normalized so
   * 1.0 = THRUST_MAX, matching the regenerated brushless gains (params_brushless.h).
   * The torque mixing is the exact inverse of powerDistributionForceTorque
   * (roll[-,-,+,+] pitch[-,+,+,-] yaw[-,+,-,+]); round-trips to f_i on the motors. */
  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torqueX = 0.0f; control->torqueY = 0.0f; control->torqueZ = 0.0f;
  } else {
    const float Tmax = 0.1625f;               // cf21bl THRUST_MAX (N per motor)
    const float arm  = 0.70710678f * 0.050f;  // 0.707 * ARM_LENGTH (matches power dist)
    const float ttq  = 0.004899994f;          // THRUST2TORQUE
    // ctrlGainScale softens/sharpens the correction; ctrlUHover trims hover. Both live PARAMs.
    const float f0 = (ctrlGainScale * ZU_new[0](0) + ctrlUHover) * Tmax;
    const float f1 = (ctrlGainScale * ZU_new[0](1) + ctrlUHover) * Tmax;
    const float f2 = (ctrlGainScale * ZU_new[0](2) + ctrlUHover) * Tmax;
    const float f3 = (ctrlGainScale * ZU_new[0](3) + ctrlUHover) * Tmax;
    control->thrustSi = f0 + f1 + f2 + f3;
    control->torqueX  = arm * (-f0 - f1 + f2 + f3);
    control->torqueY  = arm * (-f0 + f1 + f2 - f3);
    control->torqueZ  = ttq * (-f0 + f1 - f2 + f3);
  }
  control->controlMode = controlModeForceTorque;
  // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control->normalizedForces[0]), (double)(control->normalizedForces[1]));

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;
}

/**
 * Tunning variables for the full state quaternion LQR controller
 */
// PARAM_GROUP_START(ctrlMPC)
// /**
//  * @brief K gain
//  */
// PARAM_ADD(PARAM_FLOAT, u_hover, &u_hover)

// PARAM_GROUP_STOP(ctrlMPC)

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

// Note: LOG macros disabled due to C++ string literal compatibility with new firmware
/*
LOG_GROUP_START(ctrlMPC)

LOG_ADD(LOG_INT8, result, &result)
LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

LOG_ADD(LOG_FLOAT, u0, &(Uhrz[0](0)))
LOG_ADD(LOG_FLOAT, u1, &(Uhrz[0](1)))
LOG_ADD(LOG_FLOAT, u2, &(Uhrz[0](2)))
LOG_ADD(LOG_FLOAT, u3, &(Uhrz[0](3)))

LOG_ADD(LOG_FLOAT, zu0, &(ZU_new[0](0)))
LOG_ADD(LOG_FLOAT, zu1, &(ZU_new[0](1)))
LOG_ADD(LOG_FLOAT, zu2, &(ZU_new[0](2)))
LOG_ADD(LOG_FLOAT, zu3, &(ZU_new[0](3)))

LOG_GROUP_STOP(ctrlMPC)
*/

#ifdef __cplusplus
}
#endif
