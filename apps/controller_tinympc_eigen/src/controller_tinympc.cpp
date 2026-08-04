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
#include "semphr.h"
#include "static_mem.h"

#include "controller.h"
#include "controller_pid.h"   // stock PID cascade -- the cascade output feeds this
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM

#include "gate8_link.h"   // AI-deck gate-corner UART receiver
#include "perception_map_link.h"
#include "perception_danger.h"
#include "perception_corridor.h"
#include "perception_model_qparams.h"
#include "sequential_obstacle_link.h"
#include "sequential_obstacle_control.h"
#include "gate_pnp.h"     // corners -> world-frame gate center (Stage 1: perception only)

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (4 * configMINIMAL_STACK_SIZE)
#define TINYMPC_TASK_NAME             "TINYMPC"
#define TINYMPC_TASK_PRI              2

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

// need quaternion, not Rodrigues, parameters to compute the attitude reference error
static inline struct quat rp2quat(struct vec r) {
  // Inverse of quat2rp: given Rodrigues r = q_v/q_w, rebuild the unit quaternion.
  return qnormalize(quatvw(r, 1.0f));
}

// Wrap an angle to (-180, 180] degrees.
static inline float wrap_deg(float d) {
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// Macro variables - define locally to avoid dependency issues
#define DT 0.020f       // dt; matches the demo-2 50 Hz constrained cache
#define NHORIZON 25     // horizon steps (must match constants.h if used)
#define MPC_RATE RATE_50_HZ  // demo-2 constrained model/cache rate
#define LQR_RATE RATE_500_HZ  // control frequency

/* Include trajectory to track */
#include "traj_fig8_12.h"
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
static VectorNf YX[NHORIZON];
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
static VectorNf ZX[NHORIZON];
static VectorNf ZX_new[NHORIZON];

static VectorNf x0;
static VectorNf xg;
static VectorMf ug;

static MatrixNf Acx;
static VectorNf lcx;
static VectorNf ucx;

// Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

static bool isInit = false;
// static uint32_t mpcTime = 0;  // UNUSED (was for logging), commented out
static float u_hover[4] = {0.583f, 0.583f, 0.583f, 0.583f};  // demo-2 cf21bl normalized hover
// static float u_hover[4] = {0.7467, 0.667f, 0.78, 0.7f};  // cf2 not correct
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
static struct quat q_meas;   // measured attitude, stashed by updateInitialState
static struct quat q_ref0;   // reference-attitude frame for the multiplicative error

// --- Cascade output (ishaan/debug-traj approach): TinyMPC plans position; the stock
// Crazyflie PID does all low-level attitude/rate/motor control, INCLUDING yaw (which it
// handles robustly at any angle). The MPC itself never tracks yaw. mpc_setpoint_pid is
// the position + heading setpoint handed to controllerPid every tick; the heading is the
// trajectory's velocity direction (face-forward). ---
static setpoint_t mpc_setpoint_pid;
static float mpc_heading_yaw_deg = 0.0f;   // heading commanded to the PID [deg]
static bool  mpc_has_run = false;          // hold current pose until the first MPC solve
static uint32_t last_controller_tick = 0;
static uint32_t controller_activate_tick = 0;
static const bool enable_pid_face_forward_yaw = true;
static bool obs_pid_passthrough_seen = false;
static uint8_t last_obs_pid_passthrough = 0;

// Demo-2 watchdog fix: run TinyMPC in its own lower-priority task. The stabilizer loop
// only snapshots inputs, wakes this task at MPC_RATE, and feeds the latest MPC setpoint
// through the stock PID at 500 Hz.
static SemaphoreHandle_t runTaskSemaphore = NULL;
static SemaphoreHandle_t dataMutex = NULL;
static StaticSemaphore_t dataMutexBuffer;
static setpoint_t setpoint_data;
static sensorData_t sensors_data;
static state_t state_data;
static setpoint_t mpc_setpoint_data;
static bool mpc_reset_requested = false;
static void tinympcControllerTask(void *parameters);
STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);
// Let cfclient command yaw live (Parameters tab, group visYaw). useRef=1 overrides the
// heading with yawRefDeg; non-static so the C param file links against them.
uint8_t yawUseRef = 0;      // PARAM: 1 = command heading from yawRefDeg below
float   yawRefDeg = 0.0f;   // PARAM: commanded absolute heading [deg] when yawUseRef=1

// --- Gate vision (Stage 1: perception only, does NOT affect control) ---
// Poll the AI-deck corner link, project to a world-frame gate center, and let the
// visGate LOG group expose it. Params/logs live in gate_pnp_params.c.
static GateVisionPacket g_gate_vision;
static float g_gate_corners_pixels[8];

// --- demo-2/eigen-task modeled obstacle -> TinyMPC half-space constraints ---
// This bypasses the AI-deck ray pipeline for bring-up. The controller assumes a static
// modeled obstacle in world coordinates and writes one linearized position half-space per
// horizon knot, matching the demo-2 Eigen-task obstacle-avoidance structure.
uint8_t obsEnable = 0;       // PARAM: master enable for modeled cylinder constraints
uint8_t obsLogOnly = 1;      // PARAM: 1 = compute/log only, 0 = write constraints to ADMM
uint8_t obsPidPassthrough = 0;// PARAM: run perception task but pass commander setpoints to PID
float   obsCx = 0.5f;        // PARAM: obstacle center x [m]
float   obsCy = 0.15f;       // PARAM: obstacle center y [m]
float   obsCz = 0.5f;        // PARAM: cylinder center z [m]
float   obsRadius = 0.10f;   // PARAM: modeled obstacle radius [m]
float   obsHeight = 1.0f;    // PARAM: cylinder height [m], used for activation/logging
float   obsSafety = 0.00f;   // PARAM: extra clearance added to radius [m]
float   obsActMargin = 1000.0f;// PARAM: horizon activation margin [m]
float   obsSide = 1.0f;      // PARAM: +1/-1 lateral side preference for centered paths
uint32_t obsDelayMs = 0;     // PARAM: wait after controller activation before constraints
uint8_t obsKStart = 0;       // PARAM: first horizon knot allowed to receive constraints

uint8_t  g_obs_active = 0;
uint8_t  g_obs_applied = 0;
uint8_t  g_obs_count = 0;
uint8_t  g_obs_first_k = 0;
float    g_obs_a0 = 0.0f;
float    g_obs_a1 = 0.0f;
float    g_obs_a2 = 0.0f;
float    g_obs_b = 0.0f;
float    g_obs_margin = 0.0f;
float    g_obs_violation = 0.0f;
float    g_obs_clearance = 0.0f;
uint32_t g_mpc_solve_us = 0;
uint8_t  g_mpc_iter = 0;
int8_t   g_mpc_status = TINY_UNSOLVED;
float    g_mpc_primal_residual = 0.0f;
float    g_mpc_dual_residual = 0.0f;
uint8_t  g_mpc_health_hold = 0;
uint32_t g_mpc_health_faults = 0;
uint32_t g_mpc_solve_started = 0;
uint32_t g_mpc_solve_completed = 0;
uint32_t g_mpc_heartbeat_age_ms = 0;
uint8_t  g_mpc_stall_hold = 0;
uint32_t g_mpc_warm_resets = 0;
uint8_t  g_mpc_last_reset_reason = 0;
uint8_t  g_mpc_solve_route_phase = 0;
uint32_t g_mpc_stack_free_words = 0;
uint32_t g_mpc_scan_bypass_cycles = 0;
uint32_t g_mpc_pid_bypass_cycles = 0;
static TickType_t g_mpc_last_complete_tick = 0;

enum {
  MPC_RESET_ACTIVATION = 1,
};

static const uint32_t MPC_HEARTBEAT_TIMEOUT_MS = 250;

// NanoCockpit multi-task CNN maps. The quantized model is accepted for
// obstacle-avoidance use; packet validity, age checks, confidence margins,
// and soft constraint slack remain the runtime safety mechanisms.
uint8_t perceptEnable = 1;
uint8_t perceptConstraintEnable = 1;
uint8_t perceptLogOnly = 0;
float perceptHorizonS = NHORIZON * DT;
float perceptLatencyS = 0.08f;
float perceptMaxRangeM = 6.0f;
float perceptNominalSpeedMps = 1.0f;
float perceptDangerThreshold = PERCEPTION_MODEL_DANGER_THRESHOLD;
float perceptBaseMarginPx = 4.0f;
float perceptDroneRadiusM = 0.10f;
float perceptSafetyMarginM = 0.10f;
float perceptLookaheadS = 0.20f;
float perceptSlackPenalty = 1000.0f;
uint8_t perceptKStart = 1;
uint8_t perceptGateOpeningEnable = 1;
float perceptGateOpeningThreshold = 0.80f;
float perceptGateOpeningInsetPx = 8.0f;
float perceptGateOpeningSafeCap = 0.20f;
float perceptGateOpeningRangeGuardM = 0.15f;
float perceptGateOpeningMaxUncertainty = 0.50f;
uint8_t g_percept_valid = 0;
uint32_t g_percept_age_ms = 0;
uint32_t g_percept_sample = 0;
float g_percept_max_danger = 0.0f;
float g_percept_center_danger = 0.0f;
float g_percept_min_ttc = 0.0f;
uint8_t g_percept_corridor_valid = 0;
uint8_t g_percept_constraints = 0;
float g_percept_pixel_margin = 0.0f;
float g_percept_max_slack = 0.0f;
float g_percept_total_slack = 0.0f;
float g_percept_slack_cost = 0.0f;
uint32_t g_percept_failed_solves = 0;
uint32_t g_percept_near_infeasible_solves = 0;
float g_percept_left_n[3] = {0.0f, 0.0f, 0.0f};
float g_percept_right_n[3] = {0.0f, 0.0f, 0.0f};
uint8_t g_percept_gate_open_cells = 0;
static perception_danger_map_t g_perception_danger_map;
static uint32_t perception_sample_seen = 0;
static uint32_t gate_capture_tick = 0;

// Four fixed body-frame direction scores from the sequential Tiny Racer network.
// This path is opt-in and supersedes the dense-map corridor. It classifies the
// scores into open/blocked directions and uses them only to select waypoints.
uint8_t seqAvoidEnable = 0;
uint8_t seqAvoidConstraintEnable = 1;
uint8_t seqAvoidLogOnly = 0;
float seqAvoidConfidenceMin = 0.0f;
uint32_t seqAvoidMaxAgeMs = 250;
uint32_t seqAvoidDropoutGraceMs = 300;
float seqAvoidDroneRadiusM = 0.10f;
float seqAvoidTrackingMarginM = 0.08f;
float seqAvoidLatencyS = 0.08f;
float seqAvoidPerceptionMarginM = 0.03f;
float seqAvoidConfidenceGainM = 0.05f;
float seqAvoidDefaultOffsetM = 0.25f;
float seqAvoidMaxRangeM = 6.0f;
float seqAvoidDirectionSafeMinM = 0.30f;
uint8_t seqAvoidAverageWindow = 5;
float seqAvoidTriggerAverage = 3.5f;
float seqAvoidClearAverage = 1.0f;
float seqAvoidTriggerM = 1.20f;
float seqAvoidActivationEpsM = 0.25f;
float seqAvoidReferenceShiftM = 0.35f;
float seqAvoidReturnDelayS = 1.0f;
float seqAvoidReturnRateMps = 0.20f;
uint8_t seqAvoidReturnScanEnable = 1;
float seqAvoidReturnScanYawDeg = 50.0f;
float seqAvoidReturnScanYawRateDegS = 45.0f;
float seqAvoidReturnScanSettleS = 0.50f;
uint8_t seqAvoidReturnScanWindow = 5;
float seqAvoidReturnScanClearAverage = 1.0f;
float seqAvoidReturnScanRetryS = 3.0f;
float seqAvoidSideBiasM = 0.03f;
float seqAvoidScanGradientM = 0.06f;
float seqAvoidScanImprovementM = 0.04f;
float seqAvoidScanRetryMinS = 1.5f;
float seqAvoidScanRetryMaxS = 6.0f;
float seqAvoidCruiseSpeedMps = 0.40f;
float seqAvoidCruiseAccelMps2 = 0.40f;
float seqAvoidBrakingAccelMps2 = 0.60f;
float seqAvoidGoalToleranceM = 0.10f;
float seqAvoidSideMinimumM = 0.35f;
uint8_t seqAvoidSideVotesRequired = 3;
float seqAvoidForwardMinimumM = 0.35f;
float seqAvoidForwardStepM = 0.40f;
float seqAvoidPassDistanceM = 0.80f;
float seqAvoidWaypointToleranceM = 0.10f;
uint8_t seqAvoidClearVotesRequired = 3;
float seqAvoidMaxLateralM = 0.75f;
float seqAvoidProbeProgressM = 0.08f;
uint8_t seqAvoidProbeVotesRequired = 8;
/* The five-iteration flight solve routinely ends at TINY_MAX_ITER. These
 * limits reject grossly pathological output rather than treating an ordinary
 * truncated-ADMM residual as an immediate fault. */
float seqAvoidMaxPrimalResidual = 0.50f;
float seqAvoidMaxDualResidual = 300.0f;
uint32_t seqAvoidMaxSolveUs = 20000;
float seqAvoidSlackPenalty = 5000.0f;
float seqAvoidDistanceWeight = 0.45f;
float seqAvoidGoalWeight = 0.40f;
float seqAvoidHysteresisWeight = 0.15f;
float seqAvoidDynamicWeight = 0.10f;
uint8_t seqAvoidKStart = 1;

uint8_t g_seq_valid = 0;
uint8_t g_seq_stop = 0;
uint8_t g_seq_reliable_mask = 0;
int8_t g_seq_chosen = -1;
uint8_t g_seq_constraints = 0;
uint8_t g_seq_grace = 0;
uint32_t g_seq_grace_age_ms = 0;
uint32_t g_seq_age_ms = 0;
uint32_t g_seq_sample = 0;
float g_seq_pressure = 0.0f;
float g_seq_danger_average = 0.0f;
uint8_t g_seq_average_count = 0;
float g_seq_score = 0.0f;
float g_seq_max_slack = 0.0f;
float g_seq_effective[SEQUENTIAL_CONTROL_DIRECTIONS] = {0};
float g_seq_margin[SEQUENTIAL_CONTROL_DIRECTIONS] = {0};
static sequential_control_result_t g_seq_plan;
static sequential_danger_average_t g_seq_danger_history;
static uint32_t g_seq_average_sample_seen = 0;
static bool g_seq_average_clear = false;
static int g_seq_previous_direction = -1;
int8_t g_seq_side = -1;
uint8_t g_seq_side_votes = 0;
static int8_t g_seq_side_candidate = -1;
static uint32_t g_seq_side_sample_seen = 0;
enum {
  SEQ_ROUTE_IDLE = 0,
  SEQ_ROUTE_SIDESTEP = 1,
  SEQ_ROUTE_ADVANCE = 2,
  SEQ_ROUTE_HOLD = 3,
  SEQ_ROUTE_RETURN_SCAN = 4,
};
uint8_t g_seq_route_phase = SEQ_ROUTE_IDLE;
uint8_t g_seq_clear_votes = 0;
uint32_t g_seq_route_replans = 0;
float g_seq_waypoint_x = 0.0f;
float g_seq_waypoint_y = 0.0f;
float g_seq_waypoint_z = 0.0f;
float g_seq_path_offset_x = 0.0f;
float g_seq_path_offset_y = 0.0f;
static float g_seq_circle_offset_m = 0.0f;
float g_seq_return_delay_s = 0.0f;
float g_seq_scan_average = 0.0f;
uint8_t g_seq_scan_count = 0;
uint32_t g_seq_scan_retries = 0;
float g_seq_scan_target_yaw_deg = 0.0f;
float g_seq_scan_goal_yaw_deg = 0.0f;
float g_seq_scan_settle_s = 0.0f;
int8_t g_seq_scan_decision = 0;
uint8_t g_seq_scan_pid_yaw_override = 0;
uint8_t g_seq_barrier_active = 0;
float g_seq_barrier_a0 = 0.0f;
float g_seq_barrier_a1 = 0.0f;
float g_seq_barrier_b = 0.0f;
float g_seq_forward_clearance_m = 0.0f;
float g_seq_cruise_speed_mps = 0.0f;
float g_seq_goal_distance_m = 0.0f;
uint8_t g_seq_goal_reached = 0;
float g_seq_forward_open_average[SEQUENTIAL_CONTROL_DIRECTIONS] = {0};
float g_seq_scan_clearance_average[SEQUENTIAL_CONTROL_DIRECTIONS] = {0};
float g_seq_side_score_0 = 0.0f;
float g_seq_side_score_3 = 0.0f;
float g_seq_scan_forward_slope = 0.0f;
float g_seq_scan_retry_delay_s = 0.0f;
static bool g_seq_cruise_active = false;
static float g_seq_cruise_ref_x = 0.0f;
static float g_seq_cruise_ref_y = 0.0f;
static uint32_t g_seq_route_sample_seen = 0;
static float g_seq_route_side_world[3] = {0.0f, 0.0f, 0.0f};
static float g_seq_route_start_world[3] = {0.0f, 0.0f, 0.0f};
static int8_t g_seq_route_side_index = -1;
static float g_seq_route_altitude_m = 0.0f;
static bool g_seq_route_lateral_limited = false;
static bool g_seq_route_completed_step = false;
static bool g_seq_halfspace_was_applied = false;
static bool g_seq_return_scan_pending = false;
static float g_seq_return_base_yaw_deg = 0.0f;
static int8_t g_seq_return_evasion_side = -1;
static sequential_danger_average_t g_seq_scan_history;
static uint32_t g_seq_scan_sample_seen = 0;
static float g_seq_forward_average_inhibit_s = 0.0f;
static bool g_seq_scan_yaw_slewing = false;
static float g_seq_latest_clearance[SEQUENTIAL_CONTROL_DIRECTIONS] = {0};
static sequential_clearance_average_t g_seq_forward_open_history;
static sequential_clearance_average_t g_seq_scan_clearance_history;
static uint32_t g_seq_forward_clearance_sample_seen = 0;
static int8_t g_seq_last_evasion_side = -1;
static bool g_seq_previous_scan_valid = false;
static float g_seq_previous_scan_mean_m = 0.0f;

// --- Stage 2: gate navigation (fly a trajectory THROUGH the detected gate) ---
// When gateNavEn=1, override the commander setpoint with a gate waypoint: on the first
// valid detection, latch the world gate center and the approach direction (drone->gate
// at latch time), then steer the MPC toward a through-point `gateThrough` metres PAST
// the center along that direction, facing the gate. This flies a straight line through
// the gate center. Latching commits the target so the path doesn't wander as the vision
// estimate degrades on close approach; it re-arms once the drone passes the gate.
// The MPC plans position to the through-point and the stock PID does low-level + yaw
// (face the gate) -- reusing the cascade unchanged. Params/logs in gate_pnp_params.c.
// NOTE: gate_pnp gives only the gate CENTER (no gate-plane normal), so "through" is
// along the drone's line of sight -- correct head-on, approximate for angled gates.
uint8_t gateNavEn   = 0;      // PARAM: 1 = navigate through the detected gate
float   gateThrough = 0.5f;   // PARAM: through-point distance past the gate center [m]
float   gateSpeed   = 0.3f;   // PARAM: approach speed toward the gate [m/s]
uint8_t g_gate_latched = 0;   // LOG: 1 = a gate target is currently latched
float   g_gate_tx = 0.0f;     // LOG: through-point target x/y/z (world)
float   g_gate_ty = 0.0f;
float   g_gate_tz = 0.0f;
static float gate_lx = 0.0f, gate_ly = 0.0f, gate_lz = 0.0f;   // latched gate center (world)
static float gate_dirx = 1.0f, gate_diry = 0.0f;               // latched approach unit dir (xy)

// --- Stage 3: two-gate racetrack circuit (surveyed) ---
// Fly a repeating loop through two SURVEYED gate world positions. The path is an
// ellipse with the two gate centers at the ends of its major axis and semi-minor axis
// = loopWidth, so the drone passes through both gates each lap and loops around the
// sides. circEn=1 engages it (overrides the commander setpoint); the MPC tracks the
// moving target and the stock PID does low-level + face-forward yaw (tangent to the
// loop).
//
// Default preset (world frame = takeoff origin, +x forward, +y left, +z up): gate A is
// 1 m in front and 0.5 m up; gate B is 1.5 m to the RIGHT of A (-y), same height, same
// plane (both at x=1). loopWidth = 0.75 = half the gate spacing makes the ellipse a
// true CIRCLE of radius 0.75 m centred at (1, -0.75): the drone crosses the gate plane
// perpendicularly at each gate (through B heading +x, around the front, back through A
// heading -x), which is what the gate detector needs to see them head-on.
// Non-static for the C param file.
uint8_t circEn      = 0;      // PARAM: 1 = fly the two-gate circuit
float   gAx = 1.0f, gAy =  0.0f, gAz = 0.5f;   // PARAM: gate A center (world) [m]
float   gBx = 1.0f, gBy = -1.5f, gBz = 0.5f;   // PARAM: gate B center (world) [m]
float   loopWidth   = 0.75f;  // PARAM: loop half-width (semi-minor axis) [m]; 0.75 = circle
float   circSpeed   = 0.3f;   // PARAM: circuit tangential speed [m/s]
// Which way round the loop. The phase always ADVANCES, so this picks the sense by
// flipping the semi-minor axis, and with it the direction each gate is flown through.
// +1: from the takeoff origin the drone joins the loop heading FORWARD and reaches gate A
//     (the near one, 1 m in front) after ~45 deg of arc -- through A in +x, around the
//     front, back through B in -x, around the back. This is the sane one.
// -1: the mirror image. From the origin the join sends the drone BACKWARD, away from the
//     gates and around the back arc, reaching the far gate B only after 120 deg of arc --
//     it turns its back on a gate that is directly in front of it. (This was the original
//     default and it is why the drone flew backwards on engage.)
int8_t  circDir     = 1;      // PARAM: +1 / -1 = which way round the loop
float   g_circ_phase = 0.0f;  // LOG: current loop phase [rad]
uint32_t g_circ_laps = 0;     // LOG: completed circuit revolutions
static float circ_phase = 0.0f;
static uint8_t circ_armed = 0;

// Ellipse circuit point: gate A/B at the major-axis ends, semi-minor = loopWidth.
// Fills world position P[3] and (non-unit) tangent T[3]=dP/dtheta at phase theta.
static inline void circuitPoint(float theta, float P[3], float T[3]) {
  float ux = gBx - gAx, uy = gBy - gAy;
  float L = sqrtf(ux * ux + uy * uy);
  if (L < 1e-3f) L = 1e-3f;
  ux /= L; uy /= L;                       // major-axis unit (A->B)
  float a  = 0.5f * L;                     // semi-major
  const float dir = (circDir >= 0) ? 1.0f : -1.0f;
  float nx = dir * uy, ny = -dir * ux;     // semi-minor axis; its sign is the loop sense
  float mx = 0.5f * (gAx + gBx), my = 0.5f * (gAy + gBy);
  float c = cosf(theta), s = sinf(theta);
  P[0] = mx + a * c * ux + loopWidth * s * nx;
  P[1] = my + a * c * uy + loopWidth * s * ny;
  P[2] = 0.5f * (gAz + gBz) - 0.5f * (gAz - gBz) * c;   // theta=pi -> A.z, 0 -> B.z
  T[0] = -a * s * ux + loopWidth * c * nx;
  T[1] = -a * s * uy + loopWidth * c * ny;
  T[2] = 0.5f * (gAz - gBz) * s;
}

static void armCircuitAtNearestPoint(float x, float y) {
  if (circ_armed) return;
  float best = 1e30f;
  for (int k = 0; k < 48; ++k) {
    const float theta = (2.0f * M_PI_F * k) / 48.0f;
    float point[3], tangent[3];
    circuitPoint(theta, point, tangent);
    const float dx = point[0] - x;
    const float dy = point[1] - y;
    const float distance_sq = dx * dx + dy * dy;
    if (distance_sq < best) {
      best = distance_sq;
      circ_phase = theta;
    }
  }
  g_circ_laps = 0;
  circ_armed = 1;
}

static void advanceCircuitPhase(float speed_mps, float tangent_norm) {
  if (tangent_norm < 1e-6f || speed_mps <= 0.0f) return;
  circ_phase += speed_mps * (1.0f / (float)MPC_RATE) / tangent_norm;
  while (circ_phase >= 2.0f * M_PI_F) {
    circ_phase -= 2.0f * M_PI_F;
    if (g_circ_laps < UINT32_MAX) ++g_circ_laps;
  }
}

static void resetMpcWarmStart(uint8_t reason) {
  for (int k = 0; k < NHORIZON; ++k) {
    Xhrz[k] = x0;
    YX[k].setZero();
    ZX[k] = x0;
    ZX_new[k] = x0;
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    Uhrz[k].setZero();
    YU[k].setZero();
    ZU[k].setZero();
    ZU_new[k].setZero();
    d[k].setZero();
  }
  tiny_ClearPositionHalfspaces(&work);
  stgs.en_cstr_states = 0;
  g_seq_halfspace_was_applied = false;
  work.first_run = 1;
  g_mpc_last_reset_reason = reason;
  if (g_mpc_warm_resets < UINT32_MAX) g_mpc_warm_resets++;
}

static void resetStateConstraintWarmStart(void) {
  /* State constraints may be disabled for hundreds of solves, during which
   * their ADMM buffers do not advance. Seed them from the current unconstrained
   * horizon before enabling a new plane instead of injecting stale state. */
  for (int k = 0; k < NHORIZON; ++k) {
    YX[k].setZero();
    ZX[k] = Xhrz[k];
    ZX_new[k] = Xhrz[k];
  }
}

void updateInitialState(const sensorData_t *sensors, const state_t *state) {
  x0(0) = state->position.x;
  x0(1) = state->position.y;
  x0(2) = state->position.z;
  // Body velocity error, [m/s]
  x0(6) = state->velocity.x;
  x0(7) = state->velocity.y;
  x0(8) = state->velocity.z;
  // Angular rate error, [rad/s]
  x0(9)  = radians(sensors->gyro.x);
  x0(10) = radians(sensors->gyro.y);
  x0(11) = radians(sensors->gyro.z);
  // Stash the measured attitude. The multiplicative Rodrigues error written into
  // x0(3..5) is built in the 100 Hz block, once the reference frame q_ref0 is known.
  q_meas = qnormalize(mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w));  // current attitude
}

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      // Reference-attitude frame at the horizon head (rebuilt from the stored Rodrigues).
      // Valid while |reference yaw| < 180 deg; a full-yaw trajectory would need the
      // heading stored non-singularly (e.g. an angle), not as Rodrigues parameters.
      q_ref0 = rp2quat(mkvec(X_ref_data[traj_idx][3],
                             X_ref_data[traj_idx][4],
                             X_ref_data[traj_idx][5]));
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < NSTATES; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        // Horizon is a constant point, so the reference attitude is identity in the
        // q_ref0 frame -> zero cols 3..5 to match the multiplicative error in x0.
        Xref[i](3) = 0.0f;
        Xref[i](4) = 0.0f;
        Xref[i](5) = 0.0f;
        if (i < NHORIZON - 1) {
          for (int j = 0; j < NINPUTS; ++j) {
            Uref[i](j) = U_ref_data[traj_idx][j];
          }
        }
      }
      // Face-forward heading: yaw along the reference velocity direction. Handled by the
      // stock PID (not the MPC); held at the previous value when nearly stationary.
      float vx = X_ref_data[traj_idx][6], vy = X_ref_data[traj_idx][7];
      if (vx * vx + vy * vy > 1e-6f) {
        mpc_heading_yaw_deg = wrap_deg(atan2f(vy, vx) * (180.0f / M_PI_F));
      }
    }
  }
  else {
    bool  gate_cmd = false;
    float ref_yaw_deg, ref_roll_deg, ref_pitch_deg;

    // --- Stage 3: two-gate racetrack circuit override (highest priority) ---
    if (circEn && !seqAvoidEnable) {
      armCircuitAtNearestPoint(x0(0), x0(1));
      float P[3], T[3];
      circuitPoint(circ_phase, P, T);
      float Tn = sqrtf(T[0]*T[0] + T[1]*T[1] + T[2]*T[2]);
      if (Tn < 1e-6f) Tn = 1e-6f;
      advanceCircuitPhase(circSpeed, Tn);
      xg(0) = P[0]; xg(1) = P[1]; xg(2) = P[2];
      xg(6) = circSpeed * T[0] / Tn; xg(7) = circSpeed * T[1] / Tn; xg(8) = circSpeed * T[2] / Tn;
      xg(9) = 0.0f; xg(10) = 0.0f; xg(11) = 0.0f;
      ref_yaw_deg  = wrap_deg(atan2f(T[1], T[0]) * (180.0f / M_PI_F));  // face along the loop
      ref_roll_deg = 0.0f; ref_pitch_deg = 0.0f;
      g_gate_tx = P[0]; g_gate_ty = P[1]; g_gate_tz = P[2];  // reuse the target logs
      g_circ_phase = circ_phase;
      gate_cmd = true;
    } else if (!circEn) {
      circ_armed = 0;   // re-arm nearest-phase entry on next enable
      g_circ_laps = 0;
    }

    // --- Stage 2: single-gate navigation override (fly through the detected gate) ---
    if (!gate_cmd && gateNavEn) {
      // Latch a fresh valid detection: gate center + approach direction (drone->gate).
      if (!g_gate_latched && g_gate_valid) {
        float dx = g_gate_center_x - x0(0);
        float dy = g_gate_center_y - x0(1);
        float d  = sqrtf(dx * dx + dy * dy);
        if (d > 1e-3f) {
          gate_lx = g_gate_center_x; gate_ly = g_gate_center_y; gate_lz = g_gate_center_z;
          gate_dirx = dx / d; gate_diry = dy / d;
          g_gate_latched = 1;
        }
      }
      if (g_gate_latched) {
        // Target the through-point past the gate center, moving along the approach dir.
        g_gate_tx = gate_lx + gateThrough * gate_dirx;
        g_gate_ty = gate_ly + gateThrough * gate_diry;
        g_gate_tz = gate_lz;
        xg(0) = g_gate_tx; xg(1) = g_gate_ty; xg(2) = g_gate_tz;
        xg(6) = gateSpeed * gate_dirx; xg(7) = gateSpeed * gate_diry; xg(8) = 0.0f;
        xg(9) = 0.0f; xg(10) = 0.0f; xg(11) = 0.0f;
        ref_yaw_deg  = wrap_deg(atan2f(gate_diry, gate_dirx) * (180.0f / M_PI_F));
        ref_roll_deg = 0.0f; ref_pitch_deg = 0.0f;   // level; PID owns yaw
        gate_cmd = true;
        // Re-arm once the drone passes the gate center along the approach direction.
        float px = x0(0) - gate_lx, py = x0(1) - gate_ly;
        if (px * gate_dirx + py * gate_diry > 0.0f) g_gate_latched = 0;
      }
    }

    if (!gate_cmd) {
      // Commander/hover setpoint (unchanged behavior when gate nav is off/unlatched).
      xg(0)  = setpoint->position.x;
      xg(1)  = setpoint->position.y;
      xg(2)  = setpoint->position.z;
      xg(6)  = setpoint->velocity.x;
      xg(7)  = setpoint->velocity.y;
      xg(8)  = setpoint->velocity.z;
      xg(9)  = radians(setpoint->attitudeRate.roll);
      xg(10) = radians(setpoint->attitudeRate.pitch);
      xg(11) = radians(setpoint->attitudeRate.yaw);
      ref_yaw_deg   = setpoint->attitude.yaw;
      ref_roll_deg  = setpoint->attitude.roll;
      ref_pitch_deg = setpoint->attitude.pitch;
    }

    desired_rpy = mkvec(radians(ref_roll_deg), radians(ref_pitch_deg), radians(ref_yaw_deg));
    attitude = rpy2quat(desired_rpy);
    q_ref0 = qnormalize(attitude);  // desired attitude = reference frame for the error
    mpc_heading_yaw_deg = ref_yaw_deg;             // heading fed to the stock PID [deg]
    xg(3) = 0.0f;                   // identity attitude in the q_ref0 frame
    xg(4) = 0.0f;
    xg(5) = 0.0f;
    tiny_SetGoalState(&work, Xref, &xg);
    tiny_SetGoalInput(&work, Uref, &ug);
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

// Half-space constraint function removed for basic functionality test

// Stage 1: poll the AI-deck corner link and project to a world-frame gate center.
// Updates the g_gate_* globals (logged via visGate); does not touch control.
static void pollGateVision(const state_t *state, const sensorData_t *sensors) {
  float corners[GATE8_N_CORNERS];
  uint32_t age_ms = 0;
  uint32_t capture_tick = 0;
  if (!gate8LinkGetLatestSeqTimed(
          corners, &age_ms, NULL, &capture_tick)) {
    g_gate_valid = 0;   // no corner data received yet
    return;
  }
  memcpy(g_gate_corners_pixels, corners, sizeof(g_gate_corners_pixels));
  gate_capture_tick = capture_tick;
  DroneState ds;
  ds.x  = state->position.x;  ds.y  = state->position.y;  ds.z  = state->position.z;
  ds.vx = state->velocity.x;  ds.vy = state->velocity.y;  ds.vz = state->velocity.z;
  ds.qx = state->attitudeQuaternion.x;  ds.qy = state->attitudeQuaternion.y;
  ds.qz = state->attitudeQuaternion.z;  ds.qw = state->attitudeQuaternion.w;
  ds.wx = radians(sensors->gyro.x);  ds.wy = radians(sensors->gyro.y);  ds.wz = radians(sensors->gyro.z);
  // Writes g_gate_center_x/y/z, g_gate_range_m, g_gate_valid, and the dbg globals.
  gate_pnp_project(corners, age_ms, &ds, &g_gate_vision);
}

static void pollPerceptionDanger(const state_t *state) {
  uint8_t obstacle_presence[PERCEPTION_MAP_CELLS];
  uint8_t inverse_range[PERCEPTION_MAP_CELLS];
  uint8_t uncertainty[PERCEPTION_MAP_CELLS];
  uint8_t gate_opening[PERCEPTION_MAP_CELLS];
  uint32_t age_ms = 0;
  uint32_t capture_tick = 0;
  uint32_t sample = 0;
  if (!perceptEnable ||
      !perceptionMapLinkGetLatest(obstacle_presence, inverse_range, uncertainty,
                                  gate_opening,
                                  &age_ms, &capture_tick, &sample)) {
    g_percept_valid = 0;
    return;
  }
  g_percept_valid = 1;
  g_percept_age_ms = age_ms;
  g_percept_sample = sample;
  if (sample == perception_sample_seen) {
    return;
  }
  perception_sample_seen = sample;

  perception_danger_state_t danger_state;
  danger_state.body_velocity_mps[0] = state->velocity.x;
  danger_state.body_velocity_mps[1] = state->velocity.y;
  danger_state.body_velocity_mps[2] = state->velocity.z;
  danger_state.horizon_s = perceptHorizonS;
  danger_state.perception_control_latency_s = perceptLatencyS;
  danger_state.maximum_range_m = perceptMaxRangeM;
  danger_state.nominal_target_speed_mps = perceptNominalSpeedMps;
  perceptionDangerCompute(obstacle_presence, inverse_range, uncertainty,
                          age_ms, &danger_state, &g_perception_danger_map);
  g_percept_gate_open_cells = 0;
  /*
   * Both UART products carry the STM32 capture tick echoed by GAP8. Never
   * combine a gate polygon and dense map from different camera exposures.
   */
  if (perceptGateOpeningEnable && g_gate_valid &&
      capture_tick != 0 && capture_tick == gate_capture_tick) {
    const int changed = perceptionDangerApplyGateOpening(
        gate_opening, g_gate_corners_pixels, g_gate_range_m,
        perceptGateOpeningInsetPx, perceptGateOpeningThreshold,
        perceptGateOpeningSafeCap, perceptGateOpeningRangeGuardM,
        perceptGateOpeningMaxUncertainty, &g_perception_danger_map);
    g_percept_gate_open_cells =
        changed > 255 ? 255 : (uint8_t)changed;
  }

  float maximum_danger = 0.0f;
  float minimum_ttc = INFINITY;
  for (int cell = 0; cell < PERCEPTION_MAP_CELLS; ++cell) {
    if (g_perception_danger_map.probability[cell] > maximum_danger) {
      maximum_danger = g_perception_danger_map.probability[cell];
    }
    if (g_perception_danger_map.probability[cell] >= 0.5f &&
        g_perception_danger_map.time_to_contact_s[cell] < minimum_ttc) {
      minimum_ttc = g_perception_danger_map.time_to_contact_s[cell];
    }
  }
  g_percept_max_danger = maximum_danger;
  g_percept_center_danger =
      g_perception_danger_map.probability[5 * PERCEPTION_MAP_W + 5];
  g_percept_min_ttc = isfinite(minimum_ttc) ? minimum_ttc : 0.0f;
}

static void rotateBodyToWorld(const float body[3], const state_t *state,
                              float world[3]) {
  const quaternion_t *q = &state->attitudeQuaternion;
  const float tx = 2.0f * (q->y * body[2] - q->z * body[1]);
  const float ty = 2.0f * (q->z * body[0] - q->x * body[2]);
  const float tz = 2.0f * (q->x * body[1] - q->y * body[0]);
  world[0] = body[0] + q->w * tx + (q->y * tz - q->z * ty);
  world[1] = body[1] + q->w * ty + (q->z * tx - q->x * tz);
  world[2] = body[2] + q->w * tz + (q->x * ty - q->y * tx);
}

static void rotateWorldToBody(const float world[3], const state_t *state,
                              float body[3]) {
  const quaternion_t *q = &state->attitudeQuaternion;
  const float tx = 2.0f * (-q->y * world[2] + q->z * world[1]);
  const float ty = 2.0f * (-q->z * world[0] + q->x * world[2]);
  const float tz = 2.0f * (-q->x * world[1] + q->y * world[0]);
  body[0] = world[0] + q->w * tx + (-q->y * tz + q->z * ty);
  body[1] = world[1] + q->w * ty + (-q->z * tx + q->x * tz);
  body[2] = world[2] + q->w * tz + (-q->x * ty + q->y * tx);
}

static void pollSequentialObstacle(const state_t *state,
                                   const setpoint_t *setpoint) {
  memset(&g_seq_plan, 0, sizeof(g_seq_plan));
  g_seq_plan.chosen_direction = -1;
  g_seq_valid = 0;
  g_seq_stop = 0;
  g_seq_reliable_mask = 0;
  g_seq_chosen = -1;
  g_seq_pressure = 0.0f;
  g_seq_score = 0.0f;
  g_seq_grace = 0;
  g_seq_grace_age_ms = 0;
  g_seq_forward_clearance_m = 0.0f;
  if (!seqAvoidEnable) {
    g_seq_previous_direction = -1;
    sequentialDangerAverageReset(&g_seq_danger_history);
    g_seq_average_sample_seen = 0;
    g_seq_danger_average = 0.0f;
    g_seq_average_count = 0;
    g_seq_average_clear = false;
    sequentialClearanceAverageReset(&g_seq_forward_open_history);
    sequentialClearanceAverageReset(&g_seq_scan_clearance_history);
    memset(g_seq_forward_open_average, 0,
           sizeof(g_seq_forward_open_average));
    memset(g_seq_scan_clearance_average, 0,
           sizeof(g_seq_scan_clearance_average));
    g_seq_forward_clearance_sample_seen = 0;
    g_seq_side_score_0 = 0.0f;
    g_seq_side_score_3 = 0.0f;
    g_seq_scan_retry_delay_s = 0.0f;
    g_seq_previous_scan_valid = false;
    g_seq_previous_scan_mean_m = 0.0f;
    return;
  }

  float clearance[SEQUENTIAL_CONTROL_DIRECTIONS];
  float confidence[SEQUENTIAL_CONTROL_DIRECTIONS];
  uint8_t gate_valid = 0;
  uint32_t capture_tick = 0;
  if (!sequentialObstacleLinkGetLatest(clearance, confidence, &gate_valid,
                                        &g_seq_age_ms, &capture_tick,
                                        &g_seq_sample) ||
      g_seq_age_ms > seqAvoidMaxAgeMs) {
    g_seq_stop = 1;
    g_seq_previous_direction = -1;
    return;
  }
  (void)gate_valid;   // Gate validation is separate from obstacle-plane validity.
  (void)capture_tick;
  memcpy(g_seq_latest_clearance, clearance, sizeof(g_seq_latest_clearance));
  g_seq_forward_clearance_m = fminf(clearance[1], clearance[2]);

  const float goal_world[3] = {
    setpoint->position.x - state->position.x,
    setpoint->position.y - state->position.y,
    setpoint->position.z - state->position.z,
  };
  const float velocity_world[3] = {
    state->velocity.x, state->velocity.y, state->velocity.z,
  };
  float goal_body[3], velocity_body[3];
  rotateWorldToBody(goal_world, state, goal_body);
  rotateWorldToBody(velocity_world, state, velocity_body);
  const sequential_control_config_t config = {
    seqAvoidConfidenceMin,
    seqAvoidDroneRadiusM,
    seqAvoidTrackingMarginM,
    seqAvoidLatencyS,
    seqAvoidPerceptionMarginM,
    seqAvoidConfidenceGainM,
    seqAvoidDefaultOffsetM,
    seqAvoidMaxRangeM,
    seqAvoidTriggerM,
    seqAvoidDistanceWeight,
    seqAvoidGoalWeight,
    seqAvoidHysteresisWeight,
    seqAvoidDynamicWeight,
    seqAvoidDirectionSafeMinM,
  };
  sequentialObstacleControlPlan(clearance, confidence, goal_body,
                                velocity_body, g_seq_previous_direction,
                                &config, &g_seq_plan);

  const bool return_scan_active =
      g_seq_route_phase == SEQ_ROUTE_RETURN_SCAN;
  if (!return_scan_active &&
      g_seq_sample != g_seq_forward_clearance_sample_seen) {
    g_seq_forward_clearance_sample_seen = g_seq_sample;
    float open_sample[SEQUENTIAL_CONTROL_DIRECTIONS];
    for (int direction = 0; direction < SEQUENTIAL_CONTROL_DIRECTIONS;
         ++direction) {
      open_sample[direction] =
          (g_seq_plan.reliable_mask & (1u << direction)) ? 1.0f : 0.0f;
    }
    sequentialClearanceAverageUpdate(&g_seq_forward_open_history,
                                     open_sample, seqAvoidAverageWindow,
                                     g_seq_forward_open_average);
  }
  if (!return_scan_active && g_seq_forward_average_inhibit_s <= 0.0f &&
      g_seq_sample != g_seq_average_sample_seen) {
    g_seq_average_sample_seen = g_seq_sample;
    const uint8_t open_count = (uint8_t)__builtin_popcount(
        (unsigned int)(g_seq_plan.reliable_mask & 0x0fu));
    g_seq_danger_average = sequentialDangerAverageUpdate(
        &g_seq_danger_history,
        (uint8_t)(SEQUENTIAL_CONTROL_DIRECTIONS - open_count),
        seqAvoidAverageWindow);
    g_seq_average_count = g_seq_danger_history.count;
  }
  const uint8_t effective_window = seqAvoidAverageWindow < 1 ? 1 :
      seqAvoidAverageWindow > SEQUENTIAL_DANGER_WINDOW_MAX
          ? SEQUENTIAL_DANGER_WINDOW_MAX : seqAvoidAverageWindow;
  const bool average_ready = g_seq_average_count >= effective_window;
  g_seq_average_clear = !return_scan_active && average_ready &&
      g_seq_danger_average < seqAvoidClearAverage;
  g_seq_plan.avoidance_pressure = !return_scan_active && average_ready &&
      g_seq_danger_average > seqAvoidTriggerAverage ? 1.0f : 0.0f;

  /* A high dangerous-slice average starts avoidance. Side choice compares
   * only the two outer rays' binary open frequencies across the last N
   * classifications, not metric distances or the inner rays. */
  if (g_seq_plan.avoidance_pressure > 0.0f) {
    int8_t candidate = -1;
    candidate = sequentialSelectEvasionSide(
        g_seq_forward_open_average, seqAvoidSideBiasM,
        g_seq_last_evasion_side, &g_seq_side_score_0, &g_seq_side_score_3);
    /* Once a route has started, never switch sides during the maneuver. */
    if (g_seq_route_phase != SEQ_ROUTE_IDLE &&
        g_seq_route_side_index >= 0) {
      candidate = g_seq_route_side_index;
    }

    if (g_seq_sample != g_seq_side_sample_seen) {
      g_seq_side_sample_seen = g_seq_sample;
      g_seq_side_candidate = candidate;
      g_seq_side = candidate;
      g_seq_side_votes = 1;
    }
    g_seq_plan.chosen_direction = g_seq_side;
  } else if (g_seq_route_phase != SEQ_ROUTE_IDLE &&
             g_seq_route_side_index >= 0) {
    g_seq_plan.chosen_direction = g_seq_route_side_index;
  }

  g_seq_valid = g_seq_plan.valid;
  g_seq_stop = g_seq_plan.stop;
  g_seq_reliable_mask = g_seq_plan.reliable_mask;
  g_seq_chosen = g_seq_plan.chosen_direction;
  g_seq_pressure = g_seq_plan.avoidance_pressure;
  g_seq_score = isfinite(g_seq_plan.chosen_score)
      ? g_seq_plan.chosen_score : 0.0f;
  memcpy(g_seq_effective, g_seq_plan.effective_offset_m,
         sizeof(g_seq_effective));
  memcpy(g_seq_margin, g_seq_plan.margin_m, sizeof(g_seq_margin));
  if (g_seq_plan.valid) {
    g_seq_previous_direction = g_seq_plan.chosen_direction;
  } else {
    g_seq_previous_direction = -1;
  }
}

static void applySequentialFailSafeHold(const state_t *state,
                                        setpoint_t *setpoint) {
  if (!seqAvoidEnable || !g_seq_stop) return;
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->position.x = state->position.x;
  setpoint->position.y = state->position.y;
  setpoint->position.z = state->position.z;
  setpoint->velocity.x = 0.0f;
  setpoint->velocity.y = 0.0f;
  setpoint->velocity.z = 0.0f;
}

static void updateSequentialCruiseReference(const state_t *state,
                                            setpoint_t *setpoint) {
  if (!seqAvoidEnable || obsPidPassthrough) {
    g_seq_cruise_active = false;
    g_seq_cruise_speed_mps = 0.0f;
    g_seq_goal_distance_m = 0.0f;
    g_seq_goal_reached = 0;
    return;
  }

  if (circEn) {
    armCircuitAtNearestPoint(state->position.x, state->position.y);
    float point[3], tangent[3];
    circuitPoint(circ_phase, point, tangent);
    float tangent_norm = sqrtf(tangent[0] * tangent[0] +
                               tangent[1] * tangent[1] +
                               tangent[2] * tangent[2]);
    if (tangent_norm < 1.0e-6f) tangent_norm = 1.0e-6f;
    const bool avoidance_pause = g_seq_stop || !g_seq_plan.valid ||
        g_seq_plan.avoidance_pressure > 0.0f ||
        g_seq_route_phase != SEQ_ROUTE_IDLE;
    float target_speed = fminf(fmaxf(0.0f, circSpeed),
        sequentialClearanceSpeedMps(
            g_seq_forward_clearance_m, seqAvoidDirectionSafeMinM,
            fmaxf(0.0f, circSpeed), seqAvoidBrakingAccelMps2));
    if (avoidance_pause) target_speed = 0.0f;
    g_seq_cruise_speed_mps = sequentialRateLimitSpeedMps(
        g_seq_cruise_speed_mps, target_speed, seqAvoidCruiseAccelMps2,
        seqAvoidBrakingAccelMps2, 1.0f / (float)MPC_RATE);
    if (!avoidance_pause) {
      advanceCircuitPhase(g_seq_cruise_speed_mps, tangent_norm);
    }
    g_seq_cruise_active = true;
    g_seq_goal_distance_m = 0.0f;
    g_seq_goal_reached = 0;
    setpoint->position.x = point[0];
    setpoint->position.y = point[1];
    setpoint->position.z = point[2];
    setpoint->velocity.x = g_seq_cruise_speed_mps * tangent[0] / tangent_norm;
    setpoint->velocity.y = g_seq_cruise_speed_mps * tangent[1] / tangent_norm;
    setpoint->velocity.z = g_seq_cruise_speed_mps * tangent[2] / tangent_norm;
    setpoint->attitude.yaw = wrap_deg(
        atan2f(tangent[1], tangent[0]) * (180.0f / M_PI_F));
    g_gate_tx = point[0];
    g_gate_ty = point[1];
    g_gate_tz = point[2];
    g_circ_phase = circ_phase;
    return;
  }
  g_seq_circle_offset_m = 0.0f;

  const float goal_x = setpoint->position.x;
  const float goal_y = setpoint->position.y;
  if (!g_seq_cruise_active) {
    g_seq_cruise_ref_x = state->position.x;
    g_seq_cruise_ref_y = state->position.y;
    g_seq_cruise_speed_mps = 0.0f;
    g_seq_cruise_active = true;
  }

  float dx = goal_x - g_seq_cruise_ref_x;
  float dy = goal_y - g_seq_cruise_ref_y;
  float reference_distance = hypotf(dx, dy);
  g_seq_goal_distance_m = hypotf(goal_x - state->position.x,
                                 goal_y - state->position.y);

  const bool avoidance_pause = g_seq_stop || !g_seq_plan.valid ||
      g_seq_plan.avoidance_pressure > 0.0f ||
      g_seq_route_phase != SEQ_ROUTE_IDLE;
  float target_speed = sequentialClearanceSpeedMps(
      g_seq_forward_clearance_m, seqAvoidDirectionSafeMinM,
      seqAvoidCruiseSpeedMps, seqAvoidBrakingAccelMps2);
  if (avoidance_pause || reference_distance <= 1.0e-4f) {
    target_speed = 0.0f;
  }
  g_seq_cruise_speed_mps = sequentialRateLimitSpeedMps(
      g_seq_cruise_speed_mps, target_speed, seqAvoidCruiseAccelMps2,
      seqAvoidBrakingAccelMps2, 1.0f / (float)MPC_RATE);

  if (!avoidance_pause && reference_distance > 1.0e-4f) {
    const float step = fminf(
        reference_distance,
        g_seq_cruise_speed_mps * (1.0f / (float)MPC_RATE));
    g_seq_cruise_ref_x += step * dx / reference_distance;
    g_seq_cruise_ref_y += step * dy / reference_distance;
    dx = goal_x - g_seq_cruise_ref_x;
    dy = goal_y - g_seq_cruise_ref_y;
    reference_distance = hypotf(dx, dy);
  }

  const float tolerance = fmaxf(0.01f, seqAvoidGoalToleranceM);
  g_seq_goal_reached = reference_distance <= 1.0e-4f &&
      g_seq_goal_distance_m <= tolerance;
  setpoint->position.x = g_seq_cruise_ref_x;
  setpoint->position.y = g_seq_cruise_ref_y;
  setpoint->velocity.x = 0.0f;
  setpoint->velocity.y = 0.0f;
  setpoint->velocity.z = 0.0f;
}

static void resetSequentialRoute(void) {
  g_seq_route_phase = SEQ_ROUTE_IDLE;
  g_seq_clear_votes = 0;
  g_seq_route_sample_seen = 0;
  g_seq_route_side_index = -1;
  g_seq_route_lateral_limited = false;
  g_seq_route_completed_step = false;
}

static void applySequentialPathOffset(setpoint_t *setpoint) {
  if (seqAvoidEnable && circEn) {
    const float center_x = 0.5f * (gAx + gBx);
    const float center_y = 0.5f * (gAy + gBy);
    float radial_x = setpoint->position.x - center_x;
    float radial_y = setpoint->position.y - center_y;
    const float radial_norm = hypotf(radial_x, radial_y);
    if (radial_norm > 1.0e-4f) {
      radial_x /= radial_norm;
      radial_y /= radial_norm;
      g_seq_path_offset_x = g_seq_circle_offset_m * radial_x;
      g_seq_path_offset_y = g_seq_circle_offset_m * radial_y;
    } else {
      g_seq_path_offset_x = 0.0f;
      g_seq_path_offset_y = 0.0f;
    }
  }
  setpoint->position.x += g_seq_path_offset_x;
  setpoint->position.y += g_seq_path_offset_y;
}

static void resetSequentialForwardAverage(void) {
  sequentialDangerAverageReset(&g_seq_danger_history);
  g_seq_average_sample_seen = g_seq_sample;
  g_seq_danger_average = 0.0f;
  g_seq_average_count = 0;
  g_seq_average_clear = false;
  g_seq_plan.avoidance_pressure = 0.0f;
  g_seq_pressure = 0.0f;
}

static void resetSequentialReturnScanWindow(void) {
  sequentialDangerAverageReset(&g_seq_scan_history);
  sequentialClearanceAverageReset(&g_seq_scan_clearance_history);
  g_seq_scan_sample_seen = g_seq_sample;
  g_seq_scan_average = 0.0f;
  g_seq_scan_count = 0;
  memset(g_seq_scan_clearance_average, 0,
         sizeof(g_seq_scan_clearance_average));
  g_seq_scan_forward_slope = 0.0f;
}

static float selectSequentialScanRetryDelay(void) {
  const int forward_edge = g_seq_return_evasion_side == 3 ? 3 : 0;
  const int obstacle_edge = forward_edge == 0 ? 3 : 0;
  const float scan_mean = 0.25f * (
      g_seq_scan_clearance_average[0] + g_seq_scan_clearance_average[1] +
      g_seq_scan_clearance_average[2] + g_seq_scan_clearance_average[3]);
  /* At the peer yaw, the evasion-side outer slice points back toward the
   * original forward path. A positive slope therefore means the obstacle is
   * opening toward forward travel and another check can happen sooner. */
  g_seq_scan_forward_slope =
      g_seq_scan_clearance_average[forward_edge] -
      g_seq_scan_clearance_average[obstacle_edge];
  const float improvement = g_seq_previous_scan_valid
      ? scan_mean - g_seq_previous_scan_mean_m : 0.0f;
  const float gradient = fmaxf(0.0f, seqAvoidScanGradientM);
  const float improvement_min = fmaxf(0.0f, seqAvoidScanImprovementM);
  float factor = 1.25f;
  if (g_seq_scan_forward_slope >= gradient ||
      improvement >= improvement_min) {
    factor = 0.75f;
  } else if (g_seq_scan_forward_slope <= -gradient &&
             improvement <= 0.0f) {
    factor = 1.75f;
  }
  g_seq_previous_scan_mean_m = scan_mean;
  g_seq_previous_scan_valid = true;
  const float minimum = fmaxf(0.0f, seqAvoidScanRetryMinS);
  const float maximum = fmaxf(minimum, seqAvoidScanRetryMaxS);
  return fminf(maximum, fmaxf(minimum,
      fmaxf(0.0f, seqAvoidReturnScanRetryS) * factor));
}

static void holdSequentialReturnScan(const state_t *state,
                                     setpoint_t *setpoint) {
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeAbs;
  setpoint->position.x = g_seq_waypoint_x;
  setpoint->position.y = g_seq_waypoint_y;
  setpoint->position.z = g_seq_waypoint_z;
  setpoint->velocity.x = 0.0f;
  setpoint->velocity.y = 0.0f;
  setpoint->velocity.z = 0.0f;
  /* Keep TinyMPC's attitude reference fixed at forward yaw. The peering yaw is
   * injected only into the downstream stock PID output after the MPC solve. */
  setpoint->attitude.yaw = g_seq_return_base_yaw_deg;
  (void)state;
}

static void beginSequentialReturnScan(const state_t *state,
                                      const setpoint_t *setpoint) {
  g_seq_route_phase = SEQ_ROUTE_RETURN_SCAN;
  g_seq_waypoint_x = state->position.x;
  g_seq_waypoint_y = state->position.y;
  g_seq_waypoint_z = setpoint->position.z;
  g_seq_return_base_yaw_deg = setpoint->attitude.yaw;
  g_seq_scan_goal_yaw_deg = sequentialReturnScanYawDeg(
      g_seq_return_base_yaw_deg, g_seq_return_evasion_side,
      seqAvoidReturnScanYawDeg);
  g_seq_scan_target_yaw_deg = g_seq_return_base_yaw_deg;
  g_seq_scan_yaw_slewing = true;
  g_seq_scan_pid_yaw_override = true;
  g_seq_scan_settle_s = 0.0f;
  g_seq_scan_decision = 0;
  resetSequentialReturnScanWindow();
  resetStateConstraintWarmStart();
}

static void updateSequentialPathReturn(const state_t *state,
                                       setpoint_t *setpoint) {
  const float dt = 1.0f / (float)MPC_RATE;

  if (g_seq_route_phase == SEQ_ROUTE_RETURN_SCAN) {
    if (g_seq_scan_yaw_slewing) {
      g_seq_scan_target_yaw_deg = sequentialSlewYawDeg(
          g_seq_scan_target_yaw_deg, g_seq_scan_goal_yaw_deg,
          fmaxf(1.0f, seqAvoidReturnScanYawRateDegS) * dt);
      if (g_seq_scan_target_yaw_deg == g_seq_scan_goal_yaw_deg) {
        g_seq_scan_yaw_slewing = false;
        g_seq_scan_settle_s = fmaxf(0.0f, seqAvoidReturnScanSettleS);
        if (g_seq_scan_decision == 0) {
          resetSequentialReturnScanWindow();
        }
      }
      holdSequentialReturnScan(state, setpoint);
      return;
    }

    holdSequentialReturnScan(state, setpoint);
    if (g_seq_scan_settle_s > 0.0f) {
      g_seq_scan_settle_s = fmaxf(0.0f, g_seq_scan_settle_s - dt);
      return;
    }

    if (g_seq_scan_decision != 0) {
      const bool side_clear = g_seq_scan_decision > 0;
      g_seq_scan_decision = 0;
      g_seq_scan_pid_yaw_override = false;
      /* ADMM was paused for the complete peer and never observed the PID yaw.
       * Preserve its position/constraint warm start when forward motion
       * resumes; repeated full resets here caused failures after blocked scans. */
      resetSequentialRoute();
      applySequentialPathOffset(setpoint);
      if (side_clear) {
        g_seq_return_scan_pending = false;
        g_seq_return_delay_s = 0.0f;
        g_seq_barrier_active = 0;
      } else {
        if (g_seq_scan_retries < UINT32_MAX) g_seq_scan_retries++;
        g_seq_return_scan_pending = true;
        g_seq_return_delay_s = g_seq_scan_retry_delay_s;
      }
      return;
    }

    if (g_seq_sample != g_seq_scan_sample_seen) {
      g_seq_scan_sample_seen = g_seq_sample;
      const uint8_t open_count = (uint8_t)__builtin_popcount(
          (unsigned int)(g_seq_plan.reliable_mask & 0x0fu));
      g_seq_scan_average = sequentialDangerAverageUpdate(
          &g_seq_scan_history,
          (uint8_t)(SEQUENTIAL_CONTROL_DIRECTIONS - open_count),
          seqAvoidReturnScanWindow);
      sequentialClearanceAverageUpdate(&g_seq_scan_clearance_history,
                                       g_seq_latest_clearance,
                                       seqAvoidReturnScanWindow,
                                       g_seq_scan_clearance_average);
      g_seq_scan_count = g_seq_scan_history.count;
    }

    const uint8_t window = seqAvoidReturnScanWindow < 1 ? 1 :
        seqAvoidReturnScanWindow > SEQUENTIAL_DANGER_WINDOW_MAX
            ? SEQUENTIAL_DANGER_WINDOW_MAX : seqAvoidReturnScanWindow;
    if (g_seq_scan_count < window) return;

    g_seq_scan_decision =
        g_seq_scan_average < seqAvoidReturnScanClearAverage ? 1 : -1;
    g_seq_scan_retry_delay_s = g_seq_scan_decision < 0
        ? selectSequentialScanRetryDelay() : 0.0f;
    resetSequentialForwardAverage();
    g_seq_forward_average_inhibit_s =
        fmaxf(0.0f, seqAvoidReturnScanSettleS);
    g_seq_scan_goal_yaw_deg = g_seq_return_base_yaw_deg;
    g_seq_scan_yaw_slewing = true;
    g_seq_scan_settle_s = 0.0f;
    resetStateConstraintWarmStart();
    return;
  }

  if (g_seq_forward_average_inhibit_s > 0.0f) {
    g_seq_forward_average_inhibit_s = fmaxf(
        0.0f, g_seq_forward_average_inhibit_s - dt);
  }
  if (g_seq_return_delay_s > 0.0f) {
    g_seq_return_delay_s = fmaxf(0.0f, g_seq_return_delay_s - dt);
    applySequentialPathOffset(setpoint);
    return;
  }
  if (g_seq_return_scan_pending && seqAvoidReturnScanEnable &&
      seqAvoidReturnRateMps > 0.0f) {
    beginSequentialReturnScan(state, setpoint);
    holdSequentialReturnScan(state, setpoint);
    return;
  }
  if (seqAvoidReturnRateMps <= 0.0f) {
    applySequentialPathOffset(setpoint);
    return;
  }
  /* The path is about to move back toward the centerline, so the one-sided
   * post-evasion barrier has completed its job. */
  g_seq_barrier_active = 0;

  const float offset_norm = (seqAvoidEnable && circEn)
      ? fabsf(g_seq_circle_offset_m)
      : hypotf(g_seq_path_offset_x, g_seq_path_offset_y);
  const float return_step = seqAvoidReturnRateMps * dt;
  if (offset_norm <= return_step) {
    if (seqAvoidEnable && circEn) {
      g_seq_circle_offset_m = 0.0f;
    } else {
      g_seq_path_offset_x = 0.0f;
      g_seq_path_offset_y = 0.0f;
    }
    applySequentialPathOffset(setpoint);
    return;
  }
  const float scale = (offset_norm - return_step) / offset_norm;
  if (seqAvoidEnable && circEn) {
    g_seq_circle_offset_m *= scale;
  } else {
    g_seq_path_offset_x *= scale;
    g_seq_path_offset_y *= scale;
  }
  applySequentialPathOffset(setpoint);
}

static void armSequentialLateralBarrier(const state_t *state) {
  /* Permitted side: side_world' * position >= side_world' * anchor.
   * TinyMPC stores half-spaces as a' * position <= b, hence a=-side. */
  const float anchor[3] = {
    state->position.x, state->position.y, state->position.z,
  };
  float a_position[3];
  if (sequentialLateralBarrierRow(g_seq_route_side_world, anchor,
                                  a_position, &g_seq_barrier_b)) {
    g_seq_barrier_a0 = a_position[0];
    g_seq_barrier_a1 = a_position[1];
    g_seq_barrier_active = 1;
  }
}

static void setSequentialWaypoint(const state_t *state, const float direction[3],
                                  float distance, uint8_t phase) {
  g_seq_waypoint_x = state->position.x + distance * direction[0];
  g_seq_waypoint_y = state->position.y + distance * direction[1];
  g_seq_waypoint_z = g_seq_route_altitude_m;
  g_seq_route_phase = phase;
  if (g_seq_route_replans < UINT32_MAX) g_seq_route_replans++;
}

static bool setSequentialLateralWaypoint(const state_t *state) {
  const float lateral_progress =
      (state->position.x - g_seq_route_start_world[0]) *
          g_seq_route_side_world[0] +
      (state->position.y - g_seq_route_start_world[1]) *
          g_seq_route_side_world[1];
  const float remaining = seqAvoidMaxLateralM - fmaxf(0.0f, lateral_progress);
  if (remaining <= seqAvoidWaypointToleranceM) {
    g_seq_route_phase = SEQ_ROUTE_HOLD;
    g_seq_route_lateral_limited = true;
    g_seq_waypoint_x = state->position.x;
    g_seq_waypoint_y = state->position.y;
    g_seq_waypoint_z = g_seq_route_altitude_m;
    return false;
  }
  setSequentialWaypoint(state, g_seq_route_side_world,
                        fminf(seqAvoidReferenceShiftM, remaining),
                        SEQ_ROUTE_SIDESTEP);
  return true;
}

static void beginSequentialRoute(const state_t *state,
                                 const setpoint_t *setpoint) {
  g_seq_route_side_index = g_seq_side;
  g_seq_last_evasion_side = g_seq_side;
  const float side_body[3] = {0.0f, g_seq_side == 3 ? 1.0f : -1.0f, 0.0f};
  rotateBodyToWorld(side_body, state, g_seq_route_side_world);
  g_seq_route_side_world[2] = 0.0f;
  const float side_norm = hypotf(g_seq_route_side_world[0],
                                 g_seq_route_side_world[1]);
  if (side_norm > 1e-3f) {
    g_seq_route_side_world[0] /= side_norm;
    g_seq_route_side_world[1] /= side_norm;
  }
  g_seq_route_start_world[0] = state->position.x;
  g_seq_route_start_world[1] = state->position.y;
  g_seq_route_start_world[2] = state->position.z;
  /* Preserve the commander's flight level for the complete route. Copying the
   * measured height at each replan turns ordinary tracking error into a climb. */
  g_seq_route_altitude_m = setpoint->position.z;
  g_seq_clear_votes = 0;
  g_seq_route_lateral_limited = false;
  g_seq_route_completed_step = false;
  g_seq_return_delay_s = 0.0f;
  g_seq_return_scan_pending = false;
  g_seq_scan_settle_s = 0.0f;
  g_seq_scan_decision = 0;
  g_seq_scan_yaw_slewing = false;
  g_seq_scan_pid_yaw_override = false;
  g_seq_scan_retries = 0;
  g_seq_scan_retry_delay_s = 0.0f;
  g_seq_previous_scan_valid = false;
  g_seq_previous_scan_mean_m = 0.0f;
  g_seq_forward_average_inhibit_s = 0.0f;
  g_seq_barrier_active = 0;
  setSequentialLateralWaypoint(state);
}

/* Build a persistent lateral route. Every reached waypoint produces another
 * lateral step until the dangerous-slice moving average clears. */
static void updateSequentialRoute(const state_t *state, setpoint_t *setpoint) {
  if (!seqAvoidEnable) {
    resetSequentialRoute();
    g_seq_path_offset_x = 0.0f;
    g_seq_path_offset_y = 0.0f;
    g_seq_circle_offset_m = 0.0f;
    g_seq_return_delay_s = 0.0f;
    g_seq_return_scan_pending = false;
    g_seq_scan_settle_s = 0.0f;
    g_seq_scan_decision = 0;
    g_seq_scan_yaw_slewing = false;
    g_seq_scan_pid_yaw_override = false;
    g_seq_scan_retries = 0;
    g_seq_scan_retry_delay_s = 0.0f;
    g_seq_previous_scan_valid = false;
    g_seq_previous_scan_mean_m = 0.0f;
    g_seq_forward_average_inhibit_s = 0.0f;
    resetSequentialReturnScanWindow();
    g_seq_barrier_active = 0;
    g_seq_waypoint_x = state->position.x;
    g_seq_waypoint_y = state->position.y;
    g_seq_waypoint_z = state->position.z;
    return;
  }
  if (obsPidPassthrough) {
    /* Perception continues to run during PID takeoff/landing, but a waypoint
     * created there would be stale when MPC later assumes control. Keep the
     * route disarmed and make its logged target reflect the current pose. */
    resetSequentialRoute();
    g_seq_scan_pid_yaw_override = false;
    sequentialClearanceAverageReset(&g_seq_forward_open_history);
    memset(g_seq_forward_open_average, 0,
           sizeof(g_seq_forward_open_average));
    g_seq_forward_clearance_sample_seen = g_seq_sample;
    g_seq_side_score_0 = 0.0f;
    g_seq_side_score_3 = 0.0f;
    g_seq_previous_scan_valid = false;
    g_seq_previous_scan_mean_m = 0.0f;
    g_seq_waypoint_x = state->position.x;
    g_seq_waypoint_y = state->position.y;
    g_seq_waypoint_z = state->position.z;
    return;
  }
  if (g_seq_stop || !g_seq_plan.valid) {
    g_seq_scan_pid_yaw_override = false;
    if (g_seq_route_phase != SEQ_ROUTE_IDLE) {
      g_seq_route_phase = SEQ_ROUTE_HOLD;
      g_seq_waypoint_x = state->position.x;
      g_seq_waypoint_y = state->position.y;
      g_seq_waypoint_z = state->position.z;
    }
    return;
  }

  if (g_seq_route_phase == SEQ_ROUTE_RETURN_SCAN) {
    updateSequentialPathReturn(state, setpoint);
    return;
  }

  const bool fresh = g_seq_sample != g_seq_route_sample_seen;
  const uint8_t open_count = (uint8_t)__builtin_popcount(
      (unsigned int)(g_seq_plan.reliable_mask & 0x0fu));
  if (fresh) {
    g_seq_route_sample_seen = g_seq_sample;
    g_seq_clear_votes = open_count;
  }

  if (g_seq_route_phase == SEQ_ROUTE_IDLE &&
      g_seq_plan.avoidance_pressure > 0.0f && g_seq_side >= 0) {
    beginSequentialRoute(state, setpoint);
  }

  if (g_seq_route_phase == SEQ_ROUTE_IDLE) {
    updateSequentialPathReturn(state, setpoint);
    return;
  }

  const float waypoint_distance = hypotf(
      g_seq_waypoint_x - state->position.x,
      g_seq_waypoint_y - state->position.y);
  const bool reached = waypoint_distance <= seqAvoidWaypointToleranceM;
  if (reached && !g_seq_route_completed_step) {
    g_seq_route_completed_step = true;
    armSequentialLateralBarrier(state);
  }

  /* Never release on an early clear classification before completing the
   * first lateral waypoint. Once one full step is complete, a later clear may
   * release the route without forcing completion of an additional step. */
  if (fresh && g_seq_average_clear &&
      g_seq_route_completed_step) {
    const float lateral_progress = fmaxf(0.0f,
        (state->position.x - g_seq_route_start_world[0]) *
            g_seq_route_side_world[0] +
        (state->position.y - g_seq_route_start_world[1]) *
            g_seq_route_side_world[1]);
    if (circEn) {
      const float center_x = 0.5f * (gAx + gBx);
      const float center_y = 0.5f * (gAy + gBy);
      float radial_x = state->position.x - center_x;
      float radial_y = state->position.y - center_y;
      const float radial_norm = hypotf(radial_x, radial_y);
      if (radial_norm > 1.0e-4f) {
        radial_x /= radial_norm;
        radial_y /= radial_norm;
        g_seq_circle_offset_m += lateral_progress *
            (g_seq_route_side_world[0] * radial_x +
             g_seq_route_side_world[1] * radial_y);
      }
    } else {
      g_seq_path_offset_x += lateral_progress * g_seq_route_side_world[0];
      g_seq_path_offset_y += lateral_progress * g_seq_route_side_world[1];
    }
    g_seq_return_delay_s = fmaxf(0.0f, seqAvoidReturnDelayS);
    g_seq_return_scan_pending = seqAvoidReturnScanEnable &&
        seqAvoidReturnRateMps > 0.0f;
    g_seq_scan_retries = 0;
    g_seq_return_evasion_side = g_seq_route_side_index;
    g_seq_return_base_yaw_deg = setpoint->attitude.yaw;
    g_seq_scan_target_yaw_deg = g_seq_return_base_yaw_deg;
    g_seq_scan_goal_yaw_deg = g_seq_return_base_yaw_deg;
    g_seq_scan_yaw_slewing = false;
    g_seq_scan_settle_s = 0.0f;
    resetSequentialReturnScanWindow();
    resetSequentialRoute();
    g_seq_side = -1;
    g_seq_side_candidate = -1;
    g_seq_side_votes = 0;
    applySequentialPathOffset(setpoint);
    return;
  }

  if (fresh && reached && g_seq_route_phase == SEQ_ROUTE_SIDESTEP) {
    setSequentialLateralWaypoint(state);
  }

  if (g_seq_route_phase == SEQ_ROUTE_IDLE) return;
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->position.x = g_seq_waypoint_x;
  setpoint->position.y = g_seq_waypoint_y;
  setpoint->position.z = g_seq_waypoint_z;
  setpoint->velocity.x = 0.0f;
  setpoint->velocity.y = 0.0f;
  setpoint->velocity.z = 0.0f;
}

static void updateSequentialHalfspaces(void) {
  g_seq_constraints = 0;
  g_seq_max_slack = 0.0f;
  const bool apply_halfspace = g_seq_barrier_active &&
      seqAvoidConstraintEnable && !seqAvoidLogOnly;
  if (apply_halfspace != g_seq_halfspace_was_applied) {
    resetStateConstraintWarmStart();
    g_seq_halfspace_was_applied = apply_halfspace;
  }
  if (!apply_halfspace) {
    return;
  }

  Eigen::Vector3f a_position(g_seq_barrier_a0, g_seq_barrier_a1, 0.0f);
  const Eigen::Vector3f zero_velocity(0.0f, 0.0f, 0.0f);
  const uint8_t first_k = seqAvoidKStart < NHORIZON
      ? seqAvoidKStart : NHORIZON - 1;
  for (int k = first_k; k < NHORIZON; ++k) {
    tiny_SetKinematicHalfspace(
        &work, k, 1, &a_position, &zero_velocity, g_seq_barrier_b,
        seqAvoidSlackPenalty, 1);
    if (g_seq_constraints < UINT8_MAX) g_seq_constraints++;
  }
  stgs.en_cstr_states = 1;
}

static void updateSequentialSlackDiagnostics(void) {
  g_seq_max_slack = 0.0f;
  if (!seqAvoidEnable || !g_seq_barrier_active) return;
  for (int k = 0; k < NHORIZON; ++k) {
    g_seq_max_slack = fmaxf(g_seq_max_slack,
                            data.slack_used_hs[k][1]);
  }
}

static void cameraNormalToWorld(const float camera[3],
                                const state_t *state,
                                Eigen::Vector3f *world) {
  float bx = camera[2], by = -camera[0], bz = -camera[1];
  const float cp = cosf(g_gate_mount_pitch_rad);
  const float sp = sinf(g_gate_mount_pitch_rad);
  const float pitched_x = cp * bx + sp * bz;
  const float pitched_z = -sp * bx + cp * bz;
  bx = pitched_x; bz = pitched_z;
  const float cy = cosf(g_gate_mount_yaw_rad);
  const float sy = sinf(g_gate_mount_yaw_rad);
  const float yawed_x = cy * bx - sy * by;
  const float yawed_y = sy * bx + cy * by;
  bx = yawed_x; by = yawed_y;

  const quaternion_t *q = &state->attitudeQuaternion;
  const float tx = 2.0f * (q->y * bz - q->z * by);
  const float ty = 2.0f * (q->z * bx - q->x * bz);
  const float tz = 2.0f * (q->x * by - q->y * bx);
  (*world)(0) = bx + q->w * tx + (q->y * tz - q->z * ty);
  (*world)(1) = by + q->w * ty + (q->z * tx - q->x * tz);
  (*world)(2) = bz + q->w * tz + (q->x * ty - q->y * tx);
  world->normalize();
}

static Eigen::Vector3f cameraCenterWorld(const state_t *state) {
  const float bx = g_gate_mount_fwd_m;
  const float by = 0.0f;
  const float bz = g_gate_mount_up_m;
  const quaternion_t *q = &state->attitudeQuaternion;
  const float tx = 2.0f * (q->y * bz - q->z * by);
  const float ty = 2.0f * (q->z * bx - q->x * bz);
  const float tz = 2.0f * (q->x * by - q->y * bx);
  Eigen::Vector3f center;
  center << state->position.x + bx + q->w * tx + (q->y * tz - q->z * ty),
            state->position.y + by + q->w * ty + (q->z * tx - q->x * tz),
            state->position.z + bz + q->w * tz + (q->x * ty - q->y * tx);
  return center;
}

static void updatePerceptionAngularHalfspaces(const state_t *state) {
  g_percept_corridor_valid = 0;
  g_percept_constraints = 0;
  if (!perceptEnable || !g_percept_valid || !g_gate_valid) return;

  float gate_u = 0.0f, gate_v = 0.0f;
  for (int corner = 0; corner < 4; ++corner) {
    gate_u += 0.25f * g_gate_corners_pixels[2 * corner];
    gate_v += 0.25f * g_gate_corners_pixels[2 * corner + 1];
  }
  int gate_x = (int)(gate_u / 16.0f);
  int gate_y = (int)(gate_v / 16.0f);
  if (gate_x < 0) gate_x = 0;
  if (gate_x > 9) gate_x = 9;
  if (gate_y < 0) gate_y = 0;
  if (gate_y > 9) gate_y = 9;
  const int gate_cell = gate_y * 10 + gate_x;
  const float range =
      fmaxf(0.30f, g_perception_danger_map.range_m[gate_cell]);
  const float uncertainty = g_perception_danger_map.uncertainty[gate_cell];
  const float speed = sqrtf(
      state->velocity.x * state->velocity.x
      + state->velocity.y * state->velocity.y
      + state->velocity.z * state->velocity.z);
  float pixel_margin =
      perceptBaseMarginPx
      + g_gate_fx * (perceptDroneRadiusM + perceptSafetyMarginM) / range
      + 4.0f * uncertainty
      + g_gate_fx * speed * perceptLatencyS / range;
  if (pixel_margin > 24.0f) pixel_margin = 24.0f;
  g_percept_pixel_margin = pixel_margin;

  perception_corridor_config_t config = {
    perceptDangerThreshold, pixel_margin,
    g_gate_fx, g_gate_fy, g_gate_cx, g_gate_cy
  };
  perception_corridor_t corridor;
  if (!perceptionCorridorFit(g_perception_danger_map.probability,
                             g_gate_corners_pixels, &config, &corridor)) {
    return;
  }
  g_percept_corridor_valid = 1;
  Eigen::Vector3f normals_world[2];
  cameraNormalToWorld(corridor.camera_normal[0], state, &normals_world[0]);
  cameraNormalToWorld(corridor.camera_normal[1], state, &normals_world[1]);
  for (int axis = 0; axis < 3; ++axis) {
    g_percept_left_n[axis] = normals_world[0](axis);
    g_percept_right_n[axis] = normals_world[1](axis);
  }
  if (!perceptConstraintEnable || perceptLogOnly) return;

  const Eigen::Vector3f camera_center = cameraCenterWorld(state);
  const int first_k =
      perceptKStart < NHORIZON ? perceptKStart : NHORIZON - 1;
  for (int k = first_k; k < NHORIZON; ++k) {
    const float tau = fminf(perceptLookaheadS, (k + 1) * DT);
    for (int side = 0; side < 2; ++side) {
      float normal[3], center[3], a_position_data[3], a_velocity_data[3], b;
      for (int axis = 0; axis < 3; ++axis) {
        normal[axis] = normals_world[side](axis);
        center[axis] = camera_center(axis);
      }
      perceptionAngularConstraintRow(
          normal, center, tau, a_position_data, a_velocity_data, &b);
      Eigen::Vector3f a_position(
          a_position_data[0], a_position_data[1], a_position_data[2]);
      Eigen::Vector3f a_velocity(
          a_velocity_data[0], a_velocity_data[1], a_velocity_data[2]);
      tiny_SetKinematicHalfspace(
          &work, k, side + 1, &a_position, &a_velocity, b,
          perceptSlackPenalty, 1);
      g_percept_constraints++;
    }
  }
  if (g_percept_constraints) stgs.en_cstr_states = 1;
}

static void updatePerceptionSlackDiagnostics(void) {
  float maximum = 0.0f, total = 0.0f, square_sum = 0.0f;
  for (int k = 0; k < NHORIZON; ++k) {
    for (int side = 1; side <= 2; ++side) {
      const float slack = data.slack_used_hs[k][side];
      if (slack > maximum) maximum = slack;
      total += slack;
      square_sum += slack * slack;
    }
  }
  g_percept_max_slack = maximum;
  g_percept_total_slack = total;
  g_percept_slack_cost = perceptSlackPenalty * square_sum;
  if (g_percept_constraints) {
    if (info.status_val != TINY_SOLVED) g_percept_failed_solves++;
    if (maximum >= 0.02f) {
      g_percept_near_infeasible_solves++;
    }
  }
}

static void updateObstacleHalfspace(const state_t *state) {
  tiny_ClearPositionHalfspaces(&work);
  stgs.en_cstr_states = 0;
  g_obs_active = 0;
  g_obs_applied = 0;
  g_obs_count = 0;
  g_obs_first_k = 0;
  g_obs_a0 = 0.0f;
  g_obs_a1 = 0.0f;
  g_obs_a2 = 0.0f;
  g_obs_b = 0.0f;
  g_obs_margin = 0.0f;
  g_obs_violation = 0.0f;
  g_obs_clearance = 0.0f;
  const uint32_t since_activation_ms =
      (xTaskGetTickCount() - controller_activate_tick) * portTICK_PERIOD_MS;

  float cx = obsCx;
  float cy = obsCy;
  float radius = obsRadius;

  const float dx0 = state->position.x - cx;
  const float dy0 = state->position.y - cy;
  const float dz0 = fabsf(state->position.z - obsCz);
  const float radial_dist0 = sqrtf(dx0 * dx0 + dy0 * dy0);
  const float effective_radius = radius + obsSafety;
  g_obs_margin = effective_radius;
  g_obs_clearance = radial_dist0 - effective_radius;

  if (!obsEnable || since_activation_ms < obsDelayMs) {
    return;
  }

  const uint8_t k_start = (obsKStart < NHORIZON) ? obsKStart : (NHORIZON - 1);
  const float half_height = (obsHeight > 0.0f) ? (0.5f * obsHeight) : 1000.0f;
  const bool in_height_band = dz0 <= half_height + obsSafety;

  int first_k = -1;
  for (int k = k_start; k < NHORIZON; ++k) {
    float ref_x = Xref[k](0);
    float ref_y = Xref[k](1);
    float ref_z = Xref[k](2);
    if (!en_traj) {
      const float alpha = ((float)k + 1.0f) / (float)NHORIZON;
      ref_x = x0(0) + alpha * (xg(0) - x0(0));
      ref_y = x0(1) + alpha * (xg(1) - x0(1));
      ref_z = x0(2) + alpha * (xg(2) - x0(2));
    }

    const float rx = ref_x - cx;
    float ry = ref_y - cy;
    const float rz = fabsf(ref_z - obsCz);
    float rxy = sqrtf(rx * rx + ry * ry);
    if (rxy < effective_radius + obsActMargin && fabsf(ry) < 0.05f) {
      const float side = (obsSide >= 0.0f) ? 1.0f : -1.0f;
      ry += side * 0.5f * effective_radius;
      rxy = sqrtf(rx * rx + ry * ry);
    }
    if (rxy < 1e-3f || rz > half_height + obsSafety ||
        rxy > effective_radius + obsActMargin) {
      continue;
    }

    Eigen::Vector3f a;
    a << -rx / rxy, -ry / rxy, 0.0f;
    const float b = a(0) * cx + a(1) * cy - effective_radius;

    g_obs_active = 1;
    g_obs_count++;
    if (first_k < 0) {
      first_k = k;
      g_obs_first_k = (uint8_t)k;
      g_obs_a0 = a(0);
      g_obs_a1 = a(1);
      g_obs_a2 = a(2);
      g_obs_b = b;
      g_obs_violation = a(0) * state->position.x + a(1) * state->position.y +
                        a(2) * state->position.z - b;
    }

    if (!obsLogOnly && in_height_band) {
      tiny_SetPositionHalfspace(&work, k, 0, &a, b, 1);
      g_obs_applied = 1;
    }
  }

  if (g_obs_applied) {
    stgs.en_cstr_states = 1;
  }
}

void controllerOutOfTreeInit(void) {
  /* Start MPC initialization*/

  // Precompute/Cache
  // #include "params_500hz.h"
  // #include "params_100hz.h"
  #include "params_constrained.h"  // Demo-2-style 50 Hz constrained cache (rho=63)

  // End of Precompute/Cache

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 63.0f;  // Matches params_constrained.h
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);

  // Fill in the remaining struct. State buffers are required when obstacle half-spaces
  // are enabled; passing null here makes the state-constraint ADMM path invalid.
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, ZX, ZX_new);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, YX, YU, 0, &Kinf, d, &Pinf, p);

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

  for (int i = 0; i < NSTATES; ++i) {
    lcx(i) = -1e6f;
    ucx(i) = 1e6f;
  }
  for (int k = 0; k < NHORIZON; ++k) {
    YX[k].setZero();
    ZX[k] = x0;
    ZX_new[k] = x0;
  }
  tiny_SetStateBound(&work, &Acx, &lcx, &ucx);
  tiny_ClearPositionHalfspaces(&work);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;  // Obstacle state constraints are opt-in via obs params.
  stgs.max_iter = 5;        // Match demo-2 hardcoded-obstacle solve depth.
  stgs.iters_check_rho_update = 10;  // Demo-2: no rho/cache update during 5-iter solve.
  stgs.verbose = 0;
  /* Compute residuals on every solve. This may terminate early, while still
   * retaining the five-iteration upper bound used by the flight controller. */
  stgs.check_termination = 1;
  stgs.tol_abs_dual = 1e-3f;
  stgs.tol_abs_prim = 1e-3f;

  Klqr <<
  -0.123589f,0.123635f,0.285625f,-0.394876f,-0.419547f,-0.474536f,-0.073759f,0.072612f,0.186504f,-0.031569f,-0.038547f,-0.187738f,
  0.120236f,0.119379f,0.285625f,-0.346222f,0.403763f,0.475821f,0.071330f,0.068348f,0.186504f,-0.020972f,0.037152f,0.187009f,
  0.121600f,-0.122839f,0.285625f,0.362241f,0.337953f,-0.478858f,0.069310f,-0.070833f,0.186504f,0.022379f,0.015573f,-0.185212f,
  -0.118248f,-0.120176f,0.285625f,0.378857f,-0.322169f,0.477573f,-0.066881f,-0.070128f,0.186504f,0.030162f,-0.014177f,0.185941f;

  /* End of MPC initialization */
  step = 0;
  traj_iter = 0;
  mpc_has_run = false;
  controllerPidInit();   // the cascade output drives the stock PID -- init its state
  // Bring up the AI-deck corner UART link once (re-selecting the controller must not
  // start a second RX task / re-init the UART).
  static bool s_gate_link_init = false;
  if (!s_gate_link_init) {
    gate8LinkInit();
    s_gate_link_init = true;
  }
  q_ref0 = qeye();  // level reference until the first updateHorizonReference
  memset(&mpc_setpoint_data, 0, sizeof(mpc_setpoint_data));
  mpc_setpoint_data.mode.x   = modeAbs;
  mpc_setpoint_data.mode.y   = modeAbs;
  mpc_setpoint_data.mode.z   = modeAbs;
  mpc_setpoint_data.mode.yaw = modeAbs;

  static bool s_mpc_task_init = false;
  if (!s_mpc_task_init) {
    runTaskSemaphore = xSemaphoreCreateBinary();
    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
    STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask,
                           TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);
    s_mpc_task_init = true;
  }
  isInit = true;

  if (en_traj) {
    DEBUG_PRINT("Stored trajectory enabled\n");
  } else {
    DEBUG_PRINT("Commander/setpoint mode enabled\n");
  }
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

static void tinympcControllerTask(void *parameters) {
  (void)parameters;

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    setpoint_t setpoint_task;
    sensorData_t sensors_task;
    state_t state_task;
    bool reset_requested = false;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
    memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
    memcpy(&state_task, &state_data, sizeof(state_t));
    reset_requested = mpc_reset_requested;
    mpc_reset_requested = false;
    xSemaphoreGive(dataMutex);

    if (obsPidPassthrough) {
      /* During takeoff and landing the stock PID owns the vehicle. Keep only
       * the inexpensive sequential-link poll alive so the host can perform its
       * preflight age/validity checks. Dense perception, horizon generation,
       * ADMM, and MPC debug printing stay dormant until the PID->MPC edge.
       * That edge requests a fresh warm start in controllerOutOfTree(). */
      pollSequentialObstacle(&state_task, &setpoint_task);
      updateSequentialCruiseReference(&state_task, &setpoint_task);
      updateSequentialRoute(&state_task, &setpoint_task);
      if (g_mpc_pid_bypass_cycles < UINT32_MAX) {
        g_mpc_pid_bypass_cycles++;
      }
      g_mpc_last_complete_tick = xTaskGetTickCount();
      g_mpc_heartbeat_age_ms = 0;
      g_mpc_stall_hold = 0;
      g_mpc_stack_free_words =
          (uint32_t)uxTaskGetStackHighWaterMark(NULL);
      continue;
    }

    updateInitialState(&sensors_task, &state_task);
    if (reset_requested) {
      resetMpcWarmStart(MPC_RESET_ACTIVATION);
    }

    // Perception runs in the MPC task, not in the stabilizer callback.
    pollGateVision(&state_task, &sensors_task);
    pollPerceptionDanger(&state_task);
    pollSequentialObstacle(&state_task, &setpoint_task);
    updateSequentialCruiseReference(&state_task, &setpoint_task);
    applySequentialFailSafeHold(&state_task, &setpoint_task);
    updateSequentialRoute(&state_task, &setpoint_task);

    if (g_seq_scan_pid_yaw_override) {
      /* A return scan is a stationary position hold with a PID-owned yaw.
       * Running constrained ADMM here adds no path-planning value and was the
       * common factor in repeated watchdog resets during the peer maneuver.
       * Keep perception and the scan state machine at MPC_RATE, but publish the
       * held pose directly to the stock PID until forward yaw is restored. */
      setpoint_t scan_sp;
      memset(&scan_sp, 0, sizeof(scan_sp));
      scan_sp.mode.x = modeAbs;
      scan_sp.mode.y = modeAbs;
      scan_sp.mode.z = modeAbs;
      scan_sp.mode.yaw = modeAbs;
      scan_sp.position.x = setpoint_task.position.x;
      scan_sp.position.y = setpoint_task.position.y;
      scan_sp.position.z = setpoint_task.position.z;
      scan_sp.attitude.yaw = g_seq_scan_target_yaw_deg;

      if (g_mpc_scan_bypass_cycles < UINT32_MAX) {
        g_mpc_scan_bypass_cycles++;
      }
      g_mpc_last_complete_tick = xTaskGetTickCount();
      g_mpc_heartbeat_age_ms = 0;
      g_mpc_stall_hold = 0;
      g_mpc_stack_free_words =
          (uint32_t)uxTaskGetStackHighWaterMark(NULL);

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&mpc_setpoint_data, &scan_sp, sizeof(setpoint_t));
      mpc_has_run = true;
      xSemaphoreGive(dataMutex);
      continue;
    }

    updateHorizonReference(&setpoint_task);

    // Multiplicative attitude error in the reference frame: q_err = q_ref0^-1 (x) q_meas.
    struct quat q_err = qqmul(qinv(q_ref0), q_meas);
    phi = quat2rp(q_err);
    if (g_seq_scan_pid_yaw_override) {
      /* Camera peering is owned by the downstream PID. Do not expose that
       * deliberate yaw excursion to the MPC state or its ADMM warm start. */
      phi.z = 0.0f;
      x0(11) = 0.0f;
    }
    x0(3) = phi.x;
    x0(4) = phi.y;
    x0(5) = phi.z;

    updateObstacleHalfspace(&state_task);
    if (seqAvoidEnable) {
      updateSequentialHalfspaces();
    } else {
      updatePerceptionAngularHalfspaces(&state_task);
    }
    tiny_UpdateLinearCost(&work);
    if (g_mpc_solve_started < UINT32_MAX) g_mpc_solve_started++;
    g_mpc_solve_route_phase = g_seq_route_phase;
    const uint32_t mpc_start_us = usecTimestamp();
    tiny_SolveAdmm(&work);
    if (seqAvoidEnable) {
      updateSequentialSlackDiagnostics();
    } else {
      updatePerceptionSlackDiagnostics();
    }
    g_mpc_solve_us = usecTimestamp() - mpc_start_us;
    g_mpc_iter = (uint8_t)info.iter;
    g_mpc_status = (int8_t)info.status_val;
    g_mpc_primal_residual = info.pri_res;
    g_mpc_dual_residual = info.dua_res;
    if (g_mpc_solve_completed < UINT32_MAX) g_mpc_solve_completed++;
    g_mpc_last_complete_tick = xTaskGetTickCount();
    g_mpc_stack_free_words = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    const bool nonfinite_solve =
        !isfinite(g_mpc_primal_residual) || !isfinite(g_mpc_dual_residual) ||
        !isfinite(Xhrz[NHORIZON - 1](0)) ||
        !isfinite(Xhrz[NHORIZON - 1](1)) ||
        !isfinite(Xhrz[NHORIZON - 1](2));
    const bool residual_limit = seqAvoidEnable &&
        ((seqAvoidMaxPrimalResidual > 0.0f &&
         g_mpc_primal_residual > seqAvoidMaxPrimalResidual) ||
        (seqAvoidMaxDualResidual > 0.0f &&
         g_mpc_dual_residual > seqAvoidMaxDualResidual) ||
        (seqAvoidMaxSolveUs > 0 && g_mpc_solve_us > seqAvoidMaxSolveUs));
    const bool health_fault = nonfinite_solve ||
        info.status_val == TINY_NON_CVX || residual_limit;
    if (health_fault && !g_mpc_health_hold) g_mpc_health_faults++;
    g_mpc_health_hold = health_fault;

    result = info.status_val * info.iter;

    setpoint_t next_sp;
    memset(&next_sp, 0, sizeof(next_sp));
    next_sp.mode.x   = modeAbs;
    next_sp.mode.y   = modeAbs;
    next_sp.mode.z   = modeAbs;
    next_sp.mode.yaw = modeAbs;
    next_sp.position.x = health_fault ? state_task.position.x : Xhrz[NHORIZON - 1](0);
    next_sp.position.y = health_fault ? state_task.position.y : Xhrz[NHORIZON - 1](1);
    next_sp.position.z = health_fault ? state_task.position.z : Xhrz[NHORIZON - 1](2);
    next_sp.attitude.yaw = g_seq_scan_pid_yaw_override
        ? g_seq_scan_target_yaw_deg
        : (yawUseRef
             ? yawRefDeg
             : (enable_pid_face_forward_yaw
                  ? mpc_heading_yaw_deg
                  : (quat2rpy(q_meas).z * (180.0f / M_PI_F))));

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&mpc_setpoint_data, &next_sp, sizeof(setpoint_t));
    mpc_has_run = true;
    xSemaphoreGive(dataMutex);

    static uint32_t mpc_log_counter = 0;
    if (mpc_log_counter % 25 == 0) {
      DEBUG_PRINT("MPC: pos=(%.2f,%.2f,%.2f) ref=(%.2f,%.2f,%.2f)\n",
                  (double)x0(0), (double)x0(1), (double)x0(2),
                  (double)Xref[0](0), (double)Xref[0](1), (double)Xref[0](2));
      DEBUG_PRINT("MPC: u=(%.2f,%.2f,%.2f,%.2f) iter=%d solve=%luus\n",
                  (double)(Uhrz[0](0) + u_hover[0]), (double)(Uhrz[0](1) + u_hover[1]),
                  (double)(Uhrz[0](2) + u_hover[2]), (double)(Uhrz[0](3) + u_hover[3]),
                  info.iter, (unsigned long)g_mpc_solve_us);
    }
    mpc_log_counter++;
  }
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  setpoint_t hold_sp;
  memset(&hold_sp, 0, sizeof(hold_sp));
  hold_sp.mode.x   = modeAbs;
  hold_sp.mode.y   = modeAbs;
  hold_sp.mode.z   = modeAbs;
  hold_sp.mode.yaw = modeAbs;
  hold_sp.position.x = state->position.x;
  hold_sp.position.y = state->position.y;
  hold_sp.position.z = state->position.z;
  hold_sp.attitude.yaw = quat2rpy(qnormalize(mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w))).z * (180.0f / M_PI_F);

  if (!isInit || (dataMutex == NULL) || (runTaskSemaphore == NULL)) {
    controllerPid(control, &hold_sp, sensors, state, tick);
    return;
  }

  bool has_run_snapshot = false;
  bool health_hold_snapshot = false;
  bool stall_hold_snapshot = false;
  uint32_t activate_tick_snapshot = controller_activate_tick;
  setpoint_t output_sp;
  memcpy(&output_sp, &hold_sp, sizeof(output_sp));
  const bool controller_reactivated =
      (last_controller_tick == 0) || ((tick - last_controller_tick) > M2T(200));
  const uint8_t pid_passthrough_now = obsPidPassthrough;
  const bool pid_to_mpc_handoff = obs_pid_passthrough_seen &&
      last_obs_pid_passthrough && !pid_passthrough_now;
  obs_pid_passthrough_seen = true;
  last_obs_pid_passthrough = pid_passthrough_now;

  if (xSemaphoreTake(dataMutex, M2T(2)) == pdTRUE) {
    memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
    memcpy(&sensors_data, sensors, sizeof(sensorData_t));
    memcpy(&state_data, state, sizeof(state_t));
    if (controller_reactivated || pid_to_mpc_handoff) {
      controller_activate_tick = tick;
      activate_tick_snapshot = controller_activate_tick;
      mpc_has_run = false;
      mpc_reset_requested = true;
    }
    has_run_snapshot = mpc_has_run;
    health_hold_snapshot = g_mpc_health_hold;
    if (has_run_snapshot) {
      g_mpc_heartbeat_age_ms = T2M(
          xTaskGetTickCount() - g_mpc_last_complete_tick);
      g_mpc_stall_hold =
          g_mpc_heartbeat_age_ms > MPC_HEARTBEAT_TIMEOUT_MS;
    } else {
      g_mpc_heartbeat_age_ms = 0;
      g_mpc_stall_hold = 0;
    }
    stall_hold_snapshot = g_mpc_stall_hold;
    memcpy(&output_sp, &mpc_setpoint_data, sizeof(setpoint_t));
    xSemaphoreGive(dataMutex);
  } else {
    controllerPid(control, &hold_sp, sensors, state, tick);
    return;
  }
  last_controller_tick = tick;

  if (controller_reactivated || pid_to_mpc_handoff) {
    DEBUG_PRINT("OOT %s: hold pos=(%.2f,%.2f,%.2f)\n",
                pid_to_mpc_handoff ? "PID->MPC" : "activated",
                (double)state->position.x,
                (double)state->position.y,
                (double)state->position.z);
  }

  if (RATE_DO_EXECUTE(MPC_RATE, tick)) {
    xSemaphoreGive(runTaskSemaphore);
  }

  /* Output: CASCADE (ishaan/debug-traj). Hand the MPC's planned position + face-forward
     heading to the stock Crazyflie PID, which does all low-level attitude/rate/motor
     control -- including yaw, which it handles robustly at any angle. */
  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    if (pid_passthrough_now) {
      controllerPid(control, setpoint, sensors, state, tick);
      return;
    }
    const bool hold_output =
        (!has_run_snapshot) || health_hold_snapshot || stall_hold_snapshot ||
        ((tick - activate_tick_snapshot) < M2T(250));

    if (setpoint->mode.z == modeDisable && !hold_output) {
      // Not commanded to fly -> motors off.
      control->normalizedForces[0] = 0.0f;
      control->normalizedForces[1] = 0.0f;
      control->normalizedForces[2] = 0.0f;
      control->normalizedForces[3] = 0.0f;
      control->controlMode = controlModePWM;
    } else {
      memset(&mpc_setpoint_pid, 0, sizeof(mpc_setpoint_pid));
      mpc_setpoint_pid.mode.x   = modeAbs;
      mpc_setpoint_pid.mode.y   = modeAbs;
      mpc_setpoint_pid.mode.z   = modeAbs;
      mpc_setpoint_pid.mode.yaw = modeAbs;
      if (!hold_output) {
        memcpy(&mpc_setpoint_pid, &output_sp, sizeof(mpc_setpoint_pid));
      } else {
        // Before the first solve: hold current pose so we don't dive on the switch.
        mpc_setpoint_pid.position.x = state->position.x;
        mpc_setpoint_pid.position.y = state->position.y;
        mpc_setpoint_pid.position.z = state->position.z;
        mpc_setpoint_pid.attitude.yaw = quat2rpy(q_meas).z * (180.0f / M_PI_F);
      }
      controllerPid(control, &mpc_setpoint_pid, sensors, state, tick);
    }
  }
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
