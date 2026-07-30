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
#include "estimator.h"         // estimatorEnqueuePosition (Stage 4: vision -> EKF)

#include "gate8_link.h"   // AI-deck gate-corner UART receiver
#include "perception_map_link.h"
#include "perception_danger.h"
#include "perception_model_qparams.h"
#include "gate_pnp.h"     // corners -> world-frame gate center (Stage 1: perception only)

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (4 * configMINIMAL_STACK_SIZE)
#define TINYMPC_TASK_NAME             "TINYMPC"
// Match demo-2 commit 66004e6: a long ADMM solve at priority 2 can starve
// the timer/radio service tasks and trigger a motor stop even when it fits
// nominally inside the MPC period.
#define TINYMPC_TASK_PRI              1
#define TINYMPC_PLAN_TIMEOUT_MS       60

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

// Watchdog-safe MPC/PID cascade used by the known-good demo-2 integration history.
// Direct demo-2 control consumes X[1]/U[0], but a position PID needs meaningful spatial
// lookahead. At 50 Hz, knot 10 is 200 ms ahead: enough position/altitude error for the
// cascade without exposing it to the unstable terminal knot. An infeasible raw state is
// replaced by its projected Z state.
static setpoint_t mpc_setpoint_pid;
static bool  mpc_has_run = false;          // hold current pose until the first MPC solve
static uint32_t mpc_last_finish_tick = 0;
static uint32_t last_controller_tick = 0;
static uint32_t controller_activate_tick = 0;
static constexpr int MPC_CASCADE_OUTPUT_K = 10;
static constexpr float MPC_OUTPUT_MAX_VIOLATION_M = 0.01f;

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
uint32_t g_mpc_task_us = 0;
uint32_t g_mpc_stack_free_words = 0;
uint32_t g_mpc_stale_wakes = 0;
uint32_t g_mpc_dropped_wakes = 0;
uint32_t g_mpc_plan_age_ms = 0;
static void tinympcControllerTask(void *parameters);
STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);
// Let cfclient command yaw live (Parameters tab, group visYaw). useRef=1 overrides the
// heading with yawRefDeg; non-static so the C param file links against them.
uint8_t yawUseRef = 0;      // PARAM: 1 = command heading from yawRefDeg below
float   yawRefDeg = 0.0f;   // PARAM: commanded absolute heading [deg] when yawUseRef=1

// --- Gate vision (Stage 1: perception only, does NOT affect control) ---
// Poll the AI-deck corner link and publish gate telemetry independently of
// neural obstacle avoidance. Params/logs live in gate_pnp_params.c.
static GateVisionPacket g_gate_vision;

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
float    g_obs_eff_cx = 0.0f;
float    g_obs_eff_cy = 0.0f;
float    g_obs_eff_radius = 0.0f;
uint32_t g_mpc_solve_us = 0;
uint8_t  g_mpc_iter = 0;
float    g_mpc_primal_residual = 0.0f;
float    g_mpc_dual_residual = 0.0f;
float    g_mpc_rho = 0.0f;

// Vision-independent fixed half-space test. The configured inequality is
// testPlaneA' * position <= testPlaneB. It uses half-space slot 1, so the
// hardware runner disables both perception and modeled-obstacle constraints.
uint8_t testPlaneEnable = 0;
uint8_t testPlaneLogOnly = 0;
uint8_t testPlaneKStart = 1;
uint8_t testPlaneMaxIter = 5;
float testPlaneA[3] = {1.0f, -1.0f, 0.0f};
float testPlaneB = 0.35f;
uint8_t g_test_plane_valid = 0;
uint8_t g_test_plane_applied = 0;
uint8_t g_test_plane_constraints = 0;
uint8_t g_test_plane_worst_k = 0;
float g_test_plane_a[3] = {0.0f, 0.0f, 0.0f};
float g_test_plane_b = 0.0f;
float g_test_plane_plan_violation = 0.0f;
float g_test_plane_output_violation = 0.0f;
float g_test_plane_state_violation = 0.0f;

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
uint8_t perceptDangerQ = 42;
uint8_t perceptKStart = 1;
uint8_t perceptMassMinCells = 4;
float perceptLineOffsetPx = 16.0f;
uint8_t perceptHoldSteps = 6;
uint8_t perceptPersistMaps = 2;
uint8_t perceptSafeBandCells = 2;
uint8_t perceptConstraintSteps = 8;
uint8_t perceptClearMaps = 3;
uint8_t perceptRejoinSteps = 25;
float perceptMaxLateralSpeedMps = 0.60f;
uint8_t g_percept_valid = 0;
uint32_t g_percept_age_ms = 0;
uint32_t g_percept_sample = 0;
float g_percept_max_danger = 0.0f;
float g_percept_center_danger = 0.0f;
float g_percept_min_ttc = 0.0f;
uint8_t g_percept_corridor_valid = 0;
uint8_t g_percept_constraints = 0;
uint8_t g_percept_applied = 0;
int8_t g_percept_avoid_side = 0;
uint16_t g_percept_unsafe_rows[PERCEPTION_MAP_H] = {0};
float g_percept_plane_a[3] = {0.0f, 0.0f, 0.0f};
float g_percept_plane_b = 0.0f;
float g_percept_safe_u = 0.0f;
float g_percept_line_u = 0.0f;
float g_percept_left_n[3] = {0.0f, 0.0f, 0.0f};
float g_percept_right_n[3] = {0.0f, 0.0f, 0.0f};
float g_mpc_output_position[3] = {0.0f, 0.0f, 0.0f};
float g_mpc_output_yaw_deg = 0.0f;
uint8_t g_mpc_output_safe = 0;
float g_mpc_output_violation = 0.0f;
uint8_t g_percept_hold_active = 0;
uint32_t g_percept_hold_age_ms = 0;
uint8_t g_percept_obstacle_cells = 0;
uint8_t g_percept_mass_cells = 0;
uint8_t g_percept_mass_detected = 0;
uint8_t g_percept_center_q = 0;
uint8_t g_percept_max_q = 0;
uint8_t g_percept_path_hit = 0;
uint8_t g_percept_path_knot = 0;
uint8_t g_percept_path_q = 0;
uint8_t g_percept_safe_left_cells = 255;
uint8_t g_percept_safe_right_cells = 255;
float g_percept_path_u = 0.0f;
float g_percept_path_v = 0.0f;
float g_percept_plan_violation = 0.0f;
float g_percept_state_violation = 0.0f;
uint8_t g_percept_mode = 0;
uint8_t g_percept_persist_count = 0;
uint8_t g_percept_clear_count = 0;
float g_percept_avoid_position[3] = {0.0f, 0.0f, 0.0f};
float g_percept_reference_offset = 0.0f;
static perception_danger_map_t g_perception_danger_map;
static uint8_t g_perception_danger_q[PERCEPTION_MAP_CELLS] = {0};
static uint32_t perception_sample_seen = 0;
static uint32_t perception_constraint_sample_seen = 0;
enum {
  PERCEPT_MODE_CLEAR = 0,
  PERCEPT_MODE_AVOID = 1,
  PERCEPT_MODE_REJOIN = 2,
};
static uint8_t percept_plane_latch_valid = 0;
static uint32_t percept_plane_latched_tick = 0;
static int8_t percept_latched_side = 0;
static float percept_latched_a[3] = {0.0f, 0.0f, 0.0f};
static float percept_latched_b = 0.0f;
static float percept_latched_safe_u = 0.0f;
static float percept_latched_line_u = 0.0f;
static uint8_t percept_latched_first_k = 0;
static uint8_t percept_latched_last_k = 0;
static Eigen::Vector3f percept_avoid_position = Eigen::Vector3f::Zero();
static uint8_t percept_collision_k = 0;
static uint8_t percept_candidate_k = 0;
static uint8_t percept_candidate_count = 0;
static uint8_t percept_clear_count = 0;
static uint32_t percept_avoid_start_tick = 0;
static float percept_rejoin_gain = 0.0f;

static bool isDangerCell(int cell) {
  return cell >= 0 && cell < PERCEPTION_MAP_CELLS &&
         g_perception_danger_q[cell] >= perceptDangerQ;
}

static void clearPerceptionPlaneLatch(void) {
  percept_plane_latch_valid = 0;
  percept_plane_latched_tick = 0;
  percept_latched_side = 0;
  memset(percept_latched_a, 0, sizeof(percept_latched_a));
  percept_latched_b = 0.0f;
  percept_latched_safe_u = 0.0f;
  percept_latched_line_u = 0.0f;
  percept_latched_first_k = 0;
  percept_latched_last_k = 0;
  percept_avoid_position.setZero();
  percept_collision_k = 0;
  percept_candidate_k = 0;
  percept_candidate_count = 0;
  percept_clear_count = 0;
  percept_avoid_start_tick = 0;
  percept_rejoin_gain = 0.0f;
  g_percept_mode = PERCEPT_MODE_CLEAR;
  g_percept_persist_count = 0;
  g_percept_clear_count = 0;
  memset(g_percept_avoid_position, 0, sizeof(g_percept_avoid_position));
  g_percept_reference_offset = 0.0f;
  g_percept_hold_active = 0;
  g_percept_hold_age_ms = 0;
  g_percept_path_hit = 0;
  g_percept_path_knot = 0;
  g_percept_path_q = 0;
  g_percept_safe_left_cells = 255;
  g_percept_safe_right_cells = 255;
  g_percept_path_u = 0.0f;
  g_percept_path_v = 0.0f;
  g_percept_plan_violation = 0.0f;
  g_percept_state_violation = 0.0f;
}

static uint8_t largestDangerMass(void) {
  static uint8_t visited[PERCEPTION_MAP_CELLS];
  static uint8_t queue[PERCEPTION_MAP_CELLS];
  memset(visited, 0, sizeof(visited));
  uint8_t largest = 0;

  /* Rows 0 and 9 are persistently marked dangerous by the current model and
   * are excluded from this diagnostic. Count 4-connected components in the
   * remaining image instead of treating scattered pixels as one obstacle. */
  for (int seed_y = 1; seed_y < PERCEPTION_MAP_H - 1; ++seed_y) {
    for (int seed_x = 0; seed_x < PERCEPTION_MAP_W; ++seed_x) {
      const int seed = seed_y * PERCEPTION_MAP_W + seed_x;
      if (visited[seed] || !isDangerCell(seed)) {
        continue;
      }

      uint8_t head = 0;
      uint8_t tail = 0;
      uint8_t count = 0;
      queue[tail++] = (uint8_t)seed;
      visited[seed] = 1;
      while (head < tail) {
        const int cell = queue[head++];
        const int x = cell % PERCEPTION_MAP_W;
        const int y = cell / PERCEPTION_MAP_W;
        count++;
        const int nx[4] = {x - 1, x + 1, x, x};
        const int ny[4] = {y, y, y - 1, y + 1};
        for (int neighbor = 0; neighbor < 4; ++neighbor) {
          if (nx[neighbor] < 0 || nx[neighbor] >= PERCEPTION_MAP_W ||
              ny[neighbor] < 1 ||
              ny[neighbor] >= PERCEPTION_MAP_H - 1) {
            continue;
          }
          const int next =
              ny[neighbor] * PERCEPTION_MAP_W + nx[neighbor];
          if (!visited[next] && isDangerCell(next)) {
            visited[next] = 1;
            queue[tail++] = (uint8_t)next;
          }
        }
      }
      if (count > largest) largest = count;
    }
  }
  return largest;
}

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

// Unit normal of the gate plane, in xy. Both gates are surveyed as lying in one plane,
// so the A->B line spans it and the normal is that line rotated 90 deg. This is also the
// direction the drone crosses each gate in (the loop tangent at the major-axis ends), so
// a gate seen along +/-n is seen head-on. Derived from the survey -- no extra param.
static inline void gatePlaneNormal(float *nx, float *ny) {
  float ux = gBx - gAx, uy = gBy - gAy;
  float L = sqrtf(ux * ux + uy * uy);
  if (L < 1e-3f) { *nx = 1.0f; *ny = 0.0f; return; }
  *nx = -uy / L; *ny = ux / L;
}

// --- Stage 4: vision -> EKF, weighted by where we are on the trajectory ---
//
// The gates are SURVEYED landmarks, so a sighting of one is really a measurement of the
// DRONE: gate_pnp gives the drone->gate offset in world axes (g_gate_rel, which depends
// on attitude but not on the drifting position estimate), hence
//     drone_position = gate_surveyed - g_gate_rel
// which is enqueued as an absolute position measurement. That is the correction for the
// Flow-deck's unbounded odometry drift; the MPC and the trajectory are untouched.
//
// The weight (how hard the EKF is allowed to pull on a fix) is scheduled by trajectory
// phase, which is the point of this stage. For each surveyed gate we ask what the PLAN
// says: from the planned pose on the loop, is that gate framed by the camera (inside the
// FOV cone), is it near enough to range reliably, and is it being viewed square-on rather
// than edge-on? Their product is the expected visibility w in [0,1] -- ~1 on the approach
// legs, ~0 on the far side of the loop where no gate can be in shot. It is computed from
// the SCHEDULED pose, not the estimated one, so it cannot be corrupted by the very drift
// we are trying to correct. w then does two jobs:
//   - data association: the gate with the higher w is the one we must be looking at;
//   - weighting: stdDev = sigma(range) / w. On an approach leg w~1 and the fix is trusted
//     to a few cm; where the plan says no gate is visible, w floors at visWFloor (0.02),
//     inflating stdDev ~50x (variance ~2500x) so a stray detection moves the EKF by
//     essentially nothing, exactly as if it had been ignored.
// A detection that survives all that still has to pass an innovation gate (visMaxIn), so
// a mis-associated or phantom gate can never yank the state across the room.
//
// Bring-up is deliberately two-stage: visEn=1 computes and LOGS the correction without
// touching the estimator; visInj=1 actually feeds the EKF. Fly the first, then the second.
// Non-static for the C param file (PARAM/LOG macros don't compile in this C++ TU).
uint8_t visFuseEn   = 0;      // PARAM: 1 = run the fusion + log the correction
uint8_t visFuseInj  = 0;      // PARAM: 1 = actually enqueue the fix into the EKF
uint8_t visUseSched = 1;      // PARAM: 1 = weight/associate from the PLANNED loop pose
                              //        0 = from the estimated pose (bench / no circuit)
float   visStd0     = 0.05f;  // PARAM: position-fix noise, constant term [m]
float   visStdR     = 0.05f;  // PARAM: ... plus this * range^2 (size-based range error
                              //        grows quadratically) [m/m^2]
float   visFovDeg   = 40.0f;  // PARAM: camera half-FOV used for the framing weight [deg]
float   visIncMin   = 0.35f;  // PARAM: min |cos| between line-of-sight and gate normal
                              //        (~70 deg off-normal) before the gate is edge-on
float   visRGood    = 1.2f;   // PARAM: full range weight out to here [m]
float   visRFar     = 3.0f;   // PARAM: ... falling to zero at this range [m]
float   visWFloor   = 0.02f;  // PARAM: weight floor -> "almost nothing" off-schedule
float   visWCut     = 0.01f;  // PARAM: below this expected visibility, drop the fix
float   visMaxIn    = 0.75f;  // PARAM: reject corrections larger than this [m]
float    g_vf_w     = 0.0f;   // LOG: current scheduled visibility weight [0..1]
uint8_t  g_vf_gate  = 0;      // LOG: associated gate (0=none, 1=A, 2=B)
float    g_vf_std   = 0.0f;   // LOG: stdDev handed to the EKF [m]
float    g_vf_dx    = 0.0f;   // LOG: correction applied (implied pos - estimated pos) [m]
float    g_vf_dy    = 0.0f;
float    g_vf_dz    = 0.0f;
float    g_vf_expr  = 0.0f;   // LOG: expected range to the associated gate [m]
float    g_vf_expa  = 0.0f;   // LOG: expected off-axis angle to it [deg]
uint32_t g_vf_n     = 0;      // LOG: fixes injected
uint32_t g_vf_rej   = 0;      // LOG: detections rejected (no gate scheduled / innovation)

// Fresh-sample tracking: an EKF must see each vision frame at most ONCE. Fusing a frame
// the camera has not refreshed would double-count the same evidence and make the filter
// over-confident, so the corner link's sample counter is latched and only advances fuse.
static uint32_t gate_sample_seen = 0xFFFFFFFFu;
static bool     gate_sample_fresh = false;

static void resetMpcWarmStart(void) {
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
  work.first_run = 1;
}

// Expected visibility of one gate from a given pose: the product of a framing term (gate
// inside the FOV cone), an incidence term (gate square-on, not edge-on) and a range term
// (near enough for the apparent-size range estimate to mean anything). Each is 1 when
// ideal and ramps smoothly to 0, so the weight has no cliffs for the EKF to step off.
static float gateVisibility(float px, float py, float pz, float yaw_rad,
                            float gx, float gy, float gz,
                            float nx, float ny,
                            float *out_range_m, float *out_offaxis_deg) {
  const float dx = gx - px, dy = gy - py, dz = gz - pz;
  const float rxy = sqrtf(dx * dx + dy * dy);
  const float r   = sqrtf(rxy * rxy + dz * dz);
  if (out_range_m) *out_range_m = r;
  if (out_offaxis_deg) *out_offaxis_deg = 180.0f;
  if (r < 1e-3f || rxy < 1e-3f) return 0.0f;   // on top of it: nothing framed

  // Framing: angle between the camera boresight (body +x, pitched down by the mount) and
  // the line of sight. Roll/pitch of the airframe are small on this loop, so the planned
  // heading + mount pitch is enough to say what is in shot.
  const float cp = cosf(g_gate_mount_pitch_rad), sp = sinf(g_gate_mount_pitch_rad);
  const float bx = cosf(yaw_rad) * cp, by = sinf(yaw_rad) * cp, bz = -sp;
  float cos_off = (bx * dx + by * dy + bz * dz) / r;
  if (cos_off >  1.0f) cos_off =  1.0f;
  if (cos_off < -1.0f) cos_off = -1.0f;
  const float off = acosf(cos_off);                       // off-axis angle [rad]
  if (out_offaxis_deg) *out_offaxis_deg = off * (180.0f / M_PI_F);
  const float fov = radians(visFovDeg) > 1e-3f ? radians(visFovDeg) : 1e-3f;
  float w_fov = 1.0f - off / fov;                          // 1 on-axis, 0 at the edge
  if (w_fov <= 0.0f) return 0.0f;
  if (w_fov > 1.0f) w_fov = 1.0f;

  // Incidence: a gate viewed edge-on projects to a sliver, and the apparent-size range
  // estimate (and the corner detector itself) fall apart. |cos| -- either face counts.
  float cinc = fabsf((dx * nx + dy * ny) / rxy);
  const float imin = (visIncMin < 0.99f) ? visIncMin : 0.99f;
  float w_inc = (cinc - imin) / (1.0f - imin);
  if (w_inc <= 0.0f) return 0.0f;
  if (w_inc > 1.0f) w_inc = 1.0f;

  // Range: full weight while close, fading out to visRFar. Beyond it the gate is a few
  // pixels wide and the range estimate is noise.
  float w_rng;
  if (visRFar <= visRGood) {
    w_rng = (r <= visRFar) ? 1.0f : 0.0f;
  } else {
    w_rng = (visRFar - r) / (visRFar - visRGood);
    if (w_rng > 1.0f) w_rng = 1.0f;
    if (w_rng < 0.0f) w_rng = 0.0f;
  }
  if (w_rng <= 0.0f) return 0.0f;

  return w_fov * w_inc * w_rng;
}

// Fuse the latest gate sighting into the EKF, weighted by the scheduled visibility above.
// Runs at the perception rate; consumes each corner frame at most once.
static void fuseGateIntoEkf(const state_t *state) {
  if (!visFuseEn) {
    g_vf_w = 0.0f; g_vf_gate = 0; g_vf_std = 0.0f;
    gate_sample_fresh = false;   // don't fuse a frame that went stale while disabled
    return;
  }

  // Pose the WEIGHT is computed from. On the circuit this is the planned point on the
  // loop and its tangent heading: the schedule is what "this part of the trajectory"
  // means, and unlike the estimate it cannot have drifted. Off the circuit (bench, hover)
  // fall back to the estimate -- fine there, since drift is what we are measuring, not
  // something the weight has to be robust to.
  float px, py, pz, yaw;
  if (circEn && visUseSched && circ_armed) {
    float P[3], T[3];
    circuitPoint(circ_phase, P, T);
    px = P[0]; py = P[1]; pz = P[2];
    yaw = atan2f(T[1], T[0]);
  } else {
    px = state->position.x; py = state->position.y; pz = state->position.z;
    yaw = quat2rpy(q_meas).z;
  }

  // Association: score both surveyed gates from that pose; the better-framed one is the
  // gate a detection must belong to. On this loop the two are never both in shot, so the
  // winner is unambiguous -- and when neither scores, nothing should be visible at all.
  float nx, ny;
  gatePlaneNormal(&nx, &ny);
  float rA, aA, rB, aB;
  const float wA = gateVisibility(px, py, pz, yaw, gAx, gAy, gAz, nx, ny, &rA, &aA);
  const float wB = gateVisibility(px, py, pz, yaw, gBx, gBy, gBz, nx, ny, &rB, &aB);

  float w, sel_x, sel_y, sel_z;
  if (wB > wA) {
    w = wB; sel_x = gBx; sel_y = gBy; sel_z = gBz;
    g_vf_gate = 2; g_vf_expr = rB; g_vf_expa = aB;
  } else {
    w = wA; sel_x = gAx; sel_y = gAy; sel_z = gAz;
    g_vf_gate = 1; g_vf_expr = rA; g_vf_expa = aA;
  }
  g_vf_w = w;
  if (w < visWCut) g_vf_gate = 0;          // the plan says neither gate can be in shot

  // Only ever act on a corner frame the camera has actually refreshed (see above).
  if (!gate_sample_fresh) return;
  gate_sample_fresh = false;
  if (!g_gate_valid) return;               // this frame produced no usable gate fix

  if (w < visWCut) {                       // nothing should be visible here, so whatever
    g_vf_rej++;                            // the detector saw is not a gate -- drop it
    return;
  }
  const float w_eff = (w > visWFloor) ? w : visWFloor;   // floor -> "almost nothing"

  // The measurement: a surveyed landmark minus the measured offset to it IS the drone.
  const float ix = sel_x - g_gate_rel_x;
  const float iy = sel_y - g_gate_rel_y;
  const float iz = sel_z - g_gate_rel_z;
  const float dx = ix - state->position.x;
  const float dy = iy - state->position.y;
  const float dz = iz - state->position.z;
  g_vf_dx = dx; g_vf_dy = dy; g_vf_dz = dz;

  // Innovation gate: a true sighting of the expected gate lands near the current estimate
  // (drift is slow). Anything further out is a mis-association or a phantom -- drop it
  // rather than let it teleport the state.
  if (sqrtf(dx * dx + dy * dy + dz * dz) > visMaxIn) {
    g_vf_rej++;
    return;
  }

  // Noise model: range comes from apparent size, so its error grows ~quadratically with
  // range; the schedule weight then scales it.
  const float r = g_gate_range_m;
  float sigma = visStd0 + visStdR * r * r;
  if (sigma < 0.01f) sigma = 0.01f;
  float std = sigma / w_eff;
  if (std > 10.0f) std = 10.0f;            // keep the EKF's arithmetic sane
  g_vf_std = std;

  if (!visFuseInj) return;                 // bring-up: computed and logged, not applied

  positionMeasurement_t pos;
  pos.x = ix;
  pos.y = iy;
  pos.z = iz;
  pos.stdDev = std;
  pos.source = MeasurementSourceLocationService;
  estimatorEnqueuePosition(&pos);
  g_vf_n++;
}

// Basic mode - no obstacle avoidance constraints

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
    }
  }
  else {
    bool  gate_cmd = false;
    float ref_yaw_deg, ref_roll_deg, ref_pitch_deg;

    // --- Stage 3: two-gate racetrack circuit override (highest priority) ---
    if (circEn) {
      if (!circ_armed) {
        // Join the loop at the phase whose point is nearest the drone (smooth entry).
        float best = 1e30f;
        for (int k = 0; k < 48; k++) {
          float th = (2.0f * M_PI_F * k) / 48.0f;
          float Pk[3], Tk[3]; circuitPoint(th, Pk, Tk);
          float dx = Pk[0] - x0(0), dy = Pk[1] - x0(1);
          float d = dx * dx + dy * dy;
          if (d < best) { best = d; circ_phase = th; }
        }
        circ_armed = 1;
      }
      float P[3], T[3];
      circuitPoint(circ_phase, P, T);
      float Tn = sqrtf(T[0]*T[0] + T[1]*T[1] + T[2]*T[2]);
      if (Tn < 1e-6f) Tn = 1e-6f;
      circ_phase += circSpeed * (1.0f / (float)MPC_RATE) / Tn;   // advance ~const speed
      while (circ_phase >= 2.0f * M_PI_F) circ_phase -= 2.0f * M_PI_F;
      xg(0) = P[0]; xg(1) = P[1]; xg(2) = P[2];
      xg(6) = circSpeed * T[0] / Tn; xg(7) = circSpeed * T[1] / Tn; xg(8) = circSpeed * T[2] / Tn;
      xg(9) = 0.0f; xg(10) = 0.0f; xg(11) = 0.0f;
      ref_yaw_deg  = wrap_deg(atan2f(T[1], T[0]) * (180.0f / M_PI_F));  // face along the loop
      ref_roll_deg = 0.0f; ref_pitch_deg = 0.0f;
      g_gate_tx = P[0]; g_gate_ty = P[1]; g_gate_tz = P[2];  // reuse the target logs
      g_circ_phase = circ_phase;
      gate_cmd = true;
    } else {
      circ_armed = 0;   // re-arm nearest-phase entry on next enable
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
      /* Keep the camera pointed at the position being tracked. This preserves
       * the centered-target assumption when avoidance displaces the vehicle
       * laterally from the nominal path. Inside 5 cm, bearing is undefined, so
       * retain the commander's yaw instead of amplifying position noise. */
      const float target_dx = xg(0) - x0(0);
      const float target_dy = xg(1) - x0(1);
      if (target_dx * target_dx + target_dy * target_dy > 0.05f * 0.05f) {
        ref_yaw_deg = wrap_deg(
            atan2f(target_dy, target_dx) * (180.0f / M_PI_F));
      } else {
        ref_yaw_deg = setpoint->attitude.yaw;
      }
      ref_roll_deg  = setpoint->attitude.roll;
      ref_pitch_deg = setpoint->attitude.pitch;
    }

    desired_rpy = mkvec(radians(ref_roll_deg), radians(ref_pitch_deg), radians(ref_yaw_deg));
    attitude = rpy2quat(desired_rpy);
    q_ref0 = qnormalize(attitude);  // desired attitude = reference frame for the error
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

// Stage 1: poll the AI-deck corner link and project to a world-frame gate center.
// Updates the g_gate_* globals (logged via visGate); does not touch control.
static void pollGateVision(const state_t *state, const sensorData_t *sensors) {
  float corners[GATE8_N_CORNERS];
  uint32_t age_ms = 0;
  uint32_t sample = 0;
  if (!gate8LinkGetLatestSeqTimed(
          corners, &age_ms, &sample, NULL)) {
    g_gate_valid = 0;   // no corner data received yet
    return;
  }
  // Flag a corner frame the camera has actually refreshed, so Stage 4 fuses each one at
  // most once (the poll runs at 50 Hz, well above the camera's frame rate).
  if (sample != gate_sample_seen) {
    gate_sample_seen = sample;
    gate_sample_fresh = true;
  }
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
  uint32_t age_ms = 0;
  uint32_t sample = 0;
  if (!perceptEnable ||
      !perceptionMapLinkGetLatest(obstacle_presence, inverse_range, uncertainty,
                                  NULL, &age_ms, NULL, &sample)) {
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
  memcpy(g_perception_danger_q, obstacle_presence,
         sizeof(g_perception_danger_q));
  float maximum_danger = 0.0f;
  float minimum_ttc = INFINITY;
  g_percept_obstacle_cells = 0;
  g_percept_max_q = 0;
  memset(g_percept_unsafe_rows, 0, sizeof(g_percept_unsafe_rows));
  for (int cell = 0; cell < PERCEPTION_MAP_CELLS; ++cell) {
    const int row = cell / PERCEPTION_MAP_W;
    if (g_perception_danger_map.probability[cell] > maximum_danger) {
      maximum_danger = g_perception_danger_map.probability[cell];
    }
    if (row > 0 && row < PERCEPTION_MAP_H - 1 &&
        g_perception_danger_q[cell] > g_percept_max_q) {
      g_percept_max_q = g_perception_danger_q[cell];
    }
    if (isDangerCell(cell)) {
      const int column = cell % PERCEPTION_MAP_W;
      g_percept_unsafe_rows[row] |= (uint16_t)(1u << column);
      if (row > 0 && row < PERCEPTION_MAP_H - 1) {
        g_percept_obstacle_cells++;
      }
    }
    if (g_perception_danger_map.probability[cell] >= 0.5f &&
        g_perception_danger_map.time_to_contact_s[cell] < minimum_ttc) {
      minimum_ttc = g_perception_danger_map.time_to_contact_s[cell];
    }
  }
  g_percept_max_danger = maximum_danger;
  const int center_x = std::max(0, std::min(PERCEPTION_MAP_W - 1,
      (int)(g_gate_cx / 16.0f)));
  const int center_y = std::max(0, std::min(PERCEPTION_MAP_H - 1,
      (int)(g_gate_cy / 16.0f)));
  g_percept_center_danger =
      g_perception_danger_map.probability[center_y * PERCEPTION_MAP_W + center_x];
  g_percept_center_q =
      g_perception_danger_q[center_y * PERCEPTION_MAP_W + center_x];
  g_percept_min_ttc = isfinite(minimum_ttc) ? minimum_ttc : 0.0f;

  g_percept_mass_cells = largestDangerMass();
  g_percept_mass_detected =
      perceptMassMinCells > 0 &&
      g_percept_mass_cells >= perceptMassMinCells;
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

static bool worldPointToCameraPixel(const Eigen::Vector3f &point_world,
                                    const state_t *state,
                                    float *u, float *v) {
  const Eigen::Vector3f camera_center = cameraCenterWorld(state);
  const Eigen::Vector3f world = point_world - camera_center;
  const quaternion_t *q = &state->attitudeQuaternion;

  /* Rotate world -> body with the conjugate attitude quaternion. */
  const float qx = -q->x, qy = -q->y, qz = -q->z, qw = q->w;
  const float tx = 2.0f * (qy * world(2) - qz * world(1));
  const float ty = 2.0f * (qz * world(0) - qx * world(2));
  const float tz = 2.0f * (qx * world(1) - qy * world(0));
  const float body_x =
      world(0) + qw * tx + (qy * tz - qz * ty);
  const float body_y =
      world(1) + qw * ty + (qz * tx - qx * tz);
  const float body_z =
      world(2) + qw * tz + (qx * ty - qy * tx);

  /* Undo camera mount yaw, then mount pitch. */
  const float cy = cosf(g_gate_mount_yaw_rad);
  const float sy = sinf(g_gate_mount_yaw_rad);
  const float pitched_x = cy * body_x + sy * body_y;
  const float base_by = -sy * body_x + cy * body_y;
  const float pitched_z = body_z;
  const float cp = cosf(g_gate_mount_pitch_rad);
  const float sp = sinf(g_gate_mount_pitch_rad);
  const float base_bx = cp * pitched_x - sp * pitched_z;
  const float base_bz = sp * pitched_x + cp * pitched_z;

  const float camera_x = -base_by;
  const float camera_y = -base_bz;
  const float camera_z = base_bx;
  if (!isfinite(camera_z) || camera_z <= 0.02f) return false;
  *u = g_gate_cx + g_gate_fx * camera_x / camera_z;
  *v = g_gate_cy + g_gate_fy * camera_y / camera_z;
  return isfinite(*u) && isfinite(*v);
}

struct PerceptionPathHit {
  bool valid;
  uint8_t knot;
  uint8_t row;
  uint8_t column;
  uint8_t danger_q;
  float u;
  float v;
  Eigen::Vector3f nominal_world;
};

struct PerceptionSafeBand {
  bool valid;
  uint8_t distance_cells;
  float safe_u;
  float boundary_u;
};

static Eigen::Vector3f nominalPerceptionKnot(
    int knot, const state_t *state) {
  const Eigen::Vector3f camera_center = cameraCenterWorld(state);
  const Eigen::Vector3f planned(
      Xhrz[knot](0), Xhrz[knot](1), Xhrz[knot](2));
  const Eigen::Vector3f goal(xg(0), xg(1), xg(2));
  Eigen::Vector3f goal_ray = goal - camera_center;
  const float goal_range = goal_ray.norm();
  const float planned_range = (planned - camera_center).norm();
  if (!camera_center.allFinite() || !planned.allFinite() ||
      !goal_ray.allFinite() || goal_range < 0.05f ||
      !isfinite(planned_range)) {
    return planned;
  }
  goal_ray /= goal_range;
  // Keep the range of the previously planned MPC knot. The commander may
  // stream a slowly moving setpoint only a few centimetres ahead, but that
  // must not collapse the whole perception horizon to the current command.
  const float range = std::max(
      0.05f, std::min(perceptMaxRangeM, planned_range));
  return camera_center + range * goal_ray;
}

static PerceptionPathHit findEarliestDangerousNominalKnot(
    const state_t *state) {
  PerceptionPathHit hit = {};
  hit.valid = false;
  const int first_k =
      std::max(1, std::min((int)perceptKStart, NHORIZON - 1));
  for (int k = first_k; k < NHORIZON; ++k) {
    const Eigen::Vector3f nominal = nominalPerceptionKnot(k, state);
    float u = 0.0f;
    float v = 0.0f;
    if (!worldPointToCameraPixel(nominal, state, &u, &v) ||
        u < 0.0f || u >= 160.0f || v < 16.0f || v >= 144.0f) {
      continue;
    }
    const int column = std::max(
        0, std::min(PERCEPTION_MAP_W - 1, (int)(u / 16.0f)));
    const int row = std::max(
        1, std::min(PERCEPTION_MAP_H - 2, (int)(v / 16.0f)));
    const int cell = row * PERCEPTION_MAP_W + column;
    if (!isDangerCell(cell)) continue;
    hit.valid = true;
    hit.knot = (uint8_t)k;
    hit.row = (uint8_t)row;
    hit.column = (uint8_t)column;
    hit.danger_q = g_perception_danger_q[cell];
    hit.u = u;
    hit.v = v;
    hit.nominal_world = nominal;
    break;
  }
  return hit;
}

static PerceptionSafeBand safeBandOnSide(
    int row, int column, int side) {
  PerceptionSafeBand result = {};
  result.valid = false;
  result.distance_cells = 255;
  const int width = std::max(
      1, std::min((int)perceptSafeBandCells, PERCEPTION_MAP_W));
  if (side < 0) {
    for (int near = column - 1; near - width + 1 >= 0; --near) {
      bool safe = true;
      for (int x = near - width + 1; x <= near; ++x) {
        safe = safe && !isDangerCell(row * PERCEPTION_MAP_W + x);
      }
      if (!safe) continue;
      const int far = near - width + 1;
      result.valid = true;
      result.distance_cells = (uint8_t)(column - near);
      result.safe_u = 8.0f * (float)(far + near + 1);
      result.boundary_u = 16.0f * (float)(near + 1);
      return result;
    }
  } else {
    for (int near = column + 1; near + width - 1 < PERCEPTION_MAP_W; ++near) {
      bool safe = true;
      for (int x = near; x < near + width; ++x) {
        safe = safe && !isDangerCell(row * PERCEPTION_MAP_W + x);
      }
      if (!safe) continue;
      const int far = near + width - 1;
      result.valid = true;
      result.distance_cells = (uint8_t)(near - column);
      result.safe_u = 8.0f * (float)(near + far + 1);
      result.boundary_u = 16.0f * (float)near;
      return result;
    }
  }
  return result;
}

static bool installPerceptionAvoidance(
    const PerceptionPathHit &hit,
    const PerceptionSafeBand &band,
    int side,
    const state_t *state,
    uint32_t now_tick) {
  if (!hit.valid || !band.valid || (side != -1 && side != 1)) return false;
  const float maximum_margin =
      std::max(1.0f, 16.0f * (float)perceptSafeBandCells - 8.0f);
  const float margin = std::max(
      1.0f, std::min(maximum_margin, perceptLineOffsetPx));
  const float line_u = std::max(
      1.0f, std::min(159.0f, band.boundary_u + side * margin));
  float safe_u = band.safe_u;
  if ((side < 0 && safe_u >= line_u) ||
      (side > 0 && safe_u <= line_u)) {
    safe_u = line_u + side * 4.0f;
  }
  safe_u = std::max(1.0f, std::min(159.0f, safe_u));

  Eigen::Vector3f ray_camera(
      (safe_u - g_gate_cx) / g_gate_fx,
      (hit.v - g_gate_cy) / g_gate_fy,
      1.0f);
  ray_camera.normalize();
  Eigen::Vector3f ray_world;
  cameraNormalToWorld(ray_camera.data(), state, &ray_world);
  const Eigen::Vector3f camera_center = cameraCenterWorld(state);
  const float lookahead_range =
      (hit.nominal_world - camera_center).norm();
  if (!ray_world.allFinite() || !camera_center.allFinite() ||
      !isfinite(lookahead_range) || lookahead_range < 0.05f) {
    return false;
  }
  percept_avoid_position =
      camera_center + std::min(lookahead_range, perceptMaxRangeM) * ray_world;
  // This first implementation searches horizontally and should not trade
  // altitude for lateral clearance.
  percept_avoid_position(2) = hit.nominal_world(2);

  const float line_a = side < 0 ? -1.0f : 1.0f;
  const float line_c = -line_a * line_u;
  Eigen::Vector3f normal_camera(
      g_gate_fx * line_a, 0.0f, g_gate_cx * line_a + line_c);
  normal_camera.normalize();
  Eigen::Vector3f normal_world;
  cameraNormalToWorld(normal_camera.data(), state, &normal_world);
  if (!normal_world.allFinite()) return false;

  const Eigen::Vector3f a_position = -normal_world;
  for (int axis = 0; axis < 3; ++axis) {
    percept_latched_a[axis] = a_position(axis);
    g_percept_avoid_position[axis] = percept_avoid_position(axis);
  }
  percept_latched_b = -normal_world.dot(camera_center);
  percept_latched_side = (int8_t)side;
  percept_latched_safe_u = safe_u;
  percept_latched_line_u = line_u;
  percept_collision_k = hit.knot;
  const int first_k = std::max(
      (int)perceptKStart, std::max(1, (int)hit.knot - 1));
  percept_latched_first_k = (uint8_t)first_k;
  percept_latched_last_k = (uint8_t)std::min(
      NHORIZON - 1,
      first_k + std::max(1, (int)perceptConstraintSteps) - 1);
  percept_plane_latch_valid = 1;
  percept_plane_latched_tick = now_tick;
  if (g_percept_mode != PERCEPT_MODE_AVOID) {
    percept_avoid_start_tick = now_tick;
  }
  g_percept_mode = PERCEPT_MODE_AVOID;
  percept_rejoin_gain = 1.0f;
  percept_clear_count = 0;
  return true;
}

static float smoothStep01(float value) {
  const float x = std::max(0.0f, std::min(1.0f, value));
  return x * x * (3.0f - 2.0f * x);
}

static void deformPerceptionReference(const state_t *state) {
  if (g_percept_mode == PERCEPT_MODE_CLEAR ||
      !percept_avoid_position.allFinite()) {
    g_percept_reference_offset = 0.0f;
    return;
  }
  if (g_percept_mode == PERCEPT_MODE_REJOIN) {
    const float step =
        1.0f / (float)std::max(1, (int)perceptRejoinSteps);
    percept_rejoin_gain = std::max(0.0f, percept_rejoin_gain - step);
    if (percept_rejoin_gain <= 0.0f) {
      clearPerceptionPlaneLatch();
      return;
    }
  }

  const int collision_k = std::max(
      1, std::min((int)percept_collision_k, NHORIZON - 2));
  Eigen::Vector3f nominal_collision =
      nominalPerceptionKnot(collision_k, state);
  Eigen::Vector3f offset =
      percept_rejoin_gain * (percept_avoid_position - nominal_collision);
  g_percept_reference_offset = offset.norm();
  const int end_k = std::min(
      NHORIZON - 1,
      collision_k + std::max(2, (int)perceptConstraintSteps));

  for (int k = 0; k < NHORIZON; ++k) {
    const Eigen::Vector3f nominal = nominalPerceptionKnot(k, state);
    float weight = 0.0f;
    if (k <= collision_k) {
      weight = smoothStep01((float)k / (float)collision_k);
    } else if (k < end_k) {
      weight = 1.0f - smoothStep01(
          (float)(k - collision_k) / (float)(end_k - collision_k));
    }
    const Eigen::Vector3f deformed = nominal + weight * offset;
    Xref[k](0) = deformed(0);
    Xref[k](1) = deformed(1);
    Xref[k](2) = deformed(2);
  }

  const float maximum_speed =
      std::max(0.05f, perceptMaxLateralSpeedMps);
  Eigen::Vector2f forward(
      xg(0) - state->position.x,
      xg(1) - state->position.y);
  if (!forward.allFinite() || forward.norm() < 0.05f) {
    const struct vec reference_rpy = quat2rpy(q_ref0);
    forward << cosf(reference_rpy.z), sinf(reference_rpy.z);
  } else {
    forward.normalize();
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    Eigen::Vector3f velocity(
        (Xref[k + 1](0) - Xref[k](0)) / DT,
        (Xref[k + 1](1) - Xref[k](1)) / DT,
        (Xref[k + 1](2) - Xref[k](2)) / DT);
    Eigen::Vector2f velocity_xy(velocity(0), velocity(1));
    const float forward_speed = velocity_xy.dot(forward);
    Eigen::Vector2f lateral =
        velocity_xy - forward_speed * forward;
    const float lateral_speed = lateral.norm();
    if (isfinite(lateral_speed) && lateral_speed > maximum_speed) {
      lateral *= maximum_speed / lateral_speed;
      velocity_xy = forward_speed * forward + lateral;
      velocity(0) = velocity_xy(0);
      velocity(1) = velocity_xy(1);
    }
    Xref[k](6) = velocity(0);
    Xref[k](7) = velocity(1);
    Xref[k](8) = velocity(2);
  }
  Xref[NHORIZON - 1](6) = Xref[NHORIZON - 2](6);
  Xref[NHORIZON - 1](7) = Xref[NHORIZON - 2](7);
  Xref[NHORIZON - 1](8) = Xref[NHORIZON - 2](8);

  const int face_k = std::max(
      1, std::min(MPC_CASCADE_OUTPUT_K, NHORIZON - 1));
  const float face_dx = Xref[face_k](0) - state->position.x;
  const float face_dy = Xref[face_k](1) - state->position.y;
  if (face_dx * face_dx + face_dy * face_dy > 0.05f * 0.05f) {
    struct vec reference_rpy = quat2rpy(q_ref0);
    reference_rpy.z = atan2f(face_dy, face_dx);
    q_ref0 = qnormalize(rpy2quat(reference_rpy));
  }
}

static void updatePerceptionPositionHalfspace(const state_t *state) {
  const uint32_t now_tick = (uint32_t)xTaskGetTickCount();
  g_percept_corridor_valid = 0;
  g_percept_constraints = 0;
  g_percept_applied = 0;
  g_percept_avoid_side = percept_latched_side;
  memset(g_percept_plane_a, 0, sizeof(g_percept_plane_a));
  g_percept_plane_b = 0.0f;
  g_percept_safe_u = 0.0f;
  g_percept_line_u = 0.0f;
  g_percept_hold_active = 0;
  g_percept_hold_age_ms = 0;
  if (!perceptEnable) {
    clearPerceptionPlaneLatch();
    return;
  }

  const bool new_map =
      g_percept_sample != perception_constraint_sample_seen;
  if (new_map) {
    perception_constraint_sample_seen = g_percept_sample;
  }
  if (new_map && g_percept_valid && mpc_has_run &&
      isfinite(g_gate_cx) && isfinite(g_gate_cy) &&
      isfinite(g_gate_fx) && isfinite(g_gate_fy) &&
      g_gate_fx > 0.0f && g_gate_fy > 0.0f) {
    const PerceptionPathHit hit =
        findEarliestDangerousNominalKnot(state);
    g_percept_path_hit = hit.valid ? 1 : 0;
    if (hit.valid) {
      g_percept_path_knot = hit.knot;
      g_percept_path_q = hit.danger_q;
      g_percept_path_u = hit.u;
      g_percept_path_v = hit.v;
      const PerceptionSafeBand left =
          safeBandOnSide(hit.row, hit.column, -1);
      const PerceptionSafeBand right =
          safeBandOnSide(hit.row, hit.column, 1);
      g_percept_safe_left_cells =
          left.valid ? left.distance_cells : 255;
      g_percept_safe_right_cells =
          right.valid ? right.distance_cells : 255;

      if (percept_candidate_count > 0 &&
          abs((int)hit.knot - (int)percept_candidate_k) <= 2) {
        percept_candidate_count = (uint8_t)std::min(
            255, (int)percept_candidate_count + 1);
      } else {
        percept_candidate_k = hit.knot;
        percept_candidate_count = 1;
      }

      if (g_percept_mode == PERCEPT_MODE_AVOID) {
        const PerceptionSafeBand committed =
            percept_latched_side < 0 ? left : right;
        installPerceptionAvoidance(
            hit, committed, percept_latched_side, state, now_tick);
      } else if (
          percept_candidate_count >=
          std::max(1, (int)perceptPersistMaps)) {
        int selected_side = percept_latched_side;
        if (selected_side == 0) {
          if (left.valid &&
              (!right.valid ||
               left.distance_cells <= right.distance_cells)) {
            selected_side = -1;
          } else if (right.valid) {
            selected_side = 1;
          }
        }
        const PerceptionSafeBand selected =
            selected_side < 0 ? left : right;
        installPerceptionAvoidance(
            hit, selected, selected_side, state, now_tick);
      }
      percept_clear_count = 0;
    } else {
      g_percept_path_knot = 0;
      g_percept_path_q = 0;
      g_percept_safe_left_cells = 255;
      g_percept_safe_right_cells = 255;
      percept_candidate_count = 0;
      if (g_percept_mode == PERCEPT_MODE_AVOID) {
        percept_clear_count = (uint8_t)std::min(
            255, (int)percept_clear_count + 1);
        const uint32_t active_steps =
            (now_tick - percept_avoid_start_tick) /
            std::max(1u, M2T((uint32_t)(1000.0f * DT)));
        if (percept_clear_count >=
                std::max(1, (int)perceptClearMaps) &&
            active_steps >=
                (uint32_t)std::max(1, (int)perceptHoldSteps)) {
          g_percept_mode = PERCEPT_MODE_REJOIN;
          percept_plane_latch_valid = 0;
        }
      }
    }
  }

  g_percept_persist_count = percept_candidate_count;
  g_percept_clear_count = percept_clear_count;
  if (perceptConstraintEnable && !perceptLogOnly) {
    deformPerceptionReference(state);
  } else {
    g_percept_reference_offset = 0.0f;
  }

  if (!percept_plane_latch_valid ||
      g_percept_mode != PERCEPT_MODE_AVOID) {
    return;
  }
  g_percept_hold_active = 1;
  g_percept_hold_age_ms =
      (now_tick - percept_avoid_start_tick) * portTICK_PERIOD_MS;
  g_percept_corridor_valid = 1;
  for (int axis = 0; axis < 3; ++axis) {
    g_percept_plane_a[axis] = percept_latched_a[axis];
    g_percept_left_n[axis] = -percept_latched_a[axis];
    g_percept_right_n[axis] = 0.0f;
  }
  g_percept_plane_b = percept_latched_b;
  g_percept_avoid_side = percept_latched_side;
  g_percept_safe_u = percept_latched_safe_u;
  g_percept_line_u = percept_latched_line_u;
  if (!perceptConstraintEnable || perceptLogOnly) return;

  const Eigen::Vector3f a_position(
      percept_latched_a[0], percept_latched_a[1], percept_latched_a[2]);
  for (int k = percept_latched_first_k;
       k <= percept_latched_last_k; ++k) {
    tiny_SetPositionHalfspace(
        &work, k, 1, &a_position, percept_latched_b, 1);
    g_percept_constraints++;
  }
  if (g_percept_constraints) {
    g_percept_applied = 1;
    stgs.en_cstr_states = 1;
  }
}

static void updatePerceptionViolationDiagnostics(const state_t *state) {
  if (!percept_plane_latch_valid || !g_percept_applied) {
    g_percept_plan_violation = 0.0f;
    g_percept_state_violation = 0.0f;
    return;
  }
  const Eigen::Vector3f a(
      percept_latched_a[0], percept_latched_a[1], percept_latched_a[2]);
  const Eigen::Vector3f measured(
      state->position.x, state->position.y, state->position.z);
  g_percept_state_violation = a.dot(measured) - percept_latched_b;
  float maximum = -INFINITY;
  for (int k = percept_latched_first_k;
       k <= percept_latched_last_k && k < NHORIZON; ++k) {
    const Eigen::Vector3f planned(Xhrz[k](0), Xhrz[k](1), Xhrz[k](2));
    maximum = std::max(maximum, a.dot(planned) - percept_latched_b);
  }
  g_percept_plan_violation = isfinite(maximum) ? maximum : 0.0f;
}

static bool normalizedTestPlane(Eigen::Vector3f *a, float *b) {
  const Eigen::Vector3f raw_a(
      testPlaneA[0], testPlaneA[1], testPlaneA[2]);
  const float norm = raw_a.norm();
  if (!raw_a.allFinite() || !isfinite(testPlaneB) || norm < 1e-4f) {
    return false;
  }
  *a = raw_a / norm;
  *b = testPlaneB / norm;
  return a->allFinite() && isfinite(*b);
}

static void updateTestPlaneHalfspace(void) {
  g_test_plane_valid = 0;
  g_test_plane_applied = 0;
  g_test_plane_constraints = 0;
  g_test_plane_worst_k = 0;
  memset(g_test_plane_a, 0, sizeof(g_test_plane_a));
  g_test_plane_b = 0.0f;

  if (!testPlaneEnable) return;
  Eigen::Vector3f a;
  float b = 0.0f;
  if (!normalizedTestPlane(&a, &b)) return;

  g_test_plane_valid = 1;
  for (int axis = 0; axis < 3; ++axis) {
    g_test_plane_a[axis] = a(axis);
  }
  g_test_plane_b = b;
  if (testPlaneLogOnly) return;

  const int first_k =
      std::max(1, std::min((int)testPlaneKStart, NHORIZON - 1));
  for (int k = first_k; k < NHORIZON; ++k) {
    tiny_SetPositionHalfspace(&work, k, 1, &a, b, 1);
    g_test_plane_constraints++;
  }
  if (g_test_plane_constraints) {
    g_test_plane_applied = 1;
    stgs.en_cstr_states = 1;
  }
}

static void updateTestPlaneViolationDiagnostics(const state_t *state) {
  g_test_plane_plan_violation = 0.0f;
  g_test_plane_output_violation = 0.0f;
  g_test_plane_state_violation = 0.0f;
  g_test_plane_worst_k = 0;
  if (!g_test_plane_valid) return;

  const Eigen::Vector3f a(
      g_test_plane_a[0], g_test_plane_a[1], g_test_plane_a[2]);
  const Eigen::Vector3f measured(
      state->position.x, state->position.y, state->position.z);
  g_test_plane_state_violation = a.dot(measured) - g_test_plane_b;

  const int first_k =
      std::max(1, std::min((int)testPlaneKStart, NHORIZON - 1));
  float maximum = -INFINITY;
  int worst_k = first_k;
  for (int k = first_k; k < NHORIZON; ++k) {
    const Eigen::Vector3f planned(Xhrz[k](0), Xhrz[k](1), Xhrz[k](2));
    const float violation = a.dot(planned) - g_test_plane_b;
    if (violation > maximum) {
      maximum = violation;
      worst_k = k;
    }
  }
  g_test_plane_plan_violation = isfinite(maximum) ? maximum : 0.0f;
  g_test_plane_worst_k = (uint8_t)worst_k;
  const Eigen::Vector3f output(
      g_mpc_output_position[0],
      g_mpc_output_position[1],
      g_mpc_output_position[2]);
  g_test_plane_output_violation = a.dot(output) - g_test_plane_b;
}

static float knotHalfspaceViolation(int knot, const VectorNf &state) {
  float maximum = -INFINITY;
  if (knot < 0 || knot >= NHORIZON) return maximum;
  for (int h = 0; h < MAX_HS; ++h) {
    if (!data.en_hs[knot][h]) continue;
    const Eigen::Vector3f position(state(0), state(1), state(2));
    const Eigen::Vector3f velocity(state(6), state(7), state(8));
    const float allowed_slack =
        std::max(0.0f, data.slack_used_hs[knot][h]);
    const float violation =
        data.a_pos_hs[knot][h].dot(position) +
        data.a_vel_hs[knot][h].dot(velocity) -
        data.b_hs[knot][h] - allowed_slack;
    maximum = std::max(maximum, violation);
  }
  return maximum;
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
  g_obs_eff_cx = cx;
  g_obs_eff_cy = cy;
  g_obs_eff_radius = radius;

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
  #include "params_constrained.h"  // Exact 50 Hz constrained cache (demo-2 rho=250)

  // End of Precompute/Cache

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0f;  // Demo-2 penalty; must match params_constrained.h.
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
  clearPerceptionPlaneLatch();

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;  // Obstacle state constraints are opt-in via obs params.
  stgs.max_iter = 5;        // Match demo-2 hardcoded-obstacle solve depth.
  stgs.iters_check_rho_update = 10;  // Demo-2: no rho/cache update during 5-iter solve.
  stgs.verbose = 0;
  stgs.check_termination = 0;
  // Match demo-2 diagnostics. Termination remains disabled so solve time stays
  // deterministic at five iterations.
  stgs.tol_abs_dual = 5e-2f;
  stgs.tol_abs_prim = 5e-2f;

  Klqr <<
  -0.123589f,0.123635f,0.285625f,-0.394876f,-0.419547f,-0.474536f,-0.073759f,0.072612f,0.186504f,-0.031569f,-0.038547f,-0.187738f,
  0.120236f,0.119379f,0.285625f,-0.346222f,0.403763f,0.475821f,0.071330f,0.068348f,0.186504f,-0.020972f,0.037152f,0.187009f,
  0.121600f,-0.122839f,0.285625f,0.362241f,0.337953f,-0.478858f,0.069310f,-0.070833f,0.186504f,0.022379f,0.015573f,-0.185212f,
  -0.118248f,-0.120176f,0.285625f,0.378857f,-0.322169f,0.477573f,-0.066881f,-0.070128f,0.186504f,0.030162f,-0.014177f,0.185941f;

  /* End of MPC initialization */
  step = 0;
  traj_iter = 0;
  mpc_has_run = false;
  controllerPidInit();   // Used during the synchronized activation hold/fallback.
  // Bring up the AI-deck corner UART link once (re-selecting the controller must not
  // start a second RX task / re-init the UART).
  static bool s_gate_link_init = false;
  if (!s_gate_link_init) {
    gate8LinkInit();
    s_gate_link_init = true;
  }
  q_ref0 = qeye();  // level reference until the first updateHorizonReference
  memset(&mpc_setpoint_data, 0, sizeof(mpc_setpoint_data));
  mpc_setpoint_data.mode.x = modeAbs;
  mpc_setpoint_data.mode.y = modeAbs;
  mpc_setpoint_data.mode.z = modeAbs;
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
    const uint32_t task_start_us = usecTimestamp();

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

    updateInitialState(&sensors_task, &state_task);
    if (reset_requested) {
      resetMpcWarmStart();
      clearPerceptionPlaneLatch();
    }

    // Perception runs in the MPC task, faster than the AI-deck map producer.
    pollGateVision(&state_task, &sensors_task);
    pollPerceptionDanger(&state_task);
    fuseGateIntoEkf(&state_task);

    updateHorizonReference(&setpoint_task);

    // Multiplicative attitude error in the reference frame: q_err = q_ref0^-1 (x) q_meas.
    struct quat q_err = qqmul(qinv(q_ref0), q_meas);
    phi = quat2rp(q_err);
    x0(3) = phi.x;
    x0(4) = phi.y;
    x0(5) = phi.z;

    updateObstacleHalfspace(&state_task);
    updateTestPlaneHalfspace();
    updatePerceptionPositionHalfspace(&state_task);
    tiny_UpdateLinearCost(&work);
    stgs.max_iter = testPlaneEnable
        ? std::max(1, std::min((int)testPlaneMaxIter, 20))
        : 5;
    const uint32_t mpc_start_us = usecTimestamp();
    tiny_SolveAdmm(&work);
    g_mpc_solve_us = usecTimestamp() - mpc_start_us;
    g_mpc_iter = (uint8_t)info.iter;
    // Explicit residual computation is cheap and remains active even though
    // early termination/verbose output are disabled.
    ComputePrimalResidual(&work);
    ComputeDualResidual(&work);
    g_mpc_primal_residual = info.pri_res;
    g_mpc_dual_residual = info.dua_res;
    g_mpc_rho = work.rho;
    updatePerceptionViolationDiagnostics(&state_task);

    result = info.status_val * info.iter;

    const int output_k =
        std::max(1, std::min(MPC_CASCADE_OUTPUT_K, NHORIZON - 1));
    const float raw_output_violation =
        knotHalfspaceViolation(output_k, Xhrz[output_k]);
    const bool has_output_halfspace = isfinite(raw_output_violation);
    const bool raw_output_safe =
        Xhrz[output_k].allFinite() &&
        (!has_output_halfspace ||
         raw_output_violation <= MPC_OUTPUT_MAX_VIOLATION_M);
    VectorNf executed_output_state =
        raw_output_safe ? Xhrz[output_k] : ZX_new[output_k];
    if (!executed_output_state.allFinite()) {
      executed_output_state = x0;
    }
    g_mpc_output_safe = raw_output_safe ? 1 : 0;
    g_mpc_output_violation =
        has_output_halfspace ? raw_output_violation : 0.0f;
    g_mpc_output_position[0] = executed_output_state(0);
    g_mpc_output_position[1] = executed_output_state(1);
    g_mpc_output_position[2] = executed_output_state(2);
    g_mpc_output_yaw_deg = yawUseRef
        ? yawRefDeg
        : quat2rpy(q_ref0).z * (180.0f / M_PI_F);
    updateTestPlaneViolationDiagnostics(&state_task);

    setpoint_t next_sp;
    memset(&next_sp, 0, sizeof(next_sp));
    next_sp.mode.x = modeAbs;
    next_sp.mode.y = modeAbs;
    next_sp.mode.z = modeAbs;
    next_sp.mode.yaw = modeAbs;
    next_sp.position.x = executed_output_state(0);
    next_sp.position.y = executed_output_state(1);
    next_sp.position.z = executed_output_state(2);
    next_sp.attitude.yaw = g_mpc_output_yaw_deg;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&mpc_setpoint_data, &next_sp, sizeof(mpc_setpoint_data));
    mpc_has_run = true;
    mpc_last_finish_tick = xTaskGetTickCount();
    xSemaphoreGive(dataMutex);

    g_mpc_task_us = usecTimestamp() - task_start_us;
    g_mpc_stack_free_words = uxTaskGetStackHighWaterMark(NULL);

    // If this job crossed one or more 50 Hz release boundaries, the binary
    // semaphore contains a stale wake. Drop it instead of immediately running
    // a catch-up solve: back-to-back solves can prevent the idle task from
    // feeding the hardware watchdog.
    while (xSemaphoreTake(runTaskSemaphore, 0) == pdTRUE) {
      g_mpc_stale_wakes++;
    }
    // Guarantee a blocked interval after every solve so the idle task has an
    // opportunity to run even under sustained MPC load.
    vTaskDelay(1);
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
  uint32_t last_finish_tick_snapshot = 0;
  uint32_t activate_tick_snapshot = controller_activate_tick;
  setpoint_t output_sp;
  memcpy(&output_sp, &hold_sp, sizeof(output_sp));
  const bool controller_reactivated =
      (last_controller_tick == 0) || ((tick - last_controller_tick) > M2T(200));

  if (xSemaphoreTake(dataMutex, M2T(2)) == pdTRUE) {
    memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
    memcpy(&sensors_data, sensors, sizeof(sensorData_t));
    memcpy(&state_data, state, sizeof(state_t));
    if (controller_reactivated) {
      controller_activate_tick = tick;
      activate_tick_snapshot = controller_activate_tick;
      mpc_has_run = false;
      mpc_last_finish_tick = 0;
      mpc_reset_requested = true;
    }
    has_run_snapshot = mpc_has_run;
    last_finish_tick_snapshot = mpc_last_finish_tick;
    memcpy(&output_sp, &mpc_setpoint_data, sizeof(output_sp));
    xSemaphoreGive(dataMutex);
  } else {
    controllerPid(control, &hold_sp, sensors, state, tick);
    return;
  }
  last_controller_tick = tick;

  if (controller_reactivated) {
    DEBUG_PRINT("OOT activated: hold pos=(%.2f,%.2f,%.2f)\n",
                (double)state->position.x,
                (double)state->position.y,
                (double)state->position.z);
  }

  if (RATE_DO_EXECUTE(MPC_RATE, tick)) {
    if (xSemaphoreGive(runTaskSemaphore) != pdTRUE) {
      g_mpc_dropped_wakes++;
    }
  }

  // Known-good integrated output path: the MPC supplies a guarded receding-horizon
  // position/yaw setpoint and the stock PID supplies all low-level control.
  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    if (obsPidPassthrough) {
      controllerPid(control, setpoint, sensors, state, tick);
      return;
    }
    // mpc_last_finish_tick is sampled with xTaskGetTickCount() in the worker.
    // Use that same FreeRTOS clock here; the stabilizer's `tick` has a separate
    // epoch and subtracting the two wraps the age to ~UINT32_MAX.
    const uint32_t rtos_now_tick = xTaskGetTickCount();
    const uint32_t plan_age_ticks =
        last_finish_tick_snapshot
            ? (rtos_now_tick - last_finish_tick_snapshot)
            : UINT32_MAX;
    const bool plan_fresh =
        has_run_snapshot &&
        last_finish_tick_snapshot &&
        plan_age_ticks <= M2T(TINYMPC_PLAN_TIMEOUT_MS);
    g_mpc_plan_age_ms =
        plan_age_ticks == UINT32_MAX
            ? 0
            : plan_age_ticks * portTICK_PERIOD_MS;
    const bool hold_output =
        (!plan_fresh) || ((tick - activate_tick_snapshot) < M2T(250));

    if (setpoint->mode.z == modeDisable && !hold_output) {
      // Not commanded to fly -> motors off.
      control->normalizedForces[0] = 0.0f;
      control->normalizedForces[1] = 0.0f;
      control->normalizedForces[2] = 0.0f;
      control->normalizedForces[3] = 0.0f;
      control->controlMode = controlModePWM;
    } else {
      memset(&mpc_setpoint_pid, 0, sizeof(mpc_setpoint_pid));
      mpc_setpoint_pid.mode.x = modeAbs;
      mpc_setpoint_pid.mode.y = modeAbs;
      mpc_setpoint_pid.mode.z = modeAbs;
      mpc_setpoint_pid.mode.yaw = modeAbs;
      if (hold_output) {
        memcpy(&mpc_setpoint_pid, &hold_sp, sizeof(mpc_setpoint_pid));
      } else {
        memcpy(&mpc_setpoint_pid, &output_sp, sizeof(mpc_setpoint_pid));
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
