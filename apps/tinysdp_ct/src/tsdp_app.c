/*
 * TinySDP-CT on the Crazyflie: continuous-time SDP planning with an on-board checker.
 *
 * Planner task (25 Hz loop = 40 ms control period, planning budget tsdp.budgetMs, default 30 ms):
 *   at frame k, plan from the reference state at the next period boundary t_{k+1}, and execute
 *   from t_{k+1} the first that works (same decision rule as the simulator):
 *     0 new plan        - TinySDP-CT plan, checked on board in continuous time; with the commitment
 *                         rule (tsdp.commit = 1, default) it must also arrive no later than the
 *                         committed plan and cost no more than its retained tail;
 *     1 retained tail   - the remainder of the active trajectory, if it re-certifies now;
 *     2 braking         - a straight-line stop from the reference state, if it certifies;
 *     3 failure         - keep the active reference and log it (it is no longer certified).
 * Setpoint task (100 Hz): samples the active trajectory into position/velocity/acceleration
 *   setpoints (world frame) for the on-board controller (use Mellinger: stabilizer.controller = 2).
 *
 * Obstacles arrive over the app channel, one moving sphere per packet (tsdp_obstacle_packet_t);
 * their centers are extrapolated from the receive time with the given velocity. The goal and
 * switches are parameters (group tsdp). Nothing is commanded until tsdp.run = 1.
 *
 * Commitment rule (the simulator's --acceptance arrival --strict-decrease --commit-executed): the
 * executed goal-reaching plan is the commitment, with absolute arrival A = start + duration. While its
 * retained tail still certifies, a new plan is planned with horizon <= A - t_{k+1} and replaces the
 * tail only if it arrives no later than A and its effort (integral of |acceleration|^2) is at most the
 * tail's. Under exact tracking the robot then reaches the goal by the first promise and spends no more
 * than the first plan's effort. Braking, a reset or a tail that no longer certifies ends the
 * commitment, and the next certified plan makes a new one. tsdp.commit = 0 restores first-certified
 * execution (any certified plan replaces the tail, fresh horizon every frame).
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "app_channel.h"
#include "commander.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "log.h"
#include "param.h"
#include "task.h"
#include "usec_time.h"

#include "tsdp_planner.h"

#define DEBUG_MODULE "TSDP"
#define TSDP_WORLD 8                 /* obstacle slots (must equal TSDP_MAX_WORLD in tsdp_planner.cpp) */
#define PERIOD_MS 40                 /* 25 Hz control period */
#define SETPOINT_MS 10               /* 100 Hz setpoint stream */

/* ---------------------------------------------------------------- parameters */
static uint8_t run = 0;              /* 1: plan and command; 0: release the commander */
static float goal[3] = {0.0f, 0.0f, 0.5f};
static float budget_ms = 30.0f;      /* planning budget inside the 40 ms period */
static float max_velocity = 1.5f, max_acceleration = 4.0f;
static float cruise = 0.5f;          /* horizon = distance / cruise, clamped to [1, 4] s; peak speed ~1.5x cruise */
static float safety_margin = 0.05f;  /* required certified clearance (m) */
static float robot_radius = 0.08f;   /* added to every obstacle radius */
static float reset_distance = 0.10f; /* replan from the estimate when |estimate - reference| exceeds this */
static float obstacle_timeout_s = 0.5f;
static uint8_t commit_rule = 1;      /* 1: commitment rule (arrival + effort); 0: first certified */

/* ---------------------------------------------------------------- logs */
static uint32_t log_plan_us = 0;     /* wall-clock planning time of the last frame */
static uint8_t log_decision = 3;
static uint16_t log_iterations = 0;
static uint8_t log_attempts = 0, log_obstacles = 0, log_base = 0, log_reset = 0;
static uint32_t log_frames = 0, log_failures = 0, log_late = 0;
static float log_ref[3];
static float log_rho = -1.0f;        /* certified clearance (m) of the trajectory chosen this frame; -1: none */
static uint32_t log_frame_us = 0;    /* wall-clock time of the whole frame (plan, checks, fallbacks) */
static uint16_t log_obs_age_ms = 0;  /* age of the oldest obstacle used this frame */
static float log_sp[3];              /* current setpoint (100 Hz) */
static float log_track_err = 0.0f;   /* |estimate - setpoint| (100 Hz) */
static uint16_t log_stack_planner = 0, log_stack_setpoint = 0;   /* free stack words (high-water mark) */
static uint32_t footprint_bytes = 0;
static uint8_t log_reject = 0;       /* certified new plan rejected by the rule: 1 later than promised, 2 costlier than the tail */
static int32_t log_promise_ms = -1;  /* committed arrival minus the switch time (ms); -1: no commitment */
static uint32_t log_recommits = 0;   /* promises made: the first, and after each braking / reset / lost tail */

/* ---------------------------------------------------------------- obstacles (app channel) */
typedef struct __attribute__((packed)) {
  uint8_t index;                     /* 0..TSDP_WORLD-1; 0xFF clears all */
  float center[3], velocity[3], radius;
} tsdp_obstacle_packet_t;            /* 29 bytes <= APPCHANNEL_MTU */

static tsdp_sphere_t obstacles[TSDP_WORLD];
static uint64_t obstacle_stamp_us[TSDP_WORLD];
static bool obstacle_valid[TSDP_WORLD];

/* ---------------------------------------------------------------- active trajectory (shared) */
typedef struct {
  bool valid;
  uint64_t start_us;                 /* time of s = 0 of piece 0 */
  tsdp_traj_t traj;
} active_t;

static active_t active, pending;     /* pending becomes active at pending.start_us */
/* Commitment (planner task only): absolute arrival of the committed goal-reaching plan. */
static bool commit_valid = false;
/* The previous frame replanned from the estimate after a reset. Its plan starts one period late, so
 * during that period the robot still follows the older reference, and comparing the estimate with the
 * new plan one frame later would trigger another reset, and so on (two branches leapfrogging). The
 * reset check is skipped for that one frame. */
static bool after_reset_plan = false;
static uint64_t commit_arrival_us = 0;


/* Seconds from `from_us` to `to_us`, signed (an unsigned difference would wrap before a plan starts). */
static float seconds_between(uint64_t from_us, uint64_t to_us) { return (float)((int64_t)to_us - (int64_t)from_us) * 1e-6f; }

/* Position, velocity and acceleration of a piecewise cubic Bernstein trajectory at time tau (s). */
static void traj_eval(const tsdp_traj_t* t, float tau, float p[3], float v[3], float a[3]) {
  int k = 0;
  if (tau < 0) tau = 0;
  while (k + 1 < t->pieces && tau > t->dur[k]) { tau -= t->dur[k]; ++k; }
  const float h = t->dur[k];
  float s = h > 0 ? tau / h : 1.0f;
  if (s > 1.0f) s = 1.0f;
  const float q = 1.0f - s;
  for (int c = 0; c < 3; ++c) {
    const float* P = &t->p[k][0][c];
    const float p0 = P[0], p1 = P[3], p2 = P[6], p3 = P[9];   /* stride 3 floats between points */
    p[c] = q * q * q * p0 + 3 * q * q * s * p1 + 3 * q * s * s * p2 + s * s * s * p3;
    v[c] = 3.0f / h * (q * q * (p1 - p0) + 2 * q * s * (p2 - p1) + s * s * (p3 - p2));
    a[c] = 6.0f / (h * h) * (q * (p2 - 2 * p1 + p0) + s * (p3 - 2 * p2 + p1));
  }
  if (tau > h && k == t->pieces - 1) { v[0] = v[1] = v[2] = 0; a[0] = a[1] = a[2] = 0; }
}

/* Total duration (s) of a piecewise trajectory. */
static float traj_duration(const tsdp_traj_t* t) {
  float d = 0.0f;
  for (int k = 0; k < t->pieces; ++k) d += t->dur[k];
  return d;
}

/* Effort: integral of |acceleration|^2 over the trajectory. On a cubic piece of duration h the
 * acceleration is linear between a0 = 6/h^2 (P2 - 2 P1 + P0) and a1 = 6/h^2 (P3 - 2 P2 + P1), so the
 * piece contributes h (|a0|^2 + a0.a1 + |a1|^2) / 3 exactly. */
static float traj_effort(const tsdp_traj_t* t) {
  float J = 0.0f;
  for (int k = 0; k < t->pieces; ++k) {
    const float h = t->dur[k];
    if (!(h > 0.0f)) continue;
    float aa = 0, ab = 0, bb = 0;
    for (int c = 0; c < 3; ++c) {
      const float a0 = 6.0f / (h * h) * (t->p[k][2][c] - 2 * t->p[k][1][c] + t->p[k][0][c]);
      const float a1 = 6.0f / (h * h) * (t->p[k][3][c] - 2 * t->p[k][2][c] + t->p[k][1][c]);
      aa += a0 * a0; ab += a0 * a1; bb += a1 * a1;
    }
    J += h * (aa + ab + bb) / 3.0f;
  }
  return J;
}

/* Remainder of t after time tau, as up to TSDP_MAX_PIECES pieces (de Casteljau split). */
static bool traj_tail(const tsdp_traj_t* t, float tau, tsdp_traj_t* out) {
  int k = 0;
  while (k + 1 < t->pieces && tau >= t->dur[k]) { tau -= t->dur[k]; ++k; }
  const float h = t->dur[k];
  if (!(h > 0) || tau >= h) return false;
  const float s = tau / h;
  /* Right half of the de Casteljau split at s: r[3 - level] = tri[3 - level] after each level. */
  float r[4][3], tri[4][3];
  memcpy(tri, t->p[k], sizeof(tri));
  for (int c = 0; c < 3; ++c) r[3][c] = tri[3][c];
  for (int level = 1; level <= 3; ++level) {
    for (int i = 0; i <= 3 - level; ++i)
      for (int c = 0; c < 3; ++c) tri[i][c] = (1 - s) * tri[i][c] + s * tri[i + 1][c];
    for (int c = 0; c < 3; ++c) r[3 - level][c] = tri[3 - level][c];
  }
  memcpy(out->p[0], r, sizeof(r));
  out->dur[0] = (1 - s) * h;
  out->pieces = 1;
  for (int j = k + 1; j < t->pieces && out->pieces < TSDP_MAX_PIECES; ++j) {
    memcpy(out->p[out->pieces], t->p[j], sizeof(t->p[j]));
    out->dur[out->pieces] = t->dur[j];
    ++out->pieces;
  }
  return true;
}

/* Straight-line braking at max_acceleration from (x, v), then hold for 1 s. */
static void braking(const float x[3], const float v[3], tsdp_traj_t* out) {
  const float speed = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  const float decel = 0.8f * max_acceleration;
  float stop = speed / decel;
  if (stop < 1e-3f) stop = 1e-3f;
  out->pieces = 2;
  for (int c = 0; c < 3; ++c) {
    const float a = speed > 1e-6f ? -v[c] / speed * decel : 0.0f;
    const float end = x[c] + v[c] * stop + 0.5f * a * stop * stop;
    out->p[0][0][c] = x[c];
    out->p[0][1][c] = x[c] + v[c] * stop / 3;
    out->p[0][2][c] = x[c] + 2 * v[c] * stop / 3 + a * stop * stop / 6;
    out->p[0][3][c] = end;
    for (int i = 0; i < 4; ++i) out->p[1][i][c] = end;
  }
  out->dur[0] = stop; out->dur[1] = 1.0f;
}

/* Obstacles predicted to time `at_us`; returns the count (at least 1: a far sphere if none). */
static int obstacles_at(uint64_t at_us, const float near[3], tsdp_sphere_t* out) {
  int n = 0;
  for (int i = 0; i < TSDP_WORLD; ++i) {
    if (!obstacle_valid[i]) continue;
    const float age = seconds_between(obstacle_stamp_us[i], at_us);
    if (age > obstacle_timeout_s) continue;                    /* stale: excluded */
    if (age * 1000.0f > (float)log_obs_age_ms) log_obs_age_ms = (uint16_t)(age * 1000.0f);
    for (int c = 0; c < 3; ++c) {
      out[n].center[c] = obstacles[i].center[c] + age * obstacles[i].velocity[c];
      out[n].velocity[c] = obstacles[i].velocity[c];
    }
    out[n].radius = obstacles[i].radius + robot_radius;
    ++n;
  }
  if (n == 0) {                                                /* planner needs one sphere: 10 m to the side */
    out[0].center[0] = near[0]; out[0].center[1] = near[1] + 10.0f; out[0].center[2] = near[2];
    out[0].velocity[0] = out[0].velocity[1] = out[0].velocity[2] = 0; out[0].radius = 0.1f;
    n = 1;
  }
  return n;
}

static void obstacle_receiver(void) {
  tsdp_obstacle_packet_t packet;
  while (appchannelReceiveDataPacket(&packet, sizeof(packet), 0) == sizeof(packet)) {
    if (packet.index == 0xFF) { memset(obstacle_valid, 0, sizeof(obstacle_valid)); continue; }
    if (packet.index >= TSDP_WORLD) continue;
    memcpy(obstacles[packet.index].center, packet.center, sizeof(packet.center));
    memcpy(obstacles[packet.index].velocity, packet.velocity, sizeof(packet.velocity));
    obstacles[packet.index].radius = packet.radius;
    obstacle_stamp_us[packet.index] = usecTimestamp();
    obstacle_valid[packet.index] = true;
  }
}

static bool state_estimate(float x[3], float v[3]) {
  static logVarId_t id[6];
  static bool have = false;
  if (!have) {
    const char* names[6] = {"x", "y", "z", "vx", "vy", "vz"};
    for (int i = 0; i < 6; ++i) id[i] = logGetVarId("stateEstimate", names[i]);
    have = true;
  }
  for (int i = 0; i < 3; ++i) { x[i] = logGetFloat(id[i]); v[i] = logGetFloat(id[3 + i]); }
  return true;
}

/* ---------------------------------------------------------------- reference sampling */
/* Reference at time `now` from the active trajectory (activating the pending one when due). */
static bool reference_at(uint64_t now, float p[3], float v[3], float acc[3]) {
  taskENTER_CRITICAL();
  if (pending.valid && now >= pending.start_us) { active = pending; pending.valid = false; }
  const active_t a = active;
  taskEXIT_CRITICAL();
  if (!a.valid) return false;
  traj_eval(&a.traj, seconds_between(a.start_us, now), p, v, acc);
  return true;
}

/* ---------------------------------------------------------------- one planning frame */
static bool initialized = false;
static float settings_used[3];

static bool ensure_init(void) {
  if (initialized && settings_used[0] == max_velocity && settings_used[1] == max_acceleration && settings_used[2] == safety_margin)
    return true;
  initialized = tsdp_init(max_velocity, max_acceleration, safety_margin);
  settings_used[0] = max_velocity; settings_used[1] = max_acceleration; settings_used[2] = safety_margin;
  return initialized;
}

/* Plans at frame_us from the state estimate (est_x, est_v); the result executes from frame_us + period. */
static void plan_frame(uint64_t frame_us, const float est_x[3], const float est_v[3]) {
  static tsdp_sphere_t world[TSDP_WORLD];
  static tsdp_traj_t plan, tail, stop;
  const uint64_t switch_us = frame_us + (uint64_t)PERIOD_MS * 1000u;   /* plan executes from the next period */
  float x[3], v[3], acc[3];

  /* Start state: the active reference at the switch time, unless the estimate has left it.
   * The previous frame's plan is due exactly now: activate it first, so it is never overwritten
   * before the setpoint task has switched to it. */
  taskENTER_CRITICAL();
  if (pending.valid && frame_us >= pending.start_us) { active = pending; pending.valid = false; }
  const active_t a = pending.valid ? pending : active;
  taskEXIT_CRITICAL();
  log_reset = 0;
  if (a.valid) {
    traj_eval(&a.traj, seconds_between(a.start_us, switch_us), x, v, acc);
    float now_ref[3], vr[3], ar[3];
    traj_eval(&a.traj, seconds_between(a.start_us, frame_us), now_ref, vr, ar);
    const float d = sqrtf((est_x[0] - now_ref[0]) * (est_x[0] - now_ref[0]) + (est_x[1] - now_ref[1]) * (est_x[1] - now_ref[1])
                          + (est_x[2] - now_ref[2]) * (est_x[2] - now_ref[2]));
    if (d > reset_distance && !after_reset_plan) log_reset = 1;
  }
  if (!a.valid || log_reset) {
    for (int c = 0; c < 3; ++c) { x[c] = est_x[c] + est_v[c] * PERIOD_MS * 1e-3f; v[c] = est_v[c]; }
  }
  for (int c = 0; c < 3; ++c) log_ref[c] = x[c];

  log_obs_age_ms = 0;
  const int count = obstacles_at(switch_us, x, world);
  log_obstacles = (uint8_t)count;
  const float dist = sqrtf((goal[0] - x[0]) * (goal[0] - x[0]) + (goal[1] - x[1]) * (goal[1] - x[1]) + (goal[2] - x[2]) * (goal[2] - x[2]));
  float horizon = dist / (cruise > 0.05f ? cruise : 0.05f);
  if (horizon < 1.0f) horizon = 1.0f;
  if (horizon > 4.0f) horizon = 4.0f;
  /* Commitment rule: the retained tail of the committed plan, re-checked below, is what a new plan
   * must beat, and a new plan may not end later than the committed arrival. */
  const bool have_tail = a.valid && !log_reset && traj_tail(&a.traj, seconds_between(a.start_us, switch_us), &tail);
  const bool committed = commit_rule && commit_valid && have_tail;
  bool skip_solve = false;
  if (committed) {
    const float left = seconds_between(switch_us, commit_arrival_us);
    if (left < horizon) horizon = left;
    if (horizon < 2.0f * PERIOD_MS * 1e-3f) skip_solve = true;   /* the tail ends within two periods: keep it */
  }

  tsdp_stats_t stats;
  memset(&stats, 0, sizeof(stats));
  const uint64_t deadline = frame_us + (uint64_t)(budget_ms * 1000.0f);
  bool planned = false;
  if (skip_solve) tsdp_load_world(world, count);   /* the tail is still re-checked against this frame's obstacles */
  else planned = tsdp_plan(x, v, goal, horizon, world, count, deadline, &plan, &stats);
  const uint64_t done_us = usecTimestamp();
  log_plan_us = (uint32_t)(done_us - frame_us);
  log_iterations = (uint16_t)stats.iterations;
  log_attempts = (uint8_t)stats.attempts;
  log_base = stats.base_accepted;
  ++log_frames;
  if (done_us > switch_us) ++log_late;

  active_t next = {.valid = false};
  float rho = -INFINITY;
  const float tail_rho = have_tail ? tsdp_clearance(&tail) : -INFINITY;
  const bool eligible = committed && tail_rho > -INFINITY;   /* the rule applies while the tail certifies */
  log_reject = 0;
  bool take_new = planned && done_us <= switch_us && (rho = tsdp_clearance(&plan)) > -INFINITY;
  if (take_new && eligible) {
    const uint64_t plan_arrival_us = switch_us + (uint64_t)(traj_duration(&plan) * 1e6f);
    if (plan_arrival_us > commit_arrival_us + 1000u) { take_new = false; log_reject = 1; }   /* 1 ms float slack */
    else if (traj_effort(&plan) > traj_effort(&tail)) { take_new = false; log_reject = 2; }
  }
  if (take_new) {
    next.valid = true; next.start_us = switch_us; next.traj = plan; log_decision = 0;
    if (!eligible) ++log_recommits;    /* a new promise: the first plan, or after braking / reset / a lost tail */
    commit_valid = true; commit_arrival_us = switch_us + (uint64_t)(traj_duration(&plan) * 1e6f);
  } else if (have_tail && (rho = tail_rho) > -INFINITY) {
    next.valid = true; next.start_us = switch_us; next.traj = tail; log_decision = 1;   /* keeps the committed arrival */
  } else {
    braking(x, v, &stop);
    if ((rho = tsdp_clearance(&stop)) > -INFINITY) { next.valid = true; next.start_us = switch_us; next.traj = stop; log_decision = 2; }
    else { log_decision = 3; ++log_failures; }
    commit_valid = false;              /* braking or failure: the next certified plan makes a new promise */
  }
  after_reset_plan = log_reset && log_decision == 0;
  log_promise_ms = commit_valid ? (int32_t)(((int64_t)commit_arrival_us - (int64_t)switch_us) / 1000) : -1;
  log_rho = next.valid ? rho : -1.0f;
  if (next.valid) {
    taskENTER_CRITICAL();
    pending = next;
    if (!active.valid) active = next;   /* first plan: hold its start point until the switch time */
    taskEXIT_CRITICAL();
  }
  log_frame_us = (uint32_t)(usecTimestamp() - frame_us);
}

/* ---------------------------------------------------------------- setpoint task (100 Hz) */
static void setpoint_task(void* arg) {
  (void)arg;
  setpoint_t sp;
  TickType_t wake = xTaskGetTickCount();
  bool commanding = false;
  for (;;) {
    vTaskDelayUntil(&wake, M2T(SETPOINT_MS));
    if (!run) {
      if (commanding) { commanderRelaxPriority(); commanding = false; }
      continue;
    }
    float p[3], v[3], acc[3];
    if (!reference_at(usecTimestamp(), p, v, acc)) continue;
    float est_x[3], est_v[3];
    state_estimate(est_x, est_v);
    float e2 = 0.0f;
    for (int c = 0; c < 3; ++c) { log_sp[c] = p[c]; e2 += (est_x[c] - p[c]) * (est_x[c] - p[c]); }
    log_track_err = sqrtf(e2);
    log_stack_setpoint = (uint16_t)uxTaskGetStackHighWaterMark(NULL);
    memset(&sp, 0, sizeof(sp));
    sp.mode.x = sp.mode.y = sp.mode.z = modeAbs;
    sp.mode.yaw = modeAbs;
    sp.mode.roll = sp.mode.pitch = modeDisable;
    sp.position.x = p[0]; sp.position.y = p[1]; sp.position.z = p[2];
    sp.velocity.x = v[0]; sp.velocity.y = v[1]; sp.velocity.z = v[2];
    sp.acceleration.x = acc[0]; sp.acceleration.y = acc[1]; sp.acceleration.z = acc[2];
    sp.attitude.yaw = 0;
    sp.velocity_body = false;
    commanderSetSetpoint(&sp, COMMANDER_PRIORITY_EXTRX);
    commanding = true;
  }
}

/* ---------------------------------------------------------------- planner task (25 Hz) */
void appMain(void) {
  footprint_bytes = tsdp_footprint();
  DEBUG_PRINT("TinySDP-CT planner app, planner object %lu bytes\n", (unsigned long)footprint_bytes);
  static StaticTask_t setpoint_tcb;
  static StackType_t setpoint_stack[400];
  xTaskCreateStatic(setpoint_task, "TSDPSP", 400, NULL, 2, setpoint_stack, &setpoint_tcb);
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, M2T(PERIOD_MS));
    obstacle_receiver();
    if (!run) { taskENTER_CRITICAL(); active.valid = pending.valid = false; taskEXIT_CRITICAL(); commit_valid = false; after_reset_plan = false; continue; }
    if (!ensure_init()) { DEBUG_PRINT("planner init failed\n"); run = 0; continue; }
    float est_x[3], est_v[3];
    state_estimate(est_x, est_v);
    plan_frame(usecTimestamp(), est_x, est_v);
    log_stack_planner = (uint16_t)uxTaskGetStackHighWaterMark(NULL);
  }
}

PARAM_GROUP_START(tsdp)
PARAM_ADD(PARAM_UINT8, run, &run)
PARAM_ADD(PARAM_FLOAT, gx, &goal[0])
PARAM_ADD(PARAM_FLOAT, gy, &goal[1])
PARAM_ADD(PARAM_FLOAT, gz, &goal[2])
PARAM_ADD(PARAM_FLOAT, budgetMs, &budget_ms)
PARAM_ADD(PARAM_FLOAT, vmax, &max_velocity)
PARAM_ADD(PARAM_FLOAT, amax, &max_acceleration)
PARAM_ADD(PARAM_FLOAT, cruise, &cruise)
PARAM_ADD(PARAM_FLOAT, margin, &safety_margin)
PARAM_ADD(PARAM_FLOAT, robotR, &robot_radius)
PARAM_ADD(PARAM_FLOAT, resetD, &reset_distance)
PARAM_ADD(PARAM_FLOAT, obsTimeout, &obstacle_timeout_s)
PARAM_ADD(PARAM_UINT8, commit, &commit_rule)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, footprint, &footprint_bytes)
PARAM_GROUP_STOP(tsdp)

LOG_GROUP_START(tsdp)
LOG_ADD(LOG_UINT32, planUs, &log_plan_us)
LOG_ADD(LOG_UINT8, decision, &log_decision)
LOG_ADD(LOG_UINT16, iters, &log_iterations)
LOG_ADD(LOG_UINT8, attempts, &log_attempts)
LOG_ADD(LOG_UINT8, base, &log_base)
LOG_ADD(LOG_UINT8, obs, &log_obstacles)
LOG_ADD(LOG_UINT8, reset, &log_reset)
LOG_ADD(LOG_UINT32, frames, &log_frames)
LOG_ADD(LOG_UINT32, failures, &log_failures)
LOG_ADD(LOG_UINT32, late, &log_late)
LOG_ADD(LOG_FLOAT, refx, &log_ref[0])
LOG_ADD(LOG_FLOAT, refy, &log_ref[1])
LOG_ADD(LOG_FLOAT, refz, &log_ref[2])
LOG_ADD(LOG_FLOAT, rho, &log_rho)
LOG_ADD(LOG_UINT32, frameUs, &log_frame_us)
LOG_ADD(LOG_UINT16, obsAgeMs, &log_obs_age_ms)
LOG_ADD(LOG_FLOAT, spx, &log_sp[0])
LOG_ADD(LOG_FLOAT, spy, &log_sp[1])
LOG_ADD(LOG_FLOAT, spz, &log_sp[2])
LOG_ADD(LOG_FLOAT, trackErr, &log_track_err)
LOG_ADD(LOG_UINT16, stackPl, &log_stack_planner)
LOG_ADD(LOG_UINT16, stackSp, &log_stack_setpoint)
LOG_ADD(LOG_UINT8, reject, &log_reject)
LOG_ADD(LOG_INT32, promiseMs, &log_promise_ms)
LOG_ADD(LOG_UINT32, recommits, &log_recommits)
LOG_GROUP_STOP(tsdp)
