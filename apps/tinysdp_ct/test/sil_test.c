/*
 * Software-in-the-loop test of the Crazyflie app logic (src/tsdp_app.c, included verbatim)
 * with the real planner (tsdp_planner.cpp), stubbed firmware headers (test/stubs) and a
 * simulated clock. The drone tracks the reference perfectly (this checks the planning,
 * tail/braking and timing logic, not flight dynamics). Obstacles move on straight lines.
 *
 * Every scenario runs twice, with the commitment rule (tsdp.commit = 1) and with first-certified
 * execution (tsdp.commit = 0). Exit code 0 iff every run reaches the goal with the reference clear of
 * every obstacle, and, with the rule, Proposition 1 holds under this perfect tracking: the promised
 * arrival never moves later within a commitment, and a run with a single commitment arrives by its
 * first promise (one period of slack) with executed effort at most the first plan's (2% slack for the
 * 1 ms integration of the sampled acceleration).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static uint64_t sim_us = 0;          /* simulated time at the start of the current frame */
static uint64_t wall_mark_ns = 0;    /* host time when the frame started (planning cost counts) */
static uint64_t wall_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec; }
uint64_t usecTimestamp(void) { return sim_us + (wall_ns() - wall_mark_ns) / 1000u; }

#include "../src/tsdp_app.c"

typedef struct {
  const char* name; float start[3], goal[3]; int n; tsdp_sphere_t obs[TSDP_WORLD];
  float budget;          /* planning budget (ms); 0 = default 30 */
  float appear_s;        /* obstacle n-1 only becomes known (and present) after this time; 0 = always */
  float kick_s;          /* at this time the estimate jumps 0.2 m sideways (tracking error); 0 = never */
  int expect_reach;      /* 0: only safety is required (e.g. planning disabled) */
} scenario_t;

static void true_obstacle(const tsdp_sphere_t* o, float t, float c[3]) { for (int i = 0; i < 3; ++i) c[i] = o->center[i] + t * o->velocity[i]; }

static int run_scenario(const scenario_t* sc, int rule) {
  memset(&active, 0, sizeof(active)); memset(&pending, 0, sizeof(pending));
  commit_rule = (uint8_t)rule; commit_valid = false; after_reset_plan = false; log_recommits = 0;
  uint64_t first_promise_us = 0, last_promise_us = 0; uint32_t seen_commits = 0;
  int promise_moved_later = 0; float J0 = -1.0f, executed_effort = 0.0f;
  memset(obstacle_valid, 0, sizeof(obstacle_valid));
  for (int c = 0; c < 3; ++c) goal[c] = sc->goal[c];
  run = 1; log_frames = log_failures = log_late = 0;
  budget_ms = sc->budget > 0 ? sc->budget : 30.0f;
  initialized = false;
  if (!ensure_init()) { printf("%s: init failed\n", sc->name); return 1; }
  float x[3] = {sc->start[0], sc->start[1], sc->start[2]}, v[3] = {0, 0, 0};
  int decisions[4] = {0, 0, 0, 0}, reached = 0;
  float min_clear = 1e9f, max_plan_ms = 0, sum_plan_ms = 0;
  /* Flight-log-format CSV (same columns as host/fly_tinysdp_ct.py) for host/analyze_flight.py. */
  char path[256]; snprintf(path, sizeof(path), "%s/sil_%s%s.csv", getenv("SIL_CSV_DIR") ? getenv("SIL_CSV_DIR") : "/tmp", sc->name, rule ? "" : "_first");
  for (char* c = path + strlen(path) - strlen(sc->name) - 4 - (rule ? 0 : 6); *c; ++c) if (*c == ' ') *c = '_';
  FILE* log_csv = fopen(path, "w");
  if (log_csv) {
    fprintf(log_csv, "host_t,cf_ts_ms,run,t_run,stateEstimate.x,stateEstimate.y,stateEstimate.z,stateEstimate.vx,stateEstimate.vy,stateEstimate.vz,"
                     "tsdp.spx,tsdp.spy,tsdp.spz,tsdp.trackErr,tsdp.rho,tsdp.decision,tsdp.frames,tsdp.planUs,tsdp.frameUs,tsdp.iters,"
                     "tsdp.attempts,tsdp.reset,tsdp.obs,tsdp.obsAgeMs,tsdp.late,tsdp.failures,tsdp.stackPl,tsdp.stackSp,pm.vbat,tsdp.base");
    for (int i = 0; i < sc->n; ++i) fprintf(log_csv, ",obs%d_x,obs%d_y,obs%d_z,obs%d_r", i, i, i, i);
    fprintf(log_csv, "\n");
  }
  for (int frame = 0; frame < 25 * 20; ++frame) {
    sim_us = (uint64_t)frame * PERIOD_MS * 1000u;
    const float t = (float)sim_us * 1e-6f;
    for (int i = 0; i < sc->n; ++i) {               /* host streams the current obstacle states */
      if (sc->appear_s > 0 && i == sc->n - 1 && t < sc->appear_s) { obstacle_valid[i] = false; continue; }
      true_obstacle(&sc->obs[i], t, obstacles[i].center);
      memcpy(obstacles[i].velocity, sc->obs[i].velocity, sizeof(obstacles[i].velocity));
      obstacles[i].radius = sc->obs[i].radius;
      obstacle_stamp_us[i] = sim_us; obstacle_valid[i] = true;
    }
    wall_mark_ns = wall_ns();
    float est[3] = {x[0], x[1], x[2]};
    if (sc->kick_s > 0 && fabsf(t - sc->kick_s) < 0.02f) est[1] += 0.2f;   /* one-frame tracking error */
    plan_frame(sim_us, est, v);
    if (log_promise_ms >= 0) {                      /* the promise as an absolute time */
      const uint64_t promise_us = sim_us + (uint64_t)PERIOD_MS * 1000u + (uint64_t)log_promise_ms * 1000u;
      if (log_recommits != seen_commits) {          /* a new promise */
        if (seen_commits == 0) { first_promise_us = promise_us; J0 = traj_effort(&pending.traj); }
        seen_commits = log_recommits;
      } else if (promise_us > last_promise_us + 1000u) ++promise_moved_later;   /* 1 ms of ms-rounding */
      last_promise_us = promise_us;
    }
    const float ms = (float)log_plan_us * 1e-3f;
    if (ms > max_plan_ms) max_plan_ms = ms;
    sum_plan_ms += ms;
    ++decisions[log_decision];
    if (getenv("SIL_TRACE") && log_decision != 0) {
      float oc[3] = {0, 0, 0}; if (sc->n) true_obstacle(&sc->obs[sc->n - 1], t, oc);
      printf("    t=%.2f decision %d x=(%.3f %.3f %.3f) v=(%.3f %.3f %.3f) iters %d attempts %d plan %.2f ms obs=(%.2f %.2f) reset %d\n",
             t, log_decision, x[0], x[1], x[2], v[0], v[1], v[2], log_iterations, log_attempts, (float)log_plan_us * 1e-3f, oc[0], oc[1], log_reset);
    }
    /* Execute the period [t, t + 40 ms) at 1 ms, tracking the reference perfectly. */
    for (int k = 0; k < PERIOD_MS; ++k) {
      const uint64_t now = sim_us + (uint64_t)k * 1000u;
      float p[3], vv[3], a[3];
      if (!reference_at(now, p, vv, a)) continue;
      for (int c = 0; c < 3; ++c) { x[c] = p[c]; v[c] = vv[c]; }
      executed_effort += (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) * 1e-3f;
      if (log_csv && k % 10 == 0) {   /* 100 Hz rows, perfect tracking: estimate = setpoint */
        const float tr = (float)now * 1e-6f;
        fprintf(log_csv, "%.6f,%u,1,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,0,%.5f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,1000,400,3.9,%u",
                tr, (unsigned)(now / 1000), tr, p[0], p[1], p[2], vv[0], vv[1], vv[2], p[0], p[1], p[2], log_rho, log_decision,
                (unsigned)log_frames, (unsigned)log_plan_us, (unsigned)log_frame_us, log_iterations, log_attempts, log_reset, log_obstacles,
                log_obs_age_ms, (unsigned)log_late, (unsigned)log_failures, log_base);
        for (int i = 0; i < sc->n; ++i) {
          const int present = !(sc->appear_s > 0 && i == sc->n - 1 && tr < sc->appear_s);
          float oc[3]; true_obstacle(&sc->obs[i], tr, oc);
          if (present) fprintf(log_csv, ",%.5f,%.5f,%.5f,%.4f", oc[0], oc[1], oc[2], sc->obs[i].radius);
          else fprintf(log_csv, ",,,,");
        }
        fprintf(log_csv, "\n");
      }
      for (int i = 0; i < sc->n; ++i) {
        if (sc->appear_s > 0 && i == sc->n - 1 && (float)now * 1e-6f < sc->appear_s) continue;
        float oc[3]; true_obstacle(&sc->obs[i], (float)now * 1e-6f, oc);
        const float d = sqrtf((p[0] - oc[0]) * (p[0] - oc[0]) + (p[1] - oc[1]) * (p[1] - oc[1]) + (p[2] - oc[2]) * (p[2] - oc[2]))
                        - sc->obs[i].radius - robot_radius;
        if (d < min_clear) min_clear = d;
      }
    }
    const float dg = sqrtf((x[0] - goal[0]) * (x[0] - goal[0]) + (x[1] - goal[1]) * (x[1] - goal[1]) + (x[2] - goal[2]) * (x[2] - goal[2]));
    if (dg < 0.05f && sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) < 0.05f) { reached = frame + 1; break; }
  }
  if (log_csv) fclose(log_csv);
  const int frames = (int)log_frames;
  const float arrival_s = reached * PERIOD_MS * 1e-3f, promise_s = (float)first_promise_us * 1e-6f;
  const float effort_ratio = J0 > 0 ? executed_effort / J0 : 0.0f;
  int promise_failed = 0;
  if (rule) {
    if (promise_moved_later) promise_failed = 1;
    if (seen_commits == 1 && reached && arrival_s > promise_s + PERIOD_MS * 1e-3f + 1e-4f) promise_failed = 1;
    if (seen_commits == 1 && reached && effort_ratio > 1.02f) promise_failed = 1;
  }
  printf("%-17s %-6s reached %s (%.2f s)  min clearance %+.3f m  new/tail/brake/fail %3d/%3d/%d/%d  "
         "promises %u, first %.2f s, moved later %d  effort/J0 %.3f  plan ms mean %.2f max %.2f (host)  late %u%s\n",
         sc->name, rule ? "rule" : "first", reached ? "yes" : "NO", arrival_s, min_clear,
         decisions[0], decisions[1], decisions[2], decisions[3], (unsigned)seen_commits, promise_s, promise_moved_later, effort_ratio,
         sum_plan_ms / (frames ? frames : 1), max_plan_ms, (unsigned)log_late, promise_failed ? "  PROMISE BROKEN" : "");
  return !((reached || !sc->expect_reach) && min_clear >= 0.0f) || promise_failed;
}

int main(void) {
  printf("planner object %u bytes\n", tsdp_footprint());
  enum { N = 7 };
  scenario_t s[N];
  memset(s, 0, sizeof(s));
  s[0] = (scenario_t){"empty", {0, 0, 0.5f}, {2.0f, 0, 0.5f}, 0, {{{0}}}, 0, 0, 0, 1};
  s[1] = (scenario_t){"static sphere", {0, 0, 0.5f}, {2.0f, 0, 0.5f}, 1, {{{1.0f, 0.02f, 0.5f}, {0, 0, 0}, 0.25f}}, 0, 0, 0, 1};
  s[2] = (scenario_t){"crossing sphere", {0, 0, 0.5f}, {2.0f, 0, 0.5f}, 1, {{{1.0f, -1.2f, 0.5f}, {0, 0.6f, 0}, 0.2f}}, 0, 0, 0, 1};
  s[3] = (scenario_t){"three spheres", {0, 0, 0.5f}, {2.5f, 0, 0.5f}, 3,
                      {{{0.8f, 0.1f, 0.5f}, {0, 0, 0}, 0.2f}, {{1.6f, -0.3f, 0.5f}, {0, 0.2f, 0}, 0.2f}, {{1.6f, 0.6f, 0.5f}, {0, -0.1f, 0}, 0.15f}}, 0, 0, 0, 1};
  s[4] = (scenario_t){"no planning time", {0, 0, 0.5f}, {2.0f, 0, 0.5f}, 1, {{{1.0f, 0.02f, 0.5f}, {0, 0, 0}, 0.25f}}, 0.05f, 0, 0, 0};
  s[5] = (scenario_t){"sudden obstacle", {0, 0, 0.5f}, {2.5f, 0, 0.5f}, 1, {{{1.6f, 0.0f, 0.5f}, {0, 0, 0}, 0.25f}}, 0, 1.0f, 0, 1};
  s[6] = (scenario_t){"tracking kick", {0, 0, 0.5f}, {2.0f, 0, 0.5f}, 1, {{{1.0f, 0.02f, 0.5f}, {0, 0, 0}, 0.25f}}, 0, 0, 1.0f, 1};
  int failures = 0;
  for (int i = 0; i < N; ++i) failures += run_scenario(&s[i], 1);
  for (int i = 0; i < N; ++i) failures += run_scenario(&s[i], 0);
  printf("%s: %d of %d runs failed\n", failures ? "FAIL" : "PASS", failures, 2 * N);
  return failures != 0;
}
