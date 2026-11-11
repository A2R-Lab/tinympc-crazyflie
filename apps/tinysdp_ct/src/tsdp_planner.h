// C interface to the TinySDP-CT embedded frame planner (include/tinysdp_ct_embedded/),
// so the firmware glue (tsdp_app.c) can stay in C.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSDP_SEGMENTS 2
#define TSDP_POINTS (3 * TSDP_SEGMENTS + 1)   // control points of the two C1 cubic segments
#define TSDP_MAX_PIECES 3                     // active trajectory: up to 3 cubic pieces

// A moving sphere: center at the planning instant, constant velocity, radius (robot radius included).
typedef struct { float center[3], velocity[3], radius; } tsdp_sphere_t;

// Piecewise cubic Bernstein trajectory, piece k has control points p[k][0..3] and duration dur[k].
typedef struct {
  int pieces;
  float p[TSDP_MAX_PIECES][4][3];
  float dur[TSDP_MAX_PIECES];
} tsdp_traj_t;

typedef struct {
  int iterations, attempts, selected;
  bool base_accepted, accepted;
} tsdp_stats_t;

bool tsdp_init(float max_velocity, float max_acceleration, float safety_margin);

// Plans from (x, v) to goal (zero terminal velocity) over `horizon` seconds against
// `count` spheres; stops when usecTimestamp() passes deadline_us. Returns a checked plan.
bool tsdp_plan(const float x[3], const float v[3], const float goal[3], float horizon,
               const tsdp_sphere_t* spheres, int count, uint64_t deadline_us,
               tsdp_traj_t* out, tsdp_stats_t* stats);

// Loads the spheres the checker uses (tsdp_certified / tsdp_clearance) without planning, for a
// period that only re-checks the retained tail. tsdp_plan() loads them too.
bool tsdp_load_world(const tsdp_sphere_t* spheres, int count);

// On-board check of a stored trajectory against the spheres loaded by the last tsdp_plan()
// call: true iff every piece is certified clear (radius + safety margin) in continuous time.
bool tsdp_certified(const tsdp_traj_t* traj);

// Certified clearance lower bound (m beyond each sphere's radius, robot radius included in the
// spheres) of a stored trajectory against the spheres loaded by the last tsdp_plan() call:
// the tracking budget rho it tolerates. -INFINITY unless certified with the safety margin.
float tsdp_clearance(const tsdp_traj_t* traj);

// Footprint of the planner object in bytes (for logging).
uint32_t tsdp_footprint(void);

#ifdef __cplusplus
}
#endif
