// TinySDP-CT embedded frame planner behind a C interface (see tsdp_planner.h).
// Same planner class as the simulator's `--solver embedded` path
// (benchmarks/engine_closed_loop.cpp, EmbeddedBackend): float, no heap,
// fixed capacities chosen at compile time (TSDP_MAX_OBS, TSDP_MAX_WORLD, TSDP_MEMORY).
#include "tsdp_planner.h"

#include <tinysdp_ct_embedded/frame_planner.hpp>

#include <limits>

extern "C" {
#include "usec_time.h"
}

#ifndef TSDP_MAX_OBS
#define TSDP_MAX_OBS 4      // obstacles held by the SDP
#endif
#ifndef TSDP_MAX_WORLD
#define TSDP_MAX_WORLD 8    // obstacles the checker covers
#endif
#ifndef TSDP_MEMORY
#define TSDP_MEMORY 0       // Anderson memory (0 = smallest footprint)
#endif

namespace te = tinysdp_ct_embedded;
using Frame = te::FramePlanner<float, 3, TSDP_SEGMENTS, TSDP_MAX_OBS, TSDP_MAX_WORLD, TSDP_MEMORY, 0>;

static Frame planner;
static Frame::Sphere world[TSDP_MAX_WORLD];
static int world_count = 0;
static bool ready = false;

extern "C" bool tsdp_init(float max_velocity, float max_acceleration, float safety_margin) {
    auto s = Frame::Settings::defaults();
    s.core.max_velocity = max_velocity;
    s.core.max_acceleration = max_acceleration;
    s.core.safety_margin = safety_margin;
    s.core.solve_margin = safety_margin + 0.005f;
    ready = planner.init(s);
    return ready;
}

extern "C" bool tsdp_load_world(const tsdp_sphere_t* spheres, int count) {
    if (!ready || count < 1 || count > TSDP_MAX_WORLD) return false;
    for (int i = 0; i < count; ++i) {
        for (int c = 0; c < 3; ++c) { world[i].center[c] = spheres[i].center[c]; world[i].velocity[c] = spheres[i].velocity[c]; }
        world[i].radius = spheres[i].radius;
    }
    world_count = count;
    return true;
}

extern "C" bool tsdp_plan(const float x[3], const float v[3], const float goal[3], float horizon,
                          const tsdp_sphere_t* spheres, int count, uint64_t deadline_us,
                          tsdp_traj_t* out, tsdp_stats_t* stats) {
    if (!tsdp_load_world(spheres, count)) return false;
    te::Vec<float, 3> position, velocity, target;
    for (int c = 0; c < 3; ++c) { position[c] = x[c]; velocity[c] = v[c]; target[c] = goal[c]; }
    Frame::Plan plan;
    Frame::Stats s;
    const bool ok = planner.plan(position, velocity, target, horizon, world, count, plan, s, [deadline_us] {
        const uint64_t now = usecTimestamp();
        return now >= deadline_us ? -1.0f : float(deadline_us - now) * 1e-3f;   // ms left
    });
    stats->iterations = s.iterations; stats->attempts = s.attempts; stats->selected = s.selected;
    stats->base_accepted = s.base_accepted; stats->accepted = s.accepted;
    if (!ok) return false;
    out->pieces = TSDP_SEGMENTS;
    for (int seg = 0; seg < TSDP_SEGMENTS; ++seg) {
        for (int i = 0; i < 4; ++i)
            for (int c = 0; c < 3; ++c) out->p[seg][i][c] = plan.controls[c + 3 * (3 * seg + i)];
        out->dur[seg] = plan.segment_duration;
    }
    return true;
}

extern "C" float tsdp_clearance(const tsdp_traj_t* traj) {
    const float none = -std::numeric_limits<float>::infinity();
    if (!ready || world_count < 1) return none;
    float start = 0.0f, controls[12], minimum = std::numeric_limits<float>::infinity();
    for (int k = 0; k < traj->pieces; ++k) {
        for (int i = 0; i < 4; ++i)
            for (int c = 0; c < 3; ++c) controls[c + 3 * i] = traj->p[k][i][c];
        if (!(traj->dur[k] > 0.0f)) return none;
        const float c = planner.clearance(controls, 1, traj->dur[k], start);
        if (!(c > none)) return none;
        if (c < minimum) minimum = c;
        start += traj->dur[k];
    }
    return minimum;
}

extern "C" bool tsdp_certified(const tsdp_traj_t* traj) {
    return tsdp_clearance(traj) > -std::numeric_limits<float>::infinity();
}

extern "C" uint32_t tsdp_footprint(void) { return uint32_t(Frame::footprint()); }
