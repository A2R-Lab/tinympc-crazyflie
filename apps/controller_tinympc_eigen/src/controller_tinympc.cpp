/**
 * Crazyflie TinyMPC PSD Controller with Planner-Tracker Architecture
 * 
 * Uses the LIFTED PSD solver for provably-safe obstacle avoidance.
 * Matches the architecture from tiny_psd_dynamic_demo.cpp:
 * - Planner: produces collision-free lifted trajectory with PSD
 * - Tracker: tracks the plan (reuses same solver with PSD off)
 * - Certificate gating: rejects non-rank-1 solutions
 */

#include <Eigen.h>
#include <cmath>
using namespace Eigen;

// TinyMPC includes
#include "tinympc/tiny_data.hpp"
#include "tinympc/types.hpp"
#include "tinympc/admm.hpp"
#include "tinympc/tiny_api.hpp"
#include "tinympc/psd_support.hpp"
#include "psd_params.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "controller.h"
#include "controller_pid.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"
#include "cpp_compat.h"

#define DEBUG_MODULE "TINYMPC_PSD"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// System dimensions
#define NX0 4       // Base state: x, y, vx, vy
#define NU0 2       // Base input: ax, ay
#define NXL 20      // Lifted state: NX0 + NX0*NX0
#define NUL 22      // Lifted input: NU0 + NX0*NU0 + NU0*NX0 + NU0*NU0
#define NHORIZON 25
#define DT 0.02f
#define MPC_RATE RATE_50_HZ

// ========== PLANNER-TRACKER ARCHITECTURE ==========
// Plan cache: stores base trajectory from planner
static float plan_states[NHORIZON][NX0];   // Cached base states
static float plan_inputs[NHORIZON - 1][NU0]; // Cached base inputs
static int plan_start_step = 0;
static int plan_last_iter = 0;
static bool plan_valid = false;

// Replanning parameters (from demos)
#define REPLAN_STRIDE 5      // Replan every N steps
#define HORIZON_GUARD 5      // Replan if near end of horizon

// State arrays
static float x0_base[NX0];
static float x0_lifted[NXL];
static float Xref_flat[NHORIZON * NXL];
static float Uref_flat[(NHORIZON - 1) * NUL];

static int8_t result = 0;
static uint32_t step = 0;
static uint32_t log_tick = 0;

// Output state
static float cmd_x = 0.0f;
static float cmd_y = 0.0f;
static float cmd_vx = 0.0f;
static float cmd_vy = 0.0f;

// PSD demo state
#define HOVER_WAIT_STEPS 0
static uint8_t demo_phase = 0;
static bool psd_active = false;

// Virtual disk obstacle - approaches from RIGHT (-Y) toward origin
// Drone should move LEFT (+Y) to avoid
//
// obs_vy is velocity in m/s, multiplied by DT per step
// Timeline at 50Hz MPC rate with obs_vy=0.1 m/s:
//   t=0s: obs at y=-1.0, sd=0.6 (safe, PSD off)
//   t=1s: obs at y=-0.9, sd=0.5 (PSD ACTIVATES!)
//   t=5s: obs at y=-0.5, sd=0.1 (closest approach)
//   t=7s: obs at y=-0.3, stops
//
static float obs_x = 0.0f;
static float obs_y = -1.0f;           // Start 1m to the RIGHT of origin
static const float obs_vx = 0.0f;
static const float obs_vy = 0.10f;    // 0.1 m/s toward origin (per step: 0.1*0.02=2mm)
static const float obs_r = 0.25f;     // 25cm physical radius
static const float SAFETY_MARGIN = 0.15f;  // 15cm margin -> 40cm total constraint radius
static const float obs_stop_y = -0.3f;    // Stop 30cm to the right of origin

// Time-varying disk constraints
static float disk_tv[NHORIZON * 3];
static int disk_counts[NHORIZON];

// Certificate tracking
static bool last_certified = true;
static float last_trace_gap = 0.0f;
static float last_eta_min = 0.0f;

// Build lifted state from base state: [x; vec(X*X^T)]
static void build_lifted_state(const float* x_base, float* x_lifted) {
    // Copy base state
    for (int i = 0; i < NX0; ++i) {
        x_lifted[i] = x_base[i];
    }
    // Build XX = x * x^T (column-major order)
    for (int j = 0; j < NX0; ++j) {
        for (int i = 0; i < NX0; ++i) {
            x_lifted[NX0 + j*NX0 + i] = x_base[i] * x_base[j];
        }
    }
}

// Update reference for lifted system (hover at goal)
static void update_lifted_hover_reference(float goal_x, float goal_y) {
    float goal_base[NX0] = {goal_x, goal_y, 0.0f, 0.0f};
    float goal_lifted[NXL];
    build_lifted_state(goal_base, goal_lifted);

    // Match sim demos: add diagonal lifted refs to encourage rank-1 structure
    const float q_xx = 1.0f;
    const float r_uu = 10.0f;
    const int nxu = NX0 * NU0;
    const int nux = NU0 * NX0;
    const int baseUU = NU0 + nxu + nux;

    for (int k = 0; k < NHORIZON; ++k) {
        for (int i = 0; i < NXL; ++i) {
            Xref_flat[k * NXL + i] = goal_lifted[i];
        }
        for (int i = 0; i < NX0; ++i) {
            int idx = NX0 + i * NX0 + i;
            float denom = tiny_solver.work->Q(idx);
            if (denom != 0.0f) {
                Xref_flat[k * NXL + idx] = -q_xx / denom;
            }
        }
    }
    for (int k = 0; k < NHORIZON - 1; ++k) {
        for (int i = 0; i < NUL; ++i) {
            Uref_flat[k * NUL + i] = 0.0f;
        }
        for (int j = 0; j < NU0; ++j) {
            int idx = baseUU + j * NU0 + j;
            float denom = tiny_solver.work->R(idx);
            if (denom != 0.0f) {
                Uref_flat[k * NUL + idx] = -r_uu / denom;
            }
        }
    }
}

// Compute signed distance from base position to disk
static float signed_distance_to_disk(float px, float py, float cx, float cy, float r) {
    float dx = px - cx;
    float dy = py - cy;
    return sqrtf(dx*dx + dy*dy) - r;
}

// ========== CERTIFICATE COMPUTATION (from demos) ==========
// Checks if lifted solution is rank-1 and satisfies obstacle constraints
static bool compute_certificate(const float* x_lifted, float cx, float cy, float r,
                                float* trace_gap_out, float* eta_min_out) {
    // Extract position z = (x, y)
    float zx = x_lifted[0];
    float zy = x_lifted[1];
    float z_norm_sq = zx*zx + zy*zy;
    
    // Extract XX block (top-left 2x2 of the 4x4 XX matrix)
    // XX is stored column-major starting at index NX0
    float XX_00 = x_lifted[NX0 + 0*NX0 + 0];  // XX(0,0)
    float XX_11 = x_lifted[NX0 + 1*NX0 + 1];  // XX(1,1)
    float XX_trace = XX_00 + XX_11;
    
    // Trace gap: Δ = trace(XX_{2x2}) - ||z||^2
    // For rank-1: XX = z*z^T, so trace(XX) = ||z||^2, hence Δ = 0
    float trace_gap = XX_trace - z_norm_sq;
    *trace_gap_out = trace_gap;
    
    // Lifted distance squared to disk center
    float lifted_dist2 = XX_trace - 2.0f*(cx*zx + cy*zy) + cx*cx + cy*cy;
    
    // Lifted margin: η = lifted_dist2 - r^2
    float eta = lifted_dist2 - r*r;
    *eta_min_out = eta;
    
    // Certificate: η >= 0 and |Δ| <= η
    bool certified = (eta >= 0.0f) && (fabsf(trace_gap) <= eta);
    return certified;
}

// ========== ROLLOUT PLAN (from demos) ==========
// Extract base trajectory from lifted MPC solution
static void rollout_plan(const float* x_seed) {
    // Simple double integrator: x_{k+1} = A*x_k + B*u_k
    // A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]
    // B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt]
    
    // Copy seed as first state
    for (int i = 0; i < NX0; ++i) {
        plan_states[0][i] = x_seed[i];
    }
    
    // Rollout using MPC solution (base inputs only)
    float x[NX0];
    for (int i = 0; i < NX0; ++i) x[i] = x_seed[i];
    
    for (int k = 0; k < NHORIZON - 1; ++k) {
        // Extract base input from lifted solution
        float u0 = tiny_solver.solution->u(0, k);
        float u1 = tiny_solver.solution->u(1, k);
        plan_inputs[k][0] = u0;
        plan_inputs[k][1] = u1;
        
        // Simple double integrator dynamics
        float x_new[NX0];
        x_new[0] = x[0] + DT * x[2] + 0.5f * DT * DT * u0;  // x
        x_new[1] = x[1] + DT * x[3] + 0.5f * DT * DT * u1;  // y
        x_new[2] = x[2] + DT * u0;  // vx
        x_new[3] = x[3] + DT * u1;  // vy
        
        for (int i = 0; i < NX0; ++i) {
            x[i] = x_new[i];
            plan_states[k + 1][i] = x_new[i];
        }
    }
    
    plan_last_iter = tiny_solver.solution->iter;
}

// ========== SET TRACKING REFS (from demos) ==========
// Set references to track the cached plan
static void set_tracking_refs_from_plan(int current_step, float goal_x, float goal_y) {
    const float q_xx = 1.0f;
    const float r_uu = 10.0f;
    const int nxu = NX0 * NU0;
    const int nux = NU0 * NX0;
    const int baseUU = NU0 + nxu + nux;
    
    int offset = current_step - plan_start_step;
    
    for (int k = 0; k < NHORIZON; ++k) {
        // Clamp index to valid range
        int idx = offset + k;
        if (idx < 0) idx = 0;
        if (idx >= NHORIZON) idx = NHORIZON - 1;
        
        // Build lifted state from cached plan
        float base[NX0];
        if (plan_valid && idx < NHORIZON) {
            for (int i = 0; i < NX0; ++i) base[i] = plan_states[idx][i];
        } else {
            base[0] = goal_x; base[1] = goal_y; base[2] = 0; base[3] = 0;
        }
        
        float lifted[NXL];
        build_lifted_state(base, lifted);
        
        for (int i = 0; i < NXL; ++i) {
            Xref_flat[k * NXL + i] = lifted[i];
        }
        
        // Add diagonal refs to encourage rank-1
        for (int i = 0; i < NX0; ++i) {
            int diag_idx = NX0 + i * NX0 + i;
            float denom = tiny_solver.work->Q(diag_idx);
            if (denom != 0.0f) {
                Xref_flat[k * NXL + diag_idx] = -q_xx / denom;
            }
        }
    }
    
    for (int k = 0; k < NHORIZON - 1; ++k) {
        for (int i = 0; i < NUL; ++i) {
            Uref_flat[k * NUL + i] = 0.0f;
        }
        
        // Copy base input from plan if available
        int idx = offset + k;
        if (plan_valid && idx >= 0 && idx < NHORIZON - 1) {
            Uref_flat[k * NUL + 0] = plan_inputs[idx][0];
            Uref_flat[k * NUL + 1] = plan_inputs[idx][1];
        }
        
        // Add diagonal refs for UU block
        for (int j = 0; j < NU0; ++j) {
            int diag_idx = baseUU + j * NU0 + j;
            float denom = tiny_solver.work->R(diag_idx);
            if (denom != 0.0f) {
                Uref_flat[k * NUL + diag_idx] = -r_uu / denom;
            }
        }
    }
}

void controllerOutOfTreeInit(void) {
    DEBUG_PRINT("=== TinyMPC PSD Planner-Tracker ===\n");
    DEBUG_PRINT("Lifted: nx=%d nu=%d N=%d\n", NXL, NUL, NHORIZON);
    DEBUG_PRINT("Replan stride=%d, horizon_guard=%d\n", REPLAN_STRIDE, HORIZON_GUARD);
    
    // Initialize solver settings
    tiny_solver.settings->max_iter = 10;
    tiny_solver.settings->check_termination = 0;
    tiny_solver.settings->en_state_linear = 0;
    tiny_solver.settings->en_psd = 0;

    // Enable PSD block (same as sim demos)
    tiny_enable_psd(&tiny_solver, NX0, NU0, 0.95f);
    
    memset(x0_base, 0, sizeof(x0_base));
    memset(x0_lifted, 0, sizeof(x0_lifted));
    memset(Xref_flat, 0, sizeof(Xref_flat));
    memset(Uref_flat, 0, sizeof(Uref_flat));
    memset(plan_states, 0, sizeof(plan_states));
    memset(plan_inputs, 0, sizeof(plan_inputs));
    
    step = 0;
    demo_phase = 0;
    psd_active = false;
    plan_valid = false;
    plan_start_step = 0;
    obs_x = 0.0f;
    obs_y = -1.0f;  // Reset to starting position (1m to the right)
    
    controllerPidInit();
    
    DEBUG_PRINT("Planner-Tracker arch with certificate gating\n");
}

bool controllerOutOfTreeTest() {
    return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const stabilizerStep_t stabilizerStep) {
    (void)sensors;
    
    // Get current base state
    x0_base[0] = state->position.x;
    x0_base[1] = state->position.y;
    x0_base[2] = state->velocity.x;
    x0_base[3] = state->velocity.y;
    
    const float goal_x = setpoint->position.x;
    const float goal_y = setpoint->position.y;
    const bool landing = (setpoint->mode.z == modeAbs) &&
                          (setpoint->position.z < (state->position.z - 0.02f));

    if (RATE_DO_EXECUTE(MPC_RATE, stabilizerStep)) {
        // State machine for obstacle demo
        if (demo_phase == 0 && step >= HOVER_WAIT_STEPS) {
            demo_phase = 1;
            DEBUG_PRINT("PSD: Disk starting!\n");
        }
        
        if (demo_phase == 1) {
            obs_x += obs_vx * DT;
            obs_y += obs_vy * DT;
            if (obs_y > obs_stop_y) {
                demo_phase = 2;
                DEBUG_PRINT("PSD: Disk passed!\n");
            }
        }

        float sd = signed_distance_to_disk(x0_base[0], x0_base[1], obs_x, obs_y, obs_r + SAFETY_MARGIN);
        
        // PSD activation with hysteresis (from demos)
        const float activate_dist = 0.5f;
        const float deactivate_dist = 0.6f;
        
        if (!psd_active && sd < activate_dist && demo_phase == 1) {
            psd_active = true;
            plan_valid = false;  // Force replan when PSD activates
            DEBUG_PRINT("PSD: ACTIVE sd=%.2f\n", (double)sd);
        } else if (psd_active && sd > deactivate_dist) {
            psd_active = false;
            DEBUG_PRINT("PSD: OFF sd=%.2f\n", (double)sd);
        }

        build_lifted_state(x0_base, x0_lifted);
        
        // ========== PLANNER PHASE ==========
        // Decide if we need to replan (from demos)
        int plan_age = (int)step - plan_start_step;
        bool need_replan = !plan_valid 
                        || (plan_age >= REPLAN_STRIDE)
                        || (plan_age >= NHORIZON - HORIZON_GUARD);
        
        if (need_replan && psd_active) {
            // Run planner with PSD enabled
            tiny_solver.settings->en_psd = 1;
            
            // Set time-varying disk constraints (skip stage 0 for feasibility)
            for (int k = 0; k < NHORIZON; ++k) {
                disk_counts[k] = (k == 0) ? 0 : 1;
                disk_tv[3*k + 0] = obs_x;
                disk_tv[3*k + 1] = obs_y;
                disk_tv[3*k + 2] = obs_r + SAFETY_MARGIN;
            }
            tiny_set_lifted_disks_tv_raw(&tiny_solver, disk_tv, disk_counts, NHORIZON, 1);
            
            // Set initial state
            for (int i = 0; i < NXL; ++i) {
                tiny_solver.work->x(i, 0) = x0_lifted[i];
            }
            
            // Set goal reference (planner aims for goal)
            update_lifted_hover_reference(goal_x, goal_y);
            for (int k = 0; k < NHORIZON; ++k) {
                for (int i = 0; i < NXL; ++i) {
                    tiny_solver.work->Xref(i, k) = Xref_flat[k * NXL + i];
                }
            }
            for (int k = 0; k < NHORIZON - 1; ++k) {
                for (int i = 0; i < NUL; ++i) {
                    tiny_solver.work->Uref(i, k) = Uref_flat[k * NUL + i];
                }
            }
            
            tinympc::update_linear_cost(&tiny_solver);
            tinympc::backward_pass_grad(&tiny_solver);
            result = (int8_t)tiny_solve(&tiny_solver);
            
            // CERTIFICATE GATING: Check if planner solution is valid
            float x_sol[NXL];
            for (int i = 0; i < NXL; ++i) {
                x_sol[i] = tiny_solver.solution->x(i, 0);
            }
            bool cert_ok = compute_certificate(x_sol, obs_x, obs_y, obs_r + SAFETY_MARGIN,
                                               &last_trace_gap, &last_eta_min);
            
            if (cert_ok) {
                // Plan is valid, cache it
                rollout_plan(x0_base);
                plan_start_step = step;
                plan_valid = true;
                last_certified = true;
            } else {
                // Plan rejected, keep old plan
                DEBUG_PRINT("PLANNER REJECT: gap=%.3f eta=%.3f\n", 
                           (double)last_trace_gap, (double)last_eta_min);
                last_certified = false;
            }
        }
        
        // ========== TRACKER PHASE ==========
        // Tracker tracks the cached plan (PSD off for speed)
        tiny_solver.settings->en_psd = 0;
        tiny_solver.settings->en_state_linear = 0;
        tiny_solver.work->numStateLinear = 0;
        tiny_solver.settings->en_tv_state_linear = 0;
        
        for (int i = 0; i < NXL; ++i) {
            tiny_solver.work->x(i, 0) = x0_lifted[i];
        }
        
        // Set tracking references from cached plan
        set_tracking_refs_from_plan(step, goal_x, goal_y);
        for (int k = 0; k < NHORIZON; ++k) {
            for (int i = 0; i < NXL; ++i) {
                tiny_solver.work->Xref(i, k) = Xref_flat[k * NXL + i];
            }
        }
        for (int k = 0; k < NHORIZON - 1; ++k) {
            for (int i = 0; i < NUL; ++i) {
                tiny_solver.work->Uref(i, k) = Uref_flat[k * NUL + i];
            }
        }
        
        tinympc::update_linear_cost(&tiny_solver);
        tinympc::backward_pass_grad(&tiny_solver);
        result = (int8_t)tiny_solve(&tiny_solver);
        
        // EMERGENCY CERTIFICATE CHECK on tracker output
        float x_track[NXL];
        for (int i = 0; i < NXL; ++i) {
            x_track[i] = tiny_solver.solution->x(i, 0);
        }
        
        float track_gap, track_eta;
        bool track_cert = true;
        if (psd_active) {
            track_cert = compute_certificate(x_track, obs_x, obs_y, obs_r + SAFETY_MARGIN,
                                             &track_gap, &track_eta);
        }
        
        if (track_cert) {
            // Normal operation: use MPC output
            cmd_x = tiny_solver.solution->x(0, 1);
            cmd_y = tiny_solver.solution->x(1, 1);
            cmd_vx = tiny_solver.solution->x(2, 1);
            cmd_vy = tiny_solver.solution->x(3, 1);
        } else {
            // EMERGENCY: tracker failed certificate, safe stop
            DEBUG_PRINT("EMERGENCY STOP: gap=%.3f eta=%.3f\n", (double)track_gap, (double)track_eta);
            cmd_vx = 0.0f;
            cmd_vy = 0.0f;
            // Keep current position (don't move into obstacle)
        }

        // Update logs
        psd_log_plan_mode = psd_active ? 1 : 0;
        psd_log_plan_status = plan_valid ? 1 : 0;
        psd_log_plan_age = plan_age;
        psd_log_disk_count = psd_active ? 1 : 0;
        psd_log_certified = last_certified ? 1 : 0;
        psd_log_trace_gap = last_trace_gap;
        psd_log_eta_min = last_eta_min;
        psd_log_cmd_x = cmd_x;
        psd_log_cmd_y = cmd_y;
        psd_log_cmd_vx = cmd_vx;
        psd_log_cmd_vy = cmd_vy;
        psd_log_goal_x = goal_x;
        psd_log_goal_y = goal_y;
        psd_log_result = result;

        step++;
    }

    setpoint_t sp = *setpoint;
    
    if (step > 0 && !landing) {
        sp.mode.x = modeAbs;
        sp.mode.y = modeAbs;
        sp.position.x = cmd_x;
        sp.position.y = cmd_y;
        sp.velocity.x = cmd_vx;
        sp.velocity.y = cmd_vy;
        sp.velocity_body = false;
    }

    controllerPid(control, &sp, sensors, state, stabilizerStep);

    if ((stabilizerStep - log_tick) > 500) {
        DEBUG_PRINT("ph%d age=%d cert=%d cmd=(%.2f,%.2f) %s\n",
                    demo_phase, (int)(step - plan_start_step), last_certified ? 1 : 0,
                    (double)cmd_x, (double)cmd_y, psd_active ? "PSD" : "nom");
        log_tick = stabilizerStep;
    }
}

#ifdef __cplusplus
}
#endif
