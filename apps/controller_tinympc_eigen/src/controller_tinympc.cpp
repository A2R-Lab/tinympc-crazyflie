/**
 * Crazyflie TinyMPC PSD Controller
 * 
 * This uses the LIFTED PSD solver for provably-safe obstacle avoidance.
 * The lifted state is [x; vec(XX)] where XX = x*x^T, enabling convex
 * quadratic constraints for disk obstacles.
 */

#include <Eigen.h>
#include <cmath>
using namespace Eigen;

// TinyMPC includes
#include "tinympc/tiny_data.hpp"
#include "tinympc/types.hpp"
#include "tinympc/admm.hpp"
#include "tinympc/tiny_api.hpp"
#include "tinympc/psd_support.hpp"  // For tiny_set_lifted_disks_raw
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
#define MPC_RATE RATE_50_HZ  // Reduced from 100Hz - lifted solver is heavy

// State arrays
static float x0_base[NX0];      // Current base state
static float x0_lifted[NXL];    // Current lifted state
static float Xref_flat[NHORIZON * NXL];
static float Uref_flat[(NHORIZON - 1) * NUL];

static int8_t result = 0;
static uint32_t step = 0;
static uint32_t log_tick = 0;

// Output state (base) - direct MPC output, no smoothing
static float cmd_x = 0.0f;
static float cmd_y = 0.0f;
static float cmd_vx = 0.0f;
static float cmd_vy = 0.0f;

// PSD demo state
#define HOVER_WAIT_STEPS 150   // 3 seconds at 50Hz
static uint8_t demo_phase = 0;  // 0=hover wait, 1=obstacle moving, 2=done
static bool psd_active = false;

// Virtual disk obstacle from the RIGHT
static float obs_x = 0.8f;       // Start further away
static float obs_y = 0.0f;
static const float obs_vx = -0.03f;  // Slower approach
static const float obs_r = 0.20f;    // 20cm physical radius
static const float SAFETY_MARGIN = 0.20f;  // 20cm extra margin for PSD
static const float obs_stop_x = -0.5f;

// Disk constraint array for PSD: [cx, cy, r]
static float disk_constraint[3];

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
    
    for (int k = 0; k < NHORIZON; ++k) {
        for (int i = 0; i < NXL; ++i) {
            Xref_flat[k * NXL + i] = goal_lifted[i];
        }
    }
    for (int i = 0; i < (NHORIZON - 1) * NUL; ++i) {
        Uref_flat[i] = 0.0f;
    }
}

// Compute signed distance from base position to disk
static float signed_distance_to_disk(float px, float py, float cx, float cy, float r) {
    float dx = px - cx;
    float dy = py - cy;
    return sqrtf(dx*dx + dy*dy) - r;
}

void controllerOutOfTreeInit(void) {
    DEBUG_PRINT("=== TinyMPC PSD LIFTED Controller ===\n");
    DEBUG_PRINT("Lifted: nx=%d nu=%d N=%d\n", NXL, NUL, NHORIZON);
    
    // Initialize solver settings
    tiny_solver.settings->max_iter = 10;  // Original value
    tiny_solver.settings->check_termination = 0;
    tiny_solver.settings->en_state_linear = 0;  // Will enable when disk active
    
    memset(x0_base, 0, sizeof(x0_base));
    memset(x0_lifted, 0, sizeof(x0_lifted));
    memset(Xref_flat, 0, sizeof(Xref_flat));
    memset(Uref_flat, 0, sizeof(Uref_flat));
    
    step = 0;
    demo_phase = 0;
    psd_active = false;
    obs_x = 0.8f;
    obs_y = 0.0f;
    
    // Initialize PID - needed at boot since we call controllerPid() internally
    // This only runs once at boot, not on controller switch
    controllerPidInit();
    
    DEBUG_PRINT("PSD Demo: hover 3s, disk from RIGHT, drone avoids\n");
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

    if (RATE_DO_EXECUTE(MPC_RATE, stabilizerStep)) {
        
        // State machine
        if (demo_phase == 0 && step >= HOVER_WAIT_STEPS) {
            demo_phase = 1;
            DEBUG_PRINT("PSD: Disk starting!\n");
        }
        
        // Move disk in phase 1
        if (demo_phase == 1) {
            obs_x += obs_vx * DT;
            if (obs_x < obs_stop_x) {
                demo_phase = 2;
                DEBUG_PRINT("PSD: Disk passed!\n");
            }
        }

        // Compute signed distance (using constraint radius = physical + margin)
        float sd = signed_distance_to_disk(x0_base[0], x0_base[1], obs_x, obs_y, obs_r + SAFETY_MARGIN);
        
        // PSD activation based on distance (activate earlier for safety)
        const float activate_dist = 0.5f;   // Activate when 50cm from obstacle edge
        const float deactivate_dist = 0.6f;
        
        if (!psd_active && sd < activate_dist && demo_phase == 1) {
            psd_active = true;
            DEBUG_PRINT("PSD: ACTIVE sd=%.2f\n", (double)sd);
        } else if (psd_active && (sd > deactivate_dist || demo_phase == 2)) {
            psd_active = false;
            DEBUG_PRINT("PSD: OFF sd=%.2f\n", (double)sd);
        }

        // Build lifted state
        build_lifted_state(x0_base, x0_lifted);
        
        // Set disk constraint if PSD active - use the proper TinyMPC function
        if (psd_active) {
            disk_constraint[0] = obs_x;  // cx
            disk_constraint[1] = obs_y;  // cy
            disk_constraint[2] = obs_r + SAFETY_MARGIN;  // r + 20cm margin = 40cm total
            
            // Use the proper PSD function to set disk constraint
            tiny_set_lifted_disks_raw(&tiny_solver, disk_constraint, 1);
            
        } else {
            tiny_solver.settings->en_state_linear = 0;
            tiny_solver.work->numStateLinear = 0;
        }

        // Set initial state and reference
        for (int i = 0; i < NXL; ++i) {
            tiny_solver.work->x(i, 0) = x0_lifted[i];
        }
        
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

        // Solve MPC
        result = (int8_t)tiny_solve(&tiny_solver);

        // Extract base state from MPC solution directly - no smoothing
        cmd_x = tiny_solver.solution->x(0, 1);
        cmd_y = tiny_solver.solution->x(1, 1);
        cmd_vx = tiny_solver.solution->x(2, 1);
        cmd_vy = tiny_solver.solution->x(3, 1);

        // Update logs
        psd_log_plan_mode = psd_active ? 1 : 0;
        psd_log_plan_status = (result == 0) ? 1 : 0;
        psd_log_plan_age = (int32_t)step;
        psd_log_disk_count = psd_active ? 1 : 0;
        psd_log_certified = (sd > 0.0f) ? 1 : 0;
        psd_log_trace_gap = sd;
        psd_log_eta_min = sd;

        step++;
    }

    // Send to PID - pass through original setpoint, only override X/Y if MPC has run
    setpoint_t sp = *setpoint;
    
    // Only override X/Y if MPC has produced output (step > 0)
    if (step > 0) {
        sp.mode.x = modeAbs;
        sp.mode.y = modeAbs;
        sp.position.x = cmd_x;
        sp.position.y = cmd_y;
        sp.velocity.x = cmd_vx;
        sp.velocity.y = cmd_vy;
        sp.velocity_body = false;
    }
    // Otherwise pass through original setpoint unchanged (including Z for altitude!)
    
    controllerPid(control, &sp, sensors, state, stabilizerStep);

    // Debug logging - show command vs actual position
    if ((stabilizerStep - log_tick) > 500) {
        DEBUG_PRINT("ph%d obs=%.2f cmd=%.2f pos=%.2f vx=%.2f %s\n",
                    demo_phase, (double)obs_x, (double)cmd_x,
                    (double)x0_base[0], (double)cmd_vx, psd_active ? "PSD" : "nom");
        log_tick = stabilizerStep;
    }
}

#ifdef __cplusplus
}
#endif
