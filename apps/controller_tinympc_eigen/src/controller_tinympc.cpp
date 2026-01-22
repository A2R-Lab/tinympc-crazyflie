/**
 * TinyMPC Lifted Controller - Step 3: PSD + Disk Constraints
 * 
 * Uses 20-state lifted solver with PSD and disk obstacle avoidance.
 */

#include <Eigen.h>
#include <cmath>
using namespace Eigen;

#include "tinympc/tiny_data.hpp"
#include "tinympc/types.hpp"
#include "tinympc/admm.hpp"
#include "tinympc/tiny_api.hpp"
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

#define DEBUG_MODULE "TINYMPC"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("TinyMPC Lifted+PSD\n");
  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// Dimensions
#define NX0 4       // Base state: x, y, vx, vy
#define NU0 2       // Base input: ax, ay
#define NXL 20      // Lifted: NX0 + NX0*NX0 = 4 + 16
#define NUL 22      // Lifted: NU0 + NX0*NU0 + NU0*NX0 + NU0*NU0
#define NHORIZON 25
#define MPC_RATE RATE_50_HZ

// Maximum number of disk obstacles
#define MAX_DISKS 4

static float x_lifted[NXL];
static uint32_t step = 0;
static float cmd_x = 0, cmd_y = 0, cmd_vx = 0, cmd_vy = 0;

// Obstacle parameters from psd_params.c (settable via cfclient)
// psd_obs_x, psd_obs_y, psd_obs_r, psd_obs_enabled are extern in psd_params.h

// Build lifted state: [x; vec(x*x^T)]
static void build_lifted(const float* x, float* xl) {
    for (int i = 0; i < NX0; ++i) xl[i] = x[i];
    for (int j = 0; j < NX0; ++j)
        for (int i = 0; i < NX0; ++i)
            xl[NX0 + j*NX0 + i] = x[i] * x[j];
}

// Set up lifted disk constraint for single obstacle
// Disk constraint: ||pos - obs||^2 >= r^2
// In lifted form: m^T [x; vec(XX)] >= n
// where m = [-2ox, -2oy, 0, 0, 1, 0, ..., 0, 1, ...] (1s at xx11, xx22)
// and n = r^2 - ox^2 - oy^2
// Store as a^T z <= b: a = -m, b = -n
static void set_disk_constraint(float ox, float oy, float r) {
    const int nxL = NXL;
    const int idx_xx11 = NX0 + 0 + 0*NX0;  // index 4
    const int idx_xx22 = NX0 + 1 + 1*NX0;  // index 9
    
    // Enable state linear constraints (1 constraint)
    tiny_solver.settings->en_state_linear = 1;
    tiny_solver.work->numStateLinear = 1;
    
    // Allocate matrices if not already done
    if (tiny_solver.work->Alin_x.rows() != 1 || 
        tiny_solver.work->Alin_x.cols() != nxL) {
        tiny_solver.work->Alin_x = tinyMatrix::Zero(1, nxL);
        tiny_solver.work->blin_x = tinyVector::Zero(1);
        tiny_solver.work->vlnew = tiny_solver.work->x;
        tiny_solver.work->gl = tinyMatrix::Zero(nxL, NHORIZON);
    }
    
    // Build constraint: a = -m, b = -n
    // m = [-2ox, -2oy, 0, 0, 1, 0, 0, 0, 0, 1, ...]
    // a = [2ox, 2oy, 0, 0, -1, 0, 0, 0, 0, -1, ...]
    tiny_solver.work->Alin_x.setZero();
    tiny_solver.work->Alin_x(0, 0) = 2.0f * ox;
    tiny_solver.work->Alin_x(0, 1) = 2.0f * oy;
    tiny_solver.work->Alin_x(0, idx_xx11) = -1.0f;
    tiny_solver.work->Alin_x(0, idx_xx22) = -1.0f;
    
    // b = -n = -(r^2 - ox^2 - oy^2) = ox^2 + oy^2 - r^2
    tiny_solver.work->blin_x(0) = ox*ox + oy*oy - r*r;
}

static void disable_disk_constraint() {
    tiny_solver.settings->en_state_linear = 0;
    tiny_solver.work->numStateLinear = 0;
}

void controllerOutOfTreeInit(void) {
    DEBUG_PRINT("=== TinyMPC Step 3: PSD+Disk ===\n");
    
    // ENABLE PSD - buffers are pre-allocated in generated code
    tiny_solver.settings->en_psd = 1;
    tiny_solver.settings->nx0_psd = NX0;
    tiny_solver.settings->nu0_psd = NU0;
    tiny_solver.cache->rho_psd = 0.95f;
    tiny_solver.settings->en_state_linear = 0;
    tiny_solver.settings->en_tv_state_linear = 0;
    tiny_solver.settings->max_iter = 5;
    
    // Set up initial disk constraint
    if (psd_obs_enabled) {
        set_disk_constraint(psd_obs_x, psd_obs_y, psd_obs_r);
        DEBUG_PRINT("Disk obs=(%.2f,%.2f) r=%.2f\n", 
                   (double)psd_obs_x, (double)psd_obs_y, (double)psd_obs_r);
    }
    
    step = 0;
    controllerPidInit();
    DEBUG_PRINT("Lifted nx=%d nu=%d - PSD+Disk ON\n", NXL, NUL);
}

bool controllerOutOfTreeTest() { return true; }

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const stabilizerStep_t stabilizerStep) {
    (void)sensors;
    
    float x_base[NX0] = {state->position.x, state->position.y, 
                         state->velocity.x, state->velocity.y};
    float goal_x = setpoint->position.x;
    float goal_y = setpoint->position.y;

    if (RATE_DO_EXECUTE(MPC_RATE, stabilizerStep)) {
        // Update disk constraint dynamically (can change via cfclient params)
        if (psd_obs_enabled) {
            set_disk_constraint(psd_obs_x, psd_obs_y, psd_obs_r);
        } else {
            disable_disk_constraint();
        }
        
        // Build lifted state
        build_lifted(x_base, x_lifted);
        
        // Set initial state
        for (int i = 0; i < NXL; ++i)
            tiny_solver.work->x(i, 0) = x_lifted[i];
        
        // Set goal reference (lifted)
        float goal_base[NX0] = {goal_x, goal_y, 0.0f, 0.0f};
        float goal_lifted[NXL];
        build_lifted(goal_base, goal_lifted);
        
        for (int k = 0; k < NHORIZON; ++k)
            for (int i = 0; i < NXL; ++i)
                tiny_solver.work->Xref(i, k) = goal_lifted[i];
        
        for (int k = 0; k < NHORIZON - 1; ++k)
            for (int i = 0; i < NUL; ++i)
                tiny_solver.work->Uref(i, k) = 0.0f;
        
        // Solve MPC
        tinympc::update_linear_cost(&tiny_solver);
        tinympc::backward_pass_grad(&tiny_solver);
        int result = tiny_solve(&tiny_solver);
        
        // Extract base state from solution (solution->x = vnew after solve)
        // Use timestep 1 as next position target for PID
        cmd_x = tiny_solver.solution->x(0, 1);
        cmd_y = tiny_solver.solution->x(1, 1);
        cmd_vx = tiny_solver.solution->x(2, 1);
        cmd_vy = tiny_solver.solution->x(3, 1);
        
        // Safety clamps
        if (cmd_x < -2.0f) cmd_x = -2.0f;
        if (cmd_x > 2.0f) cmd_x = 2.0f;
        if (cmd_y < -2.0f) cmd_y = -2.0f;
        if (cmd_y > 2.0f) cmd_y = 2.0f;
        
        // Update logs
        psd_log_cmd_x = cmd_x;
        psd_log_cmd_y = cmd_y;
        psd_log_cmd_vx = cmd_vx;
        psd_log_cmd_vy = cmd_vy;
        psd_log_goal_x = goal_x;
        psd_log_goal_y = goal_y;
        psd_log_result = result;
        
        step++;
        
        if (step % 100 == 0) {
            DEBUG_PRINT("s%lu pos=(%.2f,%.2f) cmd=(%.2f,%.2f)\n", step,
                       (double)x_base[0], (double)x_base[1],
                       (double)cmd_x, (double)cmd_y);
        }
    }

    // Pass MPC output to PID
    setpoint_t sp = *setpoint;
    if (step > 5) {
        sp.mode.x = modeAbs;
        sp.mode.y = modeAbs;
        sp.position.x = cmd_x;
        sp.position.y = cmd_y;
        sp.velocity.x = cmd_vx;
        sp.velocity.y = cmd_vy;
        sp.velocity_body = false;
    }
    
    controllerPid(control, &sp, sensors, state, stabilizerStep);
}

#ifdef __cplusplus
}
#endif
