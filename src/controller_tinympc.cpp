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

#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"
#include "system.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "eventtrigger.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "cpp_compat.h" // needed to compile Cpp to C

// TinyMPC and PID controllers
#include "tinympc/admm.hpp"
#include "tinympc/tiny_api.hpp"
#include "controller_pid.h"

//Params
#include "quadrotor_50hz_params_unconstrained.hpp"
#include "quadrotor_50hz_params_constrained.hpp"

//Trajectory
#include "quadrotor_50hz_line_9s_xyz.hpp"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPCTASK"
#include "debug.h"

//control frequency
#define MPC_RATE RATE_100_HZ
#define LOWLEVEL_RATE RATE_500_HZ

// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

static void tinympcControllerTask(void *parameters);

STATIC_MEM_TASK_ALLOC(tinympcControllerTask, SYSTEM_TASK_STACKSIZE);

// declares eventTrigger_[name] and eventTrigger_[name]_payload
EVENTTRIGGER(horizon_x_part1, float, h0, float, h1, float, h2, float, h3, float, h4);
EVENTTRIGGER(horizon_x_part2, float, h5, float, h6, float, h7, float, h8, float, h9);
EVENTTRIGGER(horizon_x_part3, float, h10, float, h11, float, h12, float, h13, float, h14);
EVENTTRIGGER(horizon_x_part4, float, h15, float, h16, float, h17, float, h18, float, h19);
EVENTTRIGGER(horizon_y_part1, float, h0, float, h1, float, h2, float, h3, float, h4);
EVENTTRIGGER(horizon_y_part2, float, h5, float, h6, float, h7, float, h8, float, h9);
EVENTTRIGGER(horizon_y_part3, float, h10, float, h11, float, h12, float, h13, float, h14);
EVENTTRIGGER(horizon_y_part4, float, h15, float, h16, float, h17, float, h18, float, h19);
EVENTTRIGGER(horizon_z_part1, float, h0, float, h1, float, h2, float, h3, float, h4);
EVENTTRIGGER(horizon_z_part2, float, h5, float, h6, float, h7, float, h8, float, h9);
EVENTTRIGGER(horizon_z_part3, float, h10, float, h11, float, h12, float, h13, float, h14);
EVENTTRIGGER(horizon_z_part4, float, h15, float, h16, float, h17, float, h18, float, h19);
EVENTTRIGGER(problem_data_event, int32, solvetime_us, int32, iters, int32, cache_level);
EVENTTRIGGER(problem_residuals_event, float, prim_resid_state, float, prim_resid_input, float, dual_resid_state, float, dual_resid_input);


// Structs to keep track of data sent to and received by stabilizer loop
// Stabilizer loop updates/uses these
control_t control_data;
setpoint_t setpoint_data;
sensorData_t sensors_data;
state_t state_data;
tinyVector mpc_setpoint;
setpoint_t mpc_setpoint_pid;
// Copies that stay constant for duration of MPC loop
setpoint_t setpoint_task;
sensorData_t sensors_task;
state_t state_task;
control_t control_task;
tinyVector mpc_setpoint_task;

/* Allocate global variables for MPC */
// static tinytype u_hover[4] = {.65, .65, .65, .65};
static tinytype u_hover[4] = {.583, .583, .583, .583};
static TinyCache cache;
static TinyWorkspace work;
static TinySettings settings;
static TinySolver solver;
static tinyMatrix problem_x;
static float horizon_nh_z;
static float init_vel_z;
// static Eigen::Matrix<tinytype, NSTATES, NTOTAL, Eigen::ColMajor> Xref_total;
static Eigen::Matrix<tinytype, 3, NTOTAL, Eigen::ColMajor> Xref_total;
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen::ColMajor> Xref_origin; // Start position for trajectory
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen:: ColMajor> Xref_end; // End position for trajectory
static tinyVector u_lqr;
static tinyVector current_state;

// Helper variables
static bool enable_traj = false;
static int traj_index = 0;
static int max_traj_index = 0;
// static int mpc_steps_taken = 0;
static uint32_t startTimestamp;
// static uint32_t timestamp;
static uint32_t mpc_start_timestamp;
static uint32_t mpc_time_us;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters
static bool isInit = false;
static float obs_velocity_scale = 1;
static float use_obs_offset = 0;

// Obstacle constraint variables
static Eigen::Matrix<tinytype, 3, 1> obs_center;
static Eigen::Matrix<tinytype, 3, 1> obs_predicted_center;
static Eigen::Matrix<tinytype, 3, 1> obs_velocity;
static Eigen::Matrix<tinytype, 3, 1> obs_offset;
static float r_obs = .5;

static Eigen::Matrix<tinytype, 3, 1> xc;
static Eigen::Matrix<tinytype, 3, 1> a_norm;
static Eigen::Matrix<tinytype, 3, 1> q_c;

static inline float quat_dot(quaternion_t a, quaternion_t b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline quaternion_t make_quat(float x, float y, float z, float w)
{
  quaternion_t q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

static inline quaternion_t normalize_quat(quaternion_t q)
{
  float s = 1.0f / sqrtf(quat_dot(q, q));
  return make_quat(s * q.x, s * q.y, s * q.z, s * q.w);
}

static inline struct vec quat_2_rp(quaternion_t q)
{
  struct vec v;
  v.x = q.x / q.w;
  v.y = q.y / q.w;
  v.z = q.z / q.w;
  return v;
}
  
static void resetProblem(void) {
  // Copy problem data
  work.x = tinyMatrix::Zero(NSTATES, NHORIZON);
  work.q = tinyMatrix::Zero(NSTATES, NHORIZON);
  work.p = tinyMatrix::Zero(NSTATES, NHORIZON);
  work.v = tinyMatrix::Zero(NSTATES, NHORIZON);
  work.vnew = tinyMatrix::Zero(NSTATES, NHORIZON);
  work.g = tinyMatrix::Zero(NSTATES, NHORIZON);

  work.u = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  work.r = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  work.d = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  work.z = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  work.znew = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  work.y = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
}


  // We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
  void appMain()
  {
    DEBUG_PRINT("Waiting for activation ...\n");

    while (1)
    {
      vTaskDelay(M2T(2000));
    }
  }

// The new controller goes here --------------------------------------------
// Move the includes to the the top of the file if you want to
#include "controller.h"

// Call the PID controller in this example to make it possible to fly. When you implement you own controller, there is
// no need to include the pid controller.
#include "controller_pid.h"

  void controllerOutOfTreeInit()
  {
    // Initialize your controller data here...

    // Call the PID controller instead in this example to make it possible to fly
    controllerPidInit();

    // Basic setup
    solver.work = &work; // work is a Workspace
    solver.cache = &cache;
    solver.settings = &settings;

    // Copy cache data from problem_data/quadrotor*.hpp
    // I think that should be done first
    cache.rho = rho_unconstrained_value;
    cache.Kinf = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_constrained_data, NINPUTS, NSTATES);
    cache.Pinf = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_constrained_data, NSTATES, NSTATES);
    cache.Quu_inv = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_constrained_data, NINPUTS, NINPUTS);
    cache.AmBKt = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_constrained_data, NSTATES, NSTATES);


    // Basic parameters first
    work.nx = NSTATES;
    work.nu = NINPUTS;
    work.N  = NHORIZON;

    // Initialize problem data to zero before any other operations
    resetProblem();

    // Initialize simple scalar values
    work.primal_residual_state = 0;
    work.primal_residual_input = 0;
    work.dual_residual_state = 0;
    work.dual_residual_input = 0;
    work.status = 0;
    work.iter = 0;

    // Use the constrained version of Q and R
    // work.Q = Eigen::Map<Matrix<tinytype, NSTATES, 1>>(Q_constrained_data);
    // DEBUG_PRINT("After Q init\n");
    
    // work.R = Eigen::Map<Matrix<tinytype, NINPUTS, 1>>(R_constrained_data);
    // DEBUG_PRINT("After R init\n");

    // First resize the vectors
    work.Q.resize(NSTATES);
    work.R.resize(NINPUTS);
    
    // Then copy the data
    for(int i = 0; i < NSTATES; i++) {
        work.Q(i) = Q_constrained_data[i];
    }
    DEBUG_PRINT("After Q init\n");
    
    for(int i = 0; i < NINPUTS; i++) {
        work.R(i) = R_constrained_data[i];
    }
    DEBUG_PRINT("After R init\n");

    // Copy/set workspace data - just the Q and R mapping first

    // THIS DOES NOT WORK1!!!!!! because it uses dynamic size
    // recommended to use fixed size for embedded systems

    // work.Q = Eigen::Map<tinyVector>(Q_constrained_data, NSTATES, 1);
    // work.R = Eigen::Map<tinyVector>(R_constrained_data, NINPUTS, 1);

    // Defined in here: quadrotor_50hz_params_constrained.hpp
    // tinyVector is defined in tinympc/types.h

    // Resize vectors before mapping just in case
    // work.Q.resize(NSTATES); // states
    // work.R.resize(NINPUTS); // actions

    // Instead of using Dynamic sizing, let's try fixed size
    // work.Q = Matrix<tinytype, NSTATES, 1>::Zero();
    // DEBUG_PRINT("After Q init\n");
    
    // work.R = Matrix<tinytype, NINPUTS, 1>::Zero();
    // DEBUG_PRINT("After R init\n");

    // Use the constrained version of Q and R
    // work.Q = Eigen::Map<Matrix<tinytype, NSTATES, 1>>(Q_constrained_data);
    // DEBUG_PRINT("After Q init\n");
    
    // work.R = Eigen::Map<Matrix<tinytype, NINPUTS, 1>>(R_constrained_data);
    // DEBUG_PRINT("After R init\n");

    /*
    DEBUG_PRINT("Before Q copy\n");

    for(int i = 0; i < NSTATES; i++) {
        work.Q(i) = Q_constrained_data[i];
    }

    // Q copy operations
    DEBUG_PRINT("After Q copy\n");
    DEBUG_PRINT("Before R copy\n");
    
    for(int i = 0; i < NINPUTS; i++) {
        work.R(i) = R_constrained_data[i];
    }

    // R copy operations
    DEBUG_PRINT("After R copy\n");
    */

    /*
    // work.Q = Eigen::Map<tinyVector>(Q_constrained_data, NSTATES, 1);
    // work.R = Eigen::Map<tinyVector>(R_constrained_data, NINPUTS, 1);

    // Now map the data, explicitly setting type instead of using tinyVector
    work.Q = Eigen::Map<Matrix<tinytype, Dynamic, 1>>(Q_constrained_data, NSTATES);
    work.R = Eigen::Map<Matrix<tinytype, Dynamic, 1>>(R_constrained_data, NINPUTS);

    */

    /*
    // input constraints
    tinyVector vec(4, 1);
    vec << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
    work.u_min = vec.replicate(1, NHORIZON - 1);

    tinyVector vec1(4, 1);
    vec1 << 1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3];
    work.u_max = vec1.replicate(1, NHORIZON - 1);

    for (int i = 0; i < NHORIZON; i++)
    {
      work.x_min(i) = -1000; // Fill with -1000
      work.x_max(i) = 1000;  // Fill with 1000
    }

    work.Xref = tinyMatrix::Zero(NSTATES, NHORIZON);
    work.Uref = tinyMatrix::Zero(NINPUTS, NHORIZON);
    */



    /*
    // Initialize problem data to zero
    resetProblem();

    work.primal_residual_state = 0;
    work.primal_residual_input = 0;
    work.dual_residual_state = 0;
    work.dual_residual_input = 0;
    work.status = 0;
    work.iter = 0;

    // // Copy reference trajectory into Eigen matrix
    // Xref_total = Eigen::Map<Matrix<tinytype, NTOTAL, NSTATES, Eigen::RowMajor>>(Xref_data).transpose();
    // Xref_total = Eigen::Map<Matrix<tinytype, NTOTAL, 3, Eigen::RowMajor>>(Xref_data).transpose();
    // Xref_origin << Xref_total.col(0).head(3), 0, 0, 0, 0, 0, 0, 0, 0, 0; // Go to xyz start of traj
    // Xref_end << Xref_total.col(NTOTAL-1).head(3), 0, 0, 0, 0, 0, 0, 0, 0, 0; // Go to xyz start of traj
    // Xref_origin << Xref_total.col(0), 0, 0, 0, 0, 0, 0, 0, 0, 0; // Go to xyz start of traj
    // Xref_end << Xref_total.col(NTOTAL-1).head(3), 0, 0, 0, 0, 0, 0, 0, 0, 0; // Go to xyz start of traj
    Xref_origin << 0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0; // Always go to 0, 0, 1 (comment out enable_traj = true check in main loop)


    enable_traj = false;
    traj_index = 0;
    max_traj_index = NTOTAL - NHORIZON;

    */




    // somehow the section above this is still bricking - loading red

    /* Begin task initialization */
    // runTaskSemaphore = xSemaphoreCreateBinary();
    // ASSERT(runTaskSemaphore);

    // dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

    // STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);

    // isInit = true;
    /* End of task initialization */
  }

  bool controllerOutOfTreeTest()
  {
    // Always return true
    return true;
  }

  void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
  {
    // Implement your controller here...

    // Call the PID controller instead in this example to make it possible to fly
    controllerPid(control, setpoint, sensors, state, tick);
  }

#ifdef __cplusplus
} /* extern "C" */
#endif