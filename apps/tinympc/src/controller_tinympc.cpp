// controller_tinympc.cpp - PHASE 3.18: Test Specific Problematic Operations
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "controller.h"
#include "controller_pid.h"
#include "debug.h"
#include "stabilizer_types.h"
#include "commander.h"

#ifdef __cplusplus
}

// PHASE 3.18: Test specific problematic operations
#define EIGEN_NO_MALLOC
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_PARALLELIZE
#define EIGEN_NO_DEBUG
#define EIGEN_NO_ALIGN
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>

// PHASE 3.18: Include TinyMPC headers
#include <tinympc/types.hpp>
#include <tinympc/tiny_api.hpp>

#endif

// PHASE 3.18: Use very small matrices to isolate the issue
#define NSTATES 2  // Very small
#define NINPUTS 1  // Very small
#define NHORIZON 2

// PHASE 3.18: Don't include quadrotor parameters
// #ifdef __cplusplus
// extern "C" {
// #endif
// #include "quadrotor_50hz_params.hpp"
// #ifdef __cplusplus
// }
// #endif

#define DEBUG_MODULE "TINY_MPC_PHASE3_18"

// Global counter for appMain
static uint32_t test_counter = 0;

// PHASE 3.18: Test specific problematic operations
static void testSpecificProblematicOperations(void) {
    DEBUG_PRINT("PHASE 3.18: Testing specific problematic operations...\n");
    
    // Test 1: Basic TinyMPC matrix creation (we know this works)
    DEBUG_PRINT("PHASE 3.18: Test 1 - Basic TinyMPC matrix creation...\n");
    {
        tinyMatrix test_matrix = tinyMatrix::Identity(NSTATES, NSTATES);
        DEBUG_PRINT("PHASE 3.18: Basic matrix creation successful\n");
        DEBUG_PRINT("PHASE 3.18: test_matrix(0,0) = %.2f\n", (double)test_matrix(0,0));
    }
    
    // Test 2: Matrix transpose (this might be the issue)
    DEBUG_PRINT("PHASE 3.18: Test 2 - Matrix transpose...\n");
    {
        tinyMatrix A = tinyMatrix::Random(NSTATES, NINPUTS);
        tinyMatrix A_T = A.transpose();
        DEBUG_PRINT("PHASE 3.18: Matrix transpose successful\n");
        DEBUG_PRINT("PHASE 3.18: A_T(0,0) = %.3f\n", (double)A_T(0,0));
    }
    
    // Test 3: Matrix multiplication (this might be the issue)
    DEBUG_PRINT("PHASE 3.18: Test 3 - Matrix multiplication...\n");
    {
        tinyMatrix A = tinyMatrix::Random(NSTATES, NSTATES);
        tinyMatrix B = tinyMatrix::Random(NSTATES, NINPUTS);
        tinyMatrix C = A * B;
        DEBUG_PRINT("PHASE 3.18: Matrix multiplication successful\n");
        DEBUG_PRINT("PHASE 3.18: C(0,0) = %.3f\n", (double)C(0,0));
    }
    
    // Test 4: Matrix addition (this might be the issue)
    DEBUG_PRINT("PHASE 3.18: Test 4 - Matrix addition...\n");
    {
        tinyMatrix A = tinyMatrix::Identity(NSTATES, NSTATES);
        tinyMatrix B = tinyMatrix::Identity(NSTATES, NSTATES) * 2.0;
        tinyMatrix C = A + B;
        DEBUG_PRINT("PHASE 3.18: Matrix addition successful\n");
        DEBUG_PRINT("PHASE 3.18: C(0,0) = %.2f\n", (double)C(0,0));
    }
    
    // Test 5: Matrix inverse (this is very likely the issue)
    DEBUG_PRINT("PHASE 3.18: Test 5 - Matrix inverse...\n");
    {
        tinyMatrix A = tinyMatrix::Identity(NSTATES, NSTATES) * 2.0;
        tinyMatrix A_inv = A.inverse();
        DEBUG_PRINT("PHASE 3.18: Matrix inverse successful\n");
        DEBUG_PRINT("PHASE 3.18: A_inv(0,0) = %.2f\n", (double)A_inv(0,0));
    }
    
    // Test 6: Complex matrix expression (this might be the issue)
    DEBUG_PRINT("PHASE 3.18: Test 6 - Complex matrix expression...\n");
    {
        tinyMatrix A = tinyMatrix::Random(NSTATES, NSTATES);
        tinyMatrix B = tinyMatrix::Random(NSTATES, NINPUTS);
        tinyMatrix P = tinyMatrix::Identity(NSTATES, NSTATES) * 2.0;
        
        // This is the type of expression that might cause issues
        tinyMatrix result = B.transpose() * P * B;
        DEBUG_PRINT("PHASE 3.18: Complex matrix expression successful\n");
        DEBUG_PRINT("PHASE 3.18: result(0,0) = %.3f\n", (double)result(0,0));
    }
    
    // Test 7: Riccati-like operation (this is most likely the issue)
    DEBUG_PRINT("PHASE 3.18: Test 7 - Riccati-like operation...\n");
    {
        tinyMatrix A = tinyMatrix::Random(NSTATES, NSTATES);
        tinyMatrix B = tinyMatrix::Random(NSTATES, NINPUTS);
        tinyMatrix Q = tinyMatrix::Identity(NSTATES, NSTATES) * 10.0;
        tinyMatrix R = tinyMatrix::Identity(NINPUTS, NINPUTS) * 1.0;
        tinyMatrix P = tinyMatrix::Identity(NSTATES, NSTATES) * 2.0;
        
        // This is the exact type of operation from the Riccati recursion
        tinyMatrix R1 = R + tinyMatrix::Identity(NINPUTS, NINPUTS);
        tinyMatrix Riccati_matrix = R1 + B.transpose() * P * B;
        
        DEBUG_PRINT("PHASE 3.18: Riccati-like operation successful\n");
        DEBUG_PRINT("PHASE 3.18: Riccati_matrix(0,0) = %.3f\n", (double)Riccati_matrix(0,0));
    }
    
    // Test 8: Riccati matrix inversion (this is the most likely culprit)
    DEBUG_PRINT("PHASE 3.18: Test 8 - Riccati matrix inversion...\n");
    {
        tinyMatrix A = tinyMatrix::Random(NSTATES, NSTATES);
        tinyMatrix B = tinyMatrix::Random(NSTATES, NINPUTS);
        tinyMatrix Q = tinyMatrix::Identity(NSTATES, NSTATES) * 10.0;
        tinyMatrix R = tinyMatrix::Identity(NINPUTS, NINPUTS) * 1.0;
        tinyMatrix P = tinyMatrix::Identity(NSTATES, NSTATES) * 2.0;
        
        // This is the exact operation that might fail
        tinyMatrix R1 = R + tinyMatrix::Identity(NINPUTS, NINPUTS);
        tinyMatrix Riccati_matrix = R1 + B.transpose() * P * B;
        tinyMatrix Riccati_inv = Riccati_matrix.inverse();
        
        DEBUG_PRINT("PHASE 3.18: Riccati matrix inversion successful!\n");
        DEBUG_PRINT("PHASE 3.18: Riccati_inv(0,0) = %.3f\n", (double)Riccati_inv(0,0));
    }
    
    DEBUG_PRINT("PHASE 3.18: All specific problematic operations tests PASSED!\n");
}

// Controller interface functions
extern "C" void controllerOutOfTreeInit(void) {
    DEBUG_PRINT("TinyMPC Phase 3.18: Controller init\n");
    
    // Test specific problematic operations
    testSpecificProblematicOperations();
    
    // Initialize PID controller as fallback
    controllerPidInit();
    
    DEBUG_PRINT("TinyMPC Phase 3.18: Init complete\n");
}

extern "C" bool controllerOutOfTreeTest(void) {
    // Return true for this test
    return true;
  }

extern "C" void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t tick) {

    // Phase 3.18: Use PID controller as fallback
    controllerPid(control, setpoint, sensors, state, tick);
    
    // Add some debug output (very infrequent to avoid issues)
    static uint32_t debug_counter = 0;
    if (++debug_counter % 1000 == 0) {
        DEBUG_PRINT("TinyMPC Phase 3.18: PID control, thrust=%u\n", (unsigned int)control->thrust);
    }
}

// RTOS-compliant app main function
extern "C" void appMain(void) {
    DEBUG_PRINT("TinyMPC Phase 3.18 App: Starting...\n");
    while (1) {
        vTaskDelay(M2T(1000));  // Yield for 1 second
        DEBUG_PRINT("TinyMPC Phase 3.18 App: alive %lu\n", test_counter++);
    }
}
