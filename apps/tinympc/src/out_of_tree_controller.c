/**
 * Minimal TinyMPC out-of-tree controller scaffold (one-file layout)
 * Based on Bitcraze example app_out_of_tree_controller
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "TINY_MPC_OOT"
#include "debug.h"

void appMain(void)
{
    DEBUG_PRINT("Waiting for activation ...\n");
    while (1) {
        vTaskDelay(M2T(2000));
    }
}

// Controller hooks
#include "controller.h"
#include "controller_pid.h"

void controllerOutOfTreeInit(void)
{
    controllerPidInit();
}

bool controllerOutOfTreeTest(void)
{
    return true;
}

void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t stabilizerStep)
{
    controllerPid(control, setpoint, sensors, state, stabilizerStep);
}


