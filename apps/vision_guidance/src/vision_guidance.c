/**
 * vision_guidance.c - App-layer visual standoff/centering for the Crazyflie.
 *
 * Pairs with the GAP8 CV module in esp_color_object/comms/comms_deck.c.
 *
 * Design (perception -> guidance -> stock controller):
 *   - This app does NOT replace the flight controller. It runs alongside the
 *     stock PID position/velocity controller (set stabilizer.controller = 1).
 *   - It periodically TRIGGERS the GAP8 camera over CPX, receives the bright-blob
 *     DetectionResponse asynchronously, and converts it into a velocity setpoint
 *     that it feeds to the commander. The PID controller then flies it.
 *
 * Behavior when armed (visGuid.enable = 1):
 *   - Hold a fixed altitude (visGuid.targetZ, needs a Flow/Z-ranging deck).
 *   - Drive forward/back to hold a target blob pixel-area  -> "standoff distance".
 *   - Yaw (and/or strafe) to keep the blob horizontally centered.
 *   - On lost target / stale detection: hold position (zero horizontal velocity).
 *
 * Operational model:
 *   1. Take off and hover normally (visGuid.enable = 0; client/commander in charge).
 *   2. Set visGuid.targetZ to your current height, then visGuid.enable = 1.
 *      The app takes over at EXTRX priority and starts visual servoing.
 *   3. Set visGuid.enable = 0 to hand control back (then land normally).
 *
 * IMPORTANT distance caveat: the GAP8 does NOT measure true depth (real_z_mm is
 * defaulted, see APP_STATUS_DEPTH_DEFAULTED). So we use blob pixel-area
 * (yellow_pixels) as an inverse-distance proxy. This holds standoff only if the
 * bright target's real-world size is roughly constant. For a true metric
 * distance, add a range sensor and switch the forward law to use it.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "app.h"
#include "system.h"
#include "commander.h"
#include "stabilizer_types.h"   // setpoint_t, stab_mode_t
#include "param.h"
#include "log.h"

// CPX app-layer messaging.
// NOTE: verify these two symbols against your crazyflie-firmware version:
//   - cpxRegisterAppMessageHandler(callback)  (RX path on STM32)
//   - cpxSendPacketBlocking(packet)           (TX path on STM32)
// Older/newer trees may name the RX registration slightly differently
// (e.g. a *PacketCallback typedef). The protocol structs below are fixed.
#include "cpx.h"                  // CPXPacket_t, cpxInitRoute, cpxRegisterAppMessageHandler
#include "cpx_internal_router.h" // cpxSendPacketBlocking

#define DEBUG_MODULE "VIS-GUID"
#include "debug.h"

// ---------------------------------------------------------------------------
// Wire protocol -- MUST stay byte-identical to esp_color_object/comms/comms_deck.c
// ---------------------------------------------------------------------------
#define FRAME_WIDTH        160
#define FRAME_HEIGHT       120

#define APP_PROTO_MAGIC    0xA5
#define APP_PROTO_VERSION  0x01

typedef enum {
    APP_MSG_TRIGGER   = 0x01,
    APP_MSG_DETECTION = 0x81,
    APP_MSG_ERROR     = 0xE1
} AppMessageType_t;

typedef enum {
    APP_STATUS_TRIGGERED      = (1 << 0),
    APP_STATUS_FOUND          = (1 << 1),
    APP_STATUS_DEPTH_DEFAULTED = (1 << 2)
} AppStatus_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  msg_type;   // APP_MSG_TRIGGER
    uint8_t  seq;
    uint8_t  flags;
    uint8_t  reserved0;
    uint16_t depth_mm;   // 0 => GAP8 uses its DEFAULT_DEPTH_MM
} TriggerRequest_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  msg_type;     // APP_MSG_DETECTION or APP_MSG_ERROR
    uint8_t  seq;
    uint8_t  status;       // AppStatus_t bits
    uint8_t  error;        // AppError_t
    int16_t  centroid_x;   // pixels
    int16_t  centroid_y;   // pixels
    uint16_t yellow_pixels;
    int16_t  real_x_mm;
    int16_t  real_y_mm;
    uint16_t real_z_mm;
    uint32_t frame_id;
    uint32_t timestamp_ms;
} DetectionResponse_t;

// ---------------------------------------------------------------------------
// Tunables (exposed as params, see PARAM_GROUP at bottom)
// ---------------------------------------------------------------------------
static uint8_t enable      = 0;       // 0 = passthrough (commander in charge), 1 = visual servo
static float   targetZ     = 0.5f;    // m, altitude to hold while servoing
static uint16_t targetArea = 1500;    // px, desired blob area (standoff setpoint)

static float kpDist   = 0.0006f;      // (m/s) per pixel of area error  -> forward/back
static float kpYaw    = 0.30f;        // (deg/s) per pixel of x error   -> yaw to center
static float kpStrafe = 0.0f;         // (m/s) per pixel of x error     -> body-y (off by default)

static float vMaxXY   = 0.30f;        // m/s   clamp on body x/y velocity
static float yawRateMax = 40.0f;      // deg/s clamp on yaw rate

static uint16_t triggerPeriodMs = 20; // ~50 Hz guidance loop
static uint16_t detTimeoutMs    = 300;// detection considered stale after this

// ---------------------------------------------------------------------------
// Shared detection state (written in CPX RX callback, read in app task)
// ---------------------------------------------------------------------------
static SemaphoreHandle_t detMutex;
static DetectionResponse_t latestDet;
static uint32_t latestDetTick = 0;    // RTOS tick when latestDet was updated
static bool     latestDetValid = false;

// ---------------------------------------------------------------------------
// Telemetry (logged)
// ---------------------------------------------------------------------------
static uint8_t logFound = 0;
static int16_t logCx = -1;
static int16_t logCy = -1;
static uint16_t logArea = 0;
static float   logVx = 0.0f;
static float   logVy = 0.0f;
static float   logYawRate = 0.0f;
static uint32_t logRxCount = 0;

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// CPX RX callback: invoked (in CPX router task context) for CPX_F_APP packets
// addressed to the STM32. Keep it short: validate + copy into shared state.
static void cpxDetectionHandler(const CPXPacket_t *packet) {
    if (packet->dataLength < sizeof(DetectionResponse_t)) {
        return;
    }
    DetectionResponse_t det;
    memcpy(&det, packet->data, sizeof(det));

    if (det.magic != APP_PROTO_MAGIC || det.version != APP_PROTO_VERSION) {
        return;
    }
    if (det.msg_type != APP_MSG_DETECTION && det.msg_type != APP_MSG_ERROR) {
        return;
    }

    if (xSemaphoreTake(detMutex, 0) == pdTRUE) {
        memcpy(&latestDet, &det, sizeof(det));
        latestDetTick = xTaskGetTickCount();
        latestDetValid = true;
        logRxCount++;
        xSemaphoreGive(detMutex);
    }
}

static void sendTrigger(uint8_t seq) {
    static CPXPacket_t tx;   // static: CPXPacket_t is ~1KB, keep it off the stack
    TriggerRequest_t req = {
        .magic    = APP_PROTO_MAGIC,
        .version  = APP_PROTO_VERSION,
        .msg_type = APP_MSG_TRIGGER,
        .seq      = seq,
        .flags    = 0,
        .reserved0 = 0,
        .depth_mm = 0,   // let GAP8 use its default depth
    };
    cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &tx.route);
    memcpy(tx.data, &req, sizeof(req));
    tx.dataLength = sizeof(req);
    cpxSendPacketBlocking(&tx);
}

// Build a hover/hold setpoint (zero horizontal velocity, hold targetZ).
static void makeHoldSetpoint(setpoint_t *sp) {
    memset(sp, 0, sizeof(*sp));
    sp->mode.x   = modeVelocity;
    sp->mode.y   = modeVelocity;
    sp->mode.z   = modeAbs;
    sp->mode.yaw = modeVelocity;
    sp->velocity_body   = true;
    sp->velocity.x      = 0.0f;
    sp->velocity.y      = 0.0f;
    sp->position.z      = targetZ;
    sp->attitudeRate.yaw = 0.0f;
}

void appMain(void) {
    detMutex = xSemaphoreCreateMutex();

    // Register before the system starts so we don't miss early detections.
    cpxRegisterAppMessageHandler(cpxDetectionHandler);

    DEBUG_PRINT("Vision guidance app started, waiting for system...\n");
    systemWaitStart();
    DEBUG_PRINT("System up. trig=%ums  enable via param visGuid.enable\n",
                (unsigned)triggerPeriodMs);

    uint8_t seq = 0;
    TickType_t lastWake = xTaskGetTickCount();

    while (1) {
        // 1) Trigger a fresh capture on the GAP8 (always, so bench/log works
        //    even when not armed). The response arrives via cpxDetectionHandler.
        sendTrigger(seq++);

        // 2) Snapshot the latest detection.
        DetectionResponse_t det;
        bool haveDet = false;
        uint32_t ageMs = detTimeoutMs + 1;
        if (xSemaphoreTake(detMutex, M2T(2)) == pdTRUE) {
            if (latestDetValid) {
                memcpy(&det, &latestDet, sizeof(det));
                haveDet = true;
                ageMs = (uint32_t)((xTaskGetTickCount() - latestDetTick) * portTICK_PERIOD_MS);
            }
            xSemaphoreGive(detMutex);
        }

        bool fresh = haveDet && (ageMs <= detTimeoutMs);
        bool found = fresh
                  && det.msg_type == APP_MSG_DETECTION
                  && (det.status & APP_STATUS_FOUND);

        // Update telemetry.
        logFound = found ? 1 : 0;
        logCx = found ? det.centroid_x : -1;
        logCy = found ? det.centroid_y : -1;
        logArea = found ? det.yellow_pixels : 0;

        // 3) Drive the controller only when armed. When disarmed we leave the
        //    commander alone so normal/manual control works.
        if (enable) {
            setpoint_t sp;
            makeHoldSetpoint(&sp);

            if (found) {
                float exPix   = (float)det.centroid_x - (FRAME_WIDTH  / 2.0f); // + = target right
                float areaErr = (float)targetArea - (float)det.yellow_pixels;  // + = too far

                float vx   = clampf(kpDist   * areaErr, -vMaxXY, vMaxXY);   // forward/back
                float vy   = clampf(-kpStrafe * exPix,  -vMaxXY, vMaxXY);   // strafe (off by default)
                float yawR = clampf(-kpYaw    * exPix,  -yawRateMax, yawRateMax); // yaw toward target

                sp.velocity.x       = vx;
                sp.velocity.y       = vy;
                sp.attitudeRate.yaw = yawR;

                logVx = vx; logVy = vy; logYawRate = yawR;
            } else {
                // Armed but no target -> hold position at altitude (makeHoldSetpoint).
                logVx = 0.0f; logVy = 0.0f; logYawRate = 0.0f;
            }

            // Priority 3 (EXTRX) overrides CRTP commander while armed.
            commanderSetSetpoint(&sp, 3);
        } else {
            logVx = 0.0f; logVy = 0.0f; logYawRate = 0.0f;
        }

        vTaskDelayUntil(&lastWake, M2T(triggerPeriodMs));
    }
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
PARAM_GROUP_START(visGuid)
PARAM_ADD(PARAM_UINT8,  enable,      &enable)
PARAM_ADD(PARAM_FLOAT,  targetZ,     &targetZ)
PARAM_ADD(PARAM_UINT16, targetArea,  &targetArea)
PARAM_ADD(PARAM_FLOAT,  kpDist,      &kpDist)
PARAM_ADD(PARAM_FLOAT,  kpYaw,       &kpYaw)
PARAM_ADD(PARAM_FLOAT,  kpStrafe,    &kpStrafe)
PARAM_ADD(PARAM_FLOAT,  vMaxXY,      &vMaxXY)
PARAM_ADD(PARAM_FLOAT,  yawRateMax,  &yawRateMax)
PARAM_ADD(PARAM_UINT16, trigMs,      &triggerPeriodMs)
PARAM_ADD(PARAM_UINT16, detTimeoutMs,&detTimeoutMs)
PARAM_GROUP_STOP(visGuid)

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
LOG_GROUP_START(visGuid)
LOG_ADD(LOG_UINT8,  found,   &logFound)
LOG_ADD(LOG_INT16,  cx,      &logCx)
LOG_ADD(LOG_INT16,  cy,      &logCy)
LOG_ADD(LOG_UINT16, area,    &logArea)
LOG_ADD(LOG_FLOAT,  vx,      &logVx)
LOG_ADD(LOG_FLOAT,  vy,      &logVy)
LOG_ADD(LOG_FLOAT,  yawRate, &logYawRate)
LOG_ADD(LOG_UINT32, rxCount, &logRxCount)
LOG_GROUP_STOP(visGuid)
