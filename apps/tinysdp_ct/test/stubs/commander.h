#pragma once
#include <stdbool.h>
typedef enum { modeDisable = 0, modeAbs, modeVelocity } stab_mode_t;
typedef struct { float x, y, z; } vec3_s;
typedef struct { float roll, pitch, yaw; } attitude_t;
typedef struct { unsigned timestamp; attitude_t attitude, attitudeRate; float thrust; vec3_s position, velocity, acceleration; bool velocity_body;
  struct { stab_mode_t x, y, z, roll, pitch, yaw, quat; } mode; } setpoint_t;
#define COMMANDER_PRIORITY_EXTRX 3
static inline void commanderSetSetpoint(setpoint_t* s, int p) { (void)s; (void)p; }
static inline void commanderRelaxPriority(void) {}
