#ifndef TINYMPC_REACTIVE_POWER_LOOP_H
#define TINYMPC_REACTIVE_POWER_LOOP_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TINYMPC_REACTIVE_LOOP_WAITING_CLEAR = 0,
  TINYMPC_REACTIVE_LOOP_BRAKING = 1,
  TINYMPC_REACTIVE_LOOP_ACTIVE = 2,
  TINYMPC_REACTIVE_LOOP_RECOVERY = 3,
  TINYMPC_REACTIVE_LOOP_COMPLETE = 4,
} TinyMpcReactivePowerLoopPhase;

typedef struct {
  float maximum_center_risk;
  uint8_t clear_samples_required;
  uint8_t settled_samples_required;
  uint8_t recovery_samples_required;
} TinyMpcReactivePowerLoopConfig;

typedef struct {
  TinyMpcReactivePowerLoopPhase phase;
  uint8_t clear_samples;
  uint8_t settled_samples;
  uint8_t recovery_samples;
  uint16_t trigger_count;
} TinyMpcReactivePowerLoopState;

typedef struct {
  bool changed;
  bool request_stop;
  bool owns_reference;
  bool active;
} TinyMpcReactivePowerLoopCommand;

static inline TinyMpcReactivePowerLoopConfig
tinyMpcReactivePowerLoopDefaultConfig(void) {
  const TinyMpcReactivePowerLoopConfig config = {
      0.20f, 1u, 6u, 10u};
  return config;
}

static inline void tinyMpcReactivePowerLoopReset(
    TinyMpcReactivePowerLoopState *state) {
  if (state == NULL) {
    return;
  }
  state->phase = TINYMPC_REACTIVE_LOOP_WAITING_CLEAR;
  state->clear_samples = 0u;
  state->settled_samples = 0u;
  state->recovery_samples = 0u;
  state->trigger_count = 0u;
}

static inline bool tinyMpcReactivePowerLoopConfigValid(
    const TinyMpcReactivePowerLoopConfig *config) {
  return config != NULL && isfinite(config->maximum_center_risk) &&
      config->maximum_center_risk >= 0.0f &&
      config->maximum_center_risk < 1.0f &&
      config->clear_samples_required > 0u &&
      config->settled_samples_required > 0u &&
      config->recovery_samples_required > 0u;
}

/* Conservatively rotate the trajectory's horizontal bounding box into the
 * arena. This makes a translating loop obey containment before it becomes
 * non-interruptible without scanning every stored reference knot onboard. */
static inline bool tinyMpcReactivePowerLoopPathFitsArena(
    float anchor_x_m, float anchor_y_m, float heading_rad,
    float minimum_relative_x_m, float maximum_relative_x_m,
    float minimum_relative_y_m, float maximum_relative_y_m,
    float arena_half_extent_m, float clearance_m) {
  if (!isfinite(anchor_x_m) || !isfinite(anchor_y_m) ||
      !isfinite(heading_rad) || !isfinite(minimum_relative_x_m) ||
      !isfinite(maximum_relative_x_m) || !isfinite(minimum_relative_y_m) ||
      !isfinite(maximum_relative_y_m) || !isfinite(arena_half_extent_m) ||
      !isfinite(clearance_m) || minimum_relative_x_m > maximum_relative_x_m ||
      minimum_relative_y_m > maximum_relative_y_m || clearance_m < 0.0f ||
      arena_half_extent_m <= clearance_m) {
    return false;
  }
  const float limit_m = arena_half_extent_m - clearance_m;
  const float cosine = cosf(heading_rad);
  const float sine = sinf(heading_rad);
  const float x_values[2] = {minimum_relative_x_m, maximum_relative_x_m};
  const float y_values[2] = {minimum_relative_y_m, maximum_relative_y_m};
  for (int x_index = 0; x_index < 2; ++x_index) {
    for (int y_index = 0; y_index < 2; ++y_index) {
      const float world_x_m = anchor_x_m + cosine * x_values[x_index] -
          sine * y_values[y_index];
      const float world_y_m = anchor_y_m + sine * x_values[x_index] +
          cosine * y_values[y_index];
      if (fabsf(world_x_m) > limit_m || fabsf(world_y_m) > limit_m) {
        return false;
      }
    }
  }
  return true;
}

/* The maneuver is one-shot per controller reset. Clear vision first requests
 * a controlled stop; only a settled vehicle may enter the non-interruptible
 * cached sequence. A danger/geofence event can cancel the pre-loop stop. */
static inline TinyMpcReactivePowerLoopCommand tinyMpcReactivePowerLoopUpdate(
    TinyMpcReactivePowerLoopState *state,
    const TinyMpcReactivePowerLoopConfig *config,
    bool new_observation, float center_risk, bool ordinary_cruise,
    bool containment_clear, bool vehicle_settled,
    bool maneuver_sequence_complete, bool recovery_ready) {
  TinyMpcReactivePowerLoopCommand command = {false, false, false, false};
  if (state == NULL || !tinyMpcReactivePowerLoopConfigValid(config) ||
      !isfinite(center_risk)) {
    return command;
  }

  const float bounded_risk = fminf(fmaxf(center_risk, 0.0f), 1.0f);
  if (state->phase == TINYMPC_REACTIVE_LOOP_WAITING_CLEAR) {
    if (!ordinary_cruise || !containment_clear) {
      state->clear_samples = 0u;
    } else if (new_observation) {
      if (bounded_risk <= config->maximum_center_risk) {
        if (state->clear_samples < UINT8_MAX) {
          ++state->clear_samples;
        }
      } else {
        state->clear_samples = 0u;
      }
      if (state->clear_samples >= config->clear_samples_required) {
        state->phase = TINYMPC_REACTIVE_LOOP_BRAKING;
        state->settled_samples = 0u;
        command.changed = true;
      }
    }
  } else if (state->phase == TINYMPC_REACTIVE_LOOP_BRAKING) {
    if (!ordinary_cruise || !containment_clear ||
        bounded_risk > config->maximum_center_risk) {
      state->phase = TINYMPC_REACTIVE_LOOP_WAITING_CLEAR;
      state->clear_samples = 0u;
      state->settled_samples = 0u;
      command.changed = true;
    } else if (vehicle_settled) {
      if (state->settled_samples < UINT8_MAX) {
        ++state->settled_samples;
      }
      if (state->settled_samples >= config->settled_samples_required) {
        state->phase = TINYMPC_REACTIVE_LOOP_ACTIVE;
        state->recovery_samples = 0u;
        ++state->trigger_count;
        command.changed = true;
      }
    } else {
      state->settled_samples = 0u;
    }
  } else if (state->phase == TINYMPC_REACTIVE_LOOP_ACTIVE &&
             maneuver_sequence_complete) {
    state->phase = TINYMPC_REACTIVE_LOOP_RECOVERY;
    state->recovery_samples = 0u;
    command.changed = true;
  } else if (state->phase == TINYMPC_REACTIVE_LOOP_RECOVERY) {
    if (recovery_ready) {
      if (state->recovery_samples < UINT8_MAX) {
        ++state->recovery_samples;
      }
      if (state->recovery_samples >= config->recovery_samples_required) {
        state->phase = TINYMPC_REACTIVE_LOOP_COMPLETE;
        command.changed = true;
      }
    } else {
      state->recovery_samples = 0u;
    }
  }

  command.request_stop =
      state->phase == TINYMPC_REACTIVE_LOOP_BRAKING ||
      state->phase == TINYMPC_REACTIVE_LOOP_RECOVERY;
  command.owns_reference =
      state->phase == TINYMPC_REACTIVE_LOOP_ACTIVE ||
      state->phase == TINYMPC_REACTIVE_LOOP_RECOVERY;
  command.active = state->phase == TINYMPC_REACTIVE_LOOP_ACTIVE;
  return command;
}

#ifdef __cplusplus
}
#endif

#endif
