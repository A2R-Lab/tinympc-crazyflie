#ifndef TINYRACER_REACTIVE_ESCAPE_H
#define TINYRACER_REACTIVE_ESCAPE_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Deliberately small, route-free policy:
 *
 *   CRUISE --center risk >= trigger--> BRAKE
 *   BRAKE --vehicle settled--> TURN_SCAN (ordinary obstacle)
 *   BRAKE --confirmed rail opening--> TRANSLATE_OPENING
 *   TURN_SCAN --center risk <= release for N frames--> CRUISE
 *   TRANSLATE_OPENING --center risk <= release for N frames--> CRUISE
 *
 * The selected maneuver direction is latched at entry so frame-to-frame noise
 * cannot alternate yaw or lateral direction while the vehicle is stopped. */
typedef enum {
  TINYRACER_REACTIVE_CRUISE = 0,
  TINYRACER_REACTIVE_BRAKE = 1,
  TINYRACER_REACTIVE_TURN_SCAN = 2,
  TINYRACER_REACTIVE_TRANSLATE_OPENING = 3,
} TinyRacerReactivePhase;

typedef struct {
  float cruise_speed_mps;
  float trigger_probability;
  float release_probability;
  float turn_rate_deg_s;
  float translation_speed_mps;
  float side_ambiguity_margin;
  uint8_t settled_samples_required;
  uint8_t minimum_maneuver_samples;
  uint8_t clear_samples_required;
  uint8_t rail_cue_samples_required;
} TinyRacerReactiveConfig;

typedef struct {
  TinyRacerReactivePhase phase;
  int8_t turn_direction;
  int8_t previous_turn_direction;
  uint8_t settled_samples;
  uint8_t maneuver_samples;
  uint8_t clear_samples;
  bool scan_clear_latched;
  int8_t rail_candidate_direction;
  uint8_t rail_cue_samples;
  bool rail_direction_latched;
} TinyRacerReactiveState;

typedef struct {
  float forward_speed_mps;
  float lateral_speed_mps;
  float yaw_rate_deg_s;
  TinyRacerReactivePhase phase;
  int8_t turn_direction;
  bool changed;
} TinyRacerReactiveCommand;

static inline TinyRacerReactiveConfig tinyRacerReactiveDefaultConfig(
    float cruise_speed_mps) {
  const TinyRacerReactiveConfig config = {
      cruise_speed_mps,
      0.85f,
      0.55f,
      28.64788976f, /* 0.50 rad/s medium scan rate. */
      0.25f,
      0.05f,
      6u,
      24u,
      3u,
      2u,
  };
  return config;
}

static inline void tinyRacerReactiveReset(TinyRacerReactiveState *state) {
  if (state == NULL) {
    return;
  }
  state->phase = TINYRACER_REACTIVE_CRUISE;
  state->turn_direction = 0;
  state->previous_turn_direction = -1;
  state->settled_samples = 0u;
  state->maneuver_samples = 0u;
  state->clear_samples = 0u;
  state->scan_clear_latched = false;
  state->rail_candidate_direction = 0;
  state->rail_cue_samples = 0u;
  state->rail_direction_latched = false;
}

static inline bool tinyRacerReactiveConfigValid(
    const TinyRacerReactiveConfig *config) {
  return config != NULL && isfinite(config->cruise_speed_mps) &&
      config->cruise_speed_mps > 0.0f &&
      isfinite(config->trigger_probability) &&
      isfinite(config->release_probability) &&
      config->trigger_probability > config->release_probability &&
      config->trigger_probability <= 1.0f &&
      config->release_probability >= 0.0f &&
      isfinite(config->turn_rate_deg_s) && config->turn_rate_deg_s > 0.0f &&
      isfinite(config->translation_speed_mps) &&
      config->translation_speed_mps > 0.0f &&
      isfinite(config->side_ambiguity_margin) &&
      config->side_ambiguity_margin >= 0.0f &&
      config->settled_samples_required > 0u &&
      config->minimum_maneuver_samples > 0u &&
      config->clear_samples_required > 0u &&
      config->rail_cue_samples_required > 0u;
}

static inline int8_t tinyRacerReactiveChooseDirection(
    TinyRacerReactiveState *state, float left_probability,
    float right_probability, float ambiguity_margin) {
  if (left_probability + ambiguity_margin < right_probability) {
    return 1;
  }
  if (right_probability + ambiguity_margin < left_probability) {
    return -1;
  }
  /* Alternate genuinely ambiguous encounters instead of encoding a permanent
   * map-specific left/right preference. */
  return state->previous_turn_direction == 1 ? -1 : 1;
}

static inline TinyRacerReactiveCommand tinyRacerReactiveStepWithRail(
    TinyRacerReactiveState *state, const TinyRacerReactiveConfig *config,
    float left_probability, float center_probability,
    float right_probability, bool rail_direction_valid,
    int8_t rail_opening_direction, bool vehicle_settled) {
  TinyRacerReactiveCommand command = {
      0.0f, 0.0f, 0.0f, TINYRACER_REACTIVE_BRAKE, 0, false};
  if (state == NULL || !tinyRacerReactiveConfigValid(config) ||
      !isfinite(left_probability) || !isfinite(center_probability) ||
      !isfinite(right_probability)) {
    return command;
  }

  const float left = fminf(fmaxf(left_probability, 0.0f), 1.0f);
  const float center = fminf(fmaxf(center_probability, 0.0f), 1.0f);
  const float right = fminf(fmaxf(right_probability, 0.0f), 1.0f);

  if (state->phase == TINYRACER_REACTIVE_CRUISE &&
      center >= config->trigger_probability) {
    state->phase = TINYRACER_REACTIVE_BRAKE;
    state->turn_direction = tinyRacerReactiveChooseDirection(
        state, left, right, config->side_ambiguity_margin);
    state->previous_turn_direction = state->turn_direction;
    state->settled_samples = 0u;
    state->maneuver_samples = 0u;
    state->clear_samples = 0u;
    state->scan_clear_latched = false;
    state->rail_candidate_direction = 0;
    state->rail_cue_samples = 0u;
    state->rail_direction_latched = false;
    command.changed = true;
  } else if (state->phase == TINYRACER_REACTIVE_BRAKE) {
    if (vehicle_settled) {
      if (state->settled_samples < UINT8_MAX) {
        ++state->settled_samples;
      }
    } else {
      state->settled_samples = 0u;
    }
    /* The v9 close-rail head was trained for a stopped/near-hovering camera.
     * Require consecutive settled samples before it may replace the ordinary
     * lower-risk turn direction. Once confirmed, keep the opening direction
     * latched through the rest of this encounter. */
    if (vehicle_settled && !state->rail_direction_latched) {
      const int8_t bounded_direction = rail_opening_direction > 0 ? 1 :
          (rail_opening_direction < 0 ? -1 : 0);
      if (rail_direction_valid && bounded_direction != 0) {
        if (state->rail_candidate_direction == bounded_direction) {
          if (state->rail_cue_samples < UINT8_MAX) {
            ++state->rail_cue_samples;
          }
        } else {
          state->rail_candidate_direction = bounded_direction;
          state->rail_cue_samples = 1u;
        }
        if (state->rail_cue_samples >= config->rail_cue_samples_required) {
          state->turn_direction = bounded_direction;
          state->previous_turn_direction = bounded_direction;
          state->rail_direction_latched = true;
        }
      } else {
        state->rail_candidate_direction = 0;
        state->rail_cue_samples = 0u;
      }
    }
    if (state->settled_samples >= config->settled_samples_required) {
      state->phase = state->rail_direction_latched
          ? TINYRACER_REACTIVE_TRANSLATE_OPENING
          : TINYRACER_REACTIVE_TURN_SCAN;
      state->settled_samples = 0u;
      command.changed = true;
    }
  } else if (state->phase == TINYRACER_REACTIVE_TURN_SCAN ||
             state->phase == TINYRACER_REACTIVE_TRANSLATE_OPENING) {
    if (state->scan_clear_latched) {
      /* Once a clear view has been found, finish arresting yaw or translation
       * before reconsidering vision. Otherwise a changing camera view can
       * restart the maneuver every few frames and livelock the vehicle in
       * place. Any still-dangerous view retriggers BRAKE after CRUISE. */
      if (vehicle_settled) {
        if (state->settled_samples < UINT8_MAX) {
          ++state->settled_samples;
        }
      } else {
        state->settled_samples = 0u;
      }
      if (state->settled_samples >= config->settled_samples_required) {
        state->phase = TINYRACER_REACTIVE_CRUISE;
        state->turn_direction = 0;
        state->settled_samples = 0u;
        state->maneuver_samples = 0u;
        state->clear_samples = 0u;
        state->scan_clear_latched = false;
        command.changed = true;
      }
    } else {
      if (state->maneuver_samples < UINT8_MAX) {
        ++state->maneuver_samples;
      }
      if (center <= config->release_probability) {
        if (state->clear_samples < UINT8_MAX) {
          ++state->clear_samples;
        }
      } else {
        state->clear_samples = 0u;
      }
      if (state->maneuver_samples >= config->minimum_maneuver_samples &&
          state->clear_samples >= config->clear_samples_required) {
        state->scan_clear_latched = true;
        state->settled_samples = 0u;
        command.changed = true;
      }
    }
  }

  command.phase = state->phase;
  command.turn_direction = state->turn_direction;
  if (state->phase == TINYRACER_REACTIVE_CRUISE) {
    command.forward_speed_mps = config->cruise_speed_mps;
  } else if (state->phase == TINYRACER_REACTIVE_TURN_SCAN &&
             !state->scan_clear_latched) {
    command.yaw_rate_deg_s =
        (float)state->turn_direction * config->turn_rate_deg_s;
  } else if (state->phase == TINYRACER_REACTIVE_TRANSLATE_OPENING &&
             !state->scan_clear_latched) {
    command.lateral_speed_mps =
        (float)state->turn_direction * config->translation_speed_mps;
  }
  return command;
}

static inline TinyRacerReactiveCommand tinyRacerReactiveStep(
    TinyRacerReactiveState *state, const TinyRacerReactiveConfig *config,
    float left_probability, float center_probability,
    float right_probability, bool vehicle_settled) {
  return tinyRacerReactiveStepWithRail(
      state, config, left_probability, center_probability, right_probability,
      false, 0, vehicle_settled);
}

#ifdef __cplusplus
}
#endif

#endif
