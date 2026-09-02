#ifndef TINYMPC_BRAKING_SELECTOR_H
#define TINYMPC_BRAKING_SELECTOR_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TINYMPC_BRAKING_TIER_COUNT 5u

typedef enum {
  TINYMPC_BRAKING_MODEL_LEVEL = 0,
  TINYMPC_BRAKING_MODEL_SPEED_1_0_MPS = 11,
  TINYMPC_BRAKING_MODEL_SPEED_1_5_MPS = 12,
  TINYMPC_BRAKING_MODEL_SPEED_2_0_MPS = 13,
  TINYMPC_BRAKING_MODEL_SPEED_2_5_MPS = 14,
  TINYMPC_BRAKING_MODEL_SPEED_3_0_MPS = 15,
} TinyMpcBrakingModelId;

typedef struct {
  /* Positive braking-deceleration magnitudes in m/s^2. */
  float enter_deceleration_mps2;
  float exit_deceleration_mps2;
  /* Positive magnitude required of the negative pitch reference at entry. */
  float minimum_braking_pitch_rad;
  /* Exact negative pitch at each 1.0--3.0 m/s frozen phase point. */
  float nominal_pitch_rad[TINYMPC_BRAKING_TIER_COUNT];
  float maximum_level_roll_rad;
  float maximum_entry_speed_error_mps;
  float maximum_entry_pitch_error_rad;
  float speed_hysteresis_mps;
  uint16_t minimum_dwell_steps;
} TinyMpcBrakingSelectorConfig;

typedef struct {
  int active_model_id;
  uint16_t steps_since_switch;
  uint32_t switch_count;
} TinyMpcBrakingSelector;

typedef struct {
  int active_model_id;
  bool sample_valid;
  bool entry_local;
  bool switched;
  bool reset_optimizer;
} TinyMpcBrakingSelection;

static inline bool tinyMpcBrakingSelectorConfigValid(
    const TinyMpcBrakingSelectorConfig *config) {
  return config != NULL
      && isfinite(config->enter_deceleration_mps2)
      && isfinite(config->exit_deceleration_mps2)
      && isfinite(config->minimum_braking_pitch_rad)
      && isfinite(config->maximum_level_roll_rad)
      && isfinite(config->maximum_entry_speed_error_mps)
      && isfinite(config->maximum_entry_pitch_error_rad)
      && isfinite(config->speed_hysteresis_mps)
      && config->enter_deceleration_mps2
          > config->exit_deceleration_mps2
      && config->exit_deceleration_mps2 >= 0.0f
      && config->minimum_braking_pitch_rad > 0.0f
      && config->maximum_level_roll_rad >= 0.0f
      && config->maximum_entry_speed_error_mps >= 0.0f
      && config->maximum_entry_speed_error_mps < 0.5f
      && config->maximum_entry_pitch_error_rad >= 0.0f
      && config->speed_hysteresis_mps >= 0.0f
      && config->speed_hysteresis_mps < 0.25f
      && isfinite(config->nominal_pitch_rad[0])
      && isfinite(config->nominal_pitch_rad[1])
      && isfinite(config->nominal_pitch_rad[2])
      && isfinite(config->nominal_pitch_rad[3])
      && isfinite(config->nominal_pitch_rad[4])
      && config->nominal_pitch_rad[0] < 0.0f
      && config->nominal_pitch_rad[1] < 0.0f
      && config->nominal_pitch_rad[2] < 0.0f
      && config->nominal_pitch_rad[3] < 0.0f
      && config->nominal_pitch_rad[4] < 0.0f;
}

static inline bool tinyMpcBrakingModelIdValid(int model_id) {
  return model_id == (int)TINYMPC_BRAKING_MODEL_LEVEL
      || (model_id >= (int)TINYMPC_BRAKING_MODEL_SPEED_1_0_MPS
          && model_id <= (int)TINYMPC_BRAKING_MODEL_SPEED_3_0_MPS);
}

static inline uint8_t tinyMpcBrakingTierForModel(int model_id) {
  return model_id >= (int)TINYMPC_BRAKING_MODEL_SPEED_1_0_MPS
          && model_id <= (int)TINYMPC_BRAKING_MODEL_SPEED_3_0_MPS
      ? (uint8_t)(model_id - 10) : 0u;
}

static inline int tinyMpcBrakingModelForTier(uint8_t tier) {
  return tier >= 1u && tier <= TINYMPC_BRAKING_TIER_COUNT
      ? (int)tier + 10 : (int)TINYMPC_BRAKING_MODEL_LEVEL;
}

static inline float tinyMpcBrakingNominalSpeedMps(uint8_t tier) {
  return tier >= 1u && tier <= TINYMPC_BRAKING_TIER_COUNT
      ? 0.5f + 0.5f * (float)tier : 0.0f;
}

static inline float tinyMpcBrakingNominalPitchRad(
    uint8_t tier, const TinyMpcBrakingSelectorConfig *config) {
  return config != NULL && tier >= 1u && tier <= TINYMPC_BRAKING_TIER_COUNT
      ? config->nominal_pitch_rad[tier - 1u] : 0.0f;
}

static inline uint8_t tinyMpcNearestBrakingSpeedTier(float speed_mps) {
  if (!isfinite(speed_mps) || speed_mps < 0.0f) {
    return 0u;
  }
  uint8_t tier = 1u;
  for (uint8_t candidate = 2u;
       candidate <= TINYMPC_BRAKING_TIER_COUNT; ++candidate) {
    const float boundary = 0.5f * (
        tinyMpcBrakingNominalSpeedMps(candidate - 1u)
        + tinyMpcBrakingNominalSpeedMps(candidate));
    if (speed_mps < boundary) {
      break;
    }
    tier = candidate;
  }
  return tier;
}

static inline uint8_t tinyMpcRequestedBrakingSpeedTier(
    uint8_t active_tier,
    float measured_forward_speed_mps,
    float hysteresis_mps) {
  if (active_tier == 0u || active_tier > TINYMPC_BRAKING_TIER_COUNT) {
    return tinyMpcNearestBrakingSpeedTier(measured_forward_speed_mps);
  }
  uint8_t requested = active_tier;
  while (requested < TINYMPC_BRAKING_TIER_COUNT) {
    const float boundary = 0.5f * (
        tinyMpcBrakingNominalSpeedMps(requested)
        + tinyMpcBrakingNominalSpeedMps(requested + 1u));
    if (measured_forward_speed_mps < boundary + hysteresis_mps) {
      break;
    }
    ++requested;
  }
  while (requested > 1u) {
    const float boundary = 0.5f * (
        tinyMpcBrakingNominalSpeedMps(requested - 1u)
        + tinyMpcBrakingNominalSpeedMps(requested));
    if (measured_forward_speed_mps >= boundary - hysteresis_mps) {
      break;
    }
    --requested;
  }
  return requested;
}

static inline void tinyMpcBrakingSelectorReset(
    TinyMpcBrakingSelector *selector,
    const TinyMpcBrakingSelectorConfig *config) {
  if (selector == NULL) {
    return;
  }
  selector->active_model_id = (int)TINYMPC_BRAKING_MODEL_LEVEL;
  selector->switch_count = 0u;
  /* Reset is initialization, so a local braking sample may enter at once. */
  selector->steps_since_switch = tinyMpcBrakingSelectorConfigValid(config)
      ? config->minimum_dwell_steps : 0u;
}

static inline void tinyMpcBrakingFailClosedToLevel(
    TinyMpcBrakingSelector *selector,
    TinyMpcBrakingSelection *selection) {
  if (selector->active_model_id != (int)TINYMPC_BRAKING_MODEL_LEVEL) {
    selector->active_model_id = (int)TINYMPC_BRAKING_MODEL_LEVEL;
    selector->steps_since_switch = 0u;
    ++selector->switch_count;
    selection->switched = true;
    selection->reset_optimizer = true;
  } else if (selector->steps_since_switch < UINT16_MAX) {
    ++selector->steps_since_switch;
  }
  selection->active_model_id = selector->active_model_id;
}

/*
 * Select a straight-braking cache using measured forward speed. Entry from
 * level is authorized only by an explicit braking-deceleration reference and
 * additionally requires measured pitch to be local to the pitch reference and
 * measured speed to be local to one of the five frozen speed points. Any
 * nonfinite input or nontrivial reference/measured roll immediately selects
 * LEVEL so a coordinated-turn cache remains authoritative.
 */
static inline TinyMpcBrakingSelection tinyMpcBrakingSelectorUpdate(
    TinyMpcBrakingSelector *selector,
    const TinyMpcBrakingSelectorConfig *config,
    float reference_deceleration_mps2,
    float reference_pitch_rad,
    float measured_pitch_rad,
    float reference_roll_rad,
    float measured_roll_rad,
    float measured_forward_speed_mps) {
  TinyMpcBrakingSelection result = {
      (int)TINYMPC_BRAKING_MODEL_LEVEL, false, false, false, false};
  if (selector == NULL || !tinyMpcBrakingSelectorConfigValid(config)) {
    return result;
  }
  if (!tinyMpcBrakingModelIdValid(selector->active_model_id)) {
    selector->active_model_id = (int)TINYMPC_BRAKING_MODEL_LEVEL;
    selector->steps_since_switch = config->minimum_dwell_steps;
    selector->switch_count = 0u;
  }
  result.active_model_id = selector->active_model_id;

  result.sample_valid = isfinite(reference_deceleration_mps2)
      && reference_deceleration_mps2 >= 0.0f
      && isfinite(reference_pitch_rad)
      && isfinite(measured_pitch_rad)
      && isfinite(reference_roll_rad)
      && isfinite(measured_roll_rad)
      && isfinite(measured_forward_speed_mps)
      && measured_forward_speed_mps >= 0.0f;
  if (!result.sample_valid) {
    tinyMpcBrakingFailClosedToLevel(selector, &result);
    return result;
  }

  const bool roll_is_level =
      fabsf(reference_roll_rad) <= config->maximum_level_roll_rad
      && fabsf(measured_roll_rad) <= config->maximum_level_roll_rad;
  if (!roll_is_level) {
    tinyMpcBrakingFailClosedToLevel(selector, &result);
    return result;
  }

  const bool reference_requests_braking =
      reference_deceleration_mps2 >= config->enter_deceleration_mps2
      && reference_pitch_rad <= -config->minimum_braking_pitch_rad;
  const bool reference_requests_level =
      reference_deceleration_mps2 <= config->exit_deceleration_mps2;
  const uint8_t active_tier = tinyMpcBrakingTierForModel(
      selector->active_model_id);
  int requested_model_id = selector->active_model_id;

  if (active_tier == 0u) {
    if (reference_requests_braking) {
      const uint8_t candidate_tier = tinyMpcNearestBrakingSpeedTier(
          measured_forward_speed_mps);
      const float candidate_speed_mps = tinyMpcBrakingNominalSpeedMps(
          candidate_tier);
      const float candidate_pitch_rad = tinyMpcBrakingNominalPitchRad(
          candidate_tier, config);
      result.entry_local = candidate_tier != 0u
          && fabsf(measured_forward_speed_mps - candidate_speed_mps)
              <= config->maximum_entry_speed_error_mps
          && fabsf(reference_pitch_rad - candidate_pitch_rad)
              <= config->maximum_entry_pitch_error_rad
          && fabsf(measured_pitch_rad - candidate_pitch_rad)
              <= config->maximum_entry_pitch_error_rad;
      if (result.entry_local) {
        requested_model_id = tinyMpcBrakingModelForTier(candidate_tier);
      }
    }
  } else if (reference_requests_level) {
    requested_model_id = (int)TINYMPC_BRAKING_MODEL_LEVEL;
  } else if (reference_requests_braking) {
    const uint8_t requested_tier = tinyMpcRequestedBrakingSpeedTier(
        active_tier, measured_forward_speed_mps,
        config->speed_hysteresis_mps);
    const float requested_pitch_rad = tinyMpcBrakingNominalPitchRad(
        requested_tier, config);
    if (fabsf(reference_pitch_rad - requested_pitch_rad)
            <= config->maximum_entry_pitch_error_rad
        && fabsf(measured_pitch_rad - requested_pitch_rad)
            <= config->maximum_entry_pitch_error_rad) {
      requested_model_id = tinyMpcBrakingModelForTier(requested_tier);
    }
  }

  const bool dwell_complete = selector->steps_since_switch
      >= config->minimum_dwell_steps;
  if (requested_model_id != selector->active_model_id && dwell_complete) {
    selector->active_model_id = requested_model_id;
    selector->steps_since_switch = 0u;
    ++selector->switch_count;
    result.switched = true;
    result.reset_optimizer = true;
  } else if (selector->steps_since_switch < UINT16_MAX) {
    ++selector->steps_since_switch;
  }
  result.active_model_id = selector->active_model_id;
  return result;
}

#ifdef __cplusplus
}
#endif

#endif
