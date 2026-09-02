#ifndef TINYMPC_BANK_SELECTOR_H
#define TINYMPC_BANK_SELECTOR_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TINYMPC_BANK_MODEL_LEVEL = 0,
  TINYMPC_BANK_MODEL_LEFT_LOW = 1,
  TINYMPC_BANK_MODEL_RIGHT_LOW = 2,
  TINYMPC_BANK_MODEL_LEFT_MEDIUM = 3,
  TINYMPC_BANK_MODEL_RIGHT_MEDIUM = 4,
  TINYMPC_BANK_MODEL_LEFT_HIGH = 5,
  TINYMPC_BANK_MODEL_RIGHT_HIGH = 6,
  TINYMPC_BANK_MODEL_LEFT_VERY_HIGH = 7,
  TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH = 8,
  TINYMPC_BANK_MODEL_LEFT_MAXIMUM = 9,
  TINYMPC_BANK_MODEL_RIGHT_MAXIMUM = 10,
  TINYMPC_BANK_MODEL_BRAKE_LOW = 11,
  TINYMPC_BANK_MODEL_BRAKE_MEDIUM = 12,
  TINYMPC_BANK_MODEL_BRAKE_HIGH = 13,
  TINYMPC_BANK_MODEL_BRAKE_VERY_HIGH = 14,
  TINYMPC_BANK_MODEL_BRAKE_MAXIMUM = 15,
} TinyMpcBankModelId;

#define TINYMPC_BANK_TIER_COUNT 5u

typedef struct {
  float low_bank_rad;
  float medium_bank_rad;
  float high_bank_rad;
  float very_high_bank_rad;
  float maximum_bank_rad;
  float enter_bank_rad;
  float exit_bank_rad;
  float transition_hysteresis_rad;
  uint16_t minimum_dwell_steps;
} TinyMpcBankSelectorConfig;

typedef struct {
  TinyMpcBankModelId active_model;
  uint16_t steps_since_switch;
  uint32_t switch_count;
} TinyMpcBankSelector;

typedef struct {
  TinyMpcBankModelId active_model;
  float signed_bank_demand_rad;
  bool demand_valid;
  bool switched;
  bool reset_optimizer;
} TinyMpcBankSelection;

static inline bool tinyMpcBankSelectorConfigValid(
    const TinyMpcBankSelectorConfig *config) {
  return config != NULL && isfinite(config->enter_bank_rad)
      && isfinite(config->exit_bank_rad)
      && isfinite(config->low_bank_rad)
      && isfinite(config->medium_bank_rad)
      && isfinite(config->high_bank_rad)
      && isfinite(config->very_high_bank_rad)
      && isfinite(config->maximum_bank_rad)
      && isfinite(config->transition_hysteresis_rad)
      && config->enter_bank_rad > config->exit_bank_rad
      && config->exit_bank_rad >= 0.0f
      && config->low_bank_rad > config->enter_bank_rad
      && config->medium_bank_rad > config->low_bank_rad
      && config->high_bank_rad > config->medium_bank_rad
      && config->very_high_bank_rad > config->high_bank_rad
      && config->maximum_bank_rad > config->very_high_bank_rad
      && config->transition_hysteresis_rad >= 0.0f
      && 2.0f * config->transition_hysteresis_rad
          < config->medium_bank_rad - config->low_bank_rad
      && 2.0f * config->transition_hysteresis_rad
          < config->high_bank_rad - config->medium_bank_rad
      && 2.0f * config->transition_hysteresis_rad
          < config->very_high_bank_rad - config->high_bank_rad
      && 2.0f * config->transition_hysteresis_rad
          < config->maximum_bank_rad - config->very_high_bank_rad;
}

static inline void tinyMpcBankSelectorReset(
    TinyMpcBankSelector *selector,
    const TinyMpcBankSelectorConfig *config) {
  if (selector == NULL) {
    return;
  }
  selector->active_model = TINYMPC_BANK_MODEL_LEVEL;
  selector->switch_count = 0u;
  /* Permit immediate selection after reset; this is initialization, not a
   * model transition whose warm start must be protected by dwell time. */
  selector->steps_since_switch = tinyMpcBankSelectorConfigValid(config)
      ? config->minimum_dwell_steps : 0u;
}

static inline uint8_t tinyMpcBankModelTier(TinyMpcBankModelId model) {
  const int identifier = (int)model;
  return identifier >= (int)TINYMPC_BANK_MODEL_LEFT_LOW
          && identifier <= (int)TINYMPC_BANK_MODEL_RIGHT_MAXIMUM
      ? (uint8_t)((identifier + 1) / 2) : 0u;
}

static inline int8_t tinyMpcBankModelSide(TinyMpcBankModelId model) {
  const uint8_t tier = tinyMpcBankModelTier(model);
  return tier == 0u ? 0 : (((int)model & 1) != 0 ? -1 : 1);
}

static inline TinyMpcBankModelId tinyMpcBankModelForTierAndSide(
    uint8_t tier, int8_t side) {
  if (tier == 0u || tier > TINYMPC_BANK_TIER_COUNT) {
    return TINYMPC_BANK_MODEL_LEVEL;
  }
  return (TinyMpcBankModelId)(side < 0 ? 2u * tier - 1u : 2u * tier);
}

static inline float tinyMpcBankNominalForTier(
    uint8_t tier, const TinyMpcBankSelectorConfig *config) {
  const float values[TINYMPC_BANK_TIER_COUNT] = {
      config->low_bank_rad, config->medium_bank_rad, config->high_bank_rad,
      config->very_high_bank_rad, config->maximum_bank_rad};
  return tier >= 1u && tier <= TINYMPC_BANK_TIER_COUNT
      ? values[tier - 1u] : 0.0f;
}

static inline uint8_t tinyMpcNearestBankTier(
    float magnitude_rad, const TinyMpcBankSelectorConfig *config) {
  for (uint8_t tier = TINYMPC_BANK_TIER_COUNT; tier > 1u; --tier) {
    const float boundary = 0.5f * (
        tinyMpcBankNominalForTier(tier - 1u, config)
        + tinyMpcBankNominalForTier(tier, config));
    if (magnitude_rad >= boundary) {
      return tier;
    }
  }
  return 1u;
}

static inline float tinyMpcSignedBankDemand(
    float reference_bank_rad,
    bool reference_valid,
    float measured_bank_rad,
    bool measured_valid) {
  if (reference_valid && measured_valid
      && reference_bank_rad * measured_bank_rad >= 0.0f) {
    return fabsf(reference_bank_rad) >= fabsf(measured_bank_rad)
        ? reference_bank_rad : measured_bank_rad;
  }
  if (reference_valid && fabsf(reference_bank_rad) > 0.0f) {
    return reference_bank_rad;
  }
  return measured_valid ? measured_bank_rad : 0.0f;
}

/* When a path reference unwinds through level between opposite turns, the
 * residual measured bank may retain the current local chart but must not
 * promote to a more aggressive tier or flip chart side.  The next material
 * reference owns the subsequent side/tier transition. */
static inline float tinyMpcBankRecoveryDemand(
    TinyMpcBankModelId active_model, float measured_bank_rad,
    const TinyMpcBankSelectorConfig *config) {
  if (!isfinite(measured_bank_rad)
      || !tinyMpcBankSelectorConfigValid(config)) {
    return 0.0f;
  }
  const uint8_t active_tier = tinyMpcBankModelTier(active_model);
  const int8_t active_side = tinyMpcBankModelSide(active_model);
  if (active_tier == 0u || active_side == 0
      || fabsf(measured_bank_rad) < config->exit_bank_rad) {
    return measured_bank_rad;
  }
  const float retained_magnitude = fminf(
      fabsf(measured_bank_rad),
      tinyMpcBankNominalForTier(active_tier, config));
  return active_side < 0 ? -retained_magnitude : retained_magnitude;
}

/* A steady-turn bundle is local to both its bank angle and tangential speed.
 * Reference bank alone is therefore insufficient to enter that chart from
 * hover. Delay only level->bank transition until measured motion is near the
 * requested operating point. Once banked, recovery remains state-aware. */
static inline bool tinyMpcBankEntrySupportedByMotion(
    float reference_bank_rad,
    float measured_bank_rad,
    float reference_tangent_speed_mps,
    float measured_tangent_speed_mps) {
  if (!isfinite(reference_bank_rad) || !isfinite(measured_bank_rad)
      || !isfinite(reference_tangent_speed_mps)
      || !isfinite(measured_tangent_speed_mps)) {
    return false;
  }
  const float reference_speed = fabsf(reference_tangent_speed_mps);
  if (fabsf(reference_bank_rad) <= 0.045f) {
    return true;
  }
  if (reference_speed <= 0.15f) {
    return false;
  }
  return reference_bank_rad * measured_bank_rad > 0.0f
      && fabsf(measured_bank_rad) >= 0.35f * fabsf(reference_bank_rad)
      && measured_tangent_speed_mps >= 0.60f * reference_speed;
}

/* Bundle selection must wait until the reference and vehicle are local to the
 * candidate's nominal-speed chart. Comparing measured speed only with an
 * instantaneous ramp sample permits entry near hover. */
static inline bool tinyMpcBankEntrySupportedByNominalSpeed(
    float nominal_speed_mps,
    float reference_tangent_speed_mps,
    float measured_tangent_speed_mps,
    float minimum_nominal_fraction) {
  if (!isfinite(nominal_speed_mps)
      || !isfinite(reference_tangent_speed_mps)
      || !isfinite(measured_tangent_speed_mps)
      || !isfinite(minimum_nominal_fraction)
      || nominal_speed_mps < 0.0f
      || minimum_nominal_fraction < 0.0f
      || minimum_nominal_fraction > 1.0f) {
    return false;
  }
  const float minimum_entry_speed_mps =
      minimum_nominal_fraction * nominal_speed_mps;
  return fabsf(reference_tangent_speed_mps) >= minimum_entry_speed_mps
      && measured_tangent_speed_mps >= minimum_entry_speed_mps;
}

/* Preserve existing low/medium entry locality while tightening only the
 * medium-to-high promotion. Inputs are derived from the selected reference
 * tier and current selector state, independent of trajectory identity. */
static inline float tinyMpcBankCandidateSpeedLocalityFraction(
    bool low_demand_entry,
    bool high_tier_promotion,
    uint8_t scheduled_tier) {
  if (low_demand_entry) {
    return 0.20f;
  }
  if (high_tier_promotion) {
    return 0.75f;
  }
  return scheduled_tier >= 3u ? 0.65f : 0.70f;
}

/* Roll and speed alone do not prove that the state is local to a steady-turn
 * chart.  In particular, a lagging heading creates large lateral-velocity and
 * yaw-rate errors even when measured roll happens to match the reference. */
static inline bool tinyMpcBankEntrySupportedByChartError(
    const float error[12],
    float maximum_lateral_position_error_m,
    float maximum_lateral_velocity_error_mps,
    float maximum_yaw_rodrigues_error,
    float maximum_yaw_rate_error_rad_s) {
  if (error == NULL || maximum_lateral_position_error_m < 0.0f
      || maximum_lateral_velocity_error_mps < 0.0f
      || maximum_yaw_rodrigues_error < 0.0f
      || maximum_yaw_rate_error_rad_s < 0.0f) {
    return false;
  }
  for (int index = 0; index < 12; ++index) {
    if (!isfinite(error[index])) {
      return false;
    }
  }
  return fabsf(error[1]) <= maximum_lateral_position_error_m
      && fabsf(error[7]) <= maximum_lateral_velocity_error_mps
      && fabsf(error[5]) <= maximum_yaw_rodrigues_error
      && fabsf(error[11]) <= maximum_yaw_rate_error_rad_s;
}

/* A zero-bank terminal/hover reference must be able to leave a banked chart
 * even while the measured vehicle is still tilted.  Otherwise measured bank
 * alone retains the steady-turn model after the maneuver has ended. */
static inline float tinyMpcBankMeasuredDemandForReference(
    float reference_bank_rad,
    float measured_bank_rad,
    float reference_level_threshold_rad) {
  if (!isfinite(reference_bank_rad) || !isfinite(measured_bank_rad)
      || !isfinite(reference_level_threshold_rad)
      || reference_level_threshold_rad < 0.0f) {
    return 0.0f;
  }
  return fabsf(reference_bank_rad) <= reference_level_threshold_rad
      ? 0.0f : measured_bank_rad;
}

static inline TinyMpcBankModelId tinyMpcRequestedBankModel(
    TinyMpcBankModelId active_model,
    float signed_demand_rad,
    const TinyMpcBankSelectorConfig *config) {
  const float magnitude_rad = fabsf(signed_demand_rad);
  const int8_t demand_side = signed_demand_rad < 0.0f ? -1 : 1;
  const uint8_t active_tier = tinyMpcBankModelTier(active_model);
  const int8_t active_side = tinyMpcBankModelSide(active_model);
  if (active_tier == 0u) {
    return magnitude_rad >= config->enter_bank_rad
        ? tinyMpcBankModelForTierAndSide(
              tinyMpcNearestBankTier(magnitude_rad, config), demand_side)
        : TINYMPC_BANK_MODEL_LEVEL;
  }
  if (magnitude_rad < config->exit_bank_rad) {
    return TINYMPC_BANK_MODEL_LEVEL;
  }
  if (demand_side != active_side) {
    return magnitude_rad >= config->enter_bank_rad
        ? tinyMpcBankModelForTierAndSide(
              tinyMpcNearestBankTier(magnitude_rad, config), demand_side)
        : active_model;
  }

  uint8_t requested_tier = active_tier;
  while (requested_tier < TINYMPC_BANK_TIER_COUNT) {
    const float upper_boundary = 0.5f * (
        tinyMpcBankNominalForTier(requested_tier, config)
        + tinyMpcBankNominalForTier(requested_tier + 1u, config));
    if (magnitude_rad < upper_boundary
        + config->transition_hysteresis_rad) {
      break;
    }
    ++requested_tier;
  }
  while (requested_tier > 1u) {
    const float lower_boundary = 0.5f * (
        tinyMpcBankNominalForTier(requested_tier - 1u, config)
        + tinyMpcBankNominalForTier(requested_tier, config));
    if (magnitude_rad >= lower_boundary
        - config->transition_hysteresis_rad) {
      break;
    }
    --requested_tier;
  }
  return tinyMpcBankModelForTierAndSide(requested_tier, demand_side);
}

static inline TinyMpcBankSelection tinyMpcBankSelectorUpdate(
    TinyMpcBankSelector *selector,
    const TinyMpcBankSelectorConfig *config,
    float reference_bank_rad,
    float measured_bank_rad) {
  TinyMpcBankSelection result = {
      TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
  if (selector == NULL || !tinyMpcBankSelectorConfigValid(config)) {
    return result;
  }

  const bool reference_valid = isfinite(reference_bank_rad);
  const bool measured_valid = isfinite(measured_bank_rad);
  result.active_model = selector->active_model;
  const bool active_is_level =
      tinyMpcBankModelTier(selector->active_model) == 0u;
  result.demand_valid = active_is_level
      ? reference_valid : (reference_valid || measured_valid);
  /* Reference demand must initiate a banked chart. Once banked, measured
   * state can retain the physical chart during recovery, but cannot promote
   * a material low-bank reference into a higher-speed bundle. */
  const bool reference_owns_tier = reference_valid
      && fabsf(reference_bank_rad) >= config->exit_bank_rad;
  result.signed_bank_demand_rad = active_is_level
      ? (reference_valid ? reference_bank_rad : 0.0f)
      : (reference_owns_tier
          ? reference_bank_rad
          : tinyMpcBankRecoveryDemand(
                selector->active_model,
                measured_valid ? measured_bank_rad : 0.0f, config));

  /* A completely invalid sample cannot justify changing the active model.
   * Hold the previous bundle and let the next finite sample decide. */
  if (!result.demand_valid) {
    return result;
  }

  const TinyMpcBankModelId requested_model = tinyMpcRequestedBankModel(
      selector->active_model, result.signed_bank_demand_rad, config);

  const bool dwell_complete = selector->steps_since_switch
      >= config->minimum_dwell_steps;
  if (requested_model != selector->active_model && dwell_complete) {
    selector->active_model = requested_model;
    selector->steps_since_switch = 0u;
    ++selector->switch_count;
    result.switched = true;
    result.reset_optimizer = true;
  } else if (selector->steps_since_switch < UINT16_MAX) {
    ++selector->steps_since_switch;
  }
  result.active_model = selector->active_model;
  return result;
}

#ifdef __cplusplus
}
#endif

#endif
