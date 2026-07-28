#include "perception_danger.h"

#include "perception_model_qparams.h"

#include <math.h>

static float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

#ifndef PERCEPTION_MODEL_DANGER_ONLY
static float sigmoidfStable(float value) {
  if (value >= 0.0f) {
    const float inverse = expf(-value);
    return 1.0f / (1.0f + inverse);
  }
  const float exponential = expf(value);
  return exponential / (1.0f + exponential);
}
#endif

#ifndef PERCEPTION_MODEL_DANGER_ONLY
static float channelProbability(uint8_t q, int channel) {
  return clamp01(sigmoidfStable(perceptionModelOutputLogit(q, channel)));
}
#endif

void perceptionDangerCompute(
    const uint8_t nominal_collision_q[PERCEPTION_MAP_CELLS],
    const uint8_t inverse_range_q[PERCEPTION_MAP_CELLS],
    const uint8_t uncertainty_q[PERCEPTION_MAP_CELLS],
    uint32_t map_age_ms,
    const perception_danger_state_t *state,
    perception_danger_map_t *output) {
#ifdef PERCEPTION_MODEL_DANGER_ONLY
  const float vx = state->body_velocity_mps[0];
  const float vy = state->body_velocity_mps[1];
  const float vz = state->body_velocity_mps[2];
  const float speed = sqrtf(vx * vx + vy * vy + vz * vz);
  const float maximum_range =
      state->maximum_range_m > 0.0f ? state->maximum_range_m : 6.0f;
  const float nominal_speed =
      state->nominal_target_speed_mps > 1.0e-3f
          ? state->nominal_target_speed_mps
          : 1.0f;
  const float latency =
      fmaxf(0.0f, state->perception_control_latency_s)
      + 0.001f * (float)map_age_ms;
  const float extra_reach =
      fmaxf(0.0f, speed - nominal_speed)
          * fmaxf(0.0f, state->horizon_s)
      + speed * latency;
  const float speed_margin = clamp01(extra_reach / maximum_range);
  (void)inverse_range_q;
  (void)uncertainty_q;
  for (int cell = 0; cell < PERCEPTION_MAP_CELLS; ++cell) {
    const float nominal_danger =
        nominal_collision_q[cell] * (1.0f / 255.0f);
    const float danger = clamp01(
        nominal_danger + (1.0f - nominal_danger) * speed_margin);
    const float range = maximum_range * (1.0f - danger);
    output->probability[cell] = danger;
    output->range_m[cell] = range;
    output->uncertainty[cell] = 0.0f;
    output->time_to_contact_s[cell] =
        speed > 1.0e-3f ? fmaxf(0.0f, range - speed * latency) / speed
                        : INFINITY;
  }
#else
  const float vx = state->body_velocity_mps[0];
  const float vy = state->body_velocity_mps[1];
  const float vz = state->body_velocity_mps[2];
  const float speed = sqrtf(vx * vx + vy * vy + vz * vz);
  const float latency =
      fmaxf(0.0f, state->perception_control_latency_s)
      + 0.001f * (float)map_age_ms;
  const float reachable_distance =
      speed * fmaxf(0.0f, state->horizon_s + latency);
  const float nominal_speed =
      state->nominal_target_speed_mps > 1.0e-3f
          ? state->nominal_target_speed_mps
          : 1.0f;
  const float nominal_reachable_distance =
      nominal_speed
      * fmaxf(0.0f, state->horizon_s
          + state->perception_control_latency_s);
  const float maximum_range =
      state->maximum_range_m > 0.0f ? state->maximum_range_m : 6.0f;

  for (int cell = 0; cell < PERCEPTION_MAP_CELLS; ++cell) {
    const float nominal_logit = perceptionModelOutputLogit(
        nominal_collision_q[cell], PERCEPTION_MODEL_OBSTACLE_CHANNEL);
    const float inverse_range = channelProbability(
        inverse_range_q[cell], PERCEPTION_MODEL_INVERSE_RANGE_CHANNEL);
    const float uncertainty = channelProbability(
        uncertainty_q[cell], PERCEPTION_MODEL_UNCERTAINTY_CHANNEL);
    const float range = (1.0f - inverse_range) * maximum_range;
    const float soft_margin = 0.15f * (1.0f + 2.0f * uncertainty);
    const float geometric_collision =
        sigmoidfStable((reachable_distance - range) / soft_margin);

    const float adjusted_collision = sigmoidfStable(
        nominal_logit
        + (reachable_distance - nominal_reachable_distance) / soft_margin);
    output->probability[cell] =
        clamp01(fmaxf(adjusted_collision, geometric_collision));
    const float effective_range = fmaxf(0.0f, range - speed * latency);
    output->range_m[cell] = effective_range;
    output->uncertainty[cell] = uncertainty;
    output->time_to_contact_s[cell] =
        speed > 1.0e-3f ? effective_range / speed : INFINITY;
  }
#endif
}

static float cross2(float ax, float ay, float bx, float by,
                    float px, float py) {
  return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

int perceptionDangerApplyGateOpening(
    const uint8_t gate_opening_q[PERCEPTION_MAP_CELLS],
    const float gate_corners_xy[8],
    float gate_range_m,
    float inset_px,
    float gate_probability_threshold,
    float safe_probability_cap,
    float range_beyond_gate_m,
    float maximum_uncertainty,
    perception_danger_map_t *map) {
  if (!gate_opening_q || !gate_corners_xy || !map ||
      !isfinite(gate_range_m) || gate_range_m <= 0.0f) {
    return 0;
  }
  float center_x = 0.0f, center_y = 0.0f;
  for (int corner = 0; corner < 4; ++corner) {
    const float x = gate_corners_xy[2 * corner];
    const float y = gate_corners_xy[2 * corner + 1];
    if (!isfinite(x) || !isfinite(y)) return 0;
    center_x += 0.25f * x;
    center_y += 0.25f * y;
  }
  float polygon[8];
  for (int corner = 0; corner < 4; ++corner) {
    const float x = gate_corners_xy[2 * corner];
    const float y = gate_corners_xy[2 * corner + 1];
    const float dx = center_x - x;
    const float dy = center_y - y;
    const float radius = sqrtf(dx * dx + dy * dy);
    const float fraction =
        radius > 1.0e-3f ? fminf(fmaxf(inset_px, 0.0f) / radius, 0.45f)
                         : 0.45f;
    polygon[2 * corner] = x + fraction * dx;
    polygon[2 * corner + 1] = y + fraction * dy;
  }
  float signed_area_twice = 0.0f;
  for (int edge = 0; edge < 4; ++edge) {
    const int next = (edge + 1) & 3;
    signed_area_twice +=
        polygon[2 * edge] * polygon[2 * next + 1]
        - polygon[2 * next] * polygon[2 * edge + 1];
  }
  if (fabsf(signed_area_twice) < 128.0f) return 0;
  const bool positive = signed_area_twice > 0.0f;
  const float cell_px = 160.0f / (float)PERCEPTION_MAP_W;
  const float minimum_range =
      gate_range_m + fmaxf(0.0f, range_beyond_gate_m);
  int changed = 0;
  for (int y = 0; y < PERCEPTION_MAP_H; ++y) {
    for (int x = 0; x < PERCEPTION_MAP_W; ++x) {
      const float px = (x + 0.5f) * cell_px;
      const float py = (y + 0.5f) * cell_px;
      bool inside = true;
      for (int edge = 0; edge < 4; ++edge) {
        const int next = (edge + 1) & 3;
        const float side = cross2(
            polygon[2 * edge], polygon[2 * edge + 1],
            polygon[2 * next], polygon[2 * next + 1], px, py);
        if ((positive && side < 0.0f) || (!positive && side > 0.0f)) {
          inside = false;
          break;
        }
      }
      if (!inside) continue;
      const int cell = y * PERCEPTION_MAP_W + x;
#ifdef PERCEPTION_MODEL_DANGER_ONLY
      const float gate_probability = gate_opening_q[cell] * (1.0f / 255.0f);
#else
      const float gate_probability = channelProbability(
          gate_opening_q[cell], PERCEPTION_MODEL_GATE_CHANNEL);
#endif
      if (gate_probability < gate_probability_threshold ||
          map->range_m[cell] < minimum_range ||
          map->uncertainty[cell] > maximum_uncertainty) {
        continue;
      }
      const float capped = fminf(
          map->probability[cell], clamp01(safe_probability_cap));
      if (capped < map->probability[cell]) {
        map->probability[cell] = capped;
        changed++;
      }
    }
  }
  return changed;
}
