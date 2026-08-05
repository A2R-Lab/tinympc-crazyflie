#include "sequential_obstacle_control.h"

#include <math.h>
#include <string.h>

static const float k_direction_angles_rad[SEQUENTIAL_CONTROL_DIRECTIONS] = {
  -0.6981317008f, -0.2327105669f, 0.2327105669f, 0.6981317008f
};

static float clampf(float value, float lower, float upper) {
  return fminf(upper, fmaxf(lower, value));
}

static float norm3(const float value[3]) {
  return sqrtf(value[0] * value[0] + value[1] * value[1] +
               value[2] * value[2]);
}

void sequentialDangerAverageReset(sequential_danger_average_t *average) {
  memset(average, 0, sizeof(*average));
}

float sequentialDangerAverageUpdate(sequential_danger_average_t *average,
                                    uint8_t dangerous_slices,
                                    uint8_t requested_window) {
  const uint8_t window = requested_window < 1 ? 1 :
      requested_window > SEQUENTIAL_DANGER_WINDOW_MAX
          ? SEQUENTIAL_DANGER_WINDOW_MAX : requested_window;
  if (average->window != window) {
    sequentialDangerAverageReset(average);
    average->window = window;
  }

  const float sample = fminf((float)SEQUENTIAL_CONTROL_DIRECTIONS,
                             (float)dangerous_slices);
  if (average->count == window) {
    average->sum -= average->samples[average->next];
  } else {
    average->count++;
  }
  average->samples[average->next] = sample;
  average->sum += sample;
  average->next = (uint8_t)((average->next + 1u) % window);
  return average->sum / (float)average->count;
}

void sequentialClearanceAverageReset(sequential_clearance_average_t *average) {
  memset(average, 0, sizeof(*average));
}

void sequentialClearanceAverageUpdate(
    sequential_clearance_average_t *average,
    const float clearance_m[SEQUENTIAL_CONTROL_DIRECTIONS],
    uint8_t requested_window,
    float mean_clearance_m[SEQUENTIAL_CONTROL_DIRECTIONS]) {
  const uint8_t window = requested_window < 1 ? 1 :
      requested_window > SEQUENTIAL_DANGER_WINDOW_MAX
          ? SEQUENTIAL_DANGER_WINDOW_MAX : requested_window;
  if (average->window != window) {
    sequentialClearanceAverageReset(average);
    average->window = window;
  }
  if (average->count == window) {
    for (int direction = 0; direction < SEQUENTIAL_CONTROL_DIRECTIONS;
         ++direction) {
      average->sum[direction] -= average->samples[direction][average->next];
    }
  } else {
    average->count++;
  }
  for (int direction = 0; direction < SEQUENTIAL_CONTROL_DIRECTIONS;
       ++direction) {
    const float value = isfinite(clearance_m[direction])
        ? fmaxf(0.0f, clearance_m[direction]) : 0.0f;
    average->samples[direction][average->next] = value;
    average->sum[direction] += value;
    mean_clearance_m[direction] = average->sum[direction] /
        (float)average->count;
  }
  average->next = (uint8_t)((average->next + 1) % window);
}

int8_t sequentialSelectEvasionSide(
    const float open_fraction[SEQUENTIAL_CONTROL_DIRECTIONS],
    float score_bias, int previous_side, float *score_0, float *score_3) {
  float open[SEQUENTIAL_CONTROL_DIRECTIONS];
  for (int direction = 0; direction < SEQUENTIAL_CONTROL_DIRECTIONS;
       ++direction) {
    open[direction] = isfinite(open_fraction[direction])
        ? clampf(open_fraction[direction], 0.0f, 1.0f) : 0.0f;
  }
  /* Evasion side is determined only by the corresponding outer ray. The
   * inner rays remain available to the obstacle trigger and diagnostics, but
   * must not influence the left/right choice. */
  const float right_score = open[0];
  const float left_score = open[3];
  if (score_0 != NULL) *score_0 = right_score;
  if (score_3 != NULL) *score_3 = left_score;
  const float bias = clampf(score_bias, 0.0f, 1.0f);
  if (left_score > right_score + bias) return 3;
  if (right_score > left_score + bias) return 0;
  return previous_side == 0 ? 3 : 0;
}

bool sequentialLateralBarrierRow(const float side_world[3],
                                 const float anchor_world[3],
                                 float a_position[3], float *b) {
  const float norm = hypotf(side_world[0], side_world[1]);
  if (norm < 1.0e-4f || b == NULL) return false;
  a_position[0] = -side_world[0] / norm;
  a_position[1] = -side_world[1] / norm;
  a_position[2] = 0.0f;
  *b = a_position[0] * anchor_world[0] +
       a_position[1] * anchor_world[1];
  return true;
}

bool sequentialOffsetBarrierRow(const float nominal_world[3],
                                const float offset_world[3],
                                float a_position[3], float *b) {
  const float norm = hypotf(offset_world[0], offset_world[1]);
  if (norm < 1.0e-4f || b == NULL) return false;
  const float side_world[3] = {
    offset_world[0] / norm, offset_world[1] / norm, 0.0f,
  };
  const float anchor_world[3] = {
    nominal_world[0] + offset_world[0],
    nominal_world[1] + offset_world[1],
    nominal_world[2] + offset_world[2],
  };
  return sequentialLateralBarrierRow(side_world, anchor_world,
                                     a_position, b);
}

float sequentialReturnScanYawDeg(float base_yaw_deg, int evasion_direction,
                                 float yaw_offset_deg) {
  const float sign = evasion_direction == 0 ? 1.0f :
      evasion_direction == 3 ? -1.0f : 0.0f;
  float yaw = base_yaw_deg + sign * fabsf(yaw_offset_deg);
  while (yaw > 180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;
  return yaw;
}

float sequentialSlewYawDeg(float current_yaw_deg, float target_yaw_deg,
                           float maximum_step_deg) {
  float delta = target_yaw_deg - current_yaw_deg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  const float step = fabsf(maximum_step_deg);
  if (fabsf(delta) <= step || step <= 0.0f) {
    return step <= 0.0f ? current_yaw_deg : target_yaw_deg;
  }
  float yaw = current_yaw_deg + copysignf(step, delta);
  while (yaw > 180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;
  return yaw;
}

float sequentialClearanceSpeedMps(float forward_clearance_m,
                                  float safety_distance_m,
                                  float cruise_speed_mps,
                                  float braking_acceleration_mps2) {
  if (!isfinite(forward_clearance_m) || !isfinite(safety_distance_m) ||
      !isfinite(cruise_speed_mps) ||
      !isfinite(braking_acceleration_mps2)) {
    return 0.0f;
  }
  const float cruise = fmaxf(0.0f, cruise_speed_mps);
  const float braking = fmaxf(0.0f, braking_acceleration_mps2);
  const float available = forward_clearance_m -
      fmaxf(0.0f, safety_distance_m);
  if (cruise <= 0.0f || braking <= 0.0f || available <= 0.0f) {
    return 0.0f;
  }
  return fminf(cruise, sqrtf(2.0f * braking * available));
}

bool sequentialSpeedStopUpdate(bool was_stopped, float forward_clearance_m,
                               float stop_distance_m,
                               float resume_distance_m) {
  if (!isfinite(forward_clearance_m)) return true;
  const float stop = fmaxf(0.0f, stop_distance_m);
  const float resume = fmaxf(stop, resume_distance_m);
  return was_stopped ? forward_clearance_m < resume
                     : forward_clearance_m <= stop;
}

float sequentialRateLimitSpeedMps(float current_speed_mps,
                                  float target_speed_mps,
                                  float acceleration_mps2,
                                  float braking_acceleration_mps2,
                                  float dt_s) {
  if (!isfinite(current_speed_mps) || !isfinite(target_speed_mps) ||
      !isfinite(acceleration_mps2) ||
      !isfinite(braking_acceleration_mps2) || !isfinite(dt_s)) {
    return 0.0f;
  }
  const float current = fmaxf(0.0f, current_speed_mps);
  const float target = fmaxf(0.0f, target_speed_mps);
  const float dt = fmaxf(0.0f, dt_s);
  const float rate = target >= current
      ? fmaxf(0.0f, acceleration_mps2)
      : fmaxf(0.0f, braking_acceleration_mps2);
  const float step = rate * dt;
  if (target > current) return fminf(target, current + step);
  return fmaxf(target, current - step);
}

void sequentialObstacleControlPlan(
    const float clearance_m[SEQUENTIAL_CONTROL_DIRECTIONS],
    const float confidence[SEQUENTIAL_CONTROL_DIRECTIONS],
    const float goal_direction_body[3], const float velocity_body[3],
    int previous_direction, const sequential_control_config_t *config,
    sequential_control_result_t *result) {
  (void)confidence;
  memset(result, 0, sizeof(*result));
  result->chosen_direction = -1;
  result->chosen_score = -INFINITY;
  const float speed = norm3(velocity_body);
  const float goal_norm = norm3(goal_direction_body);
  float velocity_unit[3] = {0.0f, 0.0f, 0.0f};
  if (speed > 1.0e-4f) {
    for (int axis = 0; axis < 3; ++axis) {
      velocity_unit[axis] = velocity_body[axis] / speed;
    }
  }

  for (int direction = 0; direction < SEQUENTIAL_CONTROL_DIRECTIONS;
       ++direction) {
    result->body_normal[direction][0] =
        cosf(k_direction_angles_rad[direction]);
    result->body_normal[direction][1] =
        sinf(k_direction_angles_rad[direction]);
    result->body_normal[direction][2] = 0.0f;
  }
  for (int direction = 0; direction < SEQUENTIAL_CONTROL_DIRECTIONS;
       ++direction) {
    float *normal = result->body_normal[direction];
    /* The network's metric output is used only to classify each fixed ray.
     * Confidence and metric safety margins are deliberately ignored for this
     * experiment. */
    result->margin_m[direction] = 0.0f;
    result->effective_offset_m[direction] = clampf(
        clearance_m[direction], 0.0f, config->maximum_range_m);
    const bool direction_open = result->effective_offset_m[direction] >=
        config->direction_safe_min_m;
    if (!direction_open) continue;
    result->reliable_mask |= (uint8_t)(1u << direction);
    float goal_alignment = 0.0f;
    if (goal_norm > 1.0e-4f) {
      goal_alignment = (normal[0] * goal_direction_body[0] +
                        normal[1] * goal_direction_body[1] +
                        normal[2] * goal_direction_body[2]) / goal_norm;
    }
    float history_alignment = 0.0f;
    if (previous_direction >= 0 &&
        previous_direction < SEQUENTIAL_CONTROL_DIRECTIONS) {
      history_alignment =
          normal[0] * result->body_normal[previous_direction][0] +
          normal[1] * result->body_normal[previous_direction][1];
    }
    float dynamic_penalty = 0.0f;
    if (speed > 1.0e-4f) {
      dynamic_penalty = 1.0f -
          clampf(normal[0] * velocity_unit[0] +
                 normal[1] * velocity_unit[1], -1.0f, 1.0f);
    }
    const float normalized_distance = clampf(
        result->effective_offset_m[direction] / config->maximum_range_m,
        0.0f, 1.0f);
    const float score = config->distance_weight * normalized_distance +
        config->goal_weight * goal_alignment +
        config->hysteresis_weight * history_alignment -
        config->dynamic_weight * dynamic_penalty;
    if (score > result->chosen_score) {
      result->chosen_score = score;
      result->chosen_direction = (int8_t)direction;
    }
  }

  /* A fresh all-blocked classification starts the lateral escape; it is not a
   * fail-safe fault. Packet loss/staleness is handled by the caller. */
  result->stop = false;
  result->valid = true;
  result->avoidance_pressure = result->reliable_mask == 0 ? 1.0f : 0.0f;
}
