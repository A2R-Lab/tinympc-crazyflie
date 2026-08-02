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

void sequentialObstacleControlPlan(
    const float clearance_m[SEQUENTIAL_CONTROL_DIRECTIONS],
    const float confidence[SEQUENTIAL_CONTROL_DIRECTIONS],
    const float goal_direction_body[3], const float velocity_body[3],
    int previous_direction, const sequential_control_config_t *config,
    sequential_control_result_t *result) {
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

  float center_effective = config->maximum_range_m;
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
    const float confidence_excess =
        fmaxf(0.0f, confidence[direction] - config->confidence_min);
    const float perception_margin = config->perception_margin_m +
        config->confidence_margin_gain_m / (1.0f + confidence_excess);
    const float margin = config->drone_radius_m +
        config->tracking_margin_m + speed * config->latency_s +
        perception_margin;
    result->margin_m[direction] = margin;
    result->effective_offset_m[direction] = clearance_m[direction] - margin;
    const bool confidence_reliable =
        confidence[direction] >= config->confidence_min;
    if (!confidence_reliable) {
      result->effective_offset_m[direction] = clampf(
          config->conservative_default_offset_m, 0.0f,
          config->maximum_range_m);
    }
    if (direction == 1 || direction == 2) {
      center_effective = fminf(center_effective,
                               result->effective_offset_m[direction]);
    }
    if (!confidence_reliable) continue;
    if (result->effective_offset_m[direction] <= 0.0f) {
      result->effective_offset_m[direction] = 0.0f;
      continue;
    }
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

  result->stop = result->reliable_mask == 0;
  result->valid = !result->stop;
  if (config->trigger_distance_m > 1.0e-4f) {
    result->avoidance_pressure = clampf(
        (config->trigger_distance_m - center_effective) /
            config->trigger_distance_m,
        0.0f, 1.0f);
  }
}
