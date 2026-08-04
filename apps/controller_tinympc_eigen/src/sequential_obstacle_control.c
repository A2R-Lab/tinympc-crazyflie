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
