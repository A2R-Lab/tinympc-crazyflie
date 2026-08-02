/* Pure four-direction carrot-and-stick planning math. */
#ifndef __SEQUENTIAL_OBSTACLE_CONTROL_H__
#define __SEQUENTIAL_OBSTACLE_CONTROL_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEQUENTIAL_CONTROL_DIRECTIONS 4

typedef struct {
  float confidence_min;
  float drone_radius_m;
  float tracking_margin_m;
  float latency_s;
  float perception_margin_m;
  float confidence_margin_gain_m;
  float conservative_default_offset_m;
  float maximum_range_m;
  float trigger_distance_m;
  float distance_weight;
  float goal_weight;
  float hysteresis_weight;
  float dynamic_weight;
} sequential_control_config_t;

typedef struct {
  bool valid;
  bool stop;
  uint8_t reliable_mask;
  int8_t chosen_direction;
  float body_normal[SEQUENTIAL_CONTROL_DIRECTIONS][3];
  float effective_offset_m[SEQUENTIAL_CONTROL_DIRECTIONS];
  float margin_m[SEQUENTIAL_CONTROL_DIRECTIONS];
  float chosen_score;
  float avoidance_pressure;
} sequential_control_result_t;

void sequentialObstacleControlPlan(
    const float clearance_m[SEQUENTIAL_CONTROL_DIRECTIONS],
    const float confidence[SEQUENTIAL_CONTROL_DIRECTIONS],
    const float goal_direction_body[3], const float velocity_body[3],
    int previous_direction, const sequential_control_config_t *config,
    sequential_control_result_t *result);

#ifdef __cplusplus
}
#endif
#endif
