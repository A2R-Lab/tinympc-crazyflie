/* Pure four-direction carrot-and-stick planning math. */
#ifndef __SEQUENTIAL_OBSTACLE_CONTROL_H__
#define __SEQUENTIAL_OBSTACLE_CONTROL_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEQUENTIAL_CONTROL_DIRECTIONS 4
#define SEQUENTIAL_DANGER_WINDOW_MAX 32

typedef struct {
  float samples[SEQUENTIAL_DANGER_WINDOW_MAX];
  float sum;
  uint8_t next;
  uint8_t count;
  uint8_t window;
} sequential_danger_average_t;

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
  float direction_safe_min_m;
} sequential_control_config_t;

typedef struct {
  bool valid;
  bool stop;
  uint8_t reliable_mask;  /* Open-direction mask; name retained for log ABI. */
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

void sequentialDangerAverageReset(sequential_danger_average_t *average);
float sequentialDangerAverageUpdate(sequential_danger_average_t *average,
                                    uint8_t dangerous_slices,
                                    uint8_t requested_window);

#ifdef __cplusplus
}
#endif
#endif
