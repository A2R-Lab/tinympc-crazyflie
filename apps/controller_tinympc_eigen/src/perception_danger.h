/*
 * Motion-conditioned danger decoding for NanoCockpit CNN control maps.
 *
 * The CNN predicts nominal-speed collision probability, inverse range and
 * uncertainty. This module adjusts collision probability and time-to-contact
 * using the current vehicle/controller state.
 */
#ifndef __PERCEPTION_DANGER_H__
#define __PERCEPTION_DANGER_H__

#include "perception_map_link.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float body_velocity_mps[3];
  float horizon_s;
  float perception_control_latency_s;
  float maximum_range_m;
  float nominal_target_speed_mps;
} perception_danger_state_t;

typedef struct {
  float probability[PERCEPTION_MAP_CELLS];
  float time_to_contact_s[PERCEPTION_MAP_CELLS];
  float range_m[PERCEPTION_MAP_CELLS];
  float uncertainty[PERCEPTION_MAP_CELLS];
} perception_danger_map_t;

/*
 * Decode one received quantized map. map_age_ms is added to the configured
 * perception/control latency. Time-to-contact is INFINITY when speed is zero.
 */
void perceptionDangerCompute(
    const uint8_t nominal_collision_q[PERCEPTION_MAP_CELLS],
    const uint8_t inverse_range_q[PERCEPTION_MAP_CELLS],
    const uint8_t uncertainty_q[PERCEPTION_MAP_CELLS],
    uint32_t map_age_ms,
    const perception_danger_state_t *state,
    perception_danger_map_t *output);

/*
 * Treat a synchronized, confidently segmented gate opening as traversable.
 * The permission is deliberately narrow: a cell must lie inside the inset
 * ordered-corner polygon, have high gate-opening probability, low uncertainty,
 * and predicted free range beyond the known gate plane. Returns the number of
 * cells whose danger probability was reduced.
 */
int perceptionDangerApplyGateOpening(
    const uint8_t gate_opening_q[PERCEPTION_MAP_CELLS],
    const float gate_corners_xy[8],
    float gate_range_m,
    float inset_px,
    float gate_probability_threshold,
    float safe_probability_cap,
    float range_beyond_gate_m,
    float maximum_uncertainty,
    perception_danger_map_t *map);

#ifdef __cplusplus
}
#endif

#endif
