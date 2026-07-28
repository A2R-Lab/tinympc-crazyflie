/* Image-space safe-corridor fitting and calibrated line back-projection. */
#ifndef __PERCEPTION_CORRIDOR_H__
#define __PERCEPTION_CORRIDOR_H__

#include "perception_map_link.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float danger_threshold;
  float inward_margin_px;
  float fx;
  float fy;
  float cx;
  float cy;
} perception_corridor_config_t;

typedef struct {
  bool valid;
  int selected_cells;
  /* Pixel line l=[a,b,c], feasible when l^T[u,v,1] >= 0. */
  float pixel_line[2][3];
  /* Normal n_c=K^T l, normalized and signed toward the gate-center ray. */
  float camera_normal[2][3];
} perception_corridor_t;

/*
 * Select the 4-connected safe component nearest the gate center, fit its left
 * and right boundaries, shrink them by the configured margin, and back-project
 * the lines into camera-centered angular planes.
 */
bool perceptionCorridorFit(
    const float danger[PERCEPTION_MAP_CELLS],
    const float gate_corners_xy[8],
    const perception_corridor_config_t *config,
    perception_corridor_t *result);

/* Build -n^T p - tau*n^T v <= -n^T p_camera. */
void perceptionAngularConstraintRow(
    const float normal_world[3],
    const float camera_center_world[3],
    float lookahead_s,
    float position_coefficients[3],
    float velocity_coefficients[3],
    float *upper_bound);

#ifdef __cplusplus
}
#endif

#endif
