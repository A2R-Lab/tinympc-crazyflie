#include "perception_corridor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CELL_PX (160.0f / PERCEPTION_MAP_W)

static int cellIndex(int x, int y) {
  return y * PERCEPTION_MAP_W + x;
}

static void fitUFromV(const float *u, const float *v, int count,
                      float *slope, float *offset) {
  float sum_u = 0.0f, sum_v = 0.0f, sum_vv = 0.0f, sum_vu = 0.0f;
  for (int i = 0; i < count; ++i) {
    sum_u += u[i];
    sum_v += v[i];
    sum_vv += v[i] * v[i];
    sum_vu += v[i] * u[i];
  }
  const float denominator = count * sum_vv - sum_v * sum_v;
  if (fabsf(denominator) < 1.0e-5f) {
    *slope = 0.0f;
    *offset = sum_u / count;
  } else {
    *slope = (count * sum_vu - sum_v * sum_u) / denominator;
    *offset = (sum_u - *slope * sum_v) / count;
  }
}

static void backProjectAndSign(float line[3], float gate_u, float gate_v,
                               const perception_corridor_config_t *config,
                               float normal[3]) {
  if (line[0] * gate_u + line[1] * gate_v + line[2] < 0.0f) {
    line[0] = -line[0];
    line[1] = -line[1];
    line[2] = -line[2];
  }
  normal[0] = config->fx * line[0];
  normal[1] = config->fy * line[1];
  normal[2] =
      config->cx * line[0] + config->cy * line[1] + line[2];
  const float norm = sqrtf(
      normal[0] * normal[0] + normal[1] * normal[1]
      + normal[2] * normal[2]);
  if (norm > 1.0e-6f) {
    normal[0] /= norm;
    normal[1] /= norm;
    normal[2] /= norm;
  }
}

bool perceptionCorridorFit(
    const float danger[PERCEPTION_MAP_CELLS],
    const float gate_corners_xy[8],
    const perception_corridor_config_t *config,
    perception_corridor_t *result) {
  memset(result, 0, sizeof(*result));
  float gate_u = 0.0f, gate_v = 0.0f;
  for (int corner = 0; corner < 4; ++corner) {
    gate_u += 0.25f * gate_corners_xy[2 * corner];
    gate_v += 0.25f * gate_corners_xy[2 * corner + 1];
  }

  int seed_x = (int)(gate_u / CELL_PX);
  int seed_y = (int)(gate_v / CELL_PX);
  if (seed_x < 0) seed_x = 0;
  if (seed_x >= PERCEPTION_MAP_W) seed_x = PERCEPTION_MAP_W - 1;
  if (seed_y < 0) seed_y = 0;
  if (seed_y >= PERCEPTION_MAP_H) seed_y = PERCEPTION_MAP_H - 1;

  /* If the gate-center cell is unsafe, select the closest safe cell. */
  int best = -1;
  int best_distance = 1000;
  for (int y = 0; y < PERCEPTION_MAP_H; ++y) {
    for (int x = 0; x < PERCEPTION_MAP_W; ++x) {
      const int index = cellIndex(x, y);
      if (danger[index] >= config->danger_threshold) continue;
      const int distance = abs(x - seed_x) + abs(y - seed_y);
      if (distance < best_distance) {
        best = index;
        best_distance = distance;
      }
    }
  }
  if (best < 0) return false;

  uint8_t selected[PERCEPTION_MAP_CELLS] = {0};
  uint8_t queued[PERCEPTION_MAP_CELLS] = {0};
  int queue[PERCEPTION_MAP_CELLS];
  int head = 0, tail = 0;
  queue[tail++] = best;
  queued[best] = 1;
  static const int dx[4] = {1, -1, 0, 0};
  static const int dy[4] = {0, 0, 1, -1};
  while (head < tail) {
    const int index = queue[head++];
    const int x = index % PERCEPTION_MAP_W;
    const int y = index / PERCEPTION_MAP_W;
    selected[index] = 1;
    for (int direction = 0; direction < 4; ++direction) {
      const int nx = x + dx[direction];
      const int ny = y + dy[direction];
      if (nx < 0 || nx >= PERCEPTION_MAP_W ||
          ny < 0 || ny >= PERCEPTION_MAP_H) continue;
      const int neighbor = cellIndex(nx, ny);
      if (!queued[neighbor] &&
          danger[neighbor] < config->danger_threshold) {
        queued[neighbor] = 1;
        queue[tail++] = neighbor;
      }
    }
  }
  result->selected_cells = tail;

  float left_u[PERCEPTION_MAP_H], right_u[PERCEPTION_MAP_H];
  float rows_v[PERCEPTION_MAP_H];
  int rows = 0;
  for (int y = 0; y < PERCEPTION_MAP_H; ++y) {
    int left = PERCEPTION_MAP_W;
    int right = -1;
    for (int x = 0; x < PERCEPTION_MAP_W; ++x) {
      if (selected[cellIndex(x, y)]) {
        if (x < left) left = x;
        if (x > right) right = x;
      }
    }
    if (right >= left) {
      left_u[rows] = left * CELL_PX;
      right_u[rows] = (right + 1) * CELL_PX;
      rows_v[rows] = (y + 0.5f) * CELL_PX;
      rows++;
    }
  }
  if (rows < 2) return false;

  float left_m, left_c, right_m, right_c;
  fitUFromV(left_u, rows_v, rows, &left_m, &left_c);
  fitUFromV(right_u, rows_v, rows, &right_m, &right_c);
  const float margin = fmaxf(0.0f, config->inward_margin_px);
  const float left_at_gate = left_m * gate_v + left_c + margin;
  const float right_at_gate = right_m * gate_v + right_c - margin;
  if (left_at_gate >= right_at_gate ||
      gate_u < left_at_gate || gate_u > right_at_gate) {
    return false;
  }
  /* left: u >= m*v+c+margin; right: u <= m*v+c-margin. */
  result->pixel_line[0][0] = 1.0f;
  result->pixel_line[0][1] = -left_m;
  result->pixel_line[0][2] = -(left_c + margin);
  result->pixel_line[1][0] = -1.0f;
  result->pixel_line[1][1] = right_m;
  result->pixel_line[1][2] = right_c - margin;
  for (int side = 0; side < 2; ++side) {
    backProjectAndSign(result->pixel_line[side], gate_u, gate_v, config,
                       result->camera_normal[side]);
  }
  result->valid = true;
  return true;
}

void perceptionAngularConstraintRow(
    const float normal_world[3],
    const float camera_center_world[3],
    float lookahead_s,
    float position_coefficients[3],
    float velocity_coefficients[3],
    float *upper_bound) {
  *upper_bound = 0.0f;
  for (int axis = 0; axis < 3; ++axis) {
    position_coefficients[axis] = -normal_world[axis];
    velocity_coefficients[axis] = -lookahead_s * normal_world[axis];
    *upper_bound -= normal_world[axis] * camera_center_world[axis];
  }
}
