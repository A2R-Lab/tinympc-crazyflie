#ifndef TINYMPC_PROGRESS_PATH_H
#define TINYMPC_PROGRESS_PATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float x;
  float y;
  float z;
} TinyMpcPathPoint;

typedef struct {
  TinyMpcPathPoint position;
  TinyMpcPathPoint tangent;
  float curvature_per_m;
  float speed_mps;
} TinyMpcPathSample;

typedef struct {
  const float *point_data;
  uint16_t point_stride;
  uint16_t count;
  float progress;
  uint16_t search_back_segments;
  uint16_t search_forward_segments;
  float minimum_speed_mps;
  float maximum_speed_mps;
  float curvature_speed_gain_m;
  float maximum_projection_advance_m;
  float completion_radius_m;
  bool complete;
} TinyMpcProgressPath;

static inline float tinyMpcPathClamp(float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

static inline float tinyMpcPathSegmentLength(
    const TinyMpcProgressPath *path, uint16_t segment);
static inline float tinyMpcProgressPathAdvance(
    const TinyMpcProgressPath *path, float progress, float distance_m);

static inline TinyMpcPathPoint tinyMpcProgressPathPoint(
    const TinyMpcProgressPath *path, uint16_t index) {
  const float *point = path->point_data + (uint32_t)index * path->point_stride;
  const TinyMpcPathPoint result = {point[0], point[1], point[2]};
  return result;
}

static inline float tinyMpcPathSegmentLength(
    const TinyMpcProgressPath *path, uint16_t segment) {
  const TinyMpcPathPoint a = tinyMpcProgressPathPoint(path, segment);
  const TinyMpcPathPoint b = tinyMpcProgressPathPoint(path, segment + 1u);
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  const float dz = b.z - a.z;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

static inline void tinyMpcProgressPathInit(
    TinyMpcProgressPath *path, const float *point_data, uint16_t point_stride,
    uint16_t count,
    uint16_t search_back_segments, uint16_t search_forward_segments,
    float minimum_speed_mps, float maximum_speed_mps,
    float curvature_speed_gain_m, float maximum_projection_advance_m,
    float completion_radius_m) {
  path->point_data = point_data;
  path->point_stride = point_stride;
  path->count = count;
  path->progress = 0.0f;
  path->search_back_segments = search_back_segments;
  path->search_forward_segments = search_forward_segments;
  path->minimum_speed_mps = minimum_speed_mps;
  path->maximum_speed_mps = maximum_speed_mps;
  path->curvature_speed_gain_m = curvature_speed_gain_m;
  path->maximum_projection_advance_m = maximum_projection_advance_m;
  path->completion_radius_m = completion_radius_m;
  path->complete = count < 2u;
}

static inline float tinyMpcProgressPathProject(
    TinyMpcProgressPath *path, TinyMpcPathPoint vehicle) {
  if (path->complete || path->point_data == 0 || path->count < 2u) {
    return path->progress;
  }
  const uint16_t current = (uint16_t)tinyMpcPathClamp(
      floorf(path->progress), 0.0f, (float)(path->count - 2u));
  const uint16_t first = current > path->search_back_segments
      ? current - path->search_back_segments : 0u;
  uint32_t requested_last = (uint32_t)current + path->search_forward_segments;
  const uint16_t last = (uint16_t)(requested_last < path->count - 1u
      ? requested_last : path->count - 2u);
  const uint16_t current_segment = (uint16_t)tinyMpcPathClamp(
      floorf(path->progress), 0.0f, (float)(path->count - 2u));
  const float current_alpha = path->progress - (float)current_segment;
  const TinyMpcPathPoint current_a = tinyMpcProgressPathPoint(path, current_segment);
  const TinyMpcPathPoint current_b = tinyMpcProgressPathPoint(path, current_segment + 1u);
  const float current_x = current_a.x + current_alpha * (current_b.x - current_a.x);
  const float current_y = current_a.y + current_alpha * (current_b.y - current_a.y);
  const float current_z = current_a.z + current_alpha * (current_b.z - current_a.z);
  float best_distance_sq =
      (vehicle.x - current_x) * (vehicle.x - current_x)
      + (vehicle.y - current_y) * (vehicle.y - current_y)
      + (vehicle.z - current_z) * (vehicle.z - current_z);
  float best_progress = path->progress;
  for (uint16_t segment = first; segment <= last; ++segment) {
    const TinyMpcPathPoint a = tinyMpcProgressPathPoint(path, segment);
    const TinyMpcPathPoint b = tinyMpcProgressPathPoint(path, segment + 1u);
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float dz = b.z - a.z;
    const float length_sq = dx * dx + dy * dy + dz * dz;
    if (length_sq <= 1.0e-10f) {
      continue;
    }
    const float alpha = tinyMpcPathClamp(
        ((vehicle.x - a.x) * dx + (vehicle.y - a.y) * dy
         + (vehicle.z - a.z) * dz) / length_sq,
        0.0f, 1.0f);
    const float error_x = vehicle.x - (a.x + alpha * dx);
    const float error_y = vehicle.y - (a.y + alpha * dy);
    const float error_z = vehicle.z - (a.z + alpha * dz);
    const float distance_sq =
        error_x * error_x + error_y * error_y + error_z * error_z;
    const float candidate_progress = (float)segment + alpha;
    if (distance_sq < best_distance_sq &&
        candidate_progress + 1.0e-4f >= path->progress) {
      best_distance_sq = distance_sq;
      best_progress = candidate_progress;
    }
  }
  const float maximum_progress = tinyMpcProgressPathAdvance(
      path, path->progress, path->maximum_projection_advance_m);
  path->progress = tinyMpcPathClamp(
      best_progress, path->progress, maximum_progress);
  const TinyMpcPathPoint terminal = tinyMpcProgressPathPoint(
      path, path->count - 1u);
  const float terminal_distance_sq =
      (vehicle.x - terminal.x) * (vehicle.x - terminal.x)
      + (vehicle.y - terminal.y) * (vehicle.y - terminal.y)
      + (vehicle.z - terminal.z) * (vehicle.z - terminal.z);
  const uint16_t terminal_window_start = path->count - 1u >
      path->search_forward_segments
      ? path->count - 1u - path->search_forward_segments : 0u;
  const bool terminal_window = path->progress >= (float)terminal_window_start;
  if (path->progress >= (float)(path->count - 1u) - 1.0e-3f ||
      (terminal_window && terminal_distance_sq <=
          path->completion_radius_m * path->completion_radius_m)) {
    path->progress = (float)(path->count - 1u);
    path->complete = true;
  }
  return path->progress;
}

static inline float tinyMpcProgressPathAdvance(
    const TinyMpcProgressPath *path, float progress, float distance_m) {
  if (path->point_data == 0 || path->count < 2u || distance_m <= 0.0f) {
    return tinyMpcPathClamp(progress, 0.0f, (float)(path->count - 1u));
  }
  float advanced = tinyMpcPathClamp(
      progress, 0.0f, (float)(path->count - 1u));
  float remaining = distance_m;
  while (remaining > 0.0f && advanced < (float)(path->count - 1u)) {
    const uint16_t segment = (uint16_t)tinyMpcPathClamp(
        floorf(advanced), 0.0f, (float)(path->count - 2u));
    const float length = tinyMpcPathSegmentLength(path, segment);
    if (length <= 1.0e-6f) {
      advanced = (float)(segment + 1u);
      continue;
    }
    const float alpha = advanced - (float)segment;
    const float available = (1.0f - alpha) * length;
    if (remaining < available) {
      advanced += remaining / length;
      remaining = 0.0f;
    } else {
      remaining -= available;
      advanced = (float)(segment + 1u);
    }
  }
  return tinyMpcPathClamp(advanced, 0.0f, (float)(path->count - 1u));
}

static inline TinyMpcPathSample tinyMpcProgressPathSample(
    const TinyMpcProgressPath *path, float progress) {
  TinyMpcPathSample sample = {{0.0f, 0.0f, 0.0f},
                              {1.0f, 0.0f, 0.0f}, 0.0f, 0.0f};
  if (path->point_data == 0 || path->count == 0u) {
    return sample;
  }
  if (path->count == 1u) {
    sample.position = tinyMpcProgressPathPoint(path, 0u);
    return sample;
  }
  progress = tinyMpcPathClamp(progress, 0.0f, (float)(path->count - 1u));
  const uint16_t segment = (uint16_t)tinyMpcPathClamp(
      floorf(progress), 0.0f, (float)(path->count - 2u));
  const float alpha = progress - (float)segment;
  const TinyMpcPathPoint a = tinyMpcProgressPathPoint(path, segment);
  const TinyMpcPathPoint b = tinyMpcProgressPathPoint(path, segment + 1u);
  sample.position.x = a.x + alpha * (b.x - a.x);
  sample.position.y = a.y + alpha * (b.y - a.y);
  sample.position.z = a.z + alpha * (b.z - a.z);
  float segment_length = tinyMpcPathSegmentLength(path, segment);
  if (segment_length > 1.0e-6f) {
    sample.tangent.x = (b.x - a.x) / segment_length;
    sample.tangent.y = (b.y - a.y) / segment_length;
    sample.tangent.z = (b.z - a.z) / segment_length;
  }
  if (path->count >= 3u) {
    const uint16_t first_segment = segment + 1u < path->count - 1u
        ? segment : segment - 1u;
    const uint16_t second_segment = first_segment + 1u;
    const TinyMpcPathPoint first_a = tinyMpcProgressPathPoint(path, first_segment);
    const TinyMpcPathPoint first_b = tinyMpcProgressPathPoint(path, first_segment + 1u);
    const TinyMpcPathPoint second_b = tinyMpcProgressPathPoint(path, second_segment + 1u);
    const float first_x = first_b.x - first_a.x;
    const float first_y = first_b.y - first_a.y;
    const float second_x = second_b.x - first_b.x;
    const float second_y = second_b.y - first_b.y;
    const float first_length = hypotf(first_x, first_y);
    const float second_length = hypotf(second_x, second_y);
    if (first_length > 1.0e-6f && second_length > 1.0e-6f) {
      const float cross = first_x * second_y - first_y * second_x;
      const float dot = first_x * second_x + first_y * second_y;
      const float turn = atan2f(cross, dot);
      sample.curvature_per_m = turn / (0.5f * (first_length + second_length));
    }
  }
  const float curvature_scale = 1.0f + path->curvature_speed_gain_m
      * fabsf(sample.curvature_per_m);
  sample.speed_mps = tinyMpcPathClamp(
      path->maximum_speed_mps / curvature_scale,
      path->minimum_speed_mps, path->maximum_speed_mps);
  return sample;
}

#ifdef __cplusplus
}
#endif

#endif
