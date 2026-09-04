#ifndef TINYMPC_GATE_OLGMD_H
#define TINYMPC_GATE_OLGMD_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  bool brake_latched;
  bool has_threat_sample;
  uint32_t last_threat_sample;
  uint8_t clear_frames;
  uint8_t positive_frames;
  bool has_gate_sample;
  uint32_t last_gate_sample;
  uint8_t valid_near_gate_frames;
} TinyMpcGateOlgmdState;

typedef struct {
  bool threat_available;
  bool threat_fresh;
  uint32_t threat_sample;
  bool imminent_threat;
  bool gate_available;
  bool gate_fresh;
  uint32_t gate_sample;
  bool gate_valid;
  bool gate_align_or_transit;
  float gate_depth_m;
  float forward_speed_mps;
} TinyMpcGateOlgmdInput;

typedef struct {
  bool brake_latched;
  bool triggered;
  bool released;
  bool near_gate_suppression;
} TinyMpcGateOlgmdOutput;

static inline void tinyMpcGateOlgmdReset(TinyMpcGateOlgmdState *state) {
  const TinyMpcGateOlgmdState reset = {0};
  *state = reset;
}

/* Estimate range from the average projected top/bottom rail-center span. */
static inline bool tinyMpcGateOlgmdDepthMeters(
    const float corners_xy[8], float focal_length_normalized,
    float gate_span_m, float *depth_m) {
  if (corners_xy == NULL || depth_m == NULL ||
      !isfinite(focal_length_normalized) || focal_length_normalized <= 0.0f ||
      !isfinite(gate_span_m) || gate_span_m <= 0.0f) {
    return false;
  }
  for (int i = 0; i < 8; ++i) {
    if (!isfinite(corners_xy[i]) || corners_xy[i] < -1.0f ||
        corners_xy[i] > 2.0f) {
      return false;
    }
  }
  const float top_x = corners_xy[2] - corners_xy[0];
  const float top_y = corners_xy[3] - corners_xy[1];
  const float bottom_x = corners_xy[4] - corners_xy[6];
  const float bottom_y = corners_xy[5] - corners_xy[7];
  const float top_span = hypotf(top_x, top_y);
  const float bottom_span = hypotf(bottom_x, bottom_y);
  const float average_span = 0.5f * (top_span + bottom_span);
  float signed_area_twice = 0.0f;
  float turn_sign = 0.0f;
  for (int i = 0; i < 4; ++i) {
    const int next = (i + 1) % 4;
    const int after = (i + 2) % 4;
    const float ax = corners_xy[2 * next] - corners_xy[2 * i];
    const float ay = corners_xy[2 * next + 1] - corners_xy[2 * i + 1];
    const float bx = corners_xy[2 * after] - corners_xy[2 * next];
    const float by = corners_xy[2 * after + 1] - corners_xy[2 * next + 1];
    const float turn = ax * by - ay * bx;
    if (i == 0) {
      turn_sign = turn;
    } else if (turn * turn_sign <= 0.0f) {
      return false;
    }
    signed_area_twice +=
        corners_xy[2 * i] * corners_xy[2 * next + 1] -
        corners_xy[2 * i + 1] * corners_xy[2 * next];
  }
  if (average_span < 0.02f || fabsf(signed_area_twice) < 0.002f ||
      fabsf(turn_sign) < 1.0e-5f) {
    return false;
  }
  *depth_m = gate_span_m * focal_length_normalized / average_span;
  return isfinite(*depth_m) && *depth_m > 0.0f;
}

static inline TinyMpcGateOlgmdOutput tinyMpcGateOlgmdStep(
    TinyMpcGateOlgmdState *state, const TinyMpcGateOlgmdInput *input) {
  TinyMpcGateOlgmdOutput output = {0};
  const bool gate_near = input->gate_available && input->gate_fresh &&
      input->gate_valid &&
      isfinite(input->gate_depth_m) && input->gate_depth_m <= 1.5f &&
      input->gate_depth_m > 0.0f;
  if (!gate_near) {
    state->valid_near_gate_frames = 0u;
  } else if (!state->has_gate_sample ||
             input->gate_sample != state->last_gate_sample) {
    state->has_gate_sample = true;
    state->last_gate_sample = input->gate_sample;
    if (state->valid_near_gate_frames < UINT8_MAX) {
      ++state->valid_near_gate_frames;
    }
  }
  output.near_gate_suppression =
      gate_near && input->gate_align_or_transit &&
      state->valid_near_gate_frames >= 3u;

  const bool threat_new = input->threat_available && input->threat_fresh &&
      (!state->has_threat_sample ||
       input->threat_sample != state->last_threat_sample);
  if (threat_new) {
    state->has_threat_sample = true;
    state->last_threat_sample = input->threat_sample;
  }

  /* Missing or expired threat data fails closed. Gate context suppresses only
   * an otherwise-valid new positive inference, never transport failure. */
  const bool stale_fail_safe =
      !input->threat_available || !input->threat_fresh;
  if (threat_new) {
    if (input->imminent_threat) {
      if (state->positive_frames < UINT8_MAX) {
        ++state->positive_frames;
      }
    } else {
      state->positive_frames = 0u;
    }
  }
  const bool positive_trigger = threat_new && input->imminent_threat &&
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
      state->positive_frames >= 2u &&
#endif
      !output.near_gate_suppression;
  if (!state->brake_latched && (stale_fail_safe || positive_trigger)) {
    state->brake_latched = true;
    state->clear_frames = 0u;
    output.triggered = true;
  }

  if (state->brake_latched) {
    if (stale_fail_safe || (threat_new && input->imminent_threat)) {
      state->clear_frames = 0u;
    } else if (threat_new && !input->imminent_threat &&
               isfinite(input->forward_speed_mps) &&
               fabsf(input->forward_speed_mps) <= 0.10f) {
      if (state->clear_frames < UINT8_MAX) {
        ++state->clear_frames;
      }
      if (state->clear_frames >= 5u) {
        state->brake_latched = false;
        state->clear_frames = 0u;
        output.released = true;
      }
    } else if (threat_new || !isfinite(input->forward_speed_mps) ||
               fabsf(input->forward_speed_mps) > 0.10f) {
      state->clear_frames = 0u;
    }
  }
  output.brake_latched = state->brake_latched;
  return output;
}

#endif  // TINYMPC_GATE_OLGMD_H
