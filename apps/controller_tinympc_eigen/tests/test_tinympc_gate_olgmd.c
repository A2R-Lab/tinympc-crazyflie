#include "tinympc_gate_olgmd.h"
#include "tinyracer_vision_packet.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static TinyMpcGateOlgmdInput clearInput(uint32_t sample) {
  TinyMpcGateOlgmdInput input = {0};
  input.threat_available = true;
  input.threat_fresh = true;
  input.threat_sample = sample;
  input.forward_speed_mps = 0.0f;
  return input;
}

int main(void) {
  assert(sizeof(TinyRacerOlgmdThreatPacket) == 16u);
  assert(sizeof(TinyRacerGateObservationPacket) == 48u);
  assert(offsetof(TinyRacerOlgmdThreatPacket, payload) == 4u);
  assert(offsetof(TinyRacerOlgmdThreatPacket, checksum) == 12u);
  assert(offsetof(TinyRacerGateObservationPacket, payload) == 4u);
  assert(offsetof(TinyRacerGateObservationPacket, checksum) == 44u);

  const float corners[8] = {
    0.30f, 0.30f, 0.70f, 0.30f,
    0.70f, 0.70f, 0.30f, 0.70f,
  };
  float depth_m = 0.0f;
  assert(tinyMpcGateOlgmdDepthMeters(corners, 1.0f, 0.555f, &depth_m));
  assert(fabsf(depth_m - 1.3875f) < 1.0e-5f);
  const float off_image_corners[8] = {
    -0.20f, 0.20f, 1.20f, 0.20f,
    1.20f, 0.80f, -0.20f, 0.80f,
  };
  assert(tinyMpcGateOlgmdDepthMeters(
      off_image_corners, 1.0f, 0.555f, &depth_m));
  float out_of_contract[8] = {
    -1.01f, 0.20f, 1.20f, 0.20f,
    1.20f, 0.80f, -0.20f, 0.80f,
  };
  assert(!tinyMpcGateOlgmdDepthMeters(
      out_of_contract, 1.0f, 0.555f, &depth_m));
  float crossed[8] = {
    0.30f, 0.30f, 0.70f, 0.70f,
    0.70f, 0.30f, 0.30f, 0.70f,
  };
  assert(!tinyMpcGateOlgmdDepthMeters(crossed, 1.0f, 0.555f, &depth_m));

  TinyMpcGateOlgmdState state;
  tinyMpcGateOlgmdReset(&state);
  TinyMpcGateOlgmdInput input = {0};
  TinyMpcGateOlgmdOutput output = tinyMpcGateOlgmdStep(&state, &input);
  assert(output.triggered && output.brake_latched);  /* startup fails closed */

  /* Repeated reads of one clear frame do not advance release. */
  input = clearInput(1u);
  for (int i = 0; i < 10; ++i) {
    output = tinyMpcGateOlgmdStep(&state, &input);
  }
  assert(output.brake_latched && state.clear_frames == 1u);
  for (uint32_t sample = 2u; sample <= 5u; ++sample) {
    input = clearInput(sample);
    output = tinyMpcGateOlgmdStep(&state, &input);
  }
  assert(output.released && !output.brake_latched);

  /* Canonical mode requires two distinct positive frames; the legacy
   * compile-time default remains one frame for compatibility. */
  input = clearInput(6u);
  input.imminent_threat = true;
  input.forward_speed_mps = 2.0f;
  output = tinyMpcGateOlgmdStep(&state, &input);
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
  assert(!output.triggered && !output.brake_latched);
  input.threat_sample = 7u;
  output = tinyMpcGateOlgmdStep(&state, &input);
#endif
  assert(output.triggered && output.brake_latched);
  for (uint32_t sample = 8u; sample <= 13u; ++sample) {
    input = clearInput(sample);
    input.forward_speed_mps = sample == 8u ? 0.101f : 0.10f;
    output = tinyMpcGateOlgmdStep(&state, &input);
  }
  assert(output.released);

  /* Suppression needs three distinct valid near frames and ALIGN/TRANSIT. */
  tinyMpcGateOlgmdReset(&state);
  for (uint32_t sample = 1u; sample <= 2u; ++sample) {
    input = clearInput(sample);
    input.gate_available = input.gate_fresh = input.gate_valid = true;
    input.gate_align_or_transit = false;
    input.gate_sample = sample;
    input.gate_depth_m = 1.5f;
    output = tinyMpcGateOlgmdStep(&state, &input);
    assert(!output.near_gate_suppression);
  }
  input.threat_sample = 3u;
  input.imminent_threat = true;
  input.gate_sample = 3u;
  input.gate_align_or_transit = true;
  output = tinyMpcGateOlgmdStep(&state, &input);
  assert(output.near_gate_suppression && !output.brake_latched);

  /* Suppression never clears an already-active latch. */
  tinyMpcGateOlgmdReset(&state);
  input = clearInput(1u);
  input.imminent_threat = true;
  output = tinyMpcGateOlgmdStep(&state, &input);
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
  input.threat_sample = 2u;
  output = tinyMpcGateOlgmdStep(&state, &input);
#endif
  assert(output.brake_latched);
  for (uint32_t sample = 1u; sample <= 3u; ++sample) {
    input = clearInput(sample + 1u);
    input.gate_available = input.gate_fresh = input.gate_valid = true;
    input.gate_align_or_transit = true;
    input.gate_sample = sample;
    input.gate_depth_m = 1.0f;
    output = tinyMpcGateOlgmdStep(&state, &input);
  }
  assert(output.near_gate_suppression && output.brake_latched);

  /* A stale cycle fails closed and erases partial clear evidence. */
  input = clearInput(5u);
  tinyMpcGateOlgmdStep(&state, &input);
  input = (TinyMpcGateOlgmdInput){0};
  output = tinyMpcGateOlgmdStep(&state, &input);
  assert(output.brake_latched && state.clear_frames == 0u);
  return 0;
}
