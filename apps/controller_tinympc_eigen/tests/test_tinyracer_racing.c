#include "tinyracer_racing.h"

#include <assert.h>
#include <string.h>

static void setAllClearances(
    TinyRacerPerceptionObservation *observation, float clearance_m) {
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    observation->clearance_m[sector] = clearance_m;
  }
}

int main(void) {
  const TinyRacerRaceConfig config = {
    0.25f, 0.0f, 0.10f, 0.05f, 0.35f, 0.35f, 0.25f, 250,
    0.25f, 0.70f, 3, 0.20f, 2
  };
  TinyRacerRaceState state;
  TinyRacerRaceIntent intent;
  TinyRacerPerceptionObservation observation;
  memset(&observation, 0, sizeof(observation));
  tinyRacerRaceReset(&state);

  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
  assert(!intent.perception_fresh && !intent.constraint_active);

  observation.valid = true;
  observation.gate_valid = true;
  observation.sample = 1;
  setAllClearances(&observation, 0.30f);
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    observation.confidence[sector] = 6.0f;
  }
  observation.clearance_m[2] = 0.20f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(intent.perception_fresh && intent.gate_valid);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(intent.constraint_active && intent.constraint_changed);
  assert(intent.pass_side == 0);
  assert(intent.sector[2].boundary_distance_m > 0.099f &&
         intent.sector[2].boundary_distance_m < 0.101f);
  assert(intent.lateral_offset_m == 0.0f);
  state.pass_side = 1;  // The controller selects this from trajectory geometry.

  observation.clearance_m[2] = 0.40f;
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(intent.constraint_active && intent.constraint_changed);
  assert(!intent.sector[2].active && intent.sector[2].changed);

  setAllClearances(&observation, 0.40f);
  observation.received_age_ms = 251;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(!intent.perception_fresh && intent.constraint_active);

  observation.received_age_ms = 0;
  observation.sample++;
  state.lateral_offset_m = config.bypass_offset_m;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, true, false, true, 0x0f, &intent);
  assert(intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, true, false, true, 0x0f, &intent);
  assert(!intent.constraint_active && intent.constraint_changed);
  assert(intent.mode == TINYRACER_RACE_RECOVER);

  const float held_offset = state.lateral_offset_m;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, true, false, 0x0f, &intent);
  assert(state.lateral_offset_m == held_offset);

  observation.sample++;
  setAllClearances(&observation, 0.20f);
  observation.clearance_m[2] = 0.24f;
  observation.clearance_m[3] = 0.24f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, true, false, 0x0f, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0c, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0c, &intent);
  assert(intent.constraint_active && intent.pass_side == 0);

  tinyRacerRaceReset(&state);
  observation.sample++;
  setAllClearances(&observation, 0.40f);
  observation.clearance_m[1] = 0.24f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active);

  tinyRacerRaceReset(&state);
  observation.sample++;
  setAllClearances(&observation, 0.20f);
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active && intent.pass_side == 0);
  observation.sample++;
  setAllClearances(&observation, 0.30f);
  observation.clearance_m[0] = 0.20f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active && intent.pass_side == 0);

  tinyRacerRaceReset(&state);
  observation.sample++;
  observation.clearance_m[1] = 0.20f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active);

  tinyRacerRaceReset(&state);
  observation.sample++;
  setAllClearances(&observation, 0.20f);
  observation.confidence[2] = -0.1f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
  assert(!intent.constraint_active);

  tinyRacerRaceReset(&state);
  observation.confidence[2] = 0.0f;
  for (int sample = 0; sample < 3; ++sample) {
    observation.sample++;
    tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
    assert(!intent.constraint_active);
  }
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
  assert(intent.constraint_active);
  return 0;
}
