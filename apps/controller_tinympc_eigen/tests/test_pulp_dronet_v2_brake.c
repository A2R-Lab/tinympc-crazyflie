#include "pulp_dronet_v2_brake.h"

#include <assert.h>
#include <math.h>

static bool near(float left, float right) {
  return fabsf(left - right) < 1.0e-6f;
}

int main(void) {
  PulpDronetV2Brake brake = pulpDronetV2BrakeDefault();
  assert(pulpDronetV2BrakeValid(&brake));
  assert(near(pulpDronetV2BrakeStep(&brake, 0.0f), 0.60f));
  assert(near(pulpDronetV2BrakeStep(&brake, 0.0f), 0.84f));
  const float warning = pulpDronetV2BrakeStep(&brake, 0.80f);
  assert(near(brake.collision_integral, 0.50f));
  assert(warning < 0.40f);
  for (int index = 0; index < 10; ++index) {
    (void)pulpDronetV2BrakeStep(&brake, 1.0f);
  }
  assert(near(brake.collision_integral, 3.0f));
  assert(brake.filtered_speed_scale < 1.0e-3f);
  pulpDronetV2BrakeReset(&brake);
  assert(near(brake.collision_integral, 0.0f));
  assert(near(brake.filtered_speed_scale, 0.0f));
  return 0;
}
