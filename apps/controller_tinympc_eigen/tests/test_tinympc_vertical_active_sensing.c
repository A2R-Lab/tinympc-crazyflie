#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "tinympc_vertical_active_sensing.h"

static void assert_close(float actual, float expected) {
  assert(fabsf(actual - expected) <= 1.0e-6f);
}

static void test_default_square_wave_phase(void) {
  const TinyMpcVerticalActiveSensingConfig config = {0.10f, 2.0f};

  assert_close(tinyMpcVerticalActiveSensingOffset(&config, 0.0f), -0.10f);
  assert_close(tinyMpcVerticalActiveSensingOffset(&config, 0.999f), -0.10f);
  assert_close(tinyMpcVerticalActiveSensingOffset(&config, 1.0f), 0.10f);
  assert_close(tinyMpcVerticalActiveSensingOffset(&config, 1.999f), 0.10f);
  assert_close(tinyMpcVerticalActiveSensingOffset(&config, 2.0f), -0.10f);
  assert_close(tinyMpcVerticalActiveSensingOffset(&config, 3.0f), 0.10f);
}

static void test_horizon_uses_future_flight_time(void) {
  const TinyMpcVerticalActiveSensingConfig config = {0.10f, 2.0f};
  const float solve_time_s = 0.98f;
  const float dt_s = 0.02f;

  assert_close(
      tinyMpcVerticalActiveSensingOffset(&config, solve_time_s), -0.10f);
  assert_close(
      tinyMpcVerticalActiveSensingOffset(&config, solve_time_s + dt_s),
      0.10f);
}

static void test_invalid_inputs_are_neutral(void) {
  const TinyMpcVerticalActiveSensingConfig zero_period = {0.10f, 0.0f};
  const TinyMpcVerticalActiveSensingConfig negative_amplitude = {-0.10f, 2.0f};

  assert_close(tinyMpcVerticalActiveSensingOffset(NULL, 0.0f), 0.0f);
  assert_close(tinyMpcVerticalActiveSensingOffset(&zero_period, 0.0f), 0.0f);
  assert_close(
      tinyMpcVerticalActiveSensingOffset(&negative_amplitude, 0.0f), 0.0f);
  assert_close(
      tinyMpcVerticalActiveSensingOffset(&zero_period, NAN), 0.0f);
}

int main(void) {
  test_default_square_wave_phase();
  test_horizon_uses_future_flight_time();
  test_invalid_inputs_are_neutral();
  puts("tinympc vertical active-sensing tests passed");
  return 0;
}
