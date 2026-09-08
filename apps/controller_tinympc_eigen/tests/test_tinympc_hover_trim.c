#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../src/tinympc_hover_trim.h"
#include "../src/tinympc_generated_params.h"
int main(void) {
  float sum = 0;
  const float expected[4] = {.1043625f, .1203625f, .1163625f, .1003625f};
  for (unsigned i = 0; i < 4; ++i) {
    const float offset = tinympc_hover_default_trim[i];
    const float hover = tinympc_generated_physical_hover_thrust[i] + offset;
    sum += offset;
    assert(fabsf(hover - expected[i]) < 1e-7f);
    const unsigned k = i * (TINYMPC_GENERATED_HORIZON_KNOTS - 1);
    const float lower = tinympc_generated_input_lower[k] - offset;
    const float upper = tinympc_generated_input_upper[k] - offset;
    assert(fabsf(hover + lower) < 1e-7f);
    assert(fabsf(hover + upper - .312852f) < 1e-6f);
    assert(fabsf(tinympc_generated_normalized_command_to_thrust(
        tinympc_generated_thrust_to_normalized_command(hover)) - hover) < 1e-6f);
  }
  assert(fabsf(sum) < 1e-7f);
  puts("Fixed hover equilibrium tests passed");
}
