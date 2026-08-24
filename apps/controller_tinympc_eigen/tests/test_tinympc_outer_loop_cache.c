#include "tinympc_outer_loop_model_bank.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static float expectedAugmentedCoefficient(
    const TinyMpcOuterLoopModelData *model, int state, int input) {
  float value = 0.0f;
  for (int row = 0; row < TINYMPC_OUTER_LOOP_INPUT_DIM; ++row) {
    const float augmented_r =
        model->R[row * TINYMPC_OUTER_LOOP_INPUT_DIM + input]
        + (row == input ? TINYMPC_OUTER_LOOP_RHO : 0.0f);
    value += model->K[row * TINYMPC_OUTER_LOOP_STATE_DIM + state]
        * augmented_r;
  }
  for (int first = 0; first < TINYMPC_OUTER_LOOP_STATE_DIM; ++first) {
    for (int second = 0; second < TINYMPC_OUTER_LOOP_STATE_DIM; ++second) {
      value -= model->AmBKt[
              state * TINYMPC_OUTER_LOOP_STATE_DIM + first]
          * model->P[first * TINYMPC_OUTER_LOOP_STATE_DIM + second]
          * model->B[second * TINYMPC_OUTER_LOOP_INPUT_DIM + input];
    }
  }
  return value;
}

int main(void) {
  float maximum_error = 0.0f;
  bool raw_cache_would_be_wrong = false;
  for (int model_index = 0;
       model_index < TINYMPC_OUTER_LOOP_MODEL_COUNT; ++model_index) {
    const TinyMpcOuterLoopModelData *model =
        &tinympc_outer_loop_models[model_index];
    for (int state = 0; state < TINYMPC_OUTER_LOOP_STATE_DIM; ++state) {
      for (int input = 0; input < TINYMPC_OUTER_LOOP_INPUT_DIM; ++input) {
        const float expected = expectedAugmentedCoefficient(
            model, state, input);
        const float corrected = model->coeff_d2p[
                state * TINYMPC_OUTER_LOOP_INPUT_DIM + input]
            + TINYMPC_OUTER_LOOP_RHO
                * model->K[
                    input * TINYMPC_OUTER_LOOP_STATE_DIM + state];
        const float error = fabsf(corrected - expected);
        maximum_error = fmaxf(maximum_error, error);
        assert(error <= 0.01f + 2.0e-6f * fabsf(expected));
        raw_cache_would_be_wrong = raw_cache_would_be_wrong
            || fabsf(model->coeff_d2p[
                    state * TINYMPC_OUTER_LOOP_INPUT_DIM + input]
                - expected) > 1.0f;
      }
    }
  }
  assert(raw_cache_would_be_wrong);
  printf("outer-loop augmented cache identity max error %.8g\n",
         maximum_error);
  return 0;
}
