#pragma once

#include <math.h>
#include <stdint.h>

#ifndef TINYMPC_PROFILE_TANH
#define TINYMPC_PROFILE_TANH 0
#endif

#if TINYMPC_PROFILE_TANH
#include "stm32f4xx.h"
#endif

#include "tinympc/types.hpp"
#include "limo_barrier_params.hpp"

struct LimoBarrierEval {
  tinytype h;
  tinytype raw;
  tiny_VectorNx grad;
  // Nonzero only in PROFILE_TANH bench builds. This intentionally includes
  // two DWT reads per tanhf call; use the clean bench for total timing.
  uint32_t activation_cycles;
};

namespace limo_barrier {

static tinytype norm_input[learned_floor_barrier::kInputDim];
static tinytype hidden1[learned_floor_barrier::kHidden1];
static tinytype hidden2[learned_floor_barrier::kHidden2];
static tinytype feature_grad[learned_floor_barrier::kInputDim];

static inline tinytype clamp(tinytype value, tinytype lower, tinytype upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

static inline tinytype abs_tiny(tinytype value)
{
  return value < tinytype(0.0f) ? -value : value;
}

static inline tinytype sign_abs_grad(tinytype value)
{
  return value >= tinytype(0.0f) ? tinytype(-1.0f) : tinytype(1.0f);
}

static inline tinytype eval_tanh(tinytype value, uint32_t *activation_cycles)
{
#if TINYMPC_PROFILE_TANH
  const uint32_t activation_start_cycles = DWT->CYCCNT;
  const tinytype result = tanhf(value);
  *activation_cycles += DWT->CYCCNT - activation_start_cycles;
  return result;
#else
  (void)activation_cycles;
  return tanhf(value);
#endif
}

static inline void eval_mlp(const tinytype features[learned_floor_barrier::kInputDim], LimoBarrierEval *out)
{
  using namespace learned_floor_barrier;

  out->activation_cycles = 0;

  for (int i = 0; i < kInputDim; ++i) {
    norm_input[i] = (features[i] - kInputMean[i]) / kInputStd[i];
  }

  for (int j = 0; j < kHidden1; ++j) {
    tinytype pre = kB1[j];
    for (int i = 0; i < kInputDim; ++i) {
      pre += norm_input[i] * kW1[i * kHidden1 + j];
    }
    hidden1[j] = eval_tanh(pre, &out->activation_cycles);
  }

  for (int j = 0; j < kHidden2; ++j) {
    tinytype pre = kB2[j];
    for (int i = 0; i < kHidden1; ++i) {
      pre += hidden1[i] * kW2[i * kHidden2 + j];
    }
    hidden2[j] = eval_tanh(pre, &out->activation_cycles);
  }

  tinytype raw = kB3;
  for (int j = 0; j < kHidden2; ++j) {
    raw += hidden2[j] * kW3[j];
  }

  for (int j = 0; j < kHidden2; ++j) {
    hidden2[j] = kW3[j] * (tinytype(1.0f) - hidden2[j] * hidden2[j]);
  }

  // Reuse activation buffers for adjoints after each layer is consumed.
  for (int j = 0; j < kHidden1; ++j) {
    tinytype accum = tinytype(0.0f);
    for (int k = 0; k < kHidden2; ++k) {
      accum += hidden2[k] * kW2[j * kHidden2 + k];
    }
    hidden1[j] = accum * (tinytype(1.0f) - hidden1[j] * hidden1[j]);
  }

  for (int i = 0; i < kInputDim; ++i) {
    tinytype accum = tinytype(0.0f);
    for (int j = 0; j < kHidden1; ++j) {
      accum += hidden1[j] * kW1[i * kHidden1 + j];
    }
    feature_grad[i] = accum / kInputStd[i];
  }

  out->raw = raw;
  out->h = raw - kSafeThreshold;
}

}  // namespace limo_barrier

static inline void limo_eval_barrier(const tiny_VectorNx &x,
                                     tinytype az_coeff,
                                     tinytype gravity_comp,
                                     tinytype fail_roll_rad,
                                     tinytype fail_pitch_rad,
                                     LimoBarrierEval *out)
{
  tinytype features[learned_floor_barrier::kInputDim];

  const tinytype z = x(2);
  const tinytype roll = x(3);
  const tinytype pitch = x(4);
  const tinytype vz = x(8);
  const tinytype wx = x(9);
  const tinytype wy = x(10);
  const tinytype wz = x(11);

  const tinytype cos_roll = cosf(roll);
  const tinytype sin_roll = sinf(roll);
  const tinytype cos_pitch = cosf(pitch);
  const tinytype sin_pitch = sinf(pitch);
  const tinytype cos_tilt = limo_barrier::clamp(cos_roll * cos_pitch, tinytype(-1.0f), tinytype(1.0f));
  const tinytype az_max = az_coeff * cos_tilt - gravity_comp;
  const tinytype roll_margin = fail_roll_rad - limo_barrier::abs_tiny(roll);
  const tinytype pitch_margin = fail_pitch_rad - limo_barrier::abs_tiny(pitch);
  const tinytype tilt_margin = roll_margin <= pitch_margin ? roll_margin : pitch_margin;
  const tinytype angvel_norm = sqrtf(wx * wx + wy * wy + wz * wz);

  features[0] = z;
  features[1] = vz;
  features[2] = cos_tilt;
  features[3] = az_max;
  features[4] = tilt_margin;
  features[5] = angvel_norm;

  limo_barrier::eval_mlp(features, out);

  out->grad.setZero();
  out->grad(2) = limo_barrier::feature_grad[0];
  out->grad(8) = limo_barrier::feature_grad[1];

  const tinytype dcos_droll = -sin_roll * cos_pitch;
  const tinytype dcos_dpitch = -cos_roll * sin_pitch;
  const tinytype dcos_total = limo_barrier::feature_grad[2] + az_coeff * limo_barrier::feature_grad[3];
  out->grad(3) = dcos_total * dcos_droll;
  out->grad(4) = dcos_total * dcos_dpitch;

  if (roll_margin <= pitch_margin) {
    out->grad(3) += limo_barrier::feature_grad[4] * limo_barrier::sign_abs_grad(roll);
  } else {
    out->grad(4) += limo_barrier::feature_grad[4] * limo_barrier::sign_abs_grad(pitch);
  }

  if (angvel_norm > tinytype(1e-6f)) {
    const tinytype inv_angvel_norm = tinytype(1.0f) / angvel_norm;
    out->grad(9) = limo_barrier::feature_grad[5] * wx * inv_angvel_norm;
    out->grad(10) = limo_barrier::feature_grad[5] * wy * inv_angvel_norm;
    out->grad(11) = limo_barrier::feature_grad[5] * wz * inv_angvel_norm;
  }
}
