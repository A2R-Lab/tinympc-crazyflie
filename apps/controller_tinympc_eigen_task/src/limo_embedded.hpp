#pragma once

#include <math.h>

#include "authority_cache_bank_16x4_f32.hpp"
#include "tinympc/types.hpp"

namespace limo_embedded {

constexpr int kAuthorityLevels = 16;
constexpr int kQzLevels = 4;
constexpr tinytype kQzMin = tinytype(1.0f);
constexpr tinytype kQzMax = tinytype(1.4f);
constexpr tinytype kPolicySmoothing = tinytype(0.70f);
constexpr tinytype kAuthorityTau = tinytype(0.30f);
constexpr tinytype kAuthorityDeadband = tinytype(0.30f);
constexpr tinytype kFailAngularVelocity = tinytype(20.0f);

// Frozen CEGIS-v3 quad scheduler:
// prereg_audit/learned_authority_rl_cegis_v3/quad/final_policy.csv
static constexpr tinytype kPolicy[11] = {
    -1.98658178691f, 6.58130741222f, -1.04181888859f,
    0.431873123638f, -0.949319294365f, 1.92888487721f,
    1.96624863652f, 1.07054534043f, -0.0539227186924f,
    -0.92818277774f, 0.83834396466f,
};

struct Runtime {
  tinytype requested_w;
  tinytype applied_w;
  tinytype requested_qz;
  tinytype applied_qz;
  tinytype saturation_ema;
  int w_index;
  int qz_index;
  bool initialized;
};

static inline tinytype clamp(tinytype value, tinytype lower, tinytype upper)
{
  return value < lower ? lower : (value > upper ? upper : value);
}

static inline int nearest_index(tinytype value, tinytype lower,
                                tinytype upper, int levels)
{
  const tinytype normalized = clamp((value - lower) / (upper - lower),
                                    tinytype(0.0f), tinytype(1.0f));
  return static_cast<int>(floorf(normalized * tinytype(levels - 1) +
                                 tinytype(0.5f)));
}

static inline tinytype value_for_index(int index, tinytype lower,
                                       tinytype upper, int levels)
{
  return lower + (upper - lower) * tinytype(index) / tinytype(levels - 1);
}

static inline tinytype evaluate_policy(const tiny_VectorNx &x,
                                       const tiny_VectorNx &x_ref,
                                       tinytype h_now,
                                       tinytype margin_now,
                                       tinytype fail_roll_rad,
                                       tinytype fail_pitch_rad,
                                       const Runtime &runtime)
{
  const tinytype fixed_gate =
      clamp((kAuthorityTau - h_now) / kAuthorityDeadband, 0.0f, 1.0f);
  const tinytype floor_risk = clamp((tinytype(0.50f) - x(2)) / 0.50f,
                                    -2.0f, 2.0f);
  const tinytype descent_risk = clamp(-x(8) / 2.0f, -2.0f, 2.0f);
  const tinytype dx = x(0) - x_ref(0);
  const tinytype dy = x(1) - x_ref(1);
  const tinytype xy_error = sqrtf(dx * dx + dy * dy);
  const tinytype z_error = fabsf(x(2) - x_ref(2));
  const tinytype tilt_risk =
      clamp(fmaxf(fabsf(x(3)) / fmaxf(fail_roll_rad, 1e-6f),
                  fabsf(x(4)) / fmaxf(fail_pitch_rad, 1e-6f)),
            0.0f, 2.0f);
  const tinytype rate_norm =
      sqrtf(x(9) * x(9) + x(10) * x(10) + x(11) * x(11));
  const tinytype features[11] = {
      1.0f,
      fixed_gate,
      clamp((margin_now - h_now) / 0.50f, -2.0f, 2.0f),
      floor_risk,
      descent_risk,
      clamp(xy_error / 0.75f, 0.0f, 2.0f),
      clamp(z_error / 0.50f, 0.0f, 2.0f),
      tilt_risk,
      clamp(rate_norm / kFailAngularVelocity, 0.0f, 2.0f),
      clamp(runtime.saturation_ema, 0.0f, 1.0f),
      clamp(runtime.requested_w, 0.0f, 1.0f),
  };

  tinytype logit = 0.0f;
  for (int i = 0; i < 11; ++i) {
    logit += kPolicy[i] * features[i];
  }
  logit = clamp(logit, -30.0f, 30.0f);
  return tinytype(1.0f) / (tinytype(1.0f) + expf(-logit));
}

static inline void update(Runtime *runtime, const tiny_VectorNx &x,
                          const tiny_VectorNx &x_ref, tinytype h_now,
                          tinytype margin_now, tinytype fail_roll_rad,
                          tinytype fail_pitch_rad)
{
  const tinytype raw = evaluate_policy(x, x_ref, h_now, margin_now,
                                       fail_roll_rad, fail_pitch_rad, *runtime);
  runtime->requested_w = runtime->initialized
      ? kPolicySmoothing * runtime->requested_w +
            (tinytype(1.0f) - kPolicySmoothing) * raw
      : raw;
  runtime->initialized = true;

  runtime->w_index = nearest_index(runtime->requested_w, 0.0f, 1.0f,
                                   kAuthorityLevels);
  runtime->applied_w = value_for_index(runtime->w_index, 0.0f, 1.0f,
                                       kAuthorityLevels);

  // The deployed no-oracle layer requests up to +0.4 Qz under authority
  // intervention. Quantize it independently to the frozen four-level bank.
  runtime->requested_qz = kQzMin + (kQzMax - kQzMin) * runtime->requested_w;
  runtime->qz_index = nearest_index(runtime->requested_qz, kQzMin, kQzMax,
                                    kQzLevels);
  runtime->applied_qz = value_for_index(runtime->qz_index, kQzMin, kQzMax,
                                        kQzLevels);
}

static inline void update_saturation(Runtime *runtime,
                                     const tiny_VectorNu &command,
                                     const tiny_VectorNu &lower,
                                     const tiny_VectorNu &upper)
{
  tinytype proximity = 0.0f;
  for (int i = 0; i < NINPUTS; ++i) {
    const tinytype span = fmaxf(upper(i) - lower(i), 1e-6f);
    const tinytype centered =
        tinytype(2.0f) * (command(i) - lower(i)) / span - tinytype(1.0f);
    proximity = fmaxf(proximity, fabsf(centered));
  }
  runtime->saturation_ema =
      tinytype(0.9f) * runtime->saturation_ema + tinytype(0.1f) * proximity;
}

static inline bool install_cache(const Runtime &runtime,
                                 struct tiny_params *params)
{
  using namespace authority_cache_bank_16x4_f32;
  if (kWLevels != kAuthorityLevels || kQzLevels != limo_embedded::kQzLevels ||
      kEntries != kAuthorityLevels * limo_embedded::kQzLevels ||
      kScalarsPerEntry != 369 || fabsf(kRho - 5.0f) > 1e-6f ||
      fabsf(kBetaTrack - 1.0f) > 1e-6f ||
      fabsf(kBetaAttitude - 0.5f) > 1e-6f ||
      fabsf(kBetaReserve) > 1e-6f ||
      fabsf(authority_cache_bank_16x4_f32::kQzMin -
            limo_embedded::kQzMin) > 1e-6f ||
      fabsf(authority_cache_bank_16x4_f32::kQzMax -
            limo_embedded::kQzMax) > 1e-6f) {
    return false;
  }

  const int entry = runtime.qz_index * kAuthorityLevels + runtime.w_index;
  const float *source = kData[entry];
  int offset = 0;
  const tinytype rho = source[offset++];
  const tiny_MatrixNuNx kinf =
      Eigen::Map<const Eigen::Matrix<float, NINPUTS, NSTATES, Eigen::RowMajor>>(
          source + offset);
  offset += NINPUTS * NSTATES;
  const tiny_MatrixNxNx pinf =
      Eigen::Map<const Eigen::Matrix<float, NSTATES, NSTATES, Eigen::RowMajor>>(
          source + offset);
  offset += NSTATES * NSTATES;
  const tiny_MatrixNuNu quu_inv =
      Eigen::Map<const Eigen::Matrix<float, NINPUTS, NINPUTS, Eigen::RowMajor>>(
          source + offset);
  offset += NINPUTS * NINPUTS;
  const tiny_MatrixNxNx ambkt =
      Eigen::Map<const Eigen::Matrix<float, NSTATES, NSTATES, Eigen::RowMajor>>(
          source + offset);

  for (int level = 0; level < 2; ++level) {
    params->cache.rho[level] = rho;
    params->cache.Kinf[level] = kinf;
    params->cache.Pinf[level] = pinf;
    params->cache.Quu_inv[level] = quu_inv;
    params->cache.AmBKt[level] = ambkt;
    // The frozen bank's final 16 values are APf/BPf. This model has fdyn=0,
    // hence they are zero; the firmware's legacy coeff_d2p term must also be
    // disabled for the upstream Riccati recursion represented by this bank.
    params->cache.coeff_d2p[level].setZero();

    const tinytype tracking_scale =
        fmaxf(0.1f, 1.0f - runtime.applied_w);
    params->Q[level] << 100.0f * tracking_scale,
        100.0f * tracking_scale,
        100.0f * runtime.applied_qz,
        4.0f * (1.0f + 0.5f * runtime.applied_w),
        4.0f * (1.0f + 0.5f * runtime.applied_w),
        400.0f * (1.0f + 0.5f * runtime.applied_w),
        4.0f * tracking_scale,
        4.0f * tracking_scale,
        4.0f * runtime.applied_qz,
        2.0408163f * (1.0f + 0.5f * runtime.applied_w),
        2.0408163f * (1.0f + 0.5f * runtime.applied_w),
        4.0f * (1.0f + 0.5f * runtime.applied_w);
    params->Qf[level] = params->Q[level];
    params->R[level].setConstant(4.0f);
  }
  return true;
}

}  // namespace limo_embedded
