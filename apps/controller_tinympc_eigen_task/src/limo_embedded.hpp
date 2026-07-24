#pragma once

#include <math.h>
#include <stdint.h>

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
  tinytype saturation;
  tinytype qz_runtime;
  tinytype no_oracle_score;
  tinytype no_oracle_score_raw;
  tinytype previous_selected_margin;
  tiny_VectorNx predicted_next;
  bool prediction_valid;
  bool margin_valid;
  bool no_oracle_active;
  int no_oracle_hold;
  uint32_t step;
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

  // Qz is produced independently by the no-oracle residual detector below,
  // then quantized into the second cache-bank dimension.
  runtime->requested_qz =
      runtime->qz_runtime > tinytype(0.0f) ? runtime->qz_runtime : kQzMin;
  runtime->qz_index = nearest_index(runtime->requested_qz, kQzMin, kQzMax,
                                    kQzLevels);
  runtime->applied_qz = value_for_index(runtime->qz_index, kQzMin, kQzMax,
                                        kQzLevels);
}

static inline tinytype unit_score(tinytype value, tinytype scale, tinytype cap)
{
  return clamp(value / fmaxf(scale, tinytype(1e-9f)), 0.0f, cap);
}

static inline tinytype selected_failure_margin(const tiny_VectorNx &x,
                                               tinytype fail_roll_rad,
                                               tinytype fail_pitch_rad)
{
  const tinytype floor_margin = x(2);
  const tinytype roll_margin = fail_roll_rad - fabsf(x(3));
  const tinytype pitch_margin = fail_pitch_rad - fabsf(x(4));
  const tinytype rate_norm =
      sqrtf(x(9) * x(9) + x(10) * x(10) + x(11) * x(11));
  const tinytype rate_margin = kFailAngularVelocity - rate_norm;
  return fminf(floor_margin,
               fminf(roll_margin, fminf(pitch_margin, rate_margin)));
}

// Exact embedded form of the frozen no-oracle detector used by the quad
// evaluation. Tracking-failure terms are omitted because the canonical
// deployment leaves those thresholds disabled.
static inline void observe_no_oracle(Runtime *runtime,
                                     const tiny_VectorNx &x,
                                     tinytype h_now,
                                     tinytype margin_now,
                                     tinytype fail_roll_rad,
                                     tinytype fail_pitch_rad)
{
  if (runtime->qz_runtime <= tinytype(0.0f)) {
    runtime->qz_runtime = kQzMin;
  }
  if (!runtime->prediction_valid) {
    return;
  }

  const tinytype dx = x(0) - runtime->predicted_next(0);
  const tinytype dy = x(1) - runtime->predicted_next(1);
  const tinytype dz = x(2) - runtime->predicted_next(2);
  const tinytype dvx = x(6) - runtime->predicted_next(6);
  const tinytype dvy = x(7) - runtime->predicted_next(7);
  const tinytype dvz = x(8) - runtime->predicted_next(8);
  const tinytype droll = x(3) - runtime->predicted_next(3);
  const tinytype dpitch = x(4) - runtime->predicted_next(4);
  const tinytype dp = x(9) - runtime->predicted_next(9);
  const tinytype dq = x(10) - runtime->predicted_next(10);
  const tinytype dr = x(11) - runtime->predicted_next(11);

  const tinytype pos_residual = sqrtf(dx * dx + dy * dy);
  const tinytype z_residual = fabsf(dz);
  const tinytype velocity_residual =
      sqrtf(dvx * dvx + dvy * dvy + dvz * dvz);
  const tinytype vz_residual = fabsf(dvz);
  const tinytype attitude_residual =
      sqrtf(droll * droll + dpitch * dpitch +
            0.04f * (dp * dp + dq * dq + dr * dr));
  const tinytype selected_margin =
      selected_failure_margin(x, fail_roll_rad, fail_pitch_rad);
  const tinytype shrink_rate = runtime->margin_valid
      ? fmaxf(0.0f,
              (runtime->previous_selected_margin - selected_margin) / 0.05f)
      : 0.0f;
  const tinytype barrier_deficit = fmaxf(0.0f, margin_now - h_now);

  const tinytype score_raw =
      0.25f * unit_score(pos_residual, 0.006f, 2.5f) +
      0.20f * unit_score(z_residual, 0.004f, 2.0f) +
      0.65f * unit_score(velocity_residual, 0.075f, 3.0f) +
      0.45f * unit_score(vz_residual, 0.075f, 3.0f) +
      0.25f * unit_score(attitude_residual, 0.06f, 2.0f) +
      0.50f * unit_score(fmaxf(0.0f, 0.22f - selected_margin),
                         0.22f, 2.5f) +
      0.45f * unit_score(shrink_rate, 0.65f, 2.5f) +
      0.35f * unit_score(barrier_deficit, 0.25f, 2.0f) +
      0.20f * unit_score(fmaxf(0.0f, runtime->saturation - 0.50f),
                         0.50f, 1.5f);

  runtime->no_oracle_score_raw = score_raw;
  runtime->no_oracle_score = runtime->step == 0
      ? score_raw
      : 0.65f * runtime->no_oracle_score + 0.35f * score_raw;
  const bool severe =
      (velocity_residual >= 0.12f || vz_residual >= 0.10f) &&
      score_raw >= 1.80f && selected_margin <= 0.25f;
  const bool enter = runtime->no_oracle_score >= 1.60f ||
                     score_raw >= 2.0f || severe;
  const bool exit = runtime->no_oracle_score < 0.80f;
  const int requested_hold = severe ? 60 : 20;
  if (runtime->no_oracle_active) {
    if (enter) {
      runtime->no_oracle_hold =
          runtime->no_oracle_hold > requested_hold
              ? runtime->no_oracle_hold : requested_hold;
    } else if (runtime->no_oracle_hold > 0) {
      --runtime->no_oracle_hold;
    } else if (exit) {
      runtime->no_oracle_active = false;
    }
  } else if (enter) {
    runtime->no_oracle_active = true;
    runtime->no_oracle_hold = requested_hold;
  }

  const tinytype qz_target =
      runtime->no_oracle_active ? kQzMax : kQzMin;
  runtime->qz_runtime =
      0.80f * runtime->qz_runtime + 0.20f * qz_target;
  runtime->previous_selected_margin = selected_margin;
  runtime->margin_valid = true;
  ++runtime->step;
}

static inline void save_prediction(Runtime *runtime,
                                   const tiny_VectorNx &predicted_next)
{
  runtime->predicted_next = predicted_next;
  runtime->prediction_valid = true;
}

static inline void update_saturation(Runtime *runtime,
                                     const tiny_VectorNu &command,
                                     const tiny_VectorNu &lower,
                                     const tiny_VectorNu &upper)
{
  tinytype proximity = 0.0f;
  int saturated_inputs = 0;
  for (int i = 0; i < NINPUTS; ++i) {
    tinytype channel_proximity = 0.0f;
    if (upper(i) > tinytype(1e-9f)) {
      channel_proximity =
          fmaxf(channel_proximity, command(i) / upper(i));
    }
    if (lower(i) < tinytype(-1e-9f)) {
      channel_proximity =
          fmaxf(channel_proximity, command(i) / lower(i));
    }
    proximity = fmaxf(
        proximity, clamp(channel_proximity, tinytype(0.0f), tinytype(1.0f)));
    if (fabsf(command(i) - upper(i)) < tinytype(1e-3f) ||
        fabsf(command(i) - lower(i)) < tinytype(1e-3f)) {
      ++saturated_inputs;
    }
  }
  runtime->saturation_ema =
      tinytype(0.8f) * runtime->saturation_ema + tinytype(0.2f) * proximity;
  runtime->saturation =
      static_cast<tinytype>(saturated_inputs) /
      static_cast<tinytype>(NINPUTS);
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
