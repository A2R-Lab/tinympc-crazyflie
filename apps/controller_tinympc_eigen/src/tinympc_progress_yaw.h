#ifndef TINYMPC_PROGRESS_YAW_H
#define TINYMPC_PROGRESS_YAW_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline float tinyMpcProgressUnwrapYawNear(
    float angle_rad, float reference_rad) {
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.28318530717958647692f;
  float delta_rad = angle_rad - reference_rad;
  while (delta_rad > pi) {
    delta_rad -= two_pi;
  }
  while (delta_rad < -pi) {
    delta_rad += two_pi;
  }
  return reference_rad + delta_rad;
}

static inline float tinyMpcProgressSlewYawToward(
    float current_yaw_rad, float target_yaw_rad,
    float maximum_step_rad) {
  const float target_unwrapped_rad = tinyMpcProgressUnwrapYawNear(
      target_yaw_rad, current_yaw_rad);
  float step_rad = target_unwrapped_rad - current_yaw_rad;
  const float bounded_maximum_step_rad = fmaxf(maximum_step_rad, 0.0f);
  if (step_rad > bounded_maximum_step_rad) {
    step_rad = bounded_maximum_step_rad;
  } else if (step_rad < -bounded_maximum_step_rad) {
    step_rad = -bounded_maximum_step_rad;
  }
  return current_yaw_rad + step_rad;
}

/* Align to the initial path tangent once, then follow its unwrapped yaw
 * continuously. Re-arming the stationary alignment whenever normal tracking
 * exceeds the release tolerance creates a stop/go pitch transient. */
static inline bool tinyMpcProgressUpdateInitialHeadingAlignment(
    float target_yaw_rad, float maximum_step_rad,
    float release_error_rad, float *yaw_phase_rad,
    bool *alignment_complete, float *remaining_error_rad) {
  if (yaw_phase_rad == NULL || alignment_complete == NULL) {
    if (remaining_error_rad != NULL) {
      *remaining_error_rad = 0.0f;
    }
    return false;
  }
  if (*alignment_complete) {
    *yaw_phase_rad = tinyMpcProgressUnwrapYawNear(
        target_yaw_rad, *yaw_phase_rad);
    if (remaining_error_rad != NULL) {
      *remaining_error_rad = 0.0f;
    }
    return false;
  }
  *yaw_phase_rad = tinyMpcProgressSlewYawToward(
      *yaw_phase_rad, target_yaw_rad, maximum_step_rad);
  const float remaining = tinyMpcProgressUnwrapYawNear(
      target_yaw_rad, *yaw_phase_rad) - *yaw_phase_rad;
  const bool active = fabsf(remaining) > fmaxf(release_error_rad, 0.0f);
  if (!active) {
    *alignment_complete = true;
  }
  if (remaining_error_rad != NULL) {
    *remaining_error_rad = remaining;
  }
  return active;
}

/* Keep the geometric tangent continuous across the +/-pi chart seam. The
 * default progress policy additionally bounds every horizon knot around its
 * current reference phase; the explicitly uncapped policy does not. */
static inline float tinyMpcProgressHorizonYaw(
    float geometric_yaw_rad,
    float previous_horizon_yaw_rad,
    float reference_yaw_phase_rad,
    float maximum_local_deviation_rad,
    bool bound_local_deviation) {
  const float unwrapped_geometric_yaw_rad = tinyMpcProgressUnwrapYawNear(
      geometric_yaw_rad, previous_horizon_yaw_rad);
  if (!bound_local_deviation) {
    return unwrapped_geometric_yaw_rad;
  }
  float deviation_rad =
      unwrapped_geometric_yaw_rad - reference_yaw_phase_rad;
  if (deviation_rad > maximum_local_deviation_rad) {
    deviation_rad = maximum_local_deviation_rad;
  } else if (deviation_rad < -maximum_local_deviation_rad) {
    deviation_rad = -maximum_local_deviation_rad;
  }
  return reference_yaw_phase_rad + deviation_rad;
}

typedef struct {
  float roll_rad;
  float pitch_rad;
  float thrust_scale;
  bool valid;
} TinyMpcProgressBankReference;

/* Construct the attitude whose body-z thrust direction produces the desired
 * world acceleration while preserving the supplied tangent yaw. This is the
 * exact ZYX roll/pitch decomposition of
 *   b3_desired = normalize(acceleration_world + gravity * world_up).
 * Roll and pitch remain feedforward references; TinyMPC may deviate from them
 * to satisfy its dynamics and path-tunnel constraints. */
static inline TinyMpcProgressBankReference
tinyMpcProgressBankReferenceFromAcceleration(
    float acceleration_x_mps2,
    float acceleration_y_mps2,
    float acceleration_z_mps2,
    float yaw_rad,
    float gravity_mps2) {
  TinyMpcProgressBankReference reference = {0.0f, 0.0f, 0.0f, false};
  if (!isfinite(acceleration_x_mps2)
      || !isfinite(acceleration_y_mps2)
      || !isfinite(acceleration_z_mps2)
      || !isfinite(yaw_rad)
      || !isfinite(gravity_mps2)
      || gravity_mps2 <= 0.0f) {
    return reference;
  }
  const float thrust_x = acceleration_x_mps2;
  const float thrust_y = acceleration_y_mps2;
  const float thrust_z = acceleration_z_mps2 + gravity_mps2;
  const float thrust_norm = hypotf(hypotf(thrust_x, thrust_y), thrust_z);
  if (!(thrust_norm > 1.0e-6f) || !isfinite(thrust_norm)) {
    return reference;
  }
  const float inverse_thrust_norm = 1.0f / thrust_norm;
  const float b3_x = thrust_x * inverse_thrust_norm;
  const float b3_y = thrust_y * inverse_thrust_norm;
  const float b3_z = thrust_z * inverse_thrust_norm;
  const float heading_cos = cosf(yaw_rad);
  const float heading_sin = sinf(yaw_rad);
  const float b3_forward = heading_cos * b3_x + heading_sin * b3_y;
  float b3_left = -heading_sin * b3_x + heading_cos * b3_y;
  if (b3_left > 1.0f) {
    b3_left = 1.0f;
  } else if (b3_left < -1.0f) {
    b3_left = -1.0f;
  }
  reference.roll_rad = asinf(-b3_left);
  reference.pitch_rad = atan2f(b3_forward, b3_z);
  reference.thrust_scale = thrust_norm / gravity_mps2 - 1.0f;
  reference.valid = isfinite(reference.roll_rad)
      && isfinite(reference.pitch_rad)
      && isfinite(reference.thrust_scale);
  return reference;
}

typedef struct {
  float x;
  float y;
  float z;
  float w;
} TinyMpcProgressQuaternion;

typedef struct {
  float x;
  float y;
  float z;
} TinyMpcProgressBodyRate;

static inline TinyMpcProgressQuaternion tinyMpcProgressQuaternionMake(
    float x, float y, float z, float w) {
  TinyMpcProgressQuaternion quaternion = {x, y, z, w};
  return quaternion;
}

static inline TinyMpcProgressBodyRate tinyMpcProgressBodyRateMake(
    float x, float y, float z) {
  TinyMpcProgressBodyRate body_rate = {x, y, z};
  return body_rate;
}

static inline TinyMpcProgressQuaternion tinyMpcProgressQuaternionNormalize(
    TinyMpcProgressQuaternion quaternion) {
  const float norm_squared = quaternion.x * quaternion.x
      + quaternion.y * quaternion.y + quaternion.z * quaternion.z
      + quaternion.w * quaternion.w;
  if (!(norm_squared > 1.0e-20f) || !isfinite(norm_squared)) {
    return tinyMpcProgressQuaternionMake(0.0f, 0.0f, 0.0f, 1.0f);
  }
  const float inverse_norm = 1.0f / sqrtf(norm_squared);
  quaternion.x *= inverse_norm;
  quaternion.y *= inverse_norm;
  quaternion.z *= inverse_norm;
  quaternion.w *= inverse_norm;
  return quaternion;
}

/* Match Crazyflie's qqmul() component and argument convention exactly. In
 * that convention q^-1*q_next is qqmul(q_next, qinv(q)), and a body-frame
 * increment reconstructs as qqmul(delta_q, q). */
static inline TinyMpcProgressQuaternion tinyMpcProgressQuaternionMultiply(
    TinyMpcProgressQuaternion left, TinyMpcProgressQuaternion right) {
  return tinyMpcProgressQuaternionMake(
      left.w * right.x + left.z * right.y - left.y * right.z
          + left.x * right.w,
      -left.z * right.x + left.w * right.y + left.x * right.z
          + left.y * right.w,
      left.y * right.x - left.x * right.y + left.w * right.z
          + left.z * right.w,
      -left.x * right.x - left.y * right.y - left.z * right.z
          + left.w * right.w);
}

static inline TinyMpcProgressQuaternion tinyMpcProgressQuaternionExp(
    TinyMpcProgressBodyRate rotation_vector_rad) {
  const float angle_rad = sqrtf(
      rotation_vector_rad.x * rotation_vector_rad.x
      + rotation_vector_rad.y * rotation_vector_rad.y
      + rotation_vector_rad.z * rotation_vector_rad.z);
  float vector_scale;
  if (angle_rad < 1.0e-6f) {
    const float angle_squared = angle_rad * angle_rad;
    vector_scale = 0.5f - angle_squared / 48.0f;
  } else {
    vector_scale = sinf(0.5f * angle_rad) / angle_rad;
  }
  return tinyMpcProgressQuaternionNormalize(tinyMpcProgressQuaternionMake(
      rotation_vector_rad.x * vector_scale,
      rotation_vector_rad.y * vector_scale,
      rotation_vector_rad.z * vector_scale,
      cosf(0.5f * angle_rad)));
}

static inline TinyMpcProgressBodyRate tinyMpcProgressQuaternionBodyRate(
    TinyMpcProgressQuaternion current,
    TinyMpcProgressQuaternion next,
    float dt_s) {
  current = tinyMpcProgressQuaternionNormalize(current);
  next = tinyMpcProgressQuaternionNormalize(next);
  TinyMpcProgressQuaternion relative = tinyMpcProgressQuaternionMultiply(
      next, tinyMpcProgressQuaternionMake(
          -current.x, -current.y, -current.z, current.w));
  relative = tinyMpcProgressQuaternionNormalize(relative);
  /* q and -q encode the same attitude. Positive real part selects the
   * shortest arc and prevents a 2*pi rate impulse at the chart seam. */
  if (relative.w < 0.0f) {
    relative.x = -relative.x;
    relative.y = -relative.y;
    relative.z = -relative.z;
    relative.w = -relative.w;
  }
  const float vector_norm = sqrtf(
      relative.x * relative.x + relative.y * relative.y
      + relative.z * relative.z);
  float log_scale;
  if (vector_norm < 1.0e-7f) {
    log_scale = 2.0f;
  } else {
    log_scale = 2.0f * atan2f(vector_norm, relative.w) / vector_norm;
  }
  if (!(dt_s > 0.0f) || !isfinite(dt_s)) {
    return tinyMpcProgressBodyRateMake(0.0f, 0.0f, 0.0f);
  }
  return tinyMpcProgressBodyRateMake(
      relative.x * log_scale / dt_s,
      relative.y * log_scale / dt_s,
      relative.z * log_scale / dt_s);
}

static inline float tinyMpcProgressQuaternionReconstructionResidual(
    TinyMpcProgressQuaternion current,
    TinyMpcProgressQuaternion next,
    TinyMpcProgressBodyRate body_rate_rad_s,
    float dt_s) {
  const TinyMpcProgressBodyRate rotation_vector = {
      body_rate_rad_s.x * dt_s,
      body_rate_rad_s.y * dt_s,
      body_rate_rad_s.z * dt_s,
  };
  const TinyMpcProgressQuaternion reconstructed =
      tinyMpcProgressQuaternionNormalize(tinyMpcProgressQuaternionMultiply(
          tinyMpcProgressQuaternionExp(rotation_vector),
          tinyMpcProgressQuaternionNormalize(current)));
  next = tinyMpcProgressQuaternionNormalize(next);
  TinyMpcProgressQuaternion error = tinyMpcProgressQuaternionNormalize(
      tinyMpcProgressQuaternionMultiply(
          next, tinyMpcProgressQuaternionMake(
              -reconstructed.x, -reconstructed.y, -reconstructed.z,
              reconstructed.w)));
  if (error.w < 0.0f) {
    error.w = -error.w;
  }
  const float vector_norm = sqrtf(
      error.x * error.x + error.y * error.y + error.z * error.z);
  return 2.0f * atan2f(vector_norm, error.w);
}

/* The final knot has no forward interval. Copy the final real interval so the
 * terminal body-rate reference remains continuous instead of dropping to zero. */
static inline void tinyMpcProgressQuaternionHorizonBodyRates(
    const TinyMpcProgressQuaternion *attitudes,
    size_t count,
    float dt_s,
    TinyMpcProgressBodyRate *body_rates_rad_s,
    float *reconstruction_residual_rad) {
  if (attitudes == NULL || body_rates_rad_s == NULL || count == 0u) {
    return;
  }
  if (count == 1u) {
    body_rates_rad_s[0] = tinyMpcProgressBodyRateMake(0.0f, 0.0f, 0.0f);
    return;
  }
  for (size_t knot = 0u; knot + 1u < count; ++knot) {
    body_rates_rad_s[knot] = tinyMpcProgressQuaternionBodyRate(
        attitudes[knot], attitudes[knot + 1u], dt_s);
    if (reconstruction_residual_rad != NULL) {
      reconstruction_residual_rad[knot] =
          tinyMpcProgressQuaternionReconstructionResidual(
              attitudes[knot], attitudes[knot + 1u],
              body_rates_rad_s[knot], dt_s);
    }
  }
  body_rates_rad_s[count - 1u] = body_rates_rad_s[count - 2u];
}

#ifdef __cplusplus
}
#endif

#endif
