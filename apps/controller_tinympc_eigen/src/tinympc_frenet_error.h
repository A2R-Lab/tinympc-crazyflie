#ifndef TINYMPC_FRENET_ERROR_H
#define TINYMPC_FRENET_ERROR_H

#include <math.h>

typedef struct {
  float w;
  float x;
  float y;
  float z;
} TinyMpcFrenetQuaternion;

static inline TinyMpcFrenetQuaternion tinyMpcFrenetQuaternionNormalize(
    TinyMpcFrenetQuaternion q) {
  const float inverse_norm = 1.0f / sqrtf(
      q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  q.w *= inverse_norm;
  q.x *= inverse_norm;
  q.y *= inverse_norm;
  q.z *= inverse_norm;
  return q;
}

static inline TinyMpcFrenetQuaternion tinyMpcFrenetQuaternionMultiply(
    TinyMpcFrenetQuaternion left, TinyMpcFrenetQuaternion right) {
  const TinyMpcFrenetQuaternion product = {
      left.w * right.w - left.x * right.x
          - left.y * right.y - left.z * right.z,
      left.w * right.x + left.x * right.w
          + left.y * right.z - left.z * right.y,
      left.w * right.y - left.x * right.z
          + left.y * right.w + left.z * right.x,
      left.w * right.z + left.x * right.y
          - left.y * right.x + left.z * right.w,
  };
  return tinyMpcFrenetQuaternionNormalize(product);
}

static inline TinyMpcFrenetQuaternion tinyMpcFrenetQuaternionConjugate(
    TinyMpcFrenetQuaternion q) {
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return q;
}

static inline TinyMpcFrenetQuaternion tinyMpcFrenetQuaternionFromRpy(
    float roll, float pitch, float yaw) {
  const float cr = cosf(0.5f * roll);
  const float sr = sinf(0.5f * roll);
  const float cp = cosf(0.5f * pitch);
  const float sp = sinf(0.5f * pitch);
  const float cy = cosf(0.5f * yaw);
  const float sy = sinf(0.5f * yaw);
  const TinyMpcFrenetQuaternion q = {
      cy * cp * cr + sy * sp * sr,
      cy * cp * sr - sy * sp * cr,
      cy * sp * cr + sy * cp * sr,
      sy * cp * cr - cy * sp * sr,
  };
  return tinyMpcFrenetQuaternionNormalize(q);
}

static inline TinyMpcFrenetQuaternion tinyMpcFrenetQuaternionFromRodrigues(
    const float state[12]) {
  TinyMpcFrenetQuaternion q = {
      1.0f, state[3], state[4], state[5]};
  return tinyMpcFrenetQuaternionNormalize(q);
}

static inline void tinyMpcFrenetQuaternionRotate(
    TinyMpcFrenetQuaternion q, const float input[3], float output[3]) {
  const float tx = 2.0f * (q.y * input[2] - q.z * input[1]);
  const float ty = 2.0f * (q.z * input[0] - q.x * input[2]);
  const float tz = 2.0f * (q.x * input[1] - q.y * input[0]);
  output[0] = input[0] + q.w * tx + q.y * tz - q.z * ty;
  output[1] = input[1] + q.w * ty + q.z * tx - q.x * tz;
  output[2] = input[2] + q.w * tz + q.x * ty - q.y * tx;
}

static inline float tinyMpcFrenetQuaternionYaw(
    TinyMpcFrenetQuaternion q) {
  return atan2f(
      2.0f * (q.w * q.z + q.x * q.y),
      1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

/* Encode the 12 rigid-body states in the yaw-rotating reference frame.
 * Input states use [position, Rodrigues, world/local velocity, body rate]. */
static inline void tinyMpcFrenetErrorEncode(
    const float actual[12], const float reference[12], float error[12]) {
  const TinyMpcFrenetQuaternion actual_q =
      tinyMpcFrenetQuaternionFromRodrigues(actual);
  const TinyMpcFrenetQuaternion reference_q =
      tinyMpcFrenetQuaternionFromRodrigues(reference);
  const float yaw = tinyMpcFrenetQuaternionYaw(reference_q);
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);
  const float dx = actual[0] - reference[0];
  const float dy = actual[1] - reference[1];
  error[0] = cosine * dx + sine * dy;
  error[1] = -sine * dx + cosine * dy;
  error[2] = actual[2] - reference[2];
  TinyMpcFrenetQuaternion error_q = tinyMpcFrenetQuaternionMultiply(
      tinyMpcFrenetQuaternionConjugate(reference_q), actual_q);
  if (error_q.w < 0.0f) {
    error_q.w = -error_q.w;
    error_q.x = -error_q.x;
    error_q.y = -error_q.y;
    error_q.z = -error_q.z;
  }
  const float denominator = fabsf(error_q.w) > 1.0e-6f
      ? error_q.w : copysignf(1.0e-6f, error_q.w);
  error[3] = error_q.x / denominator;
  error[4] = error_q.y / denominator;
  error[5] = error_q.z / denominator;
  const float dvx = actual[6] - reference[6];
  const float dvy = actual[7] - reference[7];
  error[6] = cosine * dvx + sine * dvy;
  error[7] = -sine * dvx + cosine * dvy;
  error[8] = actual[8] - reference[8];
  float reference_rate_world[3];
  float reference_rate_actual_body[3];
  tinyMpcFrenetQuaternionRotate(
      reference_q, &reference[9], reference_rate_world);
  tinyMpcFrenetQuaternionRotate(
      tinyMpcFrenetQuaternionConjugate(actual_q),
      reference_rate_world, reference_rate_actual_body);
  error[9] = actual[9] - reference_rate_actual_body[0];
  error[10] = actual[10] - reference_rate_actual_body[1];
  error[11] = actual[11] - reference_rate_actual_body[2];
}

static inline void tinyMpcFrenetErrorDecode(
    const float error[12], const float reference[12], float actual[12]) {
  const TinyMpcFrenetQuaternion reference_q =
      tinyMpcFrenetQuaternionFromRodrigues(reference);
  const float yaw = tinyMpcFrenetQuaternionYaw(reference_q);
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);
  actual[0] = reference[0] + cosine * error[0] - sine * error[1];
  actual[1] = reference[1] + sine * error[0] + cosine * error[1];
  actual[2] = reference[2] + error[2];
  const TinyMpcFrenetQuaternion error_q =
      tinyMpcFrenetQuaternionFromRodrigues(error);
  const TinyMpcFrenetQuaternion actual_q =
      tinyMpcFrenetQuaternionMultiply(reference_q, error_q);
  const float actual_denominator = fabsf(actual_q.w) > 1.0e-6f
      ? actual_q.w : copysignf(1.0e-6f, actual_q.w);
  actual[3] = actual_q.x / actual_denominator;
  actual[4] = actual_q.y / actual_denominator;
  actual[5] = actual_q.z / actual_denominator;
  actual[6] = reference[6] + cosine * error[6] - sine * error[7];
  actual[7] = reference[7] + sine * error[6] + cosine * error[7];
  actual[8] = reference[8] + error[8];
  float reference_rate_world[3];
  float reference_rate_actual_body[3];
  tinyMpcFrenetQuaternionRotate(
      reference_q, &reference[9], reference_rate_world);
  tinyMpcFrenetQuaternionRotate(
      tinyMpcFrenetQuaternionConjugate(actual_q),
      reference_rate_world, reference_rate_actual_body);
  actual[9] = error[9] + reference_rate_actual_body[0];
  actual[10] = error[10] + reference_rate_actual_body[1];
  actual[11] = error[11] + reference_rate_actual_body[2];
}

static inline void tinyMpcFrenetCanonicalCircleState(
    const float anchor_position[3],
    float anchor_yaw_rad,
    float nominal_roll_rad,
    float nominal_pitch_rad,
    float nominal_speed_mps,
    float nominal_yaw_rate_rad_s,
    float time_s,
    float state[12]) {
  const float yaw_delta = nominal_yaw_rate_rad_s * time_s;
  const float yaw = anchor_yaw_rad + yaw_delta;
  float forward_m = nominal_speed_mps * time_s;
  float left_m = 0.0f;
  if (fabsf(nominal_yaw_rate_rad_s) > 1.0e-6f) {
    forward_m = nominal_speed_mps / nominal_yaw_rate_rad_s
        * sinf(yaw_delta);
    left_m = nominal_speed_mps / nominal_yaw_rate_rad_s
        * (1.0f - cosf(yaw_delta));
  }
  const float anchor_cos = cosf(anchor_yaw_rad);
  const float anchor_sin = sinf(anchor_yaw_rad);
  state[0] = anchor_position[0]
      + anchor_cos * forward_m - anchor_sin * left_m;
  state[1] = anchor_position[1]
      + anchor_sin * forward_m + anchor_cos * left_m;
  state[2] = anchor_position[2];
  const TinyMpcFrenetQuaternion q = tinyMpcFrenetQuaternionFromRpy(
      nominal_roll_rad, nominal_pitch_rad, yaw);
  const float denominator = fabsf(q.w) > 1.0e-6f
      ? q.w : copysignf(1.0e-6f, q.w);
  state[3] = q.x / denominator;
  state[4] = q.y / denominator;
  state[5] = q.z / denominator;
  state[6] = nominal_speed_mps * cosf(yaw);
  state[7] = nominal_speed_mps * sinf(yaw);
  state[8] = 0.0f;
  const float world_yaw_rate[3] = {0.0f, 0.0f, nominal_yaw_rate_rad_s};
  tinyMpcFrenetQuaternionRotate(
      tinyMpcFrenetQuaternionConjugate(q), world_yaw_rate, &state[9]);
}

#endif
