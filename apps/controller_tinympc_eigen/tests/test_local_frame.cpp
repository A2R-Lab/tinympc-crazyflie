#include <Eigen/Dense>
#include "math3d.h"
#include "tinympc_generated_params.h"
#include <cassert>
#include <cstdio>

using VectorNf = Eigen::Matrix<float, 12, 1>;
using VectorMf = Eigen::Matrix<float, 4, 1>;
constexpr int NHORIZON = TINYMPC_GENERATED_HORIZON_KNOTS;
constexpr int NINPUTS = 4;
struct sensorData_t { vec gyro; };
struct Attitude { float roll, pitch, yaw; };
struct state_t { vec position, velocity; quat attitudeQuaternion; };
constexpr float DT = TINYMPC_GENERATED_MODEL_DT_S;
constexpr int modeAbs = 1;
struct setpoint_t {
  vec position, velocity; Attitude attitude, attitudeRate;
  vec acceleration{}; quat attitudeQuaternion{};
  struct { int quat=0; } mode{};
};
static VectorNf x0, xg, Xref[NHORIZON];
static VectorMf ug = VectorMf::Zero(), Uref[NHORIZON - 1];
static int work;
static bool en_traj = false;
static uint32_t step = 0, traj_idx = 0;
// Exercise held references too, although the current firmware uses hold=1.
static constexpr uint32_t traj_hold = 3;
static constexpr uint32_t traj_length = NHORIZON + 4;
static float X_ref_data[traj_length][12] = {};
static float U_ref_data[traj_length - 1][4] = {};
static const float legacy_hover_command[4] = {0.7f, 0.663f, 0.7373f, 0.633f};
static void tiny_SetGoalState(int*, VectorNf* refs, const VectorNf* goal) {
  for (int i = 0; i < NHORIZON; ++i) refs[i] = *goal;
}
static void tiny_SetGoalInput(int*, VectorMf* refs, const VectorMf* goal) {
  for (int i = 0; i < NHORIZON - 1; ++i) refs[i] = *goal;
}
#include "production_frame.h"

static void checkNear(float actual, float expected, int line) {
  if (!std::isfinite(actual) || fabsf(actual - expected) >= 2e-5f) {
    std::fprintf(stderr, "line %d: actual %.9g expected %.9g\n", line, actual, expected);
  }
  assert(std::isfinite(actual) && fabsf(actual - expected) < 2e-5f);
}
#define near(actual, expected) checkNear(actual, expected, __LINE__)
static void same(const VectorNf& actual, const VectorNf& expected) {
  for (int i = 0; i < 12; ++i) near(actual(i), expected(i));
}
static vec rotateTranslate(vec p, quat rotation, vec translation) {
  return vadd(qvrot(rotation, p), translation);
}

int main() {
  sensorData_t sensors = {mkvec(12.0f, -23.0f, 34.0f)};
  state_t state = {mkvec(3, -4, 1), mkvec(1, 2, 3),
      rpy2quat(mkvec(0, 0, M_PI_2_F))};
  setpoint_t target = {mkvec(4, -2, 1.5f), mkvec(2, 3, 4),
      {0, 0, 90}, {1, 2, 3}};
  updateInitialState(&sensors, &state);
  updateHorizonReference(&target);
  for (int i = 0; i < 6; ++i) near(x0(i), 0);
  near(x0(6), 2); near(x0(7), -1); near(x0(8), 3);
  near(x0(9), radians(12)); near(x0(10), radians(-23));
  near(x0(11), radians(34));
  near(xg(0), 2); near(xg(1), -1); near(xg(2), 0.5f);
  near(xg(6), 3); near(xg(7), -2); near(xg(8), 4);
  near(xg(9), radians(1)); near(xg(10), radians(2)); near(xg(11), radians(3));
  for (const auto& reference : Xref) same(reference, xg);

  // A rotated/translated world must produce the same optimization state and
  // reference, including nonzero roll/pitch and body rates.
  state.attitudeQuaternion = rpy2quat(mkvec(0.2f, -0.15f, 0.4f));
  target.attitude = {degrees(-0.1f), degrees(0.12f), degrees(0.6f)};
  updateInitialState(&sensors, &state);
  updateHorizonReference(&target);
  const VectorNf base_state = x0, base_goal = xg;
  for (float yaw : {-3.1f, -1.5f, 0.0f, 1.5f, 3.1f}) {
    const quat rotation = rpy2quat(mkvec(0, 0, yaw));
    const vec translation = mkvec(-7, 5, 2);
    state_t transformed = state;
    transformed.position = rotateTranslate(state.position, rotation, translation);
    transformed.velocity = qvrot(rotation, state.velocity);
    // Compose the world-yaw rotation explicitly in Euler coordinates. This
    // firmware's legacy qqmul helper uses the opposite argument convention.
    transformed.attitudeQuaternion = rpy2quat(mkvec(0.2f, -0.15f, 0.4f + yaw));
    setpoint_t transformed_target = target;
    transformed_target.position = rotateTranslate(target.position, rotation, translation);
    transformed_target.velocity = qvrot(rotation, target.velocity);
    transformed_target.attitude.yaw += degrees(yaw);
    updateInitialState(&sensors, &transformed);
    updateHorizonReference(&transformed_target);
    same(x0, base_state); same(xg, base_goal);
    transformed.attitudeQuaternion.x *= -1;
    transformed.attitudeQuaternion.y *= -1;
    transformed.attitudeQuaternion.z *= -1;
    transformed.attitudeQuaternion.w *= -1;
    updateInitialState(&sensors, &transformed);
    same(x0, base_state);
  }

  // Crossing the +/-pi representation boundary gives a small yaw error.
  state.attitudeQuaternion = rpy2quat(mkvec(0, 0, radians(179)));
  target.attitude = {0, 0, -179};
  updateInitialState(&sensors, &state);
  updateHorizonReference(&target);
  near(xg(5), tanf(radians(1)));

  // Legacy Rodrigues tables still work, and a held point is reframed after
  // movement instead of retaining coordinates from the preceding solve.
  en_traj = true;
  state.position = mkvec(0, 0, 0);
  state.attitudeQuaternion = mkquat(0, 0, 0, 1);
  X_ref_data[0][0] = 1;
  X_ref_data[0][3] = 0.1f;
  updateInitialState(&sensors, &state);
  updateHorizonReference(&target);
  near(Xref[0](0), 1); near(Xref[0](3), 0.1f);
  state.position.x = 0.25f;
  updateInitialState(&sensors, &state);
  updateHorizonReference(&target);
  for (const auto& reference : Xref) {
    near(reference(0), 0.75f); near(reference(3), 0.1f);
  }
  for (const auto& input : Uref) assert(input.isZero());
  // Consume every row, including closure, without reading a nonexistent final
  // input row. Terminal hold must remove the periodic velocity/tilt/rate target.
  X_ref_data[traj_length - 1][0] = 0.5f;
  X_ref_data[traj_length - 1][2] = 0.5f;
  X_ref_data[traj_length - 1][4] = -0.04f;
  X_ref_data[traj_length - 1][7] = 0.628f;
  X_ref_data[traj_length - 1][9] = 0.1f;
  while (traj_idx < traj_length - 1) updateHorizonReference(&target);
  for (int repeat = 0; repeat < 10; ++repeat) updateHorizonReference(&target);
  assert(traj_idx == traj_length - 1);
  assert(step == (traj_length - 1) * traj_hold);
  for (const auto& reference : Xref) {
    near(reference(0), 0.25f); near(reference(2), 0.5f);
    for (int j = 3; j < 12; ++j) near(reference(j), 0);
  }
  for (const auto& input : Uref) assert(input.isZero());
  std::puts("PASS: production local state/reference transforms and frame invariance");
}
