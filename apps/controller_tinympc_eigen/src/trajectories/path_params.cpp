#include <cmath>

#include "tinympc/types.h"

// Future MPC tuning: we keep Cartesian states, but need to tune lag/radial/
// vertical Q depending on the path orientation. The corresponding Cartesian
// position cost is Q_xy(s) = w_l*t(s)*t(s)^T + w_n*n(s)*n(s)^T.

constexpr float kCircleRadiusMeters = 0.5f;

static float circleAngleFromArcLength(float arc_length_m) {
  return arc_length_m / kCircleRadiusMeters;
}

// Arc-length parameterized circle:
// p_d(s) = [r cos(s/r), r sin(s/r)]^T, with r = 0.5 m.
Eigen::Vector2f circleTrajectory(float arc_length_m) {
  const float theta = circleAngleFromArcLength(arc_length_m);
  return Eigen::Vector2f(
      kCircleRadiusMeters * std::cos(theta),
      kCircleRadiusMeters * std::sin(theta));
}

// Normalize dr/du to obtain dr/ds. This works for any regular planar path and
// any parameter u; callers provide the path-specific analytical derivative.
Eigen::Vector2f unitTangent(const Eigen::Vector2f &path_derivative) {
  const float derivative_norm = path_derivative.norm();
  if (derivative_norm <= 1.0e-6f) {
    return Eigen::Vector2f::Zero();
  }
  return Eigen::Vector2f(
      path_derivative.x() / derivative_norm,
      path_derivative.y() / derivative_norm);
}

Eigen::Vector2f circleDerivative(float arc_length_m) {
  const float theta = circleAngleFromArcLength(arc_length_m);
  return Eigen::Vector2f(-std::sin(theta), std::cos(theta));
}

Eigen::Vector2f circleUnitTangent(float arc_length_m) {
  return unitTangent(circleDerivative(arc_length_m));
}

// Unit normal: n(s) = [cos(s/r), sin(s/r)]^T.
Eigen::Vector2f unitNormal(float arc_length_m) {
  const float theta = circleAngleFromArcLength(arc_length_m);
  return Eigen::Vector2f(std::cos(theta), std::sin(theta));
}
