/**
 * Auto-generated Neural Network weights for Safety Certificate
 * Exported from: neural_cert_regression_profile_loose_T0.1_tstep0.01_max_max_hover_12d_wscale1.0_gust0-0_vel_only_row0_p.pt
 * 
 * Architecture: 12 → 1024 → 512 → 256 → 128 → 1
 */

#pragma once

#include <Eigen/Dense>

namespace SafetyNN {

constexpr int INPUT_DIM = 12;
constexpr int NUM_LAYERS = 0;

// StandardScaler parameters
const double SCALER_MEAN[INPUT_DIM] = {
    2.4606679355e-03, -5.0562401856e-03, -8.2492665217e-04, 3.6862561370e-05,
    7.3365123301e-05, -5.4219243442e-05, 6.3117305638e-04, -1.2465102898e-03,
    -2.1991171668e-04, 1.0823735204e-02, 4.2797721417e-05, 6.0599705849e-03
};

const double SCALER_SCALE[INPUT_DIM] = {
    5.2996748483e-01, 5.3519287937e-01, 5.3677049881e-01, 1.2448294219e-02,
    1.2490534614e-02, 1.2603533282e-02, 7.2299114128e-02, 7.1571369917e-02,
    7.1135357486e-02, 5.2914731700e-01, 5.2823155883e-01, 5.4075034751e-01
};

// Optional feature pre-scaling
const double FEATURE_SCALE[INPUT_DIM] = {
    2.0000000000e+00, 2.0000000000e+00, 2.0000000000e+00, 1.0000000000e+01,
    1.0000000000e+01, 1.0000000000e+01, 1.0000000000e+01, 1.0000000000e+01,
    1.0000000000e+01, 1.0000000000e+01, 1.0000000000e+01, 1.0000000000e+01
};

// Value function shift (aligns 0 to training threshold)
constexpr double VALUE_SHIFT = -3.2221773267e-02;

} // namespace SafetyNN
