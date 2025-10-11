/**
 * Quantized Neural Network (8-bit INT)
 * Compressed from 2744.0 KB to 686.0 KB
 * Architecture: 12 → 1024 → 512 → 256 → 128 → 1
 */

#pragma once

#include <stdint.h>

namespace SafetyNN {

constexpr int INPUT_DIM = 12;
constexpr int NUM_LAYERS = 0;

// Quantized StandardScaler (int8_t)
constexpr double SCALER_MEAN_SCALE = 1.1733472559e+04;
constexpr double SCALER_SCALE_SCALE = 2.3485884121e+02;
const int8_t SCALER_MEAN_Q[INPUT_DIM] = {
    29, -59, -10, 0, 1, -1, 7, -15, -3, 127, 1, 71
};

const int8_t SCALER_SCALE_Q[INPUT_DIM] = {
    124, 126, 126, 3, 3, 3, 17, 17, 17, 124, 124, 127
};

const float FEATURE_SCALE[INPUT_DIM] = {
    2.000000f, 2.000000f, 2.000000f, 10.000000f, 10.000000f, 10.000000f, 10.000000f, 10.000000f,
    10.000000f, 10.000000f, 10.000000f, 10.000000f
};

constexpr float VALUE_SHIFT = -3.2221773267e-02f;

} // namespace SafetyNN
