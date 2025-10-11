/**
 * INT8 Quantized Neural Network Runtime
 * 
 * Minimal INT8 FC kernel with per-channel dequantization
 * Weights stay in flash, compute in int32, output in float
 */

#pragma once

#include "safety_nn_int8_ptq.hpp"
#include <cmath>
#include <cstring>

namespace SafetyNN_INT8 {

/**
 * Fully-connected layer: INT8 weights × INT8 inputs → FP32 outputs
 * 
 * Accumulates in int32, then applies: out[i] = (acc[i] + bias[i]) * (w_scale[i] * x_scale)
 */
inline void fc_int8(
    const int8_t* weight_q,      // [out_dim][in_dim] in flash
    const float* weight_scales,  // [out_dim] per-channel scales
    const float* bias,           // [out_dim]
    const int8_t* input_q,       // [in_dim] quantized input
    float input_scale,           // input quantization scale
    float* output,               // [out_dim] float output
    int out_dim,
    int in_dim
) {
    for (int i = 0; i < out_dim; i++) {
        // Accumulate dot product in int32
        int32_t acc = 0;
        const int8_t* w_row = &weight_q[i * in_dim];
        
        for (int j = 0; j < in_dim; j++) {
            acc += (int32_t)w_row[j] * (int32_t)input_q[j];
        }
        
        // Dequantize: out = (acc * w_scale * x_scale) + bias
        float dequant = (float)acc * weight_scales[i] * input_scale;
        output[i] = dequant + bias[i];
    }
}

/**
 * ReLU activation in-place
 */
inline void relu_inplace(float* x, int n) {
    for (int i = 0; i < n; i++) {
        if (x[i] < 0.0f) x[i] = 0.0f;
    }
}

/**
 * Quantize float vector to int8
 */
inline void quantize_to_int8(const float* x_float, int8_t* x_q, float scale, int n) {
    for (int i = 0; i < n; i++) {
        float val = x_float[i] / scale;
        val = (val > 127.0f) ? 127.0f : ((val < -128.0f) ? -128.0f : val);
        x_q[i] = (int8_t)roundf(val);
    }
}

/**
 * INT8 Neural Network Evaluator
 * Computes g(x) efficiently using quantized weights
 */
class SafetyNNEvaluatorINT8 {
public:
    SafetyNNEvaluatorINT8() {
        // Nothing to initialize - weights/scales are const in flash
    }
    
    /**
     * Forward pass: x (float 12D) → g(x) (float scalar)
     * Gradient computation omitted for now (can add if needed)
     */
    float forward(const float* x) {
        // Preprocess input
        float x_proc[INPUT_DIM];
        for (int i = 0; i < INPUT_DIM; i++) {
            x_proc[i] = (x[i] / (FEATURE_SCALE[i] + 1e-12f) - SCALER_MEAN[i]) / (SCALER_SCALE[i] + 1e-9f);
        }
        
        // Quantize input
        int8_t x_q[INPUT_DIM];
        quantize_to_int8(x_proc, x_q, LAYER0_ACT_SCALE, INPUT_DIM);
        
        // Layer 0: [12] → [1024]
        float a0_float[LAYER0_OUT];
        fc_int8(
            (const int8_t*)LAYER0_WEIGHT_Q, 
            LAYER0_WEIGHT_SCALES, 
            LAYER0_BIAS,
            x_q, 
            LAYER0_ACT_SCALE,
            a0_float, 
            LAYER0_OUT, 
            LAYER0_IN
        );
        relu_inplace(a0_float, LAYER0_OUT);
        
        // Quantize for next layer
        int8_t a0_q[LAYER0_OUT];
        quantize_to_int8(a0_float, a0_q, LAYER1_ACT_SCALE, LAYER0_OUT);
        
        // Layer 1: [1024] → [512]  
        float a1_float[LAYER1_OUT];
        fc_int8(
            (const int8_t*)LAYER1_WEIGHT_Q,
            LAYER1_WEIGHT_SCALES,
            LAYER1_BIAS,
            a0_q,
            LAYER1_ACT_SCALE,
            a1_float,
            LAYER1_OUT,
            LAYER1_IN
        );
        relu_inplace(a1_float, LAYER1_OUT);
        
        // Quantize for next layer
        int8_t a1_q[LAYER1_OUT];
        quantize_to_int8(a1_float, a1_q, LAYER2_ACT_SCALE, LAYER1_OUT);
        
        // Layer 2: [512] → [256]
        float a2_float[LAYER2_OUT];
        fc_int8(
            (const int8_t*)LAYER2_WEIGHT_Q,
            LAYER2_WEIGHT_SCALES,
            LAYER2_BIAS,
            a1_q,
            LAYER2_ACT_SCALE,
            a2_float,
            LAYER2_OUT,
            LAYER2_IN
        );
        relu_inplace(a2_float, LAYER2_OUT);
        
        // Quantize for next layer
        int8_t a2_q[LAYER2_OUT];
        quantize_to_int8(a2_float, a2_q, LAYER3_ACT_SCALE, LAYER2_OUT);
        
        // Layer 3: [256] → [128]
        float a3_float[LAYER3_OUT];
        fc_int8(
            (const int8_t*)LAYER3_WEIGHT_Q,
            LAYER3_WEIGHT_SCALES,
            LAYER3_BIAS,
            a2_q,
            LAYER3_ACT_SCALE,
            a3_float,
            LAYER3_OUT,
            LAYER3_IN
        );
        relu_inplace(a3_float, LAYER3_OUT);
        
        // Quantize for final layer
        int8_t a3_q[LAYER3_OUT];
        quantize_to_int8(a3_float, a3_q, LAYER4_ACT_SCALE, LAYER3_OUT);
        
        // Layer 4: [128] → [1] (output, no ReLU)
        float output[LAYER4_OUT];
        fc_int8(
            (const int8_t*)LAYER4_WEIGHT_Q,
            LAYER4_WEIGHT_SCALES,
            LAYER4_BIAS,
            a3_q,
            LAYER4_ACT_SCALE,
            output,
            LAYER4_OUT,
            LAYER4_IN
        );
        
        // Apply value shift
        return output[0] - VALUE_SHIFT;
    }
};

} // namespace SafetyNN_INT8

