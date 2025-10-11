/**
 * Neural Network Safety Certificate Evaluator
 * 
 * Implements forward pass and gradient computation for learned CBF
 * using Eigen for efficient matrix operations.
 */

#pragma once

#include "Eigen.h"
#include "safety_nn_weights.hpp"
#include <cmath>

namespace SafetyNN {

using namespace Eigen;

// Use single precision (FP32) for M4F FPU efficiency and 2x memory savings
using Scalar = float;

/**
 * Neural Network evaluator for safety certificate
 * Computes g(x) and ∇g(x) for Control Barrier Function
 */
class SafetyNNEvaluator {
public:
    SafetyNNEvaluator() {
        // Initialize scaler vectors
        for (int i = 0; i < INPUT_DIM; i++) {
            scaler_mean_(i) = (Scalar)SCALER_MEAN[i];
            scaler_scale_(i) = (Scalar)SCALER_SCALE[i];
            feature_scale_(i) = (Scalar)FEATURE_SCALE[i];
        }
        
        // Initialize weight matrices and bias vectors
        for (int i = 0; i < LAYER0_OUT; i++) {
            for (int j = 0; j < LAYER0_IN; j++) {
                W0_(i, j) = LAYER0_WEIGHT[i][j];
            }
            b0_(i) = LAYER0_BIAS[i];
        }
        
        for (int i = 0; i < LAYER1_OUT; i++) {
            for (int j = 0; j < LAYER1_IN; j++) {
                W1_(i, j) = LAYER1_WEIGHT[i][j];
            }
            b1_(i) = LAYER1_BIAS[i];
        }
        
        for (int i = 0; i < LAYER2_OUT; i++) {
            for (int j = 0; j < LAYER2_IN; j++) {
                W2_(i, j) = LAYER2_WEIGHT[i][j];
            }
            b2_(i) = LAYER2_BIAS[i];
        }
        
        for (int i = 0; i < LAYER3_OUT; i++) {
            for (int j = 0; j < LAYER3_IN; j++) {
                W3_(i, j) = LAYER3_WEIGHT[i][j];
            }
            b3_(i) = LAYER3_BIAS[i];
        }
        
        for (int i = 0; i < LAYER4_OUT; i++) {
            for (int j = 0; j < LAYER4_IN; j++) {
                W4_(i, j) = LAYER4_WEIGHT[i][j];
            }
            b4_(i) = LAYER4_BIAS[i];
        }
    }
    
    /**
     * Compute g(x) and ∇g(x) for a given state
     * 
     * @param x Input state vector (12D)
     * @param g_out Output: certificate value g(x)
     * @param grad_out Output: gradient ∇g(x) (12D)
     */
    void evaluate(const Matrix<Scalar, Dynamic, 1>& x, Scalar& g_out, Matrix<Scalar, Dynamic, 1>& grad_out) {
        // Preprocess input: feature scaling + standardization
        Matrix<Scalar, INPUT_DIM, 1> x_proc;
        for (int i = 0; i < INPUT_DIM; i++) {
            x_proc(i) = (x(i) / (feature_scale_(i) + 1e-12f) - scaler_mean_(i)) / (scaler_scale_(i) + 1e-9f);
        }
        
        // Forward pass with activation tracking (for backprop)
        Matrix<Scalar, LAYER0_OUT, 1> z0 = W0_ * x_proc + b0_;
        Matrix<Scalar, LAYER0_OUT, 1> a0 = z0.cwiseMax(0.0f);  // ReLU
        
        Matrix<Scalar, LAYER1_OUT, 1> z1 = W1_ * a0 + b1_;
        Matrix<Scalar, LAYER1_OUT, 1> a1 = z1.cwiseMax(0.0f);  // ReLU
        
        Matrix<Scalar, LAYER2_OUT, 1> z2 = W2_ * a1 + b2_;
        Matrix<Scalar, LAYER2_OUT, 1> a2 = z2.cwiseMax(0.0f);  // ReLU
        
        Matrix<Scalar, LAYER3_OUT, 1> z3 = W3_ * a2 + b3_;
        Matrix<Scalar, LAYER3_OUT, 1> a3 = z3.cwiseMax(0.0f);  // ReLU
        
        Matrix<Scalar, LAYER4_OUT, 1> z4 = W4_ * a3 + b4_;
        // No activation on output layer
        
        // Output value (with shift)
        g_out = z4(0) - VALUE_SHIFT;
        
        // Backward pass for gradient
        // Start with dL/dz4 = 1 (gradient of output w.r.t. itself)
        Matrix<Scalar, LAYER4_OUT, 1> grad_z4;
        grad_z4(0) = 1.0f;
        
        // Backprop through Layer 4 (output layer, linear)
        Matrix<Scalar, LAYER3_OUT, 1> grad_a3 = W4_.transpose() * grad_z4;
        
        // Backprop through Layer 3 ReLU
        Matrix<Scalar, LAYER3_OUT, 1> grad_z3 = grad_a3;
        for (int i = 0; i < LAYER3_OUT; i++) {
            if (z3(i) <= 0.0f) grad_z3(i) = 0.0f;  // ReLU derivative
        }
        
        Matrix<Scalar, LAYER2_OUT, 1> grad_a2 = W3_.transpose() * grad_z3;
        
        // Backprop through Layer 2 ReLU
        Matrix<Scalar, LAYER2_OUT, 1> grad_z2 = grad_a2;
        for (int i = 0; i < LAYER2_OUT; i++) {
            if (z2(i) <= 0.0f) grad_z2(i) = 0.0f;
        }
        
        Matrix<Scalar, LAYER1_OUT, 1> grad_a1 = W2_.transpose() * grad_z2;
        
        // Backprop through Layer 1 ReLU
        Matrix<Scalar, LAYER1_OUT, 1> grad_z1 = grad_a1;
        for (int i = 0; i < LAYER1_OUT; i++) {
            if (z1(i) <= 0.0f) grad_z1(i) = 0.0f;
        }
        
        Matrix<Scalar, LAYER0_OUT, 1> grad_a0 = W1_.transpose() * grad_z1;
        
        // Backprop through Layer 0 ReLU
        Matrix<Scalar, LAYER0_OUT, 1> grad_z0 = grad_a0;
        for (int i = 0; i < LAYER0_OUT; i++) {
            if (z0(i) <= 0.0f) grad_z0(i) = 0.0f;
        }
        
        Matrix<Scalar, INPUT_DIM, 1> grad_x_proc = W0_.transpose() * grad_z0;
        
        // Unscale gradient: account for scaler and feature scaling
        grad_out.resize(INPUT_DIM);
        for (int i = 0; i < INPUT_DIM; i++) {
            grad_out(i) = grad_x_proc(i) / (scaler_scale_(i) + 1e-9f) / (feature_scale_(i) + 1e-12f);
        }
    }
    
private:
    // Scaler parameters (FP32)
    Matrix<Scalar, INPUT_DIM, 1> scaler_mean_;
    Matrix<Scalar, INPUT_DIM, 1> scaler_scale_;
    Matrix<Scalar, INPUT_DIM, 1> feature_scale_;
    
    // Network weights and biases (FP32)
    Matrix<Scalar, LAYER0_OUT, LAYER0_IN> W0_;
    Matrix<Scalar, LAYER0_OUT, 1> b0_;
    
    Matrix<Scalar, LAYER1_OUT, LAYER1_IN> W1_;
    Matrix<Scalar, LAYER1_OUT, 1> b1_;
    
    Matrix<Scalar, LAYER2_OUT, LAYER2_IN> W2_;
    Matrix<Scalar, LAYER2_OUT, 1> b2_;
    
    Matrix<Scalar, LAYER3_OUT, LAYER3_IN> W3_;
    Matrix<Scalar, LAYER3_OUT, 1> b3_;
    
    Matrix<Scalar, LAYER4_OUT, LAYER4_IN> W4_;
    Matrix<Scalar, LAYER4_OUT, 1> b4_;
};

} // namespace SafetyNN

