#pragma once

#include "types.hpp"
#include <cmath>

// ============================================================================
// PSD Support for 2D Position Lifting
// Minimal implementation for embedded: psd_dim=3 for [1, x, y] block
// ============================================================================

// svec: Pack symmetric 3x3 matrix into 6-element vector (column-wise lower triangular)
// Order: [M(0,0), M(1,0)*sqrt2, M(2,0)*sqrt2, M(1,1), M(2,1)*sqrt2, M(2,2)]
inline tiny_VectorSvec svec_3x3(const tiny_MatrixPsd& M) {
    tiny_VectorSvec v;
    const tinytype sqrt2 = tinytype(1.41421356237);
    v(0) = M(0,0);
    v(1) = sqrt2 * M(1,0);
    v(2) = sqrt2 * M(2,0);
    v(3) = M(1,1);
    v(4) = sqrt2 * M(2,1);
    v(5) = M(2,2);
    return v;
}

// smat: Unpack 6-element vector into symmetric 3x3 matrix
template<typename Derived>
inline tiny_MatrixPsd smat_3x3(const Eigen::MatrixBase<Derived>& v) {
    tiny_MatrixPsd M;
    const tinytype sqrt2_inv = tinytype(0.70710678118);
    M(0,0) = v(0);
    M(1,0) = v(1) * sqrt2_inv; M(0,1) = M(1,0);
    M(2,0) = v(2) * sqrt2_inv; M(0,2) = M(2,0);
    M(1,1) = v(3);
    M(2,1) = v(4) * sqrt2_inv; M(1,2) = M(2,1);
    M(2,2) = v(5);
    return M;
}

// Assemble 3x3 PSD block from position (px, py)
// M = [1,   px,    py ]
//     [px, px*px, px*py]
//     [py, px*py, py*py]
inline void assemble_psd_block_2d(tinytype px, tinytype py, tiny_MatrixPsd& M) {
    M(0,0) = tinytype(1.0);
    M(0,1) = px;        M(1,0) = px;
    M(0,2) = py;        M(2,0) = py;
    M(1,1) = px * px;
    M(1,2) = px * py;   M(2,1) = M(1,2);
    M(2,2) = py * py;
}

// Lightweight 3x3 eigenvalue computation using Cardano's formula
// Returns eigenvalues in descending order
inline void eigenvalues_3x3_sym(const tiny_MatrixPsd& A, tinytype eig[3]) {
    // Characteristic polynomial: det(A - λI) = -λ³ + c2*λ² + c1*λ + c0
    // For symmetric matrix, use trace and Frobenius norm
    
    const tinytype a = A(0,0), b = A(0,1), c = A(0,2);
    const tinytype d = A(1,1), e = A(1,2);
    const tinytype f = A(2,2);
    
    // Trace and other invariants
    const tinytype p1 = b*b + c*c + e*e;
    const tinytype q = (a + d + f) / tinytype(3.0);  // trace/3
    
    const tinytype p2 = (a - q)*(a - q) + (d - q)*(d - q) + (f - q)*(f - q) + tinytype(2.0)*p1;
    const tinytype p = sqrtf(p2 / tinytype(6.0));
    
    if (p < tinytype(1e-10)) {
        // Matrix is already diagonal (or very close)
        eig[0] = eig[1] = eig[2] = q;
        return;
    }
    
    // B = (1/p) * (A - q*I)
    const tinytype inv_p = tinytype(1.0) / p;
    const tinytype B00 = (a - q) * inv_p, B01 = b * inv_p, B02 = c * inv_p;
    const tinytype B11 = (d - q) * inv_p, B12 = e * inv_p;
    const tinytype B22 = (f - q) * inv_p;
    
    // det(B) / 2
    const tinytype detB_half = tinytype(0.5) * (B00*(B11*B22 - B12*B12) 
                                                - B01*(B01*B22 - B02*B12) 
                                                + B02*(B01*B12 - B02*B11));
    
    // Clamp to [-1, 1] for acos
    tinytype r = detB_half;
    if (r < tinytype(-1.0)) r = tinytype(-1.0);
    if (r > tinytype(1.0)) r = tinytype(1.0);
    
    const tinytype phi = acosf(r) / tinytype(3.0);
    const tinytype pi = tinytype(3.14159265359);
    
    // Eigenvalues
    eig[0] = q + tinytype(2.0) * p * cosf(phi);
    eig[1] = q + tinytype(2.0) * p * cosf(phi + tinytype(2.0)*pi/tinytype(3.0));
    eig[2] = q + tinytype(2.0) * p * cosf(phi + tinytype(4.0)*pi/tinytype(3.0));
}

// Project 3x3 symmetric matrix onto PSD cone
// Uses analytical Cardano eigenvalue formula
inline void project_psd_3x3(tiny_MatrixPsd& M) {
    // Ensure symmetric
    M = tinytype(0.5) * (M + M.transpose());
    
    // Compute eigenvalues analytically using Cardano's formula
    tinytype eig[3];
    eigenvalues_3x3_sym(M, eig);
    
    // Check if already PSD
    const tinytype eps = tinytype(1e-8);
    if (eig[0] >= -eps && eig[1] >= -eps && eig[2] >= -eps) {
        // Already PSD, no projection needed
        return;
    }
    
    // Simple approximation: shift all eigenvalues up by the most negative one
    // This is equivalent to M_proj = M - lambda_min * I when lambda_min < 0
    tinytype min_eig = eig[0];
    if (eig[1] < min_eig) min_eig = eig[1];
    if (eig[2] < min_eig) min_eig = eig[2];
    
    if (min_eig < tinytype(0.0)) {
        // Add (-min_eig + eps) * I to make it PSD
        tinytype shift = -min_eig + eps;
        M(0,0) += shift;
        M(1,1) += shift;
        M(2,2) += shift;
    }
}

// Enable PSD for the problem
inline void tiny_enable_psd(struct tiny_problem* prob, struct tiny_params* params, tinytype rho_psd) {
    prob->en_psd = 1;
    params->cache.rho_psd = rho_psd;
    
    // Initialize obstacle to zero (no constraint)
    prob->psd_obs_x = tinytype(0.0);
    prob->psd_obs_y = tinytype(0.0);
    prob->psd_obs_r = tinytype(0.0);
    
    // Initialize PSD slack/dual to zero
    prob->Spsd.setZero();
    prob->Spsd_new.setZero();
    prob->Hpsd.setZero();
    
    // Initialize Spsd to identity blocks (rank-1 with x=0,y=0)
    tiny_MatrixPsd M_init;
    M_init.setZero();
    M_init(0,0) = tinytype(1.0);
    tiny_VectorSvec v_init = svec_3x3(M_init);
    for (int k = 0; k < NHORIZON; ++k) {
        prob->Spsd.col(k) = v_init;
        prob->Spsd_new.col(k) = v_init;
    }
}

// Set PSD obstacle (disk in x-y plane)
inline void tiny_set_psd_obstacle(struct tiny_problem* prob, tinytype ox, tinytype oy, tinytype r) {
    prob->psd_obs_x = ox;
    prob->psd_obs_y = oy;
    prob->psd_obs_r = r;
}

// Compute lifted distance squared to disk obstacle
// For M = [1, x, y; x, xx, xy; y, xy, yy] and disk at (ox, oy) with radius r:
// lifted_dist² = M(1,1) + M(2,2) - 2*ox*M(0,1) - 2*oy*M(0,2) + ox² + oy²
//              = xx + yy - 2*ox*x - 2*oy*y + ox² + oy²
// Constraint: lifted_dist² >= r²
inline tinytype compute_lifted_disk_violation(const tiny_MatrixPsd& M, 
                                               tinytype ox, tinytype oy, tinytype r) {
    tinytype lifted_dist2 = M(1,1) + M(2,2) - tinytype(2.0)*ox*M(0,1) - tinytype(2.0)*oy*M(0,2) 
                           + ox*ox + oy*oy;
    tinytype margin = lifted_dist2 - r*r;
    return margin;  // Positive = safe, negative = violation
}

// Compute gradient of lifted disk constraint w.r.t. M entries
// grad w.r.t. M(0,1)=x: -2*ox
// grad w.r.t. M(0,2)=y: -2*oy  
// grad w.r.t. M(1,1)=xx: 1
// grad w.r.t. M(2,2)=yy: 1
inline void compute_lifted_disk_gradient(tinytype ox, tinytype oy,
                                          tinytype& grad_x, tinytype& grad_y,
                                          tinytype& grad_xx, tinytype& grad_yy) {
    grad_x = -tinytype(2.0) * ox;
    grad_y = -tinytype(2.0) * oy;
    grad_xx = tinytype(1.0);
    grad_yy = tinytype(1.0);
}
