#pragma once

#include <cstdint>
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif


struct RhoAdapter {
    float rho_min;
    float rho_max;
    bool clip;
    float tolerance;
    float rho_base;
    int nx;  // number of states
    int nu;  // number of inputs
};

struct RhoBenchmarkResult {
    uint32_t time_us;
    float initial_rho;
    float final_rho;
    float pri_res;
    float dual_res;
    float pri_norm;
    float dual_norm;
};

void benchmark_rho_adaptation(
    const float* x_prev,
    const float* u_prev,
    const float* v_prev,
    float pri_res,
    float dual_res,
    RhoBenchmarkResult* result,
    RhoAdapter* adapter,
    float rho
);

void update_cache_taylor(float new_rho, float old_rho);

#ifdef __cplusplus
}
#endif
