#pragma once

// Learned one-shot route proposals for the embedded planner: the same 15
// geometric features and MLP pair as the desktop policy
// (include/continuous/engine/oneshot_policy.hpp), in fixed-size float code
// with weights compiled in (oneshot_weights.hpp, ~2k parameters). The score
// only orders route cuts; acceptance is always the checker.

#include <tinysdp_ct_embedded/oneshot_weights.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace tinysdp_ct_embedded {

namespace detail {
template<int Rows, int Cols>
inline void dense(const float (&w)[Rows * Cols], const float (&b)[Rows], const float* x, float* y, bool relu) {
    for (int r = 0; r < Rows; ++r) {
        float v = b[r];
        for (int c = 0; c < Cols; ++c) v += w[r * Cols + c] * x[c];
        y[r] = relu && v < 0.f ? 0.f : v;
    }
}
}  // namespace detail

// log P(certified) - predicted log cost excess, from 15 features.
inline float oneshot_score(const float (&features)[15]) {
    namespace ow = oneshot_weights;
    float x[15], h1[32], h2[32], out[1];
    for (int i = 0; i < 15; ++i) x[i] = (features[i] - ow::classifier_mean[i]) / ow::classifier_std[i];
    detail::dense<32, 15>(ow::classifier_w0, ow::classifier_b0, x, h1, true);
    detail::dense<32, 32>(ow::classifier_w1, ow::classifier_b1, h1, h2, true);
    detail::dense<1, 32>(ow::classifier_w2, ow::classifier_b2, h2, out, false);
    const float logit = out[0];
    const float log_probability = logit >= 0.f ? -std::log1p(std::exp(-logit)) : logit - std::log1p(std::exp(logit));
    for (int i = 0; i < 15; ++i) x[i] = (features[i] - ow::cost_mean[i]) / ow::cost_std[i];
    detail::dense<32, 15>(ow::cost_w0, ow::cost_b0, x, h1, true);
    detail::dense<1, 32>(ow::cost_w1, ow::cost_b1, h1, out, false);
    return log_probability - out[0];
}

// Features of a cut on obstacle `o` at straight-path time `tau` in unit
// direction `u`, against spheres (center at plan time 0, radius). 3D only.
template<class Sphere>
void oneshot_features(const float* start, const float* goal, const Sphere* spheres, int n, int o, float tau,
                      const float* u, float (&x)[15]) {
    float fwd[3], dist2 = 0;
    for (int c = 0; c < 3; ++c) { fwd[c] = goal[c] - start[c]; dist2 += fwd[c] * fwd[c]; }
    const float length = std::max(1e-9f, 0.5f * std::sqrt(dist2));
    for (float& v : fwd) v /= 2 * length;
    const auto dot = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    // Straight-line clearance per obstacle (201 samples) and its rank.
    float clearance[64];
    const int m = n < 64 ? n : 64;
    for (int k = 0; k < m; ++k) {
        float best = std::numeric_limits<float>::infinity();
        for (int i = 0; i <= 200; ++i) {
            float d2 = 0;
            for (int c = 0; c < 3; ++c) {
                const float q = start[c] + (i / 200.f) * (goal[c] - start[c]) - float(spheres[k].center[c]);
                d2 += q * q;
            }
            best = std::min(best, std::sqrt(d2) - float(spheres[k].radius));
        }
        clearance[k] = best;
    }
    int rank = 0, worst = 0;
    for (int k = 0; k < m; ++k) {
        if (clearance[k] < clearance[o] || (clearance[k] == clearance[o] && k < o)) ++rank;
        if (clearance[k] < clearance[worst]) worst = k;
    }
    const float r = float(spheres[o].radius);
    float c0[3], d[3], lateral[3], clutter[3] = {0, 0, 0};
    for (int c = 0; c < 3; ++c) { c0[c] = float(spheres[o].center[c]); d[c] = c0[c] - (start[c] + tau * (goal[c] - start[c])); }
    const float along = dot(d, fwd);
    for (int c = 0; c < 3; ++c) lateral[c] = d[c] - along * fwd[c];
    const float lat = std::sqrt(dot(lateral, lateral));
    for (int k = 0; k < m; ++k) {
        if (k == o) continue;
        float off[3];
        for (int c = 0; c < 3; ++c) off[c] = float(spheres[k].center[c]) - c0[c];
        const float a = dot(off, fwd);
        for (int c = 0; c < 3; ++c) clutter[c] += off[c] - a * fwd[c];
    }
    const float clutter_norm = std::sqrt(dot(clutter, clutter));
    const auto free_space = [&](float distance) {
        if (m < 2) return 3.f;
        float best = std::numeric_limits<float>::infinity();
        for (int k = 0; k < m; ++k) {
            if (k == o) continue;
            float d2 = 0;
            for (int c = 0; c < 3; ++c) {
                const float p = c0[c] + (r + distance * length) * u[c] - float(spheres[k].center[c]);
                d2 += p * p;
            }
            best = std::min(best, std::sqrt(d2) - float(spheres[k].radius));
        }
        return best / r;
    };
    float ref[3] = {0, 0, 1};
    if (std::abs(fwd[2]) >= 0.9f) { ref[1] = 1; ref[2] = 0; }
    float e1[3] = {fwd[1] * ref[2] - fwd[2] * ref[1], fwd[2] * ref[0] - fwd[0] * ref[2], fwd[0] * ref[1] - fwd[1] * ref[0]};
    const float n1 = std::sqrt(dot(e1, e1));
    for (float& v : e1) v /= n1;
    float e2[3] = {fwd[1] * e1[2] - fwd[2] * e1[1], fwd[2] * e1[0] - fwd[0] * e1[2], fwd[0] * e1[1] - fwd[1] * e1[0]};
    const float n2 = std::sqrt(dot(e2, e2));
    for (float& v : e2) v /= n2;
    const float angle = std::atan2(dot(u, e2), dot(u, e1))
        - (lat > 1e-9f ? std::atan2(-dot(lateral, e2), -dot(lateral, e1)) : 0.f);
    const auto clip = [](float v) { return std::clamp(v, -3.f, 3.f); };
    x[0] = r / length; x[1] = lat / r; x[2] = clip(clearance[o] / r);
    x[3] = lat > 1e-9f ? -dot(u, lateral) / lat : 0.f;
    x[4] = tau; x[5] = std::abs(tau - 0.5f);
    x[6] = clip(free_space(0.1f)); x[7] = clip(free_space(0.3f));
    x[8] = clutter_norm > 1e-9f ? -dot(u, clutter) / clutter_norm : 0.f;
    x[9] = std::min(rank, 5) / 5.f; x[10] = n / 12.f;
    x[11] = clip(clearance[worst] / float(spheres[worst].radius));
    x[12] = clearance[o] < 0 ? 1.f : 0.f;
    x[13] = std::cos(angle); x[14] = std::sin(angle);
}

// Closest-approach time of obstacle `o` on the straight start-goal line.
template<class Sphere>
float straight_closest_time(const float* start, const float* goal, const Sphere& sphere) {
    float best = std::numeric_limits<float>::infinity(), when = 0.5f;
    for (int i = 0; i <= 200; ++i) {
        float d2 = 0;
        for (int c = 0; c < 3; ++c) {
            const float q = start[c] + (i / 200.f) * (goal[c] - start[c]) - float(sphere.center[c]);
            d2 += q * q;
        }
        if (d2 < best) { best = d2; when = i / 200.f; }
    }
    return when;
}

}  // namespace tinysdp_ct_embedded
