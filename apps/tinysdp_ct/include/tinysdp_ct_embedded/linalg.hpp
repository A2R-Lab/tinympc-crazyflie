#pragma once

// (Embedded copy of include/continuous/engine/projection.hpp.)
// Projection of a small symmetric matrix onto the PSD cone by cyclic Jacobi
// with a warm-started eigenbasis. ADMM projects nearly the same matrix at
// every iteration, so rotating by the previous eigenvectors leaves an almost
// diagonal matrix and one sweep usually suffices. Only rotations, square
// roots, and multiply-adds are used; no allocation.

#include <cmath>
#include <limits>

namespace tinysdp_ct_embedded {

template<class S>
struct JacobiSettings {
    // Stop when the off-diagonal Frobenius norm is below
    // tolerance * ||B||_F. Defaults near machine precision for S.
    S tolerance = S(8) * std::numeric_limits<S>::epsilon();
    int max_sweeps = 16;
};

// Symmetric positive-part projection of the d x d column-major matrix `in`.
// `q` holds the previous eigenvectors on entry (orthogonal; identity at the
// first call) and the current ones on exit. `b` and `t` are d*d scratch.
// Returns the number of sweeps, or -1 if Jacobi did not converge.
template<class S>
int project_psd_jacobi(const S* in, S* q, S* out, S* b, S* t, int d,
                       const JacobiSettings<S>& settings = {}) {
    if (d == 1) {
        out[0] = in[0] > S(0) ? in[0] : S(0);
        return 0;
    }
    // t = sym(in) * q, then b = q^T t.
    for (int j = 0; j < d; ++j)
        for (int i = 0; i < d; ++i) {
            S v = 0;
            for (int k = 0; k < d; ++k)
                v += S(0.5) * (in[i + k * d] + in[k + i * d]) * q[k + j * d];
            t[i + j * d] = v;
        }
    S norm = 0;
    for (int j = 0; j < d; ++j)
        for (int i = j; i < d; ++i) {
            S v = 0;
            for (int k = 0; k < d; ++k) v += q[k + i * d] * t[k + j * d];
            b[i + j * d] = v;
            b[j + i * d] = v;
            norm += (i == j ? S(1) : S(2)) * v * v;
        }
    const S threshold = settings.tolerance * settings.tolerance * norm;
    int sweep = 0;
    for (;; ++sweep) {
        S off = 0;
        for (int j = 0; j < d; ++j)
            for (int i = j + 1; i < d; ++i) off += S(2) * b[i + j * d] * b[i + j * d];
        if (off <= threshold || off == S(0)) break;
        if (sweep == settings.max_sweeps) return -1;
        for (int p = 0; p < d - 1; ++p)
            for (int r = p + 1; r < d; ++r) {
                const S apq = b[p + r * d];
                if (apq == S(0)) continue;
                const S theta = (b[r + r * d] - b[p + p * d]) / (S(2) * apq);
                const S tangent = (theta >= S(0) ? S(1) : S(-1))
                    / (std::abs(theta) + std::sqrt(theta * theta + S(1)));
                const S c = S(1) / std::sqrt(tangent * tangent + S(1));
                const S s = tangent * c;
                b[p + p * d] -= tangent * apq;
                b[r + r * d] += tangent * apq;
                b[p + r * d] = b[r + p * d] = S(0);
                for (int k = 0; k < d; ++k) {
                    if (k != p && k != r) {
                        const S bkp = b[k + p * d], bkr = b[k + r * d];
                        const S np = c * bkp - s * bkr, nr = s * bkp + c * bkr;
                        b[k + p * d] = b[p + k * d] = np;
                        b[k + r * d] = b[r + k * d] = nr;
                    }
                    const S qkp = q[k + p * d], qkr = q[k + r * d];
                    q[k + p * d] = c * qkp - s * qkr;
                    q[k + r * d] = s * qkp + c * qkr;
                }
            }
    }
    bool nonnegative = true, nonpositive = true;
    for (int i = 0; i < d; ++i) {
        nonnegative = nonnegative && b[i + i * d] >= S(0);
        nonpositive = nonpositive && b[i + i * d] <= S(0);
    }
    if (nonnegative) {
        for (int j = 0; j < d; ++j)
            for (int i = 0; i < d; ++i) out[i + j * d] = S(0.5) * (in[i + j * d] + in[j + i * d]);
        return sweep;
    }
    if (nonpositive) {
        for (int i = 0; i < d * d; ++i) out[i] = S(0);
        return sweep;
    }
    // out = sum over positive eigenvalues of lambda * q_i q_i^T.
    for (int i = 0; i < d * d; ++i) out[i] = S(0);
    for (int e = 0; e < d; ++e) {
        const S lambda = b[e + e * d];
        if (lambda <= S(0)) continue;
        const S* v = q + e * d;
        for (int j = 0; j < d; ++j) {
            const S w = lambda * v[j];
            for (int i = j; i < d; ++i) out[i + j * d] += w * v[i];
        }
    }
    for (int j = 0; j < d; ++j)
        for (int i = j + 1; i < d; ++i) out[j + i * d] = out[i + j * d];
    return sweep;
}

// Modified Gram-Schmidt on the columns of q. Rounding in accumulated
// rotations slowly erodes orthogonality; the solver calls this periodically.
template<class S>
void reorthonormalize(S* q, int d) {
    for (int j = 0; j < d; ++j) {
        S* v = q + j * d;
        for (int k = 0; k < j; ++k) {
            const S* w = q + k * d;
            S dot = 0;
            for (int i = 0; i < d; ++i) dot += v[i] * w[i];
            for (int i = 0; i < d; ++i) v[i] -= dot * w[i];
        }
        S norm = 0;
        for (int i = 0; i < d; ++i) norm += v[i] * v[i];
        norm = std::sqrt(norm);
        if (norm > S(0))
            for (int i = 0; i < d; ++i) v[i] /= norm;
    }
}

}  // namespace tinysdp_ct_embedded
