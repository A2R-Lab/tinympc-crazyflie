#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <tinympc/tiny_api.hpp>

// ---------------- svec/smat (half-vectorization with sqrt(2) scaling) -----------------
// Packs a symmetric p x p matrix S into a length m = p(p+1)/2 vector v
// using column-wise lower-triangular order: [S(0,0), S(1,0)*sqrt2, ..., S(p-1,0)*sqrt2,
//                                            S(1,1), S(2,1)*sqrt2, ..., S(p-1,1)*sqrt2, ...]
// Off-diagonals are scaled by sqrt(2) so that:  trace(A^T B) = svec(A)^T svec(B)
inline int svec_size(int p) { return p * (p + 1) / 2; }

inline void smat_inplace(const tinyVector& v, int p, tinyMatrix& S) {
    S.setZero(p, p);
    const tinytype sqrt2 = tinytype(M_SQRT2);
    int idx = 0;
    for (int c = 0; c < p; ++c) {
        // diagonal
        S(c, c) = v(idx++);
        // below diagonal (rows r > c)
        for (int r = c + 1; r < p; ++r) {
            tinytype x = v(idx++) / sqrt2; // invert scaling
            S(r, c) = x;
            S(c, r) = x;
        }
    }
}

inline void svec_inplace(const tinyMatrix& S, tinyVector& v) {
    const int p = S.rows();
    const tinytype sqrt2 = tinytype(M_SQRT2);
    v.setZero(svec_size(p));
    int idx = 0;
    for (int c = 0; c < p; ++c) {
        v(idx++) = S(c, c);
        for (int r = c + 1; r < p; ++r) {
            v(idx++) = sqrt2 * S(r, c);
        }
    }
}

// Manual Kronecker product implementation
template<typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, Eigen::Dynamic, Eigen::Dynamic>
kronecker(const Eigen::MatrixBase<Derived1>& A, const Eigen::MatrixBase<Derived2>& B) {
    Eigen::Matrix<typename Derived1::Scalar, Eigen::Dynamic, Eigen::Dynamic> result(A.rows()*B.rows(), A.cols()*B.cols());
    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            result.block(i*B.rows(), j*B.cols(), B.rows(), B.cols()) = A(i,j) * B;
        }
    }
    return result;
}

// Build lifted (A,B) exactly like the Julia file
inline void tiny_build_lifted_from_base(
    const Eigen::Matrix<tinytype, Eigen::Dynamic, Eigen::Dynamic>& Ad,
    const Eigen::Matrix<tinytype, Eigen::Dynamic, Eigen::Dynamic>& Bd,
    Eigen::Matrix<tinytype, Eigen::Dynamic, Eigen::Dynamic>& A_out,
    Eigen::Matrix<tinytype, Eigen::Dynamic, Eigen::Dynamic>& B_out)
{
    const int nx0 = Ad.rows();
    const int nu0 = Bd.cols();
    const int nxx = nx0*nx0;
    const int nxu = nx0*nu0;
    const int nux = nu0*nx0;
    const int nuu = nu0*nu0;

    A_out.setZero(nx0 + nxx, nx0 + nxx);
    A_out.block(0,0,nx0,nx0)            = Ad;
    A_out.block(nx0,nx0,nxx,nxx)        = kronecker(Ad, Ad);

    B_out.setZero(nx0 + nxx, nu0 + nxu + nux + nuu);
    B_out.block(0,0,nx0,nu0)            = Bd;
    B_out.block(nx0,     nu0,           nxx, nxu) = kronecker(Bd, Ad);
    B_out.block(nx0,     nu0 + nxu,     nxx, nux) = kronecker(Ad, Bd);
    B_out.block(nx0,     nu0 + nxu+nux, nxx, nuu) = kronecker(Bd, Bd);

    std::cout << "[PSD] Built lifted A(" << A_out.rows() << "x" << A_out.cols()
              << ") B(" << B_out.rows() << "x" << B_out.cols() << ")\n";
}

// Enable PSD block (stores nx0, nu0 and allocates S/H buffers)
inline int tiny_enable_psd(TinySolver* solver, int nx0, int nu0, tinytype rho_psd)
{
    if (!solver) { std::cout << "tiny_enable_psd: solver nullptr\n"; return 1; }
    solver->settings->en_psd   = 1;
    solver->settings->nx0_psd  = nx0;
    solver->settings->nu0_psd  = nu0;
    solver->cache->rho_psd     = rho_psd;

    const int psd_dim = 1 + nx0 + nu0;
    const int N = solver->work->N;
    const int m = svec_size(psd_dim);

    solver->work->Spsd     = tinyMatrix::Zero(m, N);
    solver->work->Spsd_new = tinyMatrix::Zero(m, N);
    solver->work->Hpsd     = tinyMatrix::Zero(m, N);

    // std::cout << "[PSD] Enabled: nx0=" << nx0 << " nu0=" << nu0
    //           << " psd_dim=" << psd_dim << " (half m=" << m << ")"
    //           << " rho_psd=" << rho_psd << "\n";
    return 0;
}

// ---------------- Time-varying linear constraint helpers --------------------
inline int tiny_enable_tv_state_linear(TinySolver* solver, int n_constr) {
    if (!solver) { std::cout << "tiny_enable_tv_state_linear: solver nullptr\n"; return 1; }
    solver->settings->en_tv_state_linear = 1;
    solver->work->numtvStateLinear = n_constr;
    solver->work->tv_Alin_x = tinyMatrix::Zero(n_constr * solver->work->N, solver->work->nx);
    solver->work->tv_blin_x = tinyMatrix::Zero(n_constr, solver->work->N);
    solver->work->vlnew_tv = solver->work->x;
    solver->work->gl_tv    = tinyMatrix::Zero(solver->work->nx, solver->work->N);
    return 0;
}

// Base-level tangent half-space update per-stage using the latest rollout x
// a^T z <= b form, where only base (x,y) entries in a are nonzero.
inline void tiny_update_base_tangent_avoidance_tv(
    TinySolver* solver, tinytype ox, tinytype oy, tinytype r, tinytype margin)
{
    const int N   = solver->work->N;
    const int nxL = solver->work->nx;
    const int nc  = std::max(1, solver->work->numtvStateLinear);

    // Track min distance for debugging
    tinytype min_dist = 1e9;
    int min_dist_k = 0;

    for (int k = 0; k < N; ++k) {
        tinytype x = solver->work->x(0,k);
        tinytype y = solver->work->x(1,k);
        
        // Safety check for NaN/Inf values in trajectory
        if (!std::isfinite(x) || !std::isfinite(y)) {
            std::cout << "[TV-UPDATE] WARNING: k=" << k << " non-finite x=" << x << " y=" << y 
                      << " - using previous constraint\n";
            continue; // Keep previous constraint
        }
        
        tinytype dx = x - ox;
        tinytype dy = y - oy;
        tinytype d  = std::sqrt(dx*dx + dy*dy);
        
        if (d < min_dist) {
            min_dist = d;
            min_dist_k = k;
        }

        // Safety: if too close to center, use a default separating plane
        const tinytype safety_eps = tinytype(1e-6);
        tinytype nx = 1.0, ny = 0.0;
        if (d > safety_eps) { 
            nx = dx / d; 
            ny = dy / d; 
        } else {
            // Use direction from previous stage or default
            std::cout << "[TV-UPDATE] WARNING: k=" << k << " very close to center d=" << d 
                      << " - using default normal\n";
        }

        // Half-space: n^T [x;y] >= n^T [ox;oy] + r + margin
        // Convert to a^T z <= b with a = -[n_x, n_y, 0,...], b = -(n^T o + r + margin)
        tinyVector a = tinyVector::Zero(nxL);
        a(0) = -nx; a(1) = -ny;
        tinytype b = - (nx*ox + ny*oy + r + margin);

        // Safety check on constraint coefficients
        if (!std::isfinite(b) || a.squaredNorm() < safety_eps) {
            std::cout << "[TV-UPDATE] WARNING: k=" << k << " invalid constraint ||a||=" 
                      << a.norm() << " b=" << b << " - skipping\n";
            continue;
        }

        const int row = k*nc + 0;
        if (row >= 0 && row < solver->work->tv_Alin_x.rows()) {
            solver->work->tv_Alin_x.row(row) = a.transpose();
        }
        if (solver->work->tv_blin_x.rows() >= 1 && k < solver->work->tv_blin_x.cols()) {
            solver->work->tv_blin_x(0,k) = b;
        }
    }
    
    // Print distance summary every update (sampled to reduce spam)
    tinytype signed_dist = min_dist - r;
    if (solver->work->iter % 10 == 0) {
        std::cout << "[TV-UPDATE] iter=" << solver->work->iter 
                  << " min_signed_dist=" << signed_dist << " at k=" << min_dist_k << "\n";
    }
}

// Global stores for multi-disk TV update (demo convenience to avoid API churn)
inline std::vector<std::array<tinytype,3>>& tv_disks_store() {
    static std::vector<std::array<tinytype,3>> disks;
    return disks;
}
inline tinytype& tv_disks_margin_store() {
    static tinytype margin = tinytype(0);
    return margin;
}

// Per-stage disk stores for time-varying obstacle predictions
inline std::vector<std::vector<std::array<tinytype,3>>>& tv_disks_per_stage_store() {
    static std::vector<std::vector<std::array<tinytype,3>>> disks_per_stage;
    return disks_per_stage;
}
inline bool& tv_use_per_stage_store() {
    static bool use_per_stage = false;
    return use_per_stage;
}

// Multi-disk version: update per-stage tangent half-spaces for all provided disks
// Uses the same indexing layout as update_slack(): rows are grouped per stage.
inline void tiny_update_base_tangent_avoidance_tv_multi(
    TinySolver* solver,
    const std::vector<std::array<tinytype,3>>& disks,
    tinytype margin)
{
    if (!solver) return;
    const int N   = solver->work->N;
    const int nxL = solver->work->nx;
    const int nc  = std::max(1, solver->work->numtvStateLinear);
    const int m   = static_cast<int>(disks.size());
    if (m <= 0) return;
    // We expect nc == m; if not, we only fill the first min(nc, m) rows per stage
    const int rows_per_stage = std::min(nc, m);

    for (int k = 0; k < N; ++k) {
        tinytype x = solver->work->x(0,k);
        tinytype y = solver->work->x(1,k);

        // Sanitize current state
        if (!std::isfinite(x) || !std::isfinite(y)) {
            // Keep previous constraints if trajectory is non-finite
            continue;
        }

        for (int j = 0; j < rows_per_stage; ++j) {
            const tinytype ox = disks[j][0];
            const tinytype oy = disks[j][1];
            const tinytype r  = disks[j][2];

            tinytype dx = x - ox;
            tinytype dy = y - oy;
            tinytype d  = std::sqrt(dx*dx + dy*dy);
            const tinytype safety_eps = tinytype(1e-6);
            tinytype nx = 1.0, ny = 0.0;
            if (d > safety_eps) { nx = dx / d; ny = dy / d; }

            // Half-space n^T [x;y] >= n^T o + r + margin
            tinyVector a = tinyVector::Zero(nxL);
            a(0) = -nx; a(1) = -ny; // a^T z <= b form
            tinytype b = - (nx*ox + ny*oy + r + margin);

            if (!std::isfinite(b) || !a.allFinite() || a.squaredNorm() < safety_eps) {
                continue;
            }

            const int row = k*nc + j;
            if (row >= 0 && row < solver->work->tv_Alin_x.rows()) {
                solver->work->tv_Alin_x.row(row) = a.transpose();
            }
            if (j < solver->work->tv_blin_x.rows() && k < solver->work->tv_blin_x.cols()) {
                solver->work->tv_blin_x(j, k) = b;
            }
        }
    }
}

// Per-stage multi-disk version: update per-stage tangent half-spaces with time-varying obstacles
// Uses different disk positions for each horizon stage k
inline void tiny_update_base_tangent_avoidance_tv_multi_per_stage(
    TinySolver* solver,
    const std::vector<std::vector<std::array<tinytype,3>>>& disks_per_stage,
    tinytype margin)
{
    if (!solver) return;
    const int N   = solver->work->N;
    const int nxL = solver->work->nx;
    const int nc  = std::max(1, solver->work->numtvStateLinear);
    if (disks_per_stage.empty()) return;

    for (int k = 0; k < N; ++k) {
        tinytype x = solver->work->x(0,k);
        tinytype y = solver->work->x(1,k);

        // Sanitize current state
        if (!std::isfinite(x) || !std::isfinite(y)) {
            continue;
        }

        // Get disks for this stage (use last available if beyond prediction horizon)
        const auto& stage_disks = (k < static_cast<int>(disks_per_stage.size()))
                                ? disks_per_stage[k]
                                : disks_per_stage.back();
        const int m = static_cast<int>(stage_disks.size());
        if (m <= 0) continue;

        const int rows_per_stage = std::min(nc, m);
        for (int j = 0; j < rows_per_stage; ++j) {
            const tinytype ox = stage_disks[j][0];
            const tinytype oy = stage_disks[j][1];
            const tinytype r  = stage_disks[j][2];

            tinytype dx = x - ox;
            tinytype dy = y - oy;
            tinytype d  = std::sqrt(dx*dx + dy*dy);
            const tinytype safety_eps = tinytype(1e-6);
            tinytype nx = 1.0, ny = 0.0;
            if (d > safety_eps) { nx = dx / d; ny = dy / d; }

            // Half-space n^T [x;y] >= n^T o + r + margin
            tinyVector a = tinyVector::Zero(nxL);
            a(0) = -nx; a(1) = -ny; // a^T z <= b form
            tinytype b = - (nx*ox + ny*oy + r + margin);

            if (!std::isfinite(b) || !a.allFinite() || a.squaredNorm() < safety_eps) {
                continue;
            }

            const int row = k*nc + j;
            if (row >= 0 && row < solver->work->tv_Alin_x.rows()) {
                solver->work->tv_Alin_x.row(row) = a.transpose();
            }
            if (j < solver->work->tv_blin_x.rows() && k < solver->work->tv_blin_x.cols()) {
                solver->work->tv_blin_x(j, k) = b;
            }
        }
    }
}

// Convenience wrapper using global storage set by tiny_enable_base_tangent_avoidance_2d_multi
inline void tiny_update_base_tangent_avoidance_tv_multi_global(TinySolver* solver) {
    // Check if we should use per-stage predictions
    if (tv_use_per_stage_store()) {
        auto& disks_per_stage = tv_disks_per_stage_store();
        if (!disks_per_stage.empty()) {
            tiny_update_base_tangent_avoidance_tv_multi_per_stage(
                solver, disks_per_stage, tv_disks_margin_store());
            return;
        }
    }
    // Fall back to single-stage disks
    auto& disks = tv_disks_store();
    if (disks.empty()) return;
    tiny_update_base_tangent_avoidance_tv_multi(solver, disks, tv_disks_margin_store());
}

// Convenience: enable base-tangent avoidance from user code.
inline int tiny_enable_base_tangent_avoidance(
    TinySolver* solver, tinytype ox, tinytype oy, tinytype r, tinytype margin)
{
    if (!solver) { std::cout << "tiny_enable_base_tangent_avoidance: solver nullptr\n"; return 1; }
    // Ensure time-varying state linear constraints are allocated (1 per stage)
    tiny_enable_tv_state_linear(solver, 1);
    solver->settings->en_tv_state_linear = 1;
    solver->settings->en_base_tangent_tv = 1;
    solver->settings->obs_x = ox;
    solver->settings->obs_y = oy;
    solver->settings->obs_r = r;
    solver->settings->obs_margin = margin;
    return 0;
}

// Convenience helper for multiple 2D circular obstacles using the existing
// base-tangent TV machinery. Allocates one per-stage row per disk and stores
// the list for per-iteration updates in the ADMM loop.
inline int tiny_enable_base_tangent_avoidance_2d_multi(
    TinySolver* solver,
    const std::vector<std::array<tinytype,3>>& disks,
    tinytype margin)
{
    if (!solver) { std::cout << "tiny_enable_base_tangent_avoidance_2d_multi: solver nullptr\n"; return 1; }
    const int m = static_cast<int>(disks.size());
    if (m <= 0) return 0;

    // Allocate m rows per stage
    tiny_enable_tv_state_linear(solver, m);
    solver->settings->en_tv_state_linear = 1;
    solver->settings->en_base_tangent_tv = 1;

    // Store for global multi-update use
    tv_disks_store() = disks;
    tv_disks_margin_store() = margin;
    tv_use_per_stage_store() = false;  // Disable per-stage mode

    // Also set the first obstacle into settings for backward compatibility
    solver->settings->obs_x = disks[0][0];
    solver->settings->obs_y = disks[0][1];
    solver->settings->obs_r = disks[0][2];
    solver->settings->obs_margin = margin;
    return 0;
}

// Per-stage version: takes predicted disk positions for each horizon stage
// This allows fair comparison with PSD which can use time-varying obstacle predictions
inline int tiny_enable_base_tangent_avoidance_2d_multi_per_stage(
    TinySolver* solver,
    const std::vector<std::vector<std::array<tinytype,3>>>& disks_per_stage,
    tinytype margin)
{
    if (!solver) { std::cout << "tiny_enable_base_tangent_avoidance_2d_multi_per_stage: solver nullptr\n"; return 1; }
    if (disks_per_stage.empty()) return 0;

    // Determine max number of disks across all stages
    int max_disks = 0;
    for (const auto& stage : disks_per_stage) {
        max_disks = std::max(max_disks, static_cast<int>(stage.size()));
    }
    if (max_disks <= 0) return 0;

    // Allocate max_disks rows per stage
    tiny_enable_tv_state_linear(solver, max_disks);
    solver->settings->en_tv_state_linear = 1;
    solver->settings->en_base_tangent_tv = 1;

    // Store for global per-stage update use
    tv_disks_per_stage_store() = disks_per_stage;
    tv_disks_margin_store() = margin;
    tv_use_per_stage_store() = true;  // Enable per-stage mode

    // Also set first obstacle from first stage into settings for backward compatibility
    if (!disks_per_stage[0].empty()) {
        solver->settings->obs_x = disks_per_stage[0][0][0];
        solver->settings->obs_y = disks_per_stage[0][0][1];
        solver->settings->obs_r = disks_per_stage[0][0][2];
    }
    solver->settings->obs_margin = margin;
    return 0;
}

// Enable non-time-varying linear state constraints with n_constr rows
inline int tiny_enable_state_linear(TinySolver* solver, int n_constr) {
    if (!solver) { std::cout << "tiny_enable_state_linear: solver nullptr\n"; return 1; }
    solver->settings->en_state_linear = 1;
    solver->work->numStateLinear = n_constr;
    solver->work->Alin_x = tinyMatrix::Zero(n_constr, solver->work->nx);
    solver->work->blin_x = tinyVector::Zero(n_constr);
    // Initialize associated slack/dual storage
    solver->work->vlnew = solver->work->x;
    solver->work->gl    = tinyMatrix::Zero(solver->work->nx, solver->work->N);
    return 0;
}

// Pure-PSD variant: add lifted disk constraints (no TV relinearization)
// Each disk j with center (ox,oy) and effective radius r contributes a row.
// In the demos, this "effective" radius is typically DEMO_OBS_R + DEMO_OBS_MARGIN
// so that it matches the TV-linear tangent geometry.
// m^T [x; vec(XX)] >= n  where m = [-2ox, -2oy, e11, e22] and n = r^2 - ||o||^2.
// We store a^T z <= b via a = -m, b = -n into Alin_x, blin_x and enable en_state_linear.
inline int tiny_set_lifted_disks(
    TinySolver* solver,
    const std::vector<std::array<tinytype,3>>& disks)
{
    if (!solver) { std::cout << "tiny_set_lifted_disks: solver nullptr\n"; return 1; }
    const int nx0 = solver->settings->nx0_psd;
    const int nxL = solver->work->nx;
    if (nx0 <= 0) { std::cout << "tiny_set_lifted_disks: nx0_psd not set (>0)\n"; return 1; }
    const int m = static_cast<int>(disks.size());
    if (m == 0) return 0;

    tinyMatrix Alin_x = tinyMatrix::Zero(m, nxL);
    tinyVector blin_x = tinyVector::Zero(m);

    const int idx_xx11 = nx0 + 0 + 0*nx0;
    const int idx_xx22 = nx0 + 1 + 1*nx0;

    for (int j = 0; j < m; ++j) {
        const tinytype ox = disks[j][0];
        const tinytype oy = disks[j][1];
        const tinytype r  = disks[j][2];

        tinyVector mrow = tinyVector::Zero(nxL);
        mrow(0) = -2 * ox; mrow(1) = -2 * oy;
        mrow(idx_xx11) = 1; mrow(idx_xx22) = 1;
        tinytype n = (r*r - (ox*ox + oy*oy));

        tinyVector a = -mrow;
        tinytype   b = -n;
        Alin_x.row(j) = a.transpose();
        blin_x(j) = b;
    }

    // Enable and set in solver
    tiny_enable_state_linear(solver, m);
    return tiny_set_linear_constraints(
        solver,
        Alin_x,
        blin_x,
        tinyMatrix::Zero(0, solver->work->nu),
        tinyVector::Zero(0));
}

inline tinyVector build_lifted_disk_row(int nx0,
                                        int nxL,
                                        tinytype ox,
                                        tinytype oy) {
    tinyVector mrow = tinyVector::Zero(nxL);
    mrow(0) = -2 * ox;
    mrow(1) = -2 * oy;
    const int idx_xx11 = nx0 + 0 + 0*nx0;
    const int idx_xx22 = nx0 + 1 + 1*nx0;
    mrow(idx_xx11) = 1;
    mrow(idx_xx22) = 1;
    return mrow;
}

inline int tiny_set_lifted_disks_tv(
    TinySolver* solver,
    const std::vector<std::vector<std::array<tinytype,3>>>& disks_per_stage)
{
    if (!solver) { std::cout << "tiny_set_lifted_disks_tv: solver nullptr\n"; return 1; }
    const int nx0 = solver->settings->nx0_psd;
    const int nxL = solver->work->nx;
    const int N   = solver->work->N;
    if (nx0 <= 0) { std::cout << "tiny_set_lifted_disks_tv: nx0_psd not set (>0)\n"; return 1; }
    if (disks_per_stage.empty()) return 0;

    int per_stage_rows = 0;
    const int stages = std::min<int>(N, disks_per_stage.size());
    for (int k = 0; k < stages; ++k) {
        per_stage_rows = std::max(per_stage_rows,
                                  static_cast<int>(disks_per_stage[k].size()));
    }
    if (per_stage_rows == 0) return 0;

    tiny_enable_tv_state_linear(solver, per_stage_rows);
    solver->settings->en_tv_state_linear = 1;

    const tinytype relaxed_upper = tinytype(1e6);
    for (int k = 0; k < N; ++k) {
        const auto& stage_disks = (k < static_cast<int>(disks_per_stage.size()))
                                ? disks_per_stage[k]
                                : std::vector<std::array<tinytype,3>>{};
        for (int j = 0; j < per_stage_rows; ++j) {
            const int row = k * per_stage_rows + j;
            if (j < static_cast<int>(stage_disks.size())) {
                const auto& d = stage_disks[j];
                tinyVector mrow = tinyVector::Zero(nxL);
                mrow = build_lifted_disk_row(nx0, nxL, d[0], d[1]);
                tinytype n = d[2]*d[2] - (d[0]*d[0] + d[1]*d[1]);
                solver->work->tv_Alin_x.row(row) = (-mrow).transpose();
                solver->work->tv_blin_x(j, k) = -n;
            } else {
                solver->work->tv_Alin_x.row(row).setZero();
                solver->work->tv_blin_x(j, k) = relaxed_upper;
            }
        }
    }
    return 0;
}

// Pure-PSD variant in 3D: add lifted sphere constraints (no TV relinearization)
// For each sphere with center o=(ox,oy,oz) and effective radius r, the constraint is:
//   (x-o)^T(x-o) >= r^2  =>  x^T x - 2 o^T x >= r^2 - o^T o
// In lifted form, using entries for XX_11, XX_22, XX_33 and base x[0:3):
//   m^T [x; vec(XX)] >= n where m has -2*o in base positions and 1's on XX_11,XX_22,XX_33
// We store a^T z <= b via a = -m, b = -n into Alin_x, blin_x.
inline int tiny_set_lifted_spheres(
    TinySolver* solver,
    const std::vector<std::array<tinytype,4>>& spheres)
{
    if (!solver) { std::cout << "tiny_set_lifted_spheres: solver nullptr\n"; return 1; }
    const int nx0 = solver->settings->nx0_psd;
    const int nxL = solver->work->nx;
    if (nx0 < 3) { std::cout << "tiny_set_lifted_spheres: nx0_psd must be >= 3\n"; return 1; }
    const int m = static_cast<int>(spheres.size());
    if (m == 0) return 0;

    tinyMatrix Alin_x = tinyMatrix::Zero(m, nxL);
    tinyVector blin_x = tinyVector::Zero(m);

    const int idx_xx11 = nx0 + 0 + 0*nx0;
    const int idx_xx22 = nx0 + 1 + 1*nx0;
    const int idx_xx33 = nx0 + 2 + 2*nx0;

    for (int j = 0; j < m; ++j) {
        const tinytype ox = spheres[j][0];
        const tinytype oy = spheres[j][1];
        const tinytype oz = spheres[j][2];
        const tinytype r  = spheres[j][3];

        tinyVector mrow = tinyVector::Zero(nxL);
        mrow(0) = -2 * ox; mrow(1) = -2 * oy; mrow(2) = -2 * oz;
        mrow(idx_xx11) = 1; mrow(idx_xx22) = 1; mrow(idx_xx33) = 1;
        tinytype n = (r*r - (ox*ox + oy*oy + oz*oz));

        tinyVector a = -mrow;
        tinytype   b = -n;
        Alin_x.row(j) = a.transpose();
        blin_x(j) = b;
    }

    // Enable and set in solver
    tiny_enable_state_linear(solver, m);
    return tiny_set_linear_constraints(
        solver,
        Alin_x,
        blin_x,
        tinyMatrix::Zero(0, solver->work->nu),
        tinyVector::Zero(0));
}

// Linearized workspace obstacles: ||S x - c||^2 >= r^2 with S (p x nx0)
struct ProjectedSphere {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> S;  // maps base state to workspace
    Eigen::VectorXd center;                                   // effective center in workspace coords
    double radius = 0.0;                                      // safety radius
};

inline int tiny_set_lifted_projected_spheres(
    TinySolver* solver,
    const std::vector<ProjectedSphere>& spheres)
{
    if (!solver) { std::cout << "tiny_set_lifted_projected_spheres: solver nullptr\n"; return 1; }
    const int nx0 = solver->settings->nx0_psd;
    const int nxL = solver->work->nx;
    if (nx0 <= 0) { std::cout << "tiny_set_lifted_projected_spheres: nx0_psd not set (>0)\n"; return 1; }
    const int m = static_cast<int>(spheres.size());
    if (m == 0) return 0;

    tinyMatrix Alin_x = tinyMatrix::Zero(m, nxL);
    tinyVector blin_x = tinyVector::Zero(m);

    for (int j = 0; j < m; ++j) {
        const auto& sph = spheres[j];
        if (sph.S.cols() != nx0) {
            std::cout << "[ProjectedSpheres] Expected S cols=" << nx0
                      << " but got " << sph.S.cols() << "\n";
            return 1;
        }
        if (sph.S.rows() != sph.center.size()) {
            std::cout << "[ProjectedSpheres] Center dimension mismatch (rows="
                      << sph.S.rows() << ", center=" << sph.center.size() << ")\n";
            return 1;
        }

        Eigen::MatrixXd Q = sph.S.transpose() * sph.S;           // nx0 x nx0
        Eigen::VectorXd l = -2.0 * sph.S.transpose() * sph.center; // nx0
        double n = sph.radius * sph.radius - sph.center.squaredNorm();

        tinyVector mrow = tinyVector::Zero(nxL);
        for (int i = 0; i < nx0; ++i) {
            mrow(i) = tinytype(l(i));
        }
        for (int col = 0; col < nx0; ++col) {
            for (int row = 0; row < nx0; ++row) {
                int idx_xx = nx0 + col * nx0 + row;
                mrow(idx_xx) += tinytype(Q(row, col));
            }
        }

        tinyVector a = -mrow;
        tinytype b = tinytype(-n);
        Alin_x.row(j) = a.transpose();
        blin_x(j) = b;
    }

    tiny_enable_state_linear(solver, m);
    return tiny_set_linear_constraints(
        solver,
        Alin_x,
        blin_x,
        tinyMatrix::Zero(0, solver->work->nu),
        tinyVector::Zero(0));
}

inline void tiny_set_circle_avoidance(TinySolver* solver, tinytype ox, tinytype oy, tinytype r) {
    const int nx0 = solver->settings->nx0_psd; // base state dim
    const int nxL = solver->work->nx;          // lifted state dim
    const int N   = solver->work->N;

    tinyVector m = tinyVector::Zero(nxL);
    m(0) = -2 * ox; m(1) = -2 * oy;
    const int idx_xx11 = nx0 + 0 + 0*nx0;
    const int idx_xx22 = nx0 + 1 + 1*nx0;
    m(idx_xx11) = 1; m(idx_xx22) = 1;
    // From (x-obs)'(x-obs) >= r^2  => (x'x - 2 obs'x) >= r^2 - ||obs||^2
    tinytype n = (r*r - (ox*ox + oy*oy));

    tinyVector a = -m; tinytype b = -n; // a^T z <= b
    for (int k = 0; k < N; ++k) {
        solver->work->tv_Alin_x.row(k * 1 + 0) = a.transpose();
        solver->work->tv_blin_x(0, k) = b;
    }
}

// Optional: base corridor x >= xmin as a second TV-linear constraint row
inline void tiny_set_xmin_halfspace(TinySolver* solver, tinytype xmin) {
    const int nxL = solver->work->nx;
    const int N   = solver->work->N;
    const int stride = std::max(1, solver->work->numtvStateLinear);
    if (stride < 2) return; // require allocation with at least 2 rows per stage

    tinyVector a = tinyVector::Zero(nxL);
    a(0) = -1; // -x <= -xmin  =>  x >= xmin
    tinytype b = -xmin;
    for (int k = 0; k < N; ++k) {
        solver->work->tv_Alin_x.row(k * stride + 1) = a.transpose();
        solver->work->tv_blin_x(1, k) = b;
    }
}

// ---------------- Ellipse constraint support ----------------

// Ellipse definition: (x-o)^T E (x-o) >= rho^2, where E is a 2x2 SPD matrix
struct Ellipse {
    Eigen::Matrix2d E;   // Shape matrix (SPD)
    Eigen::Vector2d o;   // Center
    double rho;          // Threshold
};

// Build lifted row for ellipse constraint: m^T * \bar{x}_k >= n
// where \bar{x}_k = [x; vec(XX)]
// Derivation: (x-o)^T E (x-o) = x^T E x - 2 o^T E x + o^T E o >= rho^2
//             = tr(E * XX) - 2(E*o)^T x + (o^T E o - rho^2) >= 0
// In lifted form: [vec(E)^T for XX block, -2(E*o)^T for x block] \bar{x} >= rho^2 - o^T E o
inline void lifted_row_for_ellipse(const Ellipse& el, int NX0, int nxL,
                                   Eigen::VectorXd& m, double& n) {
    m.setZero(nxL);
    
    // Base state terms: -2 * E * o
    Eigen::Vector2d c = -2.0 * el.E * el.o;
    m(0) = c.x();
    m(1) = c.y();
    
    // vec(XX) block: fill only position components (2x2 upper-left of XX)
    // XX is stored column-major: XX_00, XX_10, XX_01, XX_11 at indices NX0+0, NX0+1, NX0+NX0, NX0+NX0+1
    auto put = [&](int i, int j, double v) {
        m(NX0 + j*NX0 + i) += v;
    };
    put(0, 0, el.E(0,0));
    put(0, 1, el.E(0,1));
    put(1, 0, el.E(1,0));
    put(1, 1, el.E(1,1));
    
    n = el.rho * el.rho - el.o.transpose() * el.E * el.o;
}

// Set lifted ellipse constraints (analogous to tiny_set_lifted_disks)
// Each ellipse contributes one linear inequality in lifted space
inline int tiny_set_lifted_ellipses(
    TinySolver* solver,
    const std::vector<Ellipse>& ellipses)
{
    if (!solver) { std::cout << "tiny_set_lifted_ellipses: solver nullptr\n"; return 1; }
    const int nx0 = solver->settings->nx0_psd;
    const int nxL = solver->work->nx;
    if (nx0 <= 0) { std::cout << "tiny_set_lifted_ellipses: nx0_psd not set (>0)\n"; return 1; }
    const int m = static_cast<int>(ellipses.size());
    if (m == 0) return 0;

    tinyMatrix Alin_x = tinyMatrix::Zero(m, nxL);
    tinyVector blin_x = tinyVector::Zero(m);

    for (int j = 0; j < m; ++j) {
        Eigen::VectorXd mrow;
        double n;
        lifted_row_for_ellipse(ellipses[j], nx0, nxL, mrow, n);
        
        // Convert to a^T z <= b form: a = -m, b = -n
        Alin_x.row(j) = (-mrow).transpose().cast<tinytype>();
        blin_x(j) = tinytype(-n);
    }

    // Enable and set in solver
    tiny_enable_state_linear(solver, m);
    return tiny_set_linear_constraints(
        solver,
        Alin_x,
        blin_x,
        tinyMatrix::Zero(0, solver->work->nu),
        tinyVector::Zero(0));
}

// --------- PSD core hooks (minimal placeholders) ----------
inline void tiny_update_slack_psd(TinySolver* solver) {
    if (!solver) return;
    if (solver->work->Spsd.rows() == 0) return;
    solver->work->Spsd_new = solver->work->Spsd;
    solver->work->primal_residual_psd = 0;
}

inline void tiny_update_dual_psd(TinySolver* solver) {
    if (!solver) return;
    if (solver->work->Spsd.rows() == 0) return;
    solver->work->Hpsd = solver->work->Hpsd + (solver->work->Spsd - solver->work->Spsd_new);
    solver->work->dual_residual_psd = 0;
}

inline void tiny_add_psd_to_linear_cost(TinySolver* solver) {
    (void)solver;
}
