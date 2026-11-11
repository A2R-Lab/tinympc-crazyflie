#pragma once

// TinySDP-CT embedded: fixed-memory continuous-time SDP trajectory planner.
//
// The embedded counterpart of the desktop engine (include/continuous/engine),
// specialized to the reduced Bernstein/SOS formulation the way TinyMPC is
// specialized to MPC. No heap, no exceptions, no Eigen; every buffer is sized
// by template capacities, and the scalar is a template parameter (float on a
// microcontroller, double for host verification).
//
// Problem (normalized coordinates, one plan):
//   D-dimensional C1 spline of NSeg cubic pieces with equal durations, fixed
//   start/goal position and velocity. Facial reduction leaves the moment cone
//   [I Y; Y^T W] >= 0 over the free control points; each (segment, obstacle)
//   pair has an SOS certificate (Gram matrices Q0 in S^4, Q1 in S^3) that
//   ||gamma(s) - c(s)||^2 - rho^2 >= 0 on [0,1] for spheres moving with
//   constant velocity. Optional Bernstein derivative bounds and route
//   halfspaces are scalar cones on Y.
//
// Structure used by the solver (see the desktop engine for the general case):
//   * Gram variables are packed-symmetric coordinates, so each certificate's
//     Hessian block is the identity and every Gram variable maps to exactly
//     one coefficient row: the per-pair Schur complement is diagonal and the
//     same for every pair. The z-update is one kNc x kNc Cholesky solve plus
//     O(pairs) work; per frame only the coupling rows and right-hand sides
//     change, so a refactor costs O(pairs * kNc^2).
//   * One unit-rho factorization serves every rho (linear objective).
//   * ADMM runs on the projection input p with safeguarded Anderson
//     acceleration; PSD projections try a Cholesky certificate first, then
//     warm-started Jacobi.
//
// Online acceptance never relies on solver accuracy: a plan is used only if
// the independent continuous-time checker (checker.hpp) certifies clearance
// and its derivative control points satisfy the limits.

#include <tinysdp_ct_embedded/checker.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace tinysdp_ct_embedded {

template<class S, int D>
struct Vec {
    std::array<S, D> v{};
    S& operator[](int i) { return v[i]; }
    const S& operator[](int i) const { return v[i]; }
};

template<class S, int D>
struct MovingSphere {
    Vec<S, D> center, velocity;  // center at plan time 0, constant velocity
    S radius = 0;
};

template<class S, int D>
struct RouteCut {
    int segment = 0, obstacle = 0;
    S s = 0, margin = 0;
    Vec<S, D> direction;  // unit
};

struct SolveInfo {
    bool converged = false, stopped = false, valid = true;
    int iterations = 0, anderson_rejected = 0;
};

// NSeg cubic pieces in D dimensions; up to MaxObs spheres and MaxRoutes
// route halfspaces; Memory Anderson columns; Bounds enables derivative limits.
template<class S, int D, int NSeg, int MaxObs, int MaxRoutes = 1, int Memory = 5, bool Bounds = true>
class Planner {
public:
    static_assert(D == 2 || D == 3, "planar or spatial");
    static_assert(NSeg >= 2 && MaxObs >= 1 && MaxRoutes >= 0 && Memory >= 0 && Memory <= 8);
    static constexpr int kDeg = 3;
    static constexpr int kP = kDeg * NSeg + 1;              // control points
    static constexpr int kRows = 2 + (NSeg - 1) + 2;        // pos, C1 junctions, velocities
    static constexpr int kL = kP - kRows;                   // free control points
    static constexpr int kNy = D * kL, kNw = kL * (kL + 1) / 2, kNc = kNy + kNw;
    static constexpr int kMomentDim = D + kL, kMomentSize = kMomentDim * (kMomentDim + 1) / 2;
    static constexpr int kNq = 16, kMe = 7;                 // Gram variables and rows per pair
    static constexpr int kMaxPairs = NSeg * MaxObs;
    static constexpr int kMaxN = kNc + kMaxPairs * kNq;
    static constexpr int kBoundRows = Bounds ? NSeg * (3 + 2) * D * 2 : 0;
    static constexpr int kMaxNs = kMomentSize + kMaxPairs * kNq + kBoundRows + MaxRoutes;
    static constexpr int kMaxDim = kMomentDim > 4 ? kMomentDim : 4;
    static_assert(kL >= 1, "need at least one free control point");

    // Plain aggregate without default member initializers, so a planner
    // object is zero-initialized (.bss, no flash image); start from defaults().
    struct Settings {
        S rho, alpha;
        S absolute_tolerance, relative_tolerance;
        int check_interval;
        S anderson_regularization;
        S bound_weight;                  // weight of derivative-bound cones
        S normalization_floor;           // minimum length scale (physical units)
        S safety_margin;                 // required clearance (physical units)
        S solve_margin;                  // margin used inside the SDP
        S max_velocity, max_acceleration;
        // The SDP plans against tightened derivative limits so that a plan
        // meeting ADMM's tolerance still passes the exact check at the true ones
        // (the analogue of solve_margin for obstacles).
        S limit_tightening;

        static Settings defaults() {
            Settings s{};
            s.rho = S(0.5); s.alpha = S(1.8);
            s.absolute_tolerance = S(1e-6); s.relative_tolerance = S(1e-5);
            s.check_interval = 10;
            s.anderson_regularization = S(1e-8);
            s.bound_weight = S(0.3);
            s.normalization_floor = S(0.5);
            s.safety_margin = S(0.05); s.solve_margin = S(0.055);
            s.max_velocity = S(2); s.max_acceleration = S(8);
            s.limit_tightening = S(0.97);
            return s;
        }
    };

    // Physical-unit frame data.
    struct Frame {
        Vec<S, D> start, start_velocity, goal, goal_velocity;
        S horizon;                         // total duration (> 0)
        std::array<MovingSphere<S, D>, MaxObs> obstacles{};
        int obstacle_count = 0;
        std::array<RouteCut<S, D>, MaxRoutes> routes{};
        int route_count = 0;
    };

    // Physical control points of the plan (D x kP, column major).
    struct Plan {
        std::array<S, D * kP> controls{};
        S segment_duration = 0;
        Vec<S, D> point(int k) const { Vec<S, D> p; for (int i = 0; i < D; ++i) p[i] = controls[i + D * k]; return p; }
    };

    bool init(const Settings& settings = Settings::defaults()) {
        settings_ = settings;
        loaded_ = false;
        length_ = tau_ = S(1);
        n_ = kNc; ns_ = kMomentSize;
        primal_relative_ = dual_relative_ = std::numeric_limits<S>::infinity();
        return build_topology();
    }

    // Assemble and factor a frame. Keeps the iterate as a warm start when the
    // obstacle count is unchanged (receding horizon); otherwise cold starts.
    bool load(const Frame& frame) {
        if (frame.obstacle_count < 1 || frame.obstacle_count > MaxObs || frame.route_count < 0
            || frame.route_count > MaxRoutes || !(frame.horizon > S(0))) return false;
        const bool same_shape = loaded_ && frame.obstacle_count == obstacles_;
        frame_ = frame;
        obstacles_ = frame.obstacle_count;
        pairs_ = NSeg * obstacles_;
        n_ = kNc + pairs_ * kNq;
        routes_ = frame.route_count;
        ns_ = kMomentSize + pairs_ * kNq + kBoundRows + routes_;
        normalize();
        assemble();
        if (!factor()) return false;
        if (!same_shape) cold_start();
        else for (int r = 0; r < MaxRoutes; ++r) p_[route_slot(r)] = S(0);
        if (routes_) {  // new route rows start at their current affine value
            affine(z_.data(), a_.data());
            for (int r = 0; r < routes_; ++r) { const S v = a_[route_slot(r)]; p_[route_slot(r)] = v > S(0) ? v : S(0); }
        }
        loaded_ = true;
        return true;
    }

    // Replace only the route halfspaces of the loaded frame (refactors the
    // small common block; group data and the iterate are kept).
    bool set_routes(const RouteCut<S, D>* routes, int count) {
        if (!loaded_ || count < 0 || count > MaxRoutes) return false;
        for (int r = 0; r < count; ++r) frame_.routes[r] = routes[r];
        frame_.route_count = routes_ = count;
        ns_ = kMomentSize + pairs_ * kNq + kBoundRows + routes_;
        assemble_routes();
        if (!factor_common()) return false;
        for (int r = 0; r < MaxRoutes; ++r) p_[route_slot(r)] = S(0);
        affine(z_.data(), a_.data());
        for (int r = 0; r < routes_; ++r) { const S v = a_[route_slot(r)]; p_[route_slot(r)] = v > S(0) ? v : S(0); }
        return true;
    }

    template<class Stop>
    SolveInfo solve(int limit, Stop&& stop) {
        SolveInfo info;
        int count = 0, head = 0;
        bool previous = false, pending = false;
        S previous_residual = 0;
        for (int it = 1; it <= limit; ++it) {
            const bool check = it == 1 || it % settings_.check_interval == 0 || it == limit;
            bool converged = false;
            if (!evaluate(check, converged)) { info.valid = false; break; }
            info.iterations = it;
            S residual = 0;
            for (int i = 0; i < ns_; ++i) { g_[i] = tp_[i] - p_[i]; residual += g_[i] * g_[i]; }
            residual = std::sqrt(residual);
            if (Memory > 0 && pending) {
                if (!(residual <= previous_residual)) {
                    ++info.anderson_rejected;
                    for (int i = 0; i < ns_; ++i) p_[i] = fallback_[i];
                    count = head = 0; previous = pending = false;
                    continue;
                }
            }
            if (check) {
                if (converged) { info.converged = true; break; }
                if (stop(static_cast<const Planner&>(*this))) { info.stopped = true; break; }
            }
            if (Memory == 0) { for (int i = 0; i < ns_; ++i) p_[i] = tp_[i]; continue; }
            if (previous) push(count, head);
            for (int i = 0; i < ns_; ++i) { p_previous_[i] = p_[i]; g_previous_[i] = g_[i]; }
            previous_residual = residual;
            previous = true;
            if (count > 0 && extrapolate(count)) {
                for (int i = 0; i < ns_; ++i) fallback_[i] = tp_[i];
                pending = true;
            } else {
                for (int i = 0; i < ns_; ++i) p_[i] = tp_[i];
                pending = false;
            }
        }
        for (int i = 0; i < ns_; ++i) p_[i] = tp_[i];
        return info;
    }
    SolveInfo solve(int limit) { return solve(limit, [](const Planner&) { return false; }); }

    // Physical plan from the current iterate: gamma = origin + L (C + Y N^T).
    void plan(Plan& out) const {
        for (int k = 0; k < kP; ++k)
            for (int c = 0; c < D; ++c) {
                S value = constant_points_[c + D * k];
                for (int l = 0; l < kL; ++l) value += z_[y_index(c, l)] * nullspace_[k + kP * l];
                out.controls[c + D * k] = origin_[c] + length_ * value;
            }
        out.segment_duration = frame_.horizon / NSeg;
    }

    // Derivative control points of a plan within the limits (exact, conservative).
    bool kinematics_ok(const Plan& plan, S tolerance = S(1e-6)) const {
        const S tau = plan.segment_duration;
        for (int seg = 0; seg < NSeg; ++seg)
            for (int c = 0; c < D; ++c) {
                const int base = kDeg * seg;
                S x[4];
                for (int i = 0; i < 4; ++i) x[i] = plan.controls[c + D * (base + i)];
                for (int k = 0; k < 3; ++k)
                    if (std::abs(S(3) / tau * (x[k + 1] - x[k])) > settings_.max_velocity + tolerance) return false;
                for (int k = 0; k < 2; ++k)
                    if (std::abs(S(6) / (tau * tau) * (x[k + 2] - 2 * x[k + 1] + x[k])) > settings_.max_acceleration + tolerance) return false;
            }
        return true;
    }

    // Iterate snapshot for route branching (only the iterate, not the data).
    struct Snapshot { std::array<S, kMaxNs> p{}; std::array<S, kMaxN> z{}; };
    void save(Snapshot& out) const { out.p = p_; out.z = z_; }
    void restore(const Snapshot& in) { p_ = in.p; z_ = in.z; }

    // Persistent footprint of this object in bytes (everything is inline).
    static constexpr std::size_t footprint() { return sizeof(Planner); }
    // Relative residuals at the latest check (for early-acceptance quality gates).
    S primal_relative() const { return primal_relative_; }
    S dual_relative() const { return dual_relative_; }
    int variables() const { return n_; }
    int cone_space() const { return ns_; }
    S rho() const { return settings_.rho; }

private:
    // ---------------- topology (init) ----------------
    static S binom(int n, int k) {
        S r = 1;
        for (int i = 1; i <= k; ++i) r = r * S(n - k + i) / S(i);
        return r;
    }
    // Bernstein product weight: b_i b_j = w_ij b_{i+j} (degree d -> 2d).
    static S product_weight(int d, int i, int j) { return binom(d, i) * binom(d, j) / binom(2 * d, i + j); }

    bool build_topology() {
        // Boundary matrix (rows x kP), all computed in double at init.
        double B[kRows][kP] = {};
        int row = 0;
        B[row++][0] = 1;
        B[row++][kP - 1] = 1;
        for (int j = 1; j < NSeg; ++j) { B[row][3 * j - 1] = 1; B[row][3 * j] = -2; B[row][3 * j + 1] = 1; ++row; }
        B[row][0] = -1; B[row][1] = 1; ++row;                     // start velocity * tau / 3
        B[row][kP - 2] = -1; B[row][kP - 1] = 1; ++row;           // goal velocity * tau / 3
        // Orthonormal row space by Gram-Schmidt, then its complement.
        double Q[kP][kP] = {};
        int rank = 0;
        const auto orthonormal_add = [&](double* v) {
            for (int pass = 0; pass < 2; ++pass)
                for (int q = 0; q < rank; ++q) {
                    double dot = 0;
                    for (int i = 0; i < kP; ++i) dot += v[i] * Q[q][i];
                    for (int i = 0; i < kP; ++i) v[i] -= dot * Q[q][i];
                }
            double norm = 0;
            for (int i = 0; i < kP; ++i) norm += v[i] * v[i];
            norm = std::sqrt(norm);
            if (norm < 1e-9) return false;
            for (int i = 0; i < kP; ++i) Q[rank][i] = v[i] / norm;
            ++rank;
            return true;
        };
        for (int r = 0; r < kRows; ++r) { double v[kP]; for (int i = 0; i < kP; ++i) v[i] = B[r][i]; if (!orthonormal_add(v)) return false; }
        double N[kP][kL] = {};
        for (int e = 0, l = 0; e < kP && l < kL; ++e) {
            double v[kP] = {};
            v[e] = 1;
            if (orthonormal_add(v)) { for (int i = 0; i < kP; ++i) N[i][l] = Q[rank - 1][i]; ++l; }
        }
        if (rank != kP) return false;
        // Right inverse R = B^T (B B^T)^-1 (kP x kRows).
        double G[kRows * kRows] = {};
        for (int a = 0; a < kRows; ++a)
            for (int b = 0; b < kRows; ++b) for (int i = 0; i < kP; ++i) G[a + kRows * b] += B[a][i] * B[b][i];
        if (!chol(G, kRows)) return false;
        double R[kP][kRows] = {};
        for (int b = 0; b < kRows; ++b) {
            double e[kRows] = {};
            e[b] = 1;
            chol_solve(G, kRows, e);
            for (int i = 0; i < kP; ++i) { double v = 0; for (int a = 0; a < kRows; ++a) v += B[a][i] * e[a]; R[i][b] = v; }
        }
        // Unit-duration cost Gram (sum of per-segment blocks), acceleration cost.
        double gram[4][4] = {};
        segment_cost_gram(gram);
        double C0[kP][kP] = {};
        for (int seg = 0; seg < NSeg; ++seg)
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) C0[3 * seg + i][3 * seg + j] += gram[i][j];
        // Minimum-energy centering: R -= N (N^T C0 N)^-1 N^T C0 R.
        double NCN[kL * kL] = {}, CN[kP][kL] = {};
        for (int i = 0; i < kP; ++i) for (int l = 0; l < kL; ++l) for (int k = 0; k < kP; ++k) CN[i][l] += C0[i][k] * N[k][l];
        for (int a = 0; a < kL; ++a) for (int b = 0; b < kL; ++b) for (int i = 0; i < kP; ++i) NCN[a + kL * b] += N[i][a] * CN[i][b];
        if (!chol(NCN, kL)) return false;
        for (int b = 0; b < kRows; ++b) {
            double t[kL] = {};
            for (int l = 0; l < kL; ++l) for (int i = 0; i < kP; ++i) t[l] += CN[i][l] * R[i][b];
            chol_solve(NCN, kL, t);
            for (int i = 0; i < kP; ++i) for (int l = 0; l < kL; ++l) R[i][b] -= N[i][l] * t[l];
        }
        for (int i = 0; i < kP; ++i) {
            for (int l = 0; l < kL; ++l) nullspace_[i + kP * l] = S(N[i][l]);
            for (int b = 0; b < kRows; ++b) right_inverse_[i + kP * b] = S(R[i][b]);
        }
        for (int a = 0; a < kL; ++a) for (int b = 0; b < kL; ++b) reduced_cost0_[a + kL * b] = S(NCN_unfactored(N, C0, a, b));
        for (int i = 0; i < kP; ++i) for (int l = 0; l < kL; ++l) cost_n0_[i + kP * l] = S(CN[i][l]);
        for (int i = 0; i < kP; ++i) for (int k = 0; k < kP; ++k) cost0_[i + kP * k] = S(C0[i][k]);
        // Constant coefficient maps.
        build_gram_rows();
        build_w_rows();
        // Common Hessian from the moment cone: y enters twice (sqrt 2 in svec), w once.
        for (int i = 0; i < kNc * kNc; ++i) base_hcc_[i] = S(0);
        for (int i = 0; i < kNy; ++i) base_hcc_[i + kNc * i] = S(2);
        for (int i = kNy; i < kNc; ++i) base_hcc_[i + kNc * i] = S(1);
        return true;
    }

    static double NCN_unfactored(const double (&N)[kP][kL], const double (&C0)[kP][kP], int a, int b) {
        double v = 0;
        for (int i = 0; i < kP; ++i) for (int k = 0; k < kP; ++k) v += N[i][a] * C0[i][k] * N[k][b];
        return v;
    }

    // Integrated squared acceleration of a unit-duration cubic.
    static void segment_cost_gram(double (&out)[4][4]) {
        // Second-derivative map (4 x 2) times 6, degree-1 Bernstein Gram.
        const double M[4][2] = {{1, 0}, {-2, 1}, {1, -2}, {0, 1}};
        const double g1[2][2] = {{1.0 / 3, 1.0 / 6}, {1.0 / 6, 1.0 / 3}};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double v = 0;
                for (int a = 0; a < 2; ++a) for (int b = 0; b < 2; ++b) v += M[i][a] * g1[a][b] * M[j][b];
                out[i][j] = 36 * v;
            }
    }

    // Each packed Gram variable maps to one coefficient row: record the row
    // and coefficient (negated, as in the desktop builder), and the diagonal
    // per-row Schur complement E = Agq Agq^T.
    void build_gram_rows() {
        const S root2 = std::sqrt(S(2));
        int k = 0;
        for (int col = 0; col < 4; ++col)
            for (int r = col; r < 4; ++r, ++k) {
                gram_row_[k] = r + col;
                gram_coef_[k] = -(r == col ? S(1) : root2) * product_weight(3, r, col);
            }
        for (int col = 0; col < 3; ++col)
            for (int r = col; r < 3; ++r, ++k) {
                const int lower = r + col, row = lower + 1;
                gram_row_[k] = row;
                gram_coef_[k] = -(r == col ? S(1) : root2) * product_weight(2, r, col)
                    * S(row * (6 - row)) / S(30);
            }
        for (int r = 0; r < kMe; ++r) schur_inverse_[r] = S(0);
        for (int q = 0; q < kNq; ++q) schur_inverse_[gram_row_[q]] += gram_coef_[q] * gram_coef_[q];
        for (int r = 0; r < kMe; ++r) schur_inverse_[r] = S(1) / schur_inverse_[r];
    }

    // Constant W columns of each segment's coupling rows: bpc(n E_k n^T).
    void build_w_rows() {
        const S root2 = std::sqrt(S(2));
        for (int seg = 0; seg < NSeg; ++seg)
            for (int col = 0, k = 0; col < kL; ++col)
                for (int row = col; row < kL; ++row, ++k) {
                    S coef[kMe] = {};
                    for (int i = 0; i < 4; ++i)
                        for (int j = 0; j < 4; ++j) {
                            const S ni_r = nullspace_[(3 * seg + i) + kP * row], nj_c = nullspace_[(3 * seg + j) + kP * col];
                            const S ni_c = nullspace_[(3 * seg + i) + kP * col], nj_r = nullspace_[(3 * seg + j) + kP * row];
                            const S m = row == col ? ni_r * nj_c : (ni_r * nj_c + ni_c * nj_r) / root2;
                            coef[i + j] += product_weight(3, i, j) * m;
                        }
                    for (int r = 0; r < kMe; ++r) w_rows_[(seg * kMe + r) + NSeg * kMe * k] = coef[r];
                }
    }

    // ---------------- per frame ----------------
    static int y_index(int c, int l) { return l * D + c; }
    static int packed(int d, int row, int col) { return col * d - col * (col - 1) / 2 + (row - col); }
    int pair_offset(int pair) const { return kNc + pair * kNq; }
    int pair_slot(int pair) const { return kMomentSize + pair * kNq; }
    int bound_slot(int b) const { return kMomentSize + pairs_ * kNq + b; }
    int route_slot(int r) const { return kMomentSize + pairs_ * kNq + kBoundRows + r; }

    void normalize() {
        S dist2 = 0;
        for (int c = 0; c < D; ++c) {
            origin_[c] = S(0.5) * (frame_.start[c] + frame_.goal[c]);
            const S d = frame_.goal[c] - frame_.start[c];
            dist2 += d * d;
        }
        length_ = S(0.5) * std::sqrt(dist2);
        if (length_ < settings_.normalization_floor) length_ = settings_.normalization_floor;
        tau_ = frame_.horizon / NSeg;
    }

    // Sphere center controls (normalized) over a segment.
    void center_controls(int obstacle, int seg, S (&out)[D][4]) const {
        const auto& o = frame_.obstacles[obstacle];
        for (int i = 0; i < 4; ++i) {
            const S t = tau_ * (S(seg) + S(i) / S(3));
            for (int c = 0; c < D; ++c) out[c][i] = (o.center[c] + t * o.velocity[c] - origin_[c]) / length_;
        }
    }

    void assemble() {
        // Constant control points C = V R^T in normalized units.
        S V[D][kRows] = {};
        for (int c = 0; c < D; ++c) {
            V[c][0] = (frame_.start[c] - origin_[c]) / length_;
            V[c][1] = (frame_.goal[c] - origin_[c]) / length_;
            V[c][kRows - 2] = frame_.start_velocity[c] / length_ * tau_ / S(3);
            V[c][kRows - 1] = frame_.goal_velocity[c] / length_ * tau_ / S(3);
        }
        for (int c = 0; c < D; ++c)
            for (int k = 0; k < kP; ++k) {
                S v = 0;
                for (int b = 0; b < kRows; ++b) v += V[c][b] * right_inverse_[k + kP * b];
                constant_points_[c + D * k] = v;
            }
        // Objective: tau^{1-2k} (k = 2) times the unit-duration cost.
        const S weight = S(1) / (tau_ * tau_ * tau_);
        const S root2 = std::sqrt(S(2));
        for (int c = 0; c < D; ++c)
            for (int l = 0; l < kL; ++l) {
                S v = 0;
                for (int k = 0; k < kP; ++k) v += constant_points_[c + D * k] * cost_n0_[k + kP * l];
                objective_[y_index(c, l)] = S(2) * weight * v;
            }
        for (int col = 0, k = 0; col < kL; ++col)
            for (int row = col; row < kL; ++row, ++k)
                objective_[kNy + k] = weight * (row == col ? S(1) : root2) * reduced_cost0_[row + kL * col];
        for (int i = kNc; i < n_; ++i) objective_[i] = S(0);
        // Coupling rows and right-hand sides per pair.
        const S rho_sphere_extra = settings_.solve_margin / length_;
        for (int seg = 0; seg < NSeg; ++seg)
            for (int o = 0; o < obstacles_; ++o) {
                const int pair = seg * obstacles_ + o;
                S centers[D][4];
                center_controls(o, seg, centers);
                S h[D][4];
                for (int c = 0; c < D; ++c) for (int i = 0; i < 4; ++i) h[c][i] = constant_points_[c + D * (3 * seg + i)] - centers[c][i];
                const S rho = frame_.obstacles[o].radius / length_ + rho_sphere_extra;
                S* agc = agc_.data() + pair * kMe * kNc;  // kMe x kNc, column major
                for (int r = 0; r < kMe; ++r) b_[pair * kMe + r] = rho * rho;
                for (int c = 0; c < D; ++c)
                    for (int i = 0; i < 4; ++i)
                        for (int j = 0; j < 4; ++j) b_[pair * kMe + i + j] -= product_weight(3, i, j) * h[c][i] * h[c][j];
                for (int c = 0; c < D; ++c)
                    for (int l = 0; l < kL; ++l) {
                        S* column = agc + kMe * y_index(c, l);
                        for (int r = 0; r < kMe; ++r) column[r] = S(0);
                        for (int i = 0; i < 4; ++i)
                            for (int j = 0; j < 4; ++j)
                                column[i + j] += product_weight(3, i, j)
                                    * (h[c][i] * nullspace_[(3 * seg + j) + kP * l] + nullspace_[(3 * seg + i) + kP * l] * h[c][j]);
                    }
                for (int k = 0; k < kNw; ++k)
                    for (int r = 0; r < kMe; ++r) agc[r + kMe * (kNy + k)] = w_rows_[(seg * kMe + r) + NSeg * kMe * k];
            }
        assemble_bounds();
        assemble_routes();
    }

    // Scalar cone rows on y: coefficients are D*kL dense (y ordering).
    void assemble_bounds() {
        if (!Bounds) return;
        int b = 0;
        const S w = settings_.bound_weight;
        for (int seg = 0; seg < NSeg; ++seg)
            for (int order = 1; order <= 2; ++order) {
                const int controls = 4 - order;
                const S scale = order == 1 ? S(3) / tau_ : S(6) / (tau_ * tau_);
                const S limit = settings_.limit_tightening
                    * (order == 1 ? settings_.max_velocity : settings_.max_acceleration) / length_;
                for (int k = 0; k < controls; ++k) {
                    // map column k: order 1: (-1, +1) at k, k+1; order 2: (1, -2, 1) at k..k+2.
                    S map[4] = {0, 0, 0, 0};
                    if (order == 1) { map[k] = -1; map[k + 1] = 1; }
                    else { map[k] = 1; map[k + 1] = -2; map[k + 2] = 1; }
                    for (int c = 0; c < D; ++c) {
                        S fixed = 0;
                        for (int i = 0; i < 4; ++i) fixed += constant_points_[c + D * (3 * seg + i)] * map[i];
                        fixed *= scale;
                        S free[kL];
                        for (int l = 0; l < kL; ++l) {
                            S v = 0;
                            for (int i = 0; i < 4; ++i) v += nullspace_[(3 * seg + i) + kP * l] * map[i];
                            free[l] = scale * v;
                        }
                        for (int sign : {-1, 1}) {
                            S* coef = bound_coef_.data() + b * kNy;
                            for (int i = 0; i < kNy; ++i) coef[i] = S(0);
                            for (int l = 0; l < kL; ++l) coef[y_index(c, l)] = w * S(sign) * free[l];
                            bound_constant_[b] = w * (limit + S(sign) * fixed);
                            bound_inverse_scale_[b] = S(1) / w;
                            ++b;
                        }
                    }
                }
            }
    }

    void assemble_routes() {
        for (int r = 0; r < routes_; ++r) {
            const auto& cut = frame_.routes[r];
            S basis[4];
            const S s = cut.s, q = S(1) - s;
            basis[0] = q * q * q; basis[1] = S(3) * q * q * s; basis[2] = S(3) * q * s * s; basis[3] = s * s * s;
            S centers[D][4];
            center_controls(cut.obstacle, cut.segment, centers);
            S constant = -(frame_.obstacles[cut.obstacle].radius + cut.margin) / length_;
            S* coef = route_coef_.data() + r * kNy;
            S largest = 0;
            for (int i = 0; i < kNy; ++i) coef[i] = S(0);
            for (int c = 0; c < D; ++c) {
                S fixed = 0, center = 0;
                for (int i = 0; i < 4; ++i) {
                    fixed += constant_points_[c + D * (3 * cut.segment + i)] * basis[i];
                    center += centers[c][i] * basis[i];
                }
                constant += cut.direction[c] * (fixed - center);
                for (int l = 0; l < kL; ++l) {
                    S v = 0;
                    for (int i = 0; i < 4; ++i) v += nullspace_[(3 * cut.segment + i) + kP * l] * basis[i];
                    coef[y_index(c, l)] = cut.direction[c] * v;
                    largest = std::max(largest, std::abs(coef[y_index(c, l)]));
                }
            }
            const S scale = largest > S(0) ? S(1) / largest : S(1);
            for (int i = 0; i < kNy; ++i) coef[i] *= scale;
            route_constant_[r] = scale * constant;
            route_inverse_scale_[r] = S(1) / scale;
        }
    }

    // G = E^-1 Agc per pair, M = sum Agc^T G, K = Hcc + rows + M.
    bool factor() {
        for (int i = 0; i < kNc * kNc; ++i) mcc_[i] = S(0);
        for (int pair = 0; pair < pairs_; ++pair) {
            const S* agc = agc_.data() + pair * kMe * kNc;
            S* g = gall_.data() + pair * kMe * kNc;
            for (int c = 0; c < kNc; ++c)
                for (int r = 0; r < kMe; ++r) g[r + kMe * c] = schur_inverse_[r] * agc[r + kMe * c];
            for (int j = 0; j < kNc; ++j)
                for (int i = 0; i <= j; ++i) {
                    S v = 0;
                    for (int r = 0; r < kMe; ++r) v += agc[r + kMe * i] * g[r + kMe * j];
                    mcc_[i + kNc * j] += v;
                }
        }
        for (int j = 0; j < kNc; ++j) for (int i = 0; i < j; ++i) mcc_[j + kNc * i] = mcc_[i + kNc * j];
        if (Bounds) {
            for (int i = 0; i < kNc * kNc; ++i) bound_hcc_[i] = S(0);
            for (int b = 0; b < kBoundRows; ++b) {
                const S* coef = bound_coef_.data() + b * kNy;
                for (int j = 0; j < kNy; ++j) if (coef[j] != S(0)) for (int i = 0; i < kNy; ++i) bound_hcc_[i + kNc * j] += coef[i] * coef[j];
            }
        }
        return factor_common();
    }

    bool factor_common() {
        for (int i = 0; i < kNc * kNc; ++i) kfactor_[i] = base_hcc_[i] + mcc_[i] + (Bounds ? bound_hcc_[i] : S(0));
        for (int r = 0; r < routes_; ++r) {
            const S* coef = route_coef_.data() + r * kNy;
            for (int j = 0; j < kNy; ++j) for (int i = 0; i < kNy; ++i) kfactor_[i + kNc * j] += coef[i] * coef[j];
        }
        return chol(kfactor_.data(), kNc);
    }

    void cold_start() {
        for (int i = 0; i < n_; ++i) z_[i] = S(0);
        for (auto& q : q_) q = S(0);
        for (int k = 0; k < 1 + 2 * pairs_; ++k) { const int d = k == 0 ? kMomentDim : (k % 2 ? 4 : 3); S* q = jacobi_basis(k); for (int i = 0; i < d; ++i) q[i + d * i] = S(1); }
        affine(z_.data(), a_.data());
        project_all(a_.data(), p_.data());
    }

    // ---------------- ADMM ----------------
    void affine(const S* z, S* a) const {
        const S root2 = std::sqrt(S(2));
        for (int i = 0; i < kMomentSize; ++i) a[i] = S(0);
        for (int c = 0; c < D; ++c) a[packed(kMomentDim, c, c)] = S(1);
        for (int c = 0; c < D; ++c) for (int l = 0; l < kL; ++l) a[packed(kMomentDim, D + l, c)] = root2 * z[y_index(c, l)];
        for (int col = 0, k = 0; col < kL; ++col) for (int row = col; row < kL; ++row, ++k) a[packed(kMomentDim, D + row, D + col)] = z[kNy + k];
        for (int pair = 0; pair < pairs_; ++pair) for (int q = 0; q < kNq; ++q) a[pair_slot(pair) + q] = z[pair_offset(pair) + q];
        for (int b = 0; b < kBoundRows; ++b) {
            const S* coef = bound_coef_.data() + b * kNy;
            S v = bound_constant_[b];
            for (int i = 0; i < kNy; ++i) v += coef[i] * z[i];
            a[bound_slot(b)] = v;
        }
        for (int r = 0; r < routes_; ++r) {
            const S* coef = route_coef_.data() + r * kNy;
            S v = route_constant_[r];
            for (int i = 0; i < kNy; ++i) v += coef[i] * z[i];
            a[route_slot(r)] = v;
        }
    }

    void adjoint(const S* y, S* out) const {
        const S root2 = std::sqrt(S(2));
        for (int i = 0; i < n_; ++i) out[i] = S(0);
        for (int c = 0; c < D; ++c) for (int l = 0; l < kL; ++l) out[y_index(c, l)] = root2 * y[packed(kMomentDim, D + l, c)];
        for (int col = 0, k = 0; col < kL; ++col) for (int row = col; row < kL; ++row, ++k) out[kNy + k] = y[packed(kMomentDim, D + row, D + col)];
        for (int pair = 0; pair < pairs_; ++pair) for (int q = 0; q < kNq; ++q) out[pair_offset(pair) + q] = y[pair_slot(pair) + q];
        for (int b = 0; b < kBoundRows; ++b) {
            const S* coef = bound_coef_.data() + b * kNy;
            const S v = y[bound_slot(b)];
            for (int i = 0; i < kNy; ++i) out[i] += coef[i] * v;
        }
        for (int r = 0; r < routes_; ++r) {
            const S* coef = route_coef_.data() + r * kNy;
            const S v = y[route_slot(r)];
            for (int i = 0; i < kNy; ++i) out[i] += coef[i] * v;
        }
    }

    // Unit-rho KKT solve, rhs_ -> z_.
    void kkt_solve() {
        for (int i = 0; i < kNc; ++i) zc_[i] = rhs_[i];
        for (int pair = 0; pair < pairs_; ++pair) {
            const S* h = rhs_.data() + pair_offset(pair);   // H_g = I
            S* v = v_.data() + pair * kMe;
            for (int r = 0; r < kMe; ++r) v[r] = -b_[pair * kMe + r];
            for (int q = 0; q < kNq; ++q) v[gram_row_[q]] += gram_coef_[q] * h[q];
            for (int r = 0; r < kMe; ++r) v[r] *= schur_inverse_[r];
            const S* agc = agc_.data() + pair * kMe * kNc;
            for (int i = 0; i < kNc; ++i) {
                S dot = 0;
                for (int r = 0; r < kMe; ++r) dot += agc[r + kMe * i] * v[r];
                zc_[i] -= dot;
            }
        }
        chol_solve(kfactor_.data(), kNc, zc_.data());
        for (int i = 0; i < kNc; ++i) z_[i] = zc_[i];
        for (int pair = 0; pair < pairs_; ++pair) {
            S* v = v_.data() + pair * kMe;
            const S* g = gall_.data() + pair * kMe * kNc;
            for (int i = 0; i < kNc; ++i) { const S zi = zc_[i]; for (int r = 0; r < kMe; ++r) v[r] += g[r + kMe * i] * zi; }
            const int base = pair_offset(pair);
            for (int q = 0; q < kNq; ++q) z_[base + q] = rhs_[base + q] - gram_coef_[q] * v[gram_row_[q]];
        }
    }

    S* jacobi_basis(int k) { return q_.data() + (k == 0 ? 0 : kMomentDim * kMomentDim + ((k - 1) / 2) * 25 + ((k - 1) % 2 ? 16 : 0)); }

    bool project_cone(int k, const S* in, S* out) {
        const int d = k == 0 ? kMomentDim : (k % 2 ? 4 : 3);
        const S root2 = std::sqrt(S(2)), half = S(1) / root2;
        S* full = scratch_.data();
        S* result = full + kMaxDim * kMaxDim;
        S* b = result + kMaxDim * kMaxDim;
        S* t = b + kMaxDim * kMaxDim;
        for (int col = 0; col < d; ++col) {
            full[col + d * col] = in[packed(d, col, col)];
            for (int row = col + 1; row < d; ++row) full[row + d * col] = full[col + d * row] = half * in[packed(d, row, col)];
        }
        for (int i = 0; i < d * d; ++i) t[i] = full[i];
        if (chol(t, d)) { for (int i = 0; i < d * (d + 1) / 2; ++i) out[i] = in[i]; return true; }
        S* q = jacobi_basis(k);
        int sweeps = project_psd_jacobi(full, q, result, b, t, d);
        if (sweeps < 0) {
            for (int i = 0; i < d * d; ++i) q[i] = S(0);
            for (int i = 0; i < d; ++i) q[i + d * i] = S(1);
            if (project_psd_jacobi(full, q, result, b, t, d) < 0) return false;
        }
        if (++projections_ % 64 == 0) reorthonormalize(q, d);
        for (int col = 0; col < d; ++col) {
            out[packed(d, col, col)] = result[col + d * col];
            for (int row = col + 1; row < d; ++row) out[packed(d, row, col)] = root2 * result[row + d * col];
        }
        return true;
    }

    bool project_all(const S* in, S* out) {
        if (!project_cone(0, in, out)) return false;
        for (int pair = 0; pair < pairs_; ++pair) {
            if (!project_cone(1 + 2 * pair, in + pair_slot(pair), out + pair_slot(pair))) return false;
            if (!project_cone(2 + 2 * pair, in + pair_slot(pair) + 10, out + pair_slot(pair) + 10)) return false;
        }
        for (int i = kMomentSize + pairs_ * kNq; i < ns_; ++i) out[i] = in[i] > S(0) ? in[i] : S(0);
        return true;
    }

    S inverse_slot_scale(int i) const {
        const int first_bound = kMomentSize + pairs_ * kNq;
        if (i < first_bound) return S(1);
        if (i < first_bound + kBoundRows) return bound_inverse_scale_[i - first_bound];
        return route_inverse_scale_[i - first_bound - kBoundRows];
    }

    bool evaluate(bool check, bool& converged) {
        S* s = y_.data();
        if (!project_all(p_.data(), s)) return false;
        for (int i = 0; i < ns_; ++i) tp_[i] = s[i];
        // y = C + p - 2 s, where C is the constant part of the affine map.
        affine_constant(y_.data(), true);
        adjoint(y_.data(), rhs_.data());
        const S inverse_rho = S(1) / settings_.rho;
        for (int i = 0; i < n_; ++i) rhs_[i] = -objective_[i] * inverse_rho - rhs_[i];
        kkt_solve();
        for (int i = 0; i < n_; ++i) if (!std::isfinite(z_[i])) return false;
        affine(z_.data(), a_.data());
        const S alpha = settings_.alpha;
        for (int i = 0; i < ns_; ++i) {
            const S si = tp_[i];
            tp_[i] = alpha * a_[i] + (S(1) - alpha) * si + (p_[i] - si);
            y_[i] = si;
        }
        if (!check) return true;
        if (!project_all(tp_.data(), sp_.data())) return false;
        S primal = 0, affine_norm = 0, slack_norm = 0;
        for (int i = 0; i < ns_; ++i) {
            const S w = inverse_slot_scale(i), r = a_[i] - sp_[i];
            primal += w * w * r * r; affine_norm += w * w * a_[i] * a_[i]; slack_norm += w * w * sp_[i] * sp_[i];
            y_[i] = sp_[i] - y_[i];
        }
        adjoint(y_.data(), rhs_.data());
        S dual = 0;
        for (int i = 0; i < n_; ++i) dual += rhs_[i] * rhs_[i];
        for (int i = 0; i < ns_; ++i) y_[i] = tp_[i] - sp_[i];
        adjoint(y_.data(), rhs_.data());
        S dual_scale = 0;
        for (int i = 0; i < n_; ++i) dual_scale += rhs_[i] * rhs_[i];
        const S rho = settings_.rho;
        const S primal_tolerance = std::sqrt(S(ns_)) * settings_.absolute_tolerance
            + settings_.relative_tolerance * std::max(std::sqrt(affine_norm), std::sqrt(slack_norm));
        const S dual_tolerance = std::sqrt(S(n_)) * settings_.absolute_tolerance
            + settings_.relative_tolerance * rho * std::sqrt(dual_scale);
        converged = std::sqrt(primal) <= primal_tolerance && rho * std::sqrt(dual) <= dual_tolerance;
        const S tiny = std::numeric_limits<S>::min();
        primal_relative_ = std::sqrt(primal) / std::max({std::sqrt(affine_norm), std::sqrt(slack_norm), tiny});
        dual_relative_ = std::sqrt(dual) / std::max(std::sqrt(dual_scale), tiny);
        return true;
    }

    // Writes C + p - 2 s into y_ (s is in y_ on entry) using the affine map's
    // constant part: identity block of the moment cone and scalar constants.
    void affine_constant(S* y, bool) const {
        for (int i = 0; i < ns_; ++i) y[i] = p_[i] - S(2) * y[i];
        for (int c = 0; c < D; ++c) y[packed(kMomentDim, c, c)] += S(1);
        for (int b = 0; b < kBoundRows; ++b) y[bound_slot(b)] += bound_constant_[b];
        for (int r = 0; r < routes_; ++r) y[route_slot(r)] += route_constant_[r];
    }

    void push(int& count, int& head) {
        S* dp = dp_.data() + head * kMaxNs;
        S* dg = dg_.data() + head * kMaxNs;
        for (int i = 0; i < ns_; ++i) { dp[i] = p_[i] - p_previous_[i]; dg[i] = g_[i] - g_previous_[i]; }
        const int slot = head;
        head = (head + 1) % (Memory > 0 ? Memory : 1);   // never reached with Memory == 0 (see solve)
        count = count < Memory ? count + 1 : Memory;
        for (int j = 0; j < count; ++j) {
            const S* other = dg_.data() + j * kMaxNs;
            S dot = 0;
            for (int i = 0; i < ns_; ++i) dot += dg[i] * other[i];
            gram_[slot + Memory * j] = gram_[j + Memory * slot] = dot;
        }
    }

    bool extrapolate(int k) {
        S matrix[Memory * Memory > 0 ? Memory * Memory : 1], gamma[Memory > 0 ? Memory : 1];
        S trace = 0;
        for (int j = 0; j < k; ++j) trace += gram_[j + Memory * j];
        const S lambda = settings_.anderson_regularization * (trace > S(0) ? trace : S(1));
        for (int j = 0; j < k; ++j) {
            for (int i = 0; i < k; ++i) matrix[i + k * j] = gram_[i + Memory * j];
            matrix[j + k * j] += lambda;
            const S* dg = dg_.data() + j * kMaxNs;
            S dot = 0;
            for (int i = 0; i < ns_; ++i) dot += dg[i] * g_[i];
            gamma[j] = dot;
        }
        if (!chol(matrix, k)) return false;
        chol_solve(matrix, k, gamma);
        for (int j = 0; j < k; ++j) if (!std::isfinite(gamma[j])) return false;
        for (int i = 0; i < ns_; ++i) p_[i] = tp_[i];
        for (int j = 0; j < k; ++j) {
            const S* dp = dp_.data() + j * kMaxNs;
            const S* dg = dg_.data() + j * kMaxNs;
            for (int i = 0; i < ns_; ++i) p_[i] -= gamma[j] * (dp[i] + dg[i]);
        }
        return true;
    }

    template<class T>
    static bool chol(T* a, int n) {
        for (int j = 0; j < n; ++j) {
            T d = a[j + j * n];
            for (int k = 0; k < j; ++k) d -= a[j + k * n] * a[j + k * n];
            if (!(d > T(0))) return false;
            d = std::sqrt(d);
            a[j + j * n] = d;
            for (int i = j + 1; i < n; ++i) {
                T v = a[i + j * n];
                for (int k = 0; k < j; ++k) v -= a[i + k * n] * a[j + k * n];
                a[i + j * n] = v / d;
            }
        }
        return true;
    }
    template<class T>
    static void chol_solve(const T* l, int n, T* x) {
        for (int i = 0; i < n; ++i) { T v = x[i]; for (int k = 0; k < i; ++k) v -= l[i + k * n] * x[k]; x[i] = v / l[i + i * n]; }
        for (int i = n - 1; i >= 0; --i) { T v = x[i]; for (int k = i + 1; k < n; ++k) v -= l[k + i * n] * x[k]; x[i] = v / l[i + i * n]; }
    }

    // Topology constants (init).
    std::array<S, kP * kL> nullspace_{};
    std::array<S, kP * kRows> right_inverse_{};
    std::array<S, kL * kL> reduced_cost0_{};
    std::array<S, kP * kL> cost_n0_{};
    std::array<S, kP * kP> cost0_{};
    std::array<int8_t, kNq> gram_row_{};
    std::array<S, kNq> gram_coef_{};
    std::array<S, kMe> schur_inverse_{};
    std::array<S, NSeg * kMe * kNw> w_rows_{};
    std::array<S, kNc * kNc> base_hcc_{};
    // Frame data.
    Settings settings_;
    Frame frame_;
    std::array<S, D> origin_{};
    S length_, tau_;
    std::array<S, D * kP> constant_points_{};
    std::array<S, kMaxN> objective_{};
    std::array<S, kMaxPairs * kMe * kNc> agc_{}, gall_{};
    std::array<S, kMaxPairs * kMe> b_{}, v_{};
    std::array<S, (kBoundRows > 0 ? kBoundRows : 1) * kNy> bound_coef_{};
    std::array<S, (kBoundRows > 0 ? kBoundRows : 1)> bound_constant_{}, bound_inverse_scale_{};
    std::array<S, (MaxRoutes > 0 ? MaxRoutes : 1) * kNy> route_coef_{};
    std::array<S, (MaxRoutes > 0 ? MaxRoutes : 1)> route_constant_{}, route_inverse_scale_{};
    std::array<S, kNc * kNc> mcc_{}, kfactor_{};
    std::array<S, Bounds ? kNc * kNc : 1> bound_hcc_{};
    // Iterates.
    std::array<S, kMaxN> z_{}, rhs_{};
    std::array<S, kNc> zc_{};
    std::array<S, kMaxNs> p_{}, tp_{}, sp_{}, a_{}, y_{};
    std::array<S, kMomentDim * kMomentDim + kMaxPairs * 25> q_{};
    std::array<S, 4 * kMaxDim * kMaxDim> scratch_{};
    std::array<S, kMaxNs> g_{}, p_previous_{}, g_previous_{}, fallback_{};
    std::array<S, (Memory > 0 ? Memory : 1) * kMaxNs> dp_{}, dg_{};
    std::array<S, (Memory > 0 ? Memory * Memory : 1)> gram_{};
    int obstacles_, pairs_, n_, ns_, routes_;
    long projections_;
    bool loaded_;
    S primal_relative_, dual_relative_;
};

}  // namespace tinysdp_ct_embedded
