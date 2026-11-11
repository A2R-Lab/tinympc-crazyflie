#pragma once

// One receding-horizon planning frame on the microcontroller:
//   1. keep the MaxObs obstacles with the smallest predicted distance to the
//      straight start-goal motion over the horizon (the SDP's capacity);
//   2. solve; every `accept_interval` iterations, decode the plan and accept
//      it as soon as the checker certifies clearance against ALL obstacles
//      and its derivative control points satisfy the limits;
//   3. otherwise try one-cut route halfspaces at the worst closest approaches
//      (previous route first), each warm-started from the base iterate.
// The caller's `time_left()` bounds the whole frame. Safety comes only from
// the checker; obstacle selection and routing affect success, not safety.

#include <tinysdp_ct_embedded/oneshot.hpp>
#include <tinysdp_ct_embedded/planner.hpp>
#include <tinysdp_ct_embedded/set_encoder.hpp>
#include <tinysdp_ct_embedded/unified_encoder.hpp>

#include <type_traits>

namespace tinysdp_ct_embedded {

// Scorer selects the learned cut scorer: 0 the one-shot MLP (oneshot.hpp),
// 1 the distilled int8 set encoder (set_encoder.hpp), 2 the distilled int8
// unified encoder shared with the Franka (unified_encoder.hpp). Encoders
// hold their encoded tokens in the planner object.
template<class S, int D, int NSeg, int MaxObs, int MaxWorld, int Memory = 5, int Scorer = 0>
class FramePlanner {
public:
    using Core = Planner<S, D, NSeg, MaxObs, 1, Memory, true>;
    using Sphere = MovingSphere<S, D>;
    using Plan = typename Core::Plan;
    static constexpr int kDirections = D == 3 ? 8 : 2;

    struct Settings {
        typename Core::Settings core;
        int base_iterations, route_iterations, accept_interval;
        int checker_nodes, anchors;
        // Early acceptance also requires a loosely converged iterate, so that
        // safe but poor intermediate plans are not executed.
        S accept_primal, accept_dual;
        S cut_margin;
        // Learned one-shot cut ordering (oneshot.hpp), tried before the
        // anchor bank when the base solve is not certified. 3D only.
        bool learned_routes;
        int learned_tries;
        // Tabu across frames (0 = off): a learned proposal that failed within
        // the last `tabu_frames` frames is tried after all others, so the
        // proposal budget is spent over time instead of retrying the same
        // top candidate every frame.
        int tabu_frames;

        static Settings defaults() {
            Settings s{};
            s.core = Core::Settings::defaults();
            s.base_iterations = 400; s.route_iterations = 1500; s.accept_interval = 50;
            s.checker_nodes = 4096; s.anchors = 3;
            s.accept_primal = S(1e-3); s.accept_dual = S(1e-2);
            s.cut_margin = S(0.07);
            s.learned_routes = false; s.learned_tries = 8; s.tabu_frames = 0;
            return s;
        }
    };
    struct Stats { int iterations = 0, attempts = 0, selected = 0, learned_attempts = 0; bool base_accepted = false, accepted = false, learned = false; };

    bool init(const Settings& settings = Settings::defaults()) {
        settings_ = settings;
        world_count_ = 0; last_obstacle_ = last_direction_ = -1;
        frame_ = 0;
        failed_at_.fill(0);
        return core_.init(settings.core);
    }

    template<class TimeLeft>
    bool plan(const Vec<S, D>& position, const Vec<S, D>& velocity, const Vec<S, D>& goal, S horizon,
              const Sphere* world, int world_count, Plan& out, Stats& stats, TimeLeft&& time_left) {
        if (world_count < 1 || world_count > MaxWorld) return false;
        if (++frame_ == 0) frame_ = 1;  // 0 marks "never failed"
        typename Core::Frame frame{};
        frame.start = position; frame.start_velocity = velocity; frame.goal = goal;
        for (int c = 0; c < D; ++c) frame.goal_velocity[c] = S(0);
        frame.horizon = horizon;
        select(position, goal, horizon, world, world_count, frame);
        stats.selected = frame.obstacle_count;
        for (int i = 0; i < world_count; ++i) {
            for (int c = 0; c < D; ++c) { spheres_[i].center[c] = world[i].center[c]; spheres_[i].velocity[c] = world[i].velocity[c]; }
            spheres_[i].radius = world[i].radius;
        }
        world_count_ = world_count;
        if (!core_.load(frame)) return false;
        if (attempt(settings_.base_iterations, out, stats, time_left)) { stats.base_accepted = stats.accepted = true; return true; }
        core_.save(base_);
        if (D == 3 && settings_.learned_routes && learned_routes(position, goal, world, world_count, frame, out, stats, time_left))
            return true;
        Plan base_plan;
        core_.plan(base_plan);
        // Worst closest approaches among the selected obstacles (sampled).
        struct Anchor { int obstacle = 0, segment = 0; S s = 0, clearance = 0; };
        Anchor anchors[MaxObs];
        for (int o = 0; o < frame.obstacle_count; ++o) {
            anchors[o] = {o, 0, S(0), std::numeric_limits<S>::infinity()};
            for (int seg = 0; seg < NSeg; ++seg)
                for (int k = 0; k <= 16; ++k) {
                    const S s = S(k) / S(16);
                    const S d = distance(base_plan, frame.obstacles[o], seg, s);
                    if (d < anchors[o].clearance) anchors[o] = {o, seg, s, d};
                }
        }
        for (int i = 1; i < frame.obstacle_count; ++i)   // insertion sort by clearance
            for (int j = i; j > 0 && anchors[j].clearance < anchors[j - 1].clearance; --j) { const Anchor t = anchors[j]; anchors[j] = anchors[j - 1]; anchors[j - 1] = t; }
        const int count = settings_.anchors < frame.obstacle_count ? settings_.anchors : frame.obstacle_count;
        for (int pass = 0; pass < 2; ++pass)
            for (int a = 0; a < count; ++a)
                for (int dir = 0; dir < kDirections; ++dir) {
                    const bool previous = anchors[a].obstacle == last_obstacle_ && dir == last_direction_;
                    if ((pass == 0) != previous) continue;
                    if (!(time_left() > S(0))) return false;
                    RouteCut<S, D> cut;
                    cut.segment = anchors[a].segment; cut.obstacle = anchors[a].obstacle;
                    cut.s = anchors[a].s; cut.margin = settings_.cut_margin;
                    cut.direction = direction(position, goal, dir);
                    core_.restore(base_);
                    if (!core_.set_routes(&cut, 1)) continue;
                    ++stats.attempts;
                    if (attempt(settings_.route_iterations, out, stats, time_left)) {
                        last_obstacle_ = anchors[a].obstacle; last_direction_ = dir;
                        stats.accepted = true;
                        return true;
                    }
                }
        return false;
    }

    // Certify a plan (e.g. a retained suffix) against the world obstacles.
    bool certified(const S* controls, int segments, S segment_duration, S start_time = S(0)) {
        return clearance(controls, segments, segment_duration, start_time) > -std::numeric_limits<S>::infinity();
    }

    // Certified clearance lower bound (m, beyond each sphere's radius) of a plan against the world
    // obstacles; -inf unless the plan is certified with at least the safety margin. This is the
    // tracking budget rho the executed plan tolerates (any deviation below it stays clear).
    S clearance(const S* controls, int segments, S segment_duration, S start_time = S(0)) {
        const auto result = checker_.check(controls, segments, segment_duration, spheres_.data(), world_count_,
                                           settings_.core.safety_margin, settings_.checker_nodes, start_time);
        return result.status == Checker<S, D>::Status::kSafe ? result.clearance_lower_bound
                                                             : -std::numeric_limits<S>::infinity();
    }

    static constexpr std::size_t footprint() { return sizeof(FramePlanner); }

private:
    // Straight-path cuts on the planned obstacles, ordered by the learned
    // score (features use every world obstacle for context).
    template<class TimeLeft>
    bool learned_routes(const Vec<S, D>& position, const Vec<S, D>& goal, const Sphere* world, int world_count,
                        const typename Core::Frame& frame, Plan& out, Stats& stats, TimeLeft&& time_left) {
        constexpr int kMax = MaxObs * kDirections;
        float score[kMax], tau[kMax];
        int obstacle[kMax], direction_index[kMax], count = 0;
        float start[3], target[3];
        for (int c = 0; c < 3; ++c) { start[c] = float(position[c]); target[c] = float(goal[c]); }
        if constexpr (Scorer == 1) encoder_.encode(start, target, world, world_count);
        if constexpr (Scorer == 2) unified_.encode(start, target, world, world_count);
        for (int i = 0; i < frame.obstacle_count; ++i) {
            const int w = selected_index_[i];
            const float t = straight_closest_time(start, target, world[w]);
            for (int dir = 0; dir < kDirections; ++dir) {
                const Vec<S, D> u = direction(position, goal, dir);
                const float uf[3] = {float(u[0]), float(u[1]), float(u[2])};
                if constexpr (Scorer == 1) {
                    score[count] = encoder_.score(w, uf, t);
                } else if constexpr (Scorer == 2) {
                    score[count] = unified_.score(w, uf, dir, kDirections, t);
                } else {
                    float x[15];
                    oneshot_features(start, target, world, world_count, w, t, uf, x);
                    score[count] = oneshot_score(x);
                }
                if (tabu(w, dir)) score[count] -= 1e6f;  // recently failed: try last
                tau[count] = t; obstacle[count] = i; direction_index[count] = dir;
                ++count;
            }
        }
        for (int k = 0; k < settings_.learned_tries && k < count; ++k) {
            int best = k;
            for (int j = k + 1; j < count; ++j) if (score[j] > score[best]) best = j;
            std::swap(score[k], score[best]); std::swap(tau[k], tau[best]);
            std::swap(obstacle[k], obstacle[best]); std::swap(direction_index[k], direction_index[best]);
            if (!(time_left() > S(0))) return false;
            RouteCut<S, D> cut;
            cut.segment = tau[k] < 0.5f ? 0 : 1;
            cut.s = S(cut.segment == 0 ? 2 * tau[k] : 2 * tau[k] - 1);
            cut.obstacle = obstacle[k];
            cut.margin = settings_.cut_margin;
            cut.direction = direction(position, goal, direction_index[k]);
            core_.restore(base_);
            if (!core_.set_routes(&cut, 1)) continue;
            ++stats.attempts; ++stats.learned_attempts;
            if (attempt(settings_.route_iterations, out, stats, time_left)) {
                stats.accepted = stats.learned = true;
                return true;
            }
            if (settings_.tabu_frames > 0) failed_at_[selected_index_[obstacle[k]] * kDirections + direction_index[k]] = frame_;
        }
        return false;
    }

    template<class TimeLeft>
    bool attempt(int budget, Plan& out, Stats& stats, TimeLeft&& time_left) {
        bool accepted = false;
        int checks = 0;
        const int every = settings_.accept_interval / settings_.core.check_interval;
        const auto info = core_.solve(budget, [&](const Core& core) {
            if (++checks % (every > 0 ? every : 1)) return !(time_left() > S(0));
            if (core.primal_relative() > settings_.accept_primal || core.dual_relative() > settings_.accept_dual)
                return !(time_left() > S(0));
            core.plan(out);
            accepted = core.kinematics_ok(out)
                && certified(out.controls.data(), NSeg, out.segment_duration);
            return accepted || !(time_left() > S(0));
        });
        stats.iterations += info.iterations;
        if (accepted) return true;
        if (!info.valid) return false;
        if (!info.converged && (core_.primal_relative() > settings_.accept_primal
                                || core_.dual_relative() > settings_.accept_dual)) return false;
        core_.plan(out);
        return core_.kinematics_ok(out) && certified(out.controls.data(), NSeg, out.segment_duration);
    }

    // Keep the MaxObs spheres closest to the nominal straight motion.
    void select(const Vec<S, D>& start, const Vec<S, D>& goal, S horizon, const Sphere* world, int count,
                typename Core::Frame& frame) {
        S score[MaxWorld];
        int order[MaxWorld];
        for (int i = 0; i < count; ++i) {
            score[i] = std::numeric_limits<S>::infinity();
            for (int k = 0; k <= 8; ++k) {
                const S t = horizon * S(k) / S(8);
                S d2 = 0;
                for (int c = 0; c < D; ++c) {
                    const S nominal = start[c] + (goal[c] - start[c]) * S(k) / S(8);
                    const S diff = nominal - (world[i].center[c] + t * world[i].velocity[c]);
                    d2 += diff * diff;
                }
                const S d = std::sqrt(d2) - world[i].radius;
                if (d < score[i]) score[i] = d;
            }
            order[i] = i;
        }
        for (int i = 1; i < count; ++i)
            for (int j = i; j > 0 && score[order[j]] < score[order[j - 1]]; --j) { const int t = order[j]; order[j] = order[j - 1]; order[j - 1] = t; }
        frame.obstacle_count = count < MaxObs ? count : MaxObs;
        for (int i = 0; i < frame.obstacle_count; ++i) { frame.obstacles[i] = world[order[i]]; selected_index_[i] = order[i]; }
        if (last_obstacle_ >= frame.obstacle_count) last_obstacle_ = -1;
    }

    static S distance(const Plan& plan, const Sphere& sphere, int seg, S s) {
        const S q = S(1) - s, basis[4] = {q * q * q, S(3) * q * q * s, S(3) * q * s * s, s * s * s};
        const S t = plan.segment_duration * (S(seg) + s);
        S d2 = 0;
        for (int c = 0; c < D; ++c) {
            S x = 0;
            for (int i = 0; i < 4; ++i) x += basis[i] * plan.controls[c + D * (3 * seg + i)];
            const S diff = x - (sphere.center[c] + t * sphere.velocity[c]);
            d2 += diff * diff;
        }
        return std::sqrt(d2) - sphere.radius;
    }

    // Transverse directions around the start-goal axis.
    static Vec<S, D> direction(const Vec<S, D>& start, const Vec<S, D>& goal, int index) {
        Vec<S, D> f;
        S norm = 0;
        for (int c = 0; c < D; ++c) { f[c] = goal[c] - start[c]; norm += f[c] * f[c]; }
        norm = std::sqrt(norm);
        for (int c = 0; c < D; ++c) f[c] = norm > S(0) ? f[c] / norm : (c == 0 ? S(1) : S(0));
        Vec<S, D> out;
        if (D == 2) { out[0] = -f[1] * (index ? S(-1) : S(1)); out[1] = f[0] * (index ? S(-1) : S(1)); return out; }
        // e1 = f x ref, e2 = f x e1 (ref = z unless f is nearly vertical).
        S ref[3] = {0, 0, 1};
        if (std::abs(f[2]) > S(0.9)) { ref[1] = 1; ref[2] = 0; }
        S e1[3] = {f[1] * ref[2] - f[2] * ref[1], f[2] * ref[0] - f[0] * ref[2], f[0] * ref[1] - f[1] * ref[0]};
        S n1 = std::sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
        for (auto& v : e1) v /= n1;
        const S e2[3] = {f[1] * e1[2] - f[2] * e1[1], f[2] * e1[0] - f[0] * e1[2], f[0] * e1[1] - f[1] * e1[0]};
        const S angle = S(2) * S(3.14159265358979323846) * S(index) / S(kDirections);
        for (int c = 0; c < 3; ++c) out[c] = std::cos(angle) * e1[c] + std::sin(angle) * e2[c];
        return out;
    }

    Settings settings_;
    Core core_;
    typename Core::Snapshot base_;
    Checker<S, D> checker_;
    std::array<typename Checker<S, D>::Sphere, MaxWorld> spheres_;
    bool tabu(int world_index, int dir) const {
        if (settings_.tabu_frames <= 0) return false;
        const std::uint16_t at = failed_at_[world_index * kDirections + dir];
        return at != 0 && static_cast<std::uint16_t>(frame_ - at) <= settings_.tabu_frames;
    }

    int world_count_, last_obstacle_, last_direction_;
    std::uint16_t frame_;
    std::array<std::uint16_t, MaxWorld * kDirections> failed_at_;
    std::array<int, MaxObs> selected_index_;
    struct NoEncoder {};
    std::conditional_t<Scorer == 1, SetEncoder<MaxWorld>, NoEncoder> encoder_;
    struct NoUnified {};
    std::conditional_t<Scorer == 2, UnifiedEncoder<MaxWorld>, NoUnified> unified_;
};

}  // namespace tinysdp_ct_embedded
