#pragma once

// Conservative continuous-time clearance checker for cubic Bernstein pieces
// against spheres moving with constant velocity. Fixed stack, no allocation.
//
// For a segment and a sphere, the relative curve gamma(s) - c(s) is again a
// cubic Bernstein curve (c is linear in time). Interval de Casteljau
// subdivision with outward rounding proves each subcurve's bounding box is
// farther than radius + margin from the origin. Unresolved within the node
// budget is never reported safe. Requires IEEE arithmetic (no fast-math).

#include <tinysdp_ct_embedded/linalg.hpp>

#include <array>
#include <cmath>
#include <limits>

namespace tinysdp_ct_embedded {

template<class S, int D, int MaxDepth = 18>
class Checker {
public:
    static_assert(std::numeric_limits<S>::is_iec559, "IEEE arithmetic required");
#ifdef __FAST_MATH__
#error "Checker requires IEEE arithmetic without fast-math"
#endif
    enum class Status { kSafe, kUnresolved, kInvalid };
    struct Result {
        Status status = Status::kInvalid;
        int nodes = 0;
        S clearance_lower_bound = -std::numeric_limits<S>::infinity();
    };
    struct Sphere { std::array<S, D> center{}, velocity{}; S radius = 0; };

    // controls: D x (3 * segments + 1), column major; start_time is the plan
    // time of the first control point relative to the spheres' reference.
    Result check(const S* controls, int segments, S segment_duration, const Sphere* spheres, int count,
                 S margin, int node_budget, S start_time = S(0)) {
        Result result;
        if (segments < 1 || !(segment_duration > S(0)) || count < 0 || !(margin >= S(0)) || node_budget < 1)
            return result;
        // Non-finite data is invalid, never safe: NaN would otherwise vanish in the min/max below.
        if (!std::isfinite(segment_duration) || !std::isfinite(margin) || !std::isfinite(start_time)) return result;
        for (int k = 0; k < D * (3 * segments + 1); ++k) if (!std::isfinite(controls[k])) return result;
        for (int o = 0; o < count; ++o) {
            if (!std::isfinite(spheres[o].radius) || !(spheres[o].radius >= S(0))) return result;
            for (int c = 0; c < D; ++c)
                if (!std::isfinite(spheres[o].center[c]) || !std::isfinite(spheres[o].velocity[c])) return result;
        }
        S minimum = std::numeric_limits<S>::infinity();
        for (int seg = 0; seg < segments; ++seg)
            for (int o = 0; o < count; ++o) {
                const Sphere& sphere = spheres[o];
                const S required = up(sphere.radius + margin), required_squared = up(required * required);
                Node& root = stack_[0];
                for (int i = 0; i < 4; ++i) {
                    const S offset = segment_duration * (S(seg) + S(i) / S(3));
                    const S t = start_time + offset, t_abs = std::abs(start_time) + std::abs(offset);
                    for (int c = 0; c < D; ++c) {
                        // Relative control point, widened by a bound on the
                        // rounding of t, t * v, and the sums (a few ulps).
                        const S center = sphere.center[c] + t * sphere.velocity[c];
                        const S x = controls[c + D * (3 * seg + i)];
                        const S slack = S(8) * std::numeric_limits<S>::epsilon()
                            * (std::abs(x) + std::abs(sphere.center[c]) + t_abs * std::abs(sphere.velocity[c]));
                        root.lo[c][i] = down(down(x - center) - slack);
                        root.hi[c][i] = up(up(x - center) + slack);
                    }
                }
                root.depth = 0;
                int used = 1;
                while (used) {
                    if (result.nodes >= node_budget) { result.status = Status::kUnresolved; return result; }
                    const Node node = stack_[--used];
                    ++result.nodes;
                    S distance_squared = 0;
                    for (int c = 0; c < D; ++c) {
                        S lower = node.lo[c][0], upper = node.hi[c][0];
                        for (int i = 1; i < 4; ++i) { lower = lower < node.lo[c][i] ? lower : node.lo[c][i]; upper = upper > node.hi[c][i] ? upper : node.hi[c][i]; }
                        S distance = 0;
                        if (lower > S(0)) distance = down(lower);
                        else if (upper < S(0)) distance = down(-upper);
                        distance_squared = down(distance_squared + down(distance * distance));
                    }
                    if (distance_squared > required_squared) {
                        const S clearance = down(down(std::sqrt(distance_squared)) - sphere.radius);
                        minimum = minimum < clearance ? minimum : clearance;
                        continue;
                    }
                    if (node.depth == MaxDepth || used + 2 > MaxDepth + 1) { result.status = Status::kUnresolved; return result; }
                    split(node, stack_[used + 1], stack_[used]);
                    used += 2;
                }
            }
        result.status = Status::kSafe;
        result.clearance_lower_bound = minimum;
        return result;
    }

private:
    struct Node { S lo[D][4], hi[D][4]; int depth; };
    static S down(S v) { return std::nextafter(v, -std::numeric_limits<S>::infinity()); }
    static S up(S v) { return std::nextafter(v, std::numeric_limits<S>::infinity()); }
    static void split(const Node& source, Node& left, Node& right) {
        S lo[D][4], hi[D][4];
        for (int c = 0; c < D; ++c) for (int i = 0; i < 4; ++i) { lo[c][i] = source.lo[c][i]; hi[c][i] = source.hi[c][i]; }
        left.depth = right.depth = source.depth + 1;
        for (int c = 0; c < D; ++c) { left.lo[c][0] = lo[c][0]; left.hi[c][0] = hi[c][0]; right.lo[c][3] = lo[c][3]; right.hi[c][3] = hi[c][3]; }
        for (int level = 1; level <= 3; ++level) {
            for (int i = 0; i <= 3 - level; ++i)
                for (int c = 0; c < D; ++c) {
                    lo[c][i] = down(down(lo[c][i] + lo[c][i + 1]) * S(0.5));
                    hi[c][i] = up(up(hi[c][i] + hi[c][i + 1]) * S(0.5));
                }
            for (int c = 0; c < D; ++c) {
                left.lo[c][level] = lo[c][0]; left.hi[c][level] = hi[c][0];
                right.lo[c][3 - level] = lo[c][3 - level]; right.hi[c][3 - level] = hi[c][3 - level];
            }
        }
    }
    std::array<Node, MaxDepth + 2> stack_{};
};

}  // namespace tinysdp_ct_embedded
