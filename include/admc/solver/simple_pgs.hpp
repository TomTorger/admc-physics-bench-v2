#pragma once

#include "admc/admc/channels.hpp"
#include "admc/constraints/constraint_row.hpp"
#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"
#include "admc/solver/conservation_tracker.hpp"
#include "admc/solver/constraint_key.hpp"
#include "admc/solver/iteration_control.hpp"
#include "admc/solver/operators.hpp"
#include "admc/solver/warmstart.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
#include <limits>

namespace admc::solver
{
struct SolverBody
{
    ::admc::core::Vec3 linear_velocity{};
    ::admc::core::Vec3 angular_velocity{};
    float inverse_mass{1.0f};
    ::admc::core::Mat3 inverse_inertia{::admc::core::Mat3::identity()};
};

struct ConstraintInstance
{
    ::admc::constraints::ConstraintRow row{};
    std::size_t body_a{0};
    std::size_t body_b{0};
    ConstraintKey key{};
};

struct IslandState
{
    std::vector<SolverBody> bodies{};
    std::vector<ConstraintInstance> constraints{};
};

struct SolverResult
{
    int iterations{0};
    float max_residual{0.0f};
    float max_penetration_error{0.0f};
    float max_joint_error{0.0f};
    float admc_drift{0.0f};
    double warmstart_ms{0.0};
    double iteration_ms{0.0};
    double total_ms{0.0};
};

    class SimplePGSSolver
    {
    public:
        struct Options
        {
            int max_iterations{8};
            CompositeIterationGate::Settings gate_settings{};
            ManifoldWarmstartScaler::Settings warmstart_settings{};
            // Normalization velocity scale for ADMC drift (e.g., g * dt)
            float drift_velocity_scale{9.81f / 60.0f};
        };

    SimplePGSSolver() noexcept;
    explicit SimplePGSSolver(const Options& options) noexcept;

    [[nodiscard]] SolverResult solve(IslandState& island, IConservationTracker* tracker = nullptr) const noexcept;

private:
    struct MomentumSample
    {
        float mass{0.0f};
        ::admc::core::Vec3 momentum{};
    };

    static MomentumSample sample_momentum(
        const IslandState& island,
        const std::vector<BodyStateView>& states) noexcept;

    static void apply_velocity_delta(
        std::size_t body_index,
        const BodyImpulse& impulse,
        const ConstraintOperator& op,
        std::vector<BodyStateView>& states) noexcept;

    void warmstart_constraints(
        IslandState& island,
        const ConstraintOperator& op,
        std::vector<BodyStateView>& states,
        float island_scale) const noexcept;

    struct ConstraintCacheEntry
    {
        float lambda{0.0f};
        float residual{0.0f};
        float admc_drift{0.0f};
    };

    Options options_{};
    CompositeIterationGate gate_;
    ManifoldWarmstartScaler scaler_;
    mutable std::unordered_map<ConstraintKey, ConstraintCacheEntry, ConstraintKeyHasher> cache_;
};

inline SimplePGSSolver::SimplePGSSolver() noexcept : SimplePGSSolver(Options{}) {}

inline SimplePGSSolver::SimplePGSSolver(const Options& options) noexcept
    : options_(options),
      gate_(options.gate_settings),
      scaler_(options.warmstart_settings)
{
}

inline SimplePGSSolver::MomentumSample SimplePGSSolver::sample_momentum(
    const IslandState& island,
    const std::vector<BodyStateView>& states) noexcept
{
    MomentumSample sample{};
    const std::size_t count = std::min(island.bodies.size(), states.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        const SolverBody& body = island.bodies[i];
        if (body.inverse_mass <= 0.0f)
        {
            continue;
        }
        const float mass = 1.0f / body.inverse_mass;
        sample.mass += mass;
        sample.momentum += states[i].linear_velocity * mass;
    }
    if (sample.mass <= 0.0f)
    {
        sample.mass = 1.0f;
    }
    return sample;
}

inline void SimplePGSSolver::apply_velocity_delta(
    std::size_t body_index,
    const BodyImpulse& impulse,
    const ConstraintOperator& op,
    std::vector<BodyStateView>& states) noexcept
{
    if (body_index >= states.size())
    {
        return;
    }
    const BodyImpulse response = op.apply_inverse_mass(body_index, impulse);
    states[body_index].linear_velocity += response.linear;
    states[body_index].angular_velocity += response.angular;
}

inline void SimplePGSSolver::warmstart_constraints(
    IslandState& island,
    const ConstraintOperator& op,
    std::vector<BodyStateView>& states,
    float island_scale) const noexcept
{
    for (ConstraintInstance& constraint : island.constraints)
    {
        const auto cache_it = cache_.find(constraint.key);
        if (cache_it != cache_.end())
        {
            constraint.row.lambda = cache_it->second.lambda;
        }
        const ManifoldWarmstartSample sample{
            .residual_norm = (cache_it != cache_.end()) ? cache_it->second.residual : std::fabs(constraint.row.bias),
            .admc_drift = (cache_it != cache_.end()) ? cache_it->second.admc_drift : 0.0f,
            .impulse_magnitude = std::fabs(constraint.row.lambda)};
        const float scale = scaler_.scale_for(sample) * island_scale;
        const float scaled_lambda = constraint.row.lambda * scale;
        constraint.row.lambda = scaled_lambda;
        if (scaled_lambda == 0.0f)
        {
            continue;
        }

        const BodyPairImpulse pair = op.apply_jacobian_transpose(constraint.row, scaled_lambda);
        apply_velocity_delta(constraint.body_a, pair.body_a, op, states);
        apply_velocity_delta(constraint.body_b, pair.body_b, op, states);
    }
}

inline SolverResult SimplePGSSolver::solve(IslandState& island, IConservationTracker* island_tracker) const noexcept
{
    SolverResult result{};
    if (island.bodies.empty() || island.constraints.empty())
    {
        return result;
    }

    std::vector<BodyStateView> state_views(island.bodies.size());
    std::vector<BodyInertiaView> inertia_views(island.bodies.size());
    for (std::size_t i = 0; i < island.bodies.size(); ++i)
    {
        state_views[i].linear_velocity = island.bodies[i].linear_velocity;
        state_views[i].angular_velocity = island.bodies[i].angular_velocity;
        inertia_views[i].inverse_mass = island.bodies[i].inverse_mass;
        inertia_views[i].inverse_inertia = island.bodies[i].inverse_inertia;
    }

    ConstraintOperator op(state_views, inertia_views);
    const MomentumSample pre = sample_momentum(island, state_views);
    ::admc::admc::DirectionalTracker tracker;
    tracker.record_pre(pre.mass, pre.momentum);

    const auto warmstart_start = std::chrono::steady_clock::now();
    if (island_tracker)
    {
        island_tracker->on_island_begin(0);
    }
    const float island_warm_scale = island_tracker ? island_tracker->warmstart_scale(0) : 1.0f;
    warmstart_constraints(island, op, state_views, island_warm_scale);
    const auto warmstart_end = std::chrono::steady_clock::now();
    result.warmstart_ms = std::chrono::duration<double, std::milli>(warmstart_end - warmstart_start).count();

    std::vector<float> residual_samples(island.constraints.size(), 0.0f);

    const auto iteration_start = std::chrono::steady_clock::now();
    IterationSignals last_signals{};
    for (int iter = 0; iter < options_.max_iterations; ++iter)
    {
        float max_residual = 0.0f;
        float max_penetration = 0.0f;
        float max_joint = 0.0f;

        for (std::size_t ci = 0; ci < island.constraints.size(); ++ci)
        {
            ConstraintInstance& constraint = island.constraints[ci];
            if (constraint.row.effective_mass <= 0.0f)
            {
                continue;
            }

            const float relative_velocity_before = op.apply_jacobian(constraint.row, constraint.body_a, constraint.body_b);
            const float rhs = -constraint.row.bias - relative_velocity_before;
            const float delta_lambda = rhs * constraint.row.effective_mass;
            const float new_lambda = std::clamp(
                constraint.row.lambda + delta_lambda,
                constraint.row.lower_limit,
                constraint.row.upper_limit);
            const float applied = new_lambda - constraint.row.lambda;
            constraint.row.lambda = new_lambda;

            if (std::fabs(applied) > 0.0f)
            {
                const BodyPairImpulse pair = op.apply_jacobian_transpose(constraint.row, applied);
                apply_velocity_delta(constraint.body_a, pair.body_a, op, state_views);
                apply_velocity_delta(constraint.body_b, pair.body_b, op, state_views);
            }

            max_residual = std::max(max_residual, std::fabs(rhs));
            const float relative_velocity_after = op.apply_jacobian(constraint.row, constraint.body_a, constraint.body_b);
            const float corrected = -constraint.row.bias - relative_velocity_after;

            if (constraint.row.type == ::admc::constraints::ConstraintType::ContactNormal)
            {
                max_penetration = std::max(max_penetration, std::fabs(corrected));
            }
            else if (constraint.row.type == ::admc::constraints::ConstraintType::Joint)
            {
                max_joint = std::max(max_joint, std::fabs(corrected));
            }
            max_residual = std::max(max_residual, std::fabs(corrected));
            residual_samples[ci] = std::fabs(corrected);
        }

    tracker.record_post(pre.mass, sample_momentum(island, state_views).momentum);
    const DriftSample drift = make_drift_sample(tracker);
    const float denom = std::max(pre.mass * options_.drift_velocity_scale, 1.0e-9f);
    const float normalized_drift = drift.momentum_norm / denom;

        IterationSignals signals{
            .constraint_residual = max_residual,
            .penetration_error = max_penetration,
            .joint_error = max_joint,
            .admc_drift = normalized_drift};
        last_signals = signals;

        result.iterations = iter + 1;
        result.max_residual = max_residual;
        result.max_penetration_error = max_penetration;
        result.max_joint_error = max_joint;
        result.admc_drift = normalized_drift;

        const IterationPolicy policy = island_tracker ? island_tracker->on_iteration_end(0, signals) : gate_.evaluate(signals);
        if (!policy.should_continue)
        {
            break;
        }
    }

    const auto iteration_end = std::chrono::steady_clock::now();
    result.iteration_ms = std::chrono::duration<double, std::milli>(iteration_end - iteration_start).count();
    result.total_ms = result.warmstart_ms + result.iteration_ms;

    for (std::size_t i = 0; i < island.bodies.size(); ++i)
    {
        island.bodies[i].linear_velocity = state_views[i].linear_velocity;
        island.bodies[i].angular_velocity = state_views[i].angular_velocity;
    }

    for (std::size_t ci = 0; ci < island.constraints.size(); ++ci)
    {
        const ConstraintInstance& constraint = island.constraints[ci];
        cache_[constraint.key] = ConstraintCacheEntry{
            constraint.row.lambda,
            (ci < residual_samples.size()) ? residual_samples[ci] : 0.0f,
            result.admc_drift};
    }

    if (island_tracker)
    {
        island_tracker->on_island_end(0, last_signals);
    }

    return result;
}
} // namespace admc::solver
