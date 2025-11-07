#pragma once

#include "admc/admc/channels.hpp"
#include "admc/core/vec3.hpp"

#include <cmath>

namespace admc::solver
{
struct IterationSignals
{
    float constraint_residual{0.0f};
    float penetration_error{0.0f};
    float joint_error{0.0f};
    float admc_drift{0.0f};
};

struct IterationThresholds
{
    float residual{1.0e-4f};
    float penetration{1.0e-4f};
    float joint{1.0e-4f};
    float admc{1.0e-4f};
};

struct IterationPolicy
{
    bool should_continue{true};
    bool focus_island{false};
};

class CompositeIterationGate
{
public:
    struct Settings
    {
        IterationThresholds thresholds{};
        float focus_ratio{2.0f};
    };

    CompositeIterationGate() noexcept = default;
    explicit CompositeIterationGate(const Settings& settings) noexcept : settings_(settings) {}

    [[nodiscard]] IterationPolicy evaluate(const IterationSignals& signals) const noexcept
    {
        const bool residual_ok = std::fabs(signals.constraint_residual) <= settings_.thresholds.residual;
        const bool penetration_ok = std::fabs(signals.penetration_error) <= settings_.thresholds.penetration;
        const bool joint_ok = std::fabs(signals.joint_error) <= settings_.thresholds.joint;
        const bool admc_ok = std::fabs(signals.admc_drift) <= settings_.thresholds.admc;

        const bool continue_iterations = !(residual_ok && penetration_ok && joint_ok && admc_ok);

        bool focus = false;
        if (!residual_ok && std::fabs(signals.admc_drift) > settings_.thresholds.admc * settings_.focus_ratio)
        {
            focus = true;
        }

        return IterationPolicy{
            .should_continue = continue_iterations,
            .focus_island = focus};
    }

    [[nodiscard]] const Settings& settings() const noexcept { return settings_; }

private:
    Settings settings_{};
};

struct DriftSample
{
    float max_channel_drift{0.0f};
    float momentum_norm{0.0f};
};

inline DriftSample make_drift_sample(const ::admc::admc::DirectionalTracker& tracker) noexcept
{
    const float max_channel = tracker.max_abs_channel_drift();
    const ::admc::core::Vec3 drift = tracker.momentum_drift();
    return DriftSample{
        .max_channel_drift = max_channel,
        .momentum_norm = ::admc::core::length(drift)};
}
} // namespace admc::solver
