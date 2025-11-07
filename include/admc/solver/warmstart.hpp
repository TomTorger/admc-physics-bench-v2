#pragma once

#include <algorithm>
#include <span>
#include <vector>

namespace admc::solver
{
struct ManifoldWarmstartSample
{
    float residual_norm{0.0f};
    float admc_drift{0.0f};
    float impulse_magnitude{0.0f};
};

class ManifoldWarmstartScaler
{
public:
    struct Settings
    {
        float drift_threshold{2.5e-4f};
        float residual_threshold{1.0e-3f};
        float min_scale{0.25f};
        float max_scale{1.0f};
        float sleeping_impulse{1.0e-3f};
    };

    explicit ManifoldWarmstartScaler(Settings settings = Settings{}) noexcept : settings_(settings) {}

    [[nodiscard]] float scale_for(const ManifoldWarmstartSample& sample) const noexcept
    {
        if (sample.impulse_magnitude <= settings_.sleeping_impulse)
        {
            return settings_.max_scale;
        }

        const float drift_ratio = sample.admc_drift / (settings_.drift_threshold + 1.0e-9f);
        const float residual_ratio = sample.residual_norm / (settings_.residual_threshold + 1.0e-9f);
        const float severity = std::clamp(std::max(drift_ratio, residual_ratio), 0.0f, 1.0f);
        const float scale_span = settings_.max_scale - settings_.min_scale;
        return settings_.max_scale - severity * scale_span;
    }

    [[nodiscard]] std::vector<float> scale_all(std::span<const ManifoldWarmstartSample> samples) const
    {
        std::vector<float> result;
        result.reserve(samples.size());
        for (const ManifoldWarmstartSample& sample : samples)
        {
            result.push_back(scale_for(sample));
        }
        return result;
    }

private:
    Settings settings_{};
};
} // namespace admc::solver
