#pragma once

#include "admc/solver/iteration_control.hpp"

#include <cstddef>
#include <optional>
#include <span>

namespace admc::solver
{
using IslandId = std::size_t;

class IConservationTracker
{
public:
    virtual ~IConservationTracker() = default;

    virtual void on_island_begin(IslandId /*island*/) {}
    virtual float warmstart_scale(IslandId /*island*/) const noexcept { return 1.0f; }
    virtual IterationPolicy on_iteration_end(IslandId /*island*/, const IterationSignals& signals) = 0;
    virtual void on_island_end(IslandId /*island*/, const IterationSignals& /*last_signals*/) {}

    // Optional: per-iteration, solver can provide per-tile residuals and drift norms.
    // Trackers may return a tile index to focus (e.g., for extra passes or reordering).
    virtual std::optional<std::size_t> select_focus_tile(
        IslandId /*island*/, std::span<const float> /*tile_residuals*/, std::span<const float> /*tile_drifts*/) { return std::nullopt; }
};

class NullConservationTracker final : public IConservationTracker
{
public:
    IterationPolicy on_iteration_end(IslandId, const IterationSignals&) override
    {
        return IterationPolicy{};
    }
};

class AdaptiveConservationTracker final : public IConservationTracker
{
public:
    explicit AdaptiveConservationTracker(const CompositeIterationGate::Settings& settings) noexcept
        : gate_(settings)
    {
    }

    IterationPolicy on_iteration_end(IslandId, const IterationSignals& signals) override
    {
        return gate_.evaluate(signals);
    }

    std::optional<std::size_t> select_focus_tile(
        IslandId, std::span<const float> tile_residuals, std::span<const float> tile_drifts) override
    {
        // Simple policy: choose tile with max residual if it exceeds residual threshold.
        if (tile_residuals.empty()) return std::nullopt;
        std::size_t best = 0;
        float best_val = tile_residuals[0];
        for (std::size_t i = 1; i < tile_residuals.size(); ++i)
        {
            if (tile_residuals[i] > best_val)
            {
                best_val = tile_residuals[i];
                best = i;
            }
        }
        if (best_val > gate_.settings().thresholds.residual)
        {
            (void)tile_drifts; // reserved for future use
            return best;
        }
        return std::nullopt;
    }

private:
    CompositeIterationGate gate_;
};
} // namespace admc::solver
