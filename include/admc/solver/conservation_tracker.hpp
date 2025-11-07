#pragma once

#include "admc/solver/iteration_control.hpp"

#include <cstddef>

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

private:
    CompositeIterationGate gate_;
};
} // namespace admc::solver
