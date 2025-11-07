#pragma once

#include "admc/admc/channels.hpp"
#include "admc/constraints/constraint_row.hpp"
#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"
#include "admc/solver/conservation_tracker.hpp"
#include "admc/solver/constraint_batch.hpp"
#include "admc/solver/constraint_key.hpp"
#include "admc/solver/iteration_control.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace admc::solver
{
class TilePGSSolver
{
public:
    struct MomentumSample
    {
        float mass{0.0f};
        ::admc::core::Vec3 momentum{};
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

    struct Options
    {
        int max_iterations{8};
        CompositeIterationGate::Settings gate_settings{};
        ManifoldWarmstartScaler::Settings warmstart_settings{};
    };

    TilePGSSolver() noexcept : TilePGSSolver(Options{}) {}
    explicit TilePGSSolver(const Options& options) noexcept
        : options_(options),
          gate_(options.gate_settings),
          scaler_(options.warmstart_settings)
    {
    }

    SolverResult solve(
        IslandState& island,
        ConstraintBatch& batch,
        std::vector<float>& residuals,
        IConservationTracker* tracker = nullptr) const noexcept
    {
        SolverResult result{};
        if (batch.islands.empty())
        {
            return result;
        }

        ::admc::admc::DirectionalTracker momentum_tracker;
        const MomentumSample pre = sample_momentum(island);
        momentum_tracker.record_pre(pre.mass, pre.momentum);

        const auto warmstart_start = std::chrono::steady_clock::now();
        if (tracker)
        {
            tracker->on_island_begin(0);
        }
        const float island_scale = tracker ? tracker->warmstart_scale(0) : 1.0f;
        warmstart(batch, island, island_scale, residuals);
        const auto warmstart_end = std::chrono::steady_clock::now();
        result.warmstart_ms = std::chrono::duration<double, std::milli>(warmstart_end - warmstart_start).count();

        const auto iteration_start = std::chrono::steady_clock::now();
        IterationSignals last_signals{};

        const std::size_t total_row_count = total_rows(batch);
        residuals.assign(total_row_count, 0.0f);

        for (int iter = 0; iter < options_.max_iterations; ++iter)
        {
            float max_residual = 0.0f;
            float max_penetration = 0.0f;
            float max_joint = 0.0f;

            std::size_t row_cursor = 0;

            for (ConstraintIsland& island_tiles : batch.islands)
            {
                for (ConstraintTile& tile : island_tiles.tiles)
                    {
                        const std::vector<SolverBody*> body_views = make_body_views(tile, island);
                        const std::size_t rows = tile.rows.effective_mass.size();
                        for (std::size_t row = 0; row < rows; ++row, ++row_cursor)
                        {
                            SolverBody* bodyA = resolve_body(tile, body_views, row, true);
                            SolverBody* bodyB = resolve_body(tile, body_views, row, false);
                            if (!bodyA || !bodyB)
                            {
                                continue;
                            }

                            const float relative_before = compute_relative_velocity(tile.rows, row, *bodyA, *bodyB);
                            const float rhs = -tile.rows.bias[row] - relative_before;
                            const float delta_lambda = rhs * tile.rows.effective_mass[row];
                            const float new_lambda = std::clamp(
                                tile.rows.lambda[row] + delta_lambda,
                                tile.rows.lower_limit[row],
                                tile.rows.upper_limit[row]);
                            const float applied = new_lambda - tile.rows.lambda[row];
                            tile.rows.lambda[row] = new_lambda;

                            if (std::fabs(applied) > 0.0f)
                            {
                                apply_velocity_delta(tile.rows, row, applied, *bodyA, *bodyB);
                            }

                            max_residual = std::max(max_residual, std::fabs(rhs));
                            const float corrected = -tile.rows.bias[row] - compute_relative_velocity(tile.rows, row, *bodyA, *bodyB);

                            if (tile.rows.types[row] == ::admc::constraints::ConstraintType::ContactNormal)
                            {
                                max_penetration = std::max(max_penetration, std::fabs(corrected));
                            }
                            else if (tile.rows.types[row] == ::admc::constraints::ConstraintType::Joint)
                            {
                                max_joint = std::max(max_joint, std::fabs(corrected));
                            }

                            const float abs_corrected = std::fabs(corrected);
                            max_residual = std::max(max_residual, abs_corrected);
                            if (row_cursor < residuals.size())
                            {
                                residuals[row_cursor] = abs_corrected;
                            }
                        }
                    }
                }

            momentum_tracker.record_post(pre.mass, sample_momentum(island).momentum);
            const DriftSample drift = make_drift_sample(momentum_tracker);

            IterationSignals signals{
                .constraint_residual = max_residual,
                .penetration_error = max_penetration,
                .joint_error = max_joint,
                .admc_drift = drift.momentum_norm};

            last_signals = signals;
            result.iterations = iter + 1;
            result.max_residual = max_residual;
            result.max_penetration_error = max_penetration;
            result.max_joint_error = max_joint;
            result.admc_drift = drift.momentum_norm;

            const IterationPolicy policy = tracker ? tracker->on_iteration_end(0, signals) : gate_.evaluate(signals);
            if (!policy.should_continue)
            {
                break;
            }
        }

        const auto iteration_end = std::chrono::steady_clock::now();
        result.iteration_ms = std::chrono::duration<double, std::milli>(iteration_end - iteration_start).count();
        result.total_ms = result.warmstart_ms + result.iteration_ms;

        if (tracker)
        {
            tracker->on_island_end(0, last_signals);
        }

        return result;
    }

private:
    static MomentumSample sample_momentum(const IslandState& island) noexcept
    {
        MomentumSample sample{};
        for (const SolverBody& body : island.bodies)
        {
            if (body.inverse_mass <= 0.0f)
            {
                continue;
            }
            const float mass = 1.0f / body.inverse_mass;
            sample.mass += mass;
            sample.momentum += body.linear_velocity * mass;
        }
        if (sample.mass <= 0.0f)
        {
            sample.mass = 1.0f;
        }
        return sample;
    }

    static float compute_relative_velocity(
        const TileRowSoA& rows,
        std::size_t row,
        const SolverBody& bodyA,
        const SolverBody& bodyB) noexcept
    {
        const float termA = ::admc::core::dot(rows.jacobian_linear_a[row], bodyA.linear_velocity) +
                            ::admc::core::dot(rows.jacobian_angular_a[row], bodyA.angular_velocity);
        const float termB = ::admc::core::dot(rows.jacobian_linear_b[row], bodyB.linear_velocity) +
                            ::admc::core::dot(rows.jacobian_angular_b[row], bodyB.angular_velocity);
        return termA + termB;
    }

    static void apply_velocity_delta(
        const TileRowSoA& rows,
        std::size_t row,
        float applied,
        SolverBody& bodyA,
        SolverBody& bodyB) noexcept
    {
        const ::admc::core::Vec3 linA = rows.jacobian_linear_a[row] * applied;
        const ::admc::core::Vec3 angA = rows.jacobian_angular_a[row] * applied;
        const ::admc::core::Vec3 linB = rows.jacobian_linear_b[row] * applied;
        const ::admc::core::Vec3 angB = rows.jacobian_angular_b[row] * applied;

        if (bodyA.inverse_mass > 0.0f)
        {
            bodyA.linear_velocity += linA * bodyA.inverse_mass;
            bodyA.angular_velocity += bodyA.inverse_inertia * angA;
        }
        if (bodyB.inverse_mass > 0.0f)
        {
            bodyB.linear_velocity += linB * bodyB.inverse_mass;
            bodyB.angular_velocity += bodyB.inverse_inertia * angB;
        }
    }

    static std::size_t total_rows(const ConstraintBatch& batch) noexcept
    {
        std::size_t rows = 0;
        for (const ConstraintIsland& island : batch.islands)
        {
            for (const ConstraintTile& tile : island.tiles)
            {
                rows += tile.rows.effective_mass.size();
            }
        }
        return rows;
    }

    void warmstart(
        ConstraintBatch& batch,
        IslandState& island,
        float island_scale,
        const std::vector<float>& residuals) const noexcept
    {
        std::size_t row_cursor = 0;
        for (ConstraintIsland& island_tiles : batch.islands)
        {
            for (ConstraintTile& tile : island_tiles.tiles)
            {
                const std::vector<SolverBody*> body_views = make_body_views(tile, island);
                const std::size_t rows = tile.rows.effective_mass.size();
                for (std::size_t row = 0; row < rows; ++row)
                {
                    const float cached_residual = (row_cursor < residuals.size()) ? residuals[row_cursor] : std::fabs(tile.rows.bias[row]);
                    const ManifoldWarmstartSample sample{
                        .residual_norm = cached_residual,
                        .admc_drift = 0.0f,
                        .impulse_magnitude = std::fabs(tile.rows.lambda[row])};
                    const float scale = scaler_.scale_for(sample) * island_scale;
                    const float scaled_lambda = tile.rows.lambda[row] * scale;
                    tile.rows.lambda[row] = scaled_lambda;

                    if (scaled_lambda == 0.0f)
                    {
                        continue;
                    }

                    if (row >= tile.rows.body_a.size() || row >= tile.rows.body_b.size())
                    {
                        continue;
                    }
                    SolverBody* bodyA = resolve_body(tile, body_views, row, true);
                    SolverBody* bodyB = resolve_body(tile, body_views, row, false);
                    if (!bodyA || !bodyB)
                    {
                        continue;
                    }
                    apply_velocity_delta(tile.rows, row, scaled_lambda, *bodyA, *bodyB);
                    ++row_cursor;
                }
            }
        }
    }

    static std::vector<SolverBody*> make_body_views(const ConstraintTile& tile, IslandState& island)
    {
        std::vector<SolverBody*> views(tile.bodies.size(), nullptr);
        for (std::size_t i = 0; i < tile.bodies.size(); ++i)
        {
            const std::uint32_t global = tile.bodies[i].global_index;
            if (global < island.bodies.size())
            {
                views[i] = &island.bodies[global];
            }
        }
        return views;
    }

    static SolverBody* resolve_body(
        const ConstraintTile& tile,
        const std::vector<SolverBody*>& body_views,
        std::size_t row,
        bool first) noexcept
    {
        const std::vector<std::uint16_t>& indices = first ? tile.rows.body_a : tile.rows.body_b;
        if (row >= indices.size())
        {
            return nullptr;
        }
        const std::uint16_t local = indices[row];
        if (local >= body_views.size())
        {
            return nullptr;
        }
        return body_views[local];
    }

    Options options_{};
    CompositeIterationGate gate_;
    ManifoldWarmstartScaler scaler_;
};
} // namespace admc::solver
