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
#include <span>

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
        // Per-tile summaries (from last iteration):
        float tile_residual_min{0.0f};
        float tile_residual_max{0.0f};
        float tile_residual_p95{0.0f};
        float tile_drift_min{0.0f};
        float tile_drift_max{0.0f};
        float tile_drift_p95{0.0f};
    };

    struct Options
    {
        int max_iterations{8};
        CompositeIterationGate::Settings gate_settings{};
        ManifoldWarmstartScaler::Settings warmstart_settings{};
        bool collect_tile_diagnostics{true};
        bool tile_diagnostics_last_iteration_only{true};
        float sor{1.0f};
        // Normalization velocity scale for ADMC drift (e.g., g * dt)
        float drift_velocity_scale{9.81f / 60.0f};
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

        // Pre-allocate per-tile diagnostics
        const std::size_t total_tiles = count_tiles(batch);
        std::vector<float> tile_residuals(options_.collect_tile_diagnostics ? total_tiles : 0);
        std::vector<float> tile_drifts(options_.collect_tile_diagnostics ? total_tiles : 0);

        // Reusable scratch for body views to avoid per-tile allocations inside the iteration loop
        std::vector<SolverBody*> body_views_scratch;
        body_views_scratch.reserve(max_tile_bodies(batch));

        float sor_value = options_.sor;
        float prev_residual = std::numeric_limits<float>::infinity();
        for (int iter = 0; iter < options_.max_iterations; ++iter)
        {
            float max_residual = 0.0f;
            float max_penetration = 0.0f;
            float max_joint = 0.0f;

            std::size_t row_cursor = 0;
            std::size_t tile_index = 0;

            for (ConstraintIsland& island_tiles : batch.islands)
            {
                for (ConstraintTile& tile : island_tiles.tiles)
                    {
                        const bool do_tile_diag = options_.collect_tile_diagnostics &&
                            (!options_.tile_diagnostics_last_iteration_only || (iter + 1 == options_.max_iterations));
                        MomentumSample tile_pre{};
                        if (do_tile_diag)
                        {
                            tile_pre = sample_tile_momentum(tile, island);
                        }
                        float tile_max_residual = 0.0f;
                        fill_body_views(tile, island, body_views_scratch);
                        const std::size_t rows = tile.rows.effective_mass.size();
                        for (std::size_t row = 0; row < rows; ++row, ++row_cursor)
                        {
                            SolverBody* bodyA = resolve_body(tile, body_views_scratch, row, true);
                            SolverBody* bodyB = resolve_body(tile, body_views_scratch, row, false);
                            if (!bodyA || !bodyB)
                            {
                                continue;
                            }

                            const float relative_before = compute_relative_velocity(tile.rows, row, *bodyA, *bodyB);
                            const float rhs = -tile.rows.bias[row] - relative_before;
                            if (std::fabs(rhs) < 1.0e-12f)
                            {
                                if (row_cursor < residuals.size())
                                {
                                    residuals[row_cursor] = std::fabs(rhs);
                                }
                                continue;
                            }
                            float delta_lambda = rhs * tile.rows.effective_mass[row];
                            // Apply SOR only when we expect more than one iteration; avoid overshoot for single-iteration runs
                            if (sor_value != 1.0f && (options_.max_iterations > 1 || iter > 0))
                            {
                                delta_lambda *= sor_value;
                            }
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
                            // Compute corrected residual without re-evaluating Jv: corrected = rhs - applied / m_eff
                            const float inv_meff = (tile.rows.effective_mass[row] != 0.0f) ? (1.0f / tile.rows.effective_mass[row]) : 0.0f;
                            const float corrected = rhs - applied * inv_meff;

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
                            tile_max_residual = std::max(tile_max_residual, abs_corrected);
                            if (row_cursor < residuals.size())
                            {
                                residuals[row_cursor] = abs_corrected;
                            }
                        }
                        if (do_tile_diag)
                        {
                            const MomentumSample tile_post = sample_tile_momentum(tile, island);
                            const ::admc::core::Vec3 dP = tile_post.momentum - tile_pre.momentum;
                            const float denom_tile = std::max(tile_pre.mass * options_.drift_velocity_scale, 1.0e-9f);
                            const float drift_norm = ::admc::core::length(dP) / denom_tile;
                            if (tile_index < tile_residuals.size()) tile_residuals[tile_index] = tile_max_residual;
                            if (tile_index < tile_drifts.size()) tile_drifts[tile_index] = drift_norm;
                        }
                        ++tile_index;
                    }
                }

            momentum_tracker.record_post(pre.mass, sample_momentum(island).momentum);
            const DriftSample drift = make_drift_sample(momentum_tracker);
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

            // Offer per-tile diagnostics to tracker (optional focus selection)
            if (tracker && options_.collect_tile_diagnostics)
            {
                (void)tracker->select_focus_tile(0, tile_residuals, tile_drifts);
            }

            // Dynamic SOR controller (conservative)
            if (options_.max_iterations > 1)
            {
                if (signals.admc_drift > gate_.settings().thresholds.admc * 1.5f)
                {
                    sor_value = std::max(1.0f, sor_value * 0.85f);
                }
                else if (prev_residual < std::numeric_limits<float>::infinity())
                {
                    if (max_residual < prev_residual * 0.85f)
                    {
                        sor_value = std::min(1.6f, sor_value * 1.05f);
                    }
                    else if (max_residual > prev_residual * 1.05f)
                    {
                        sor_value = std::max(1.0f, sor_value * 0.95f);
                    }
                }
                prev_residual = max_residual;
            }

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

        // Finalize per-tile summary statistics from last iteration
        if (options_.collect_tile_diagnostics && !tile_residuals.empty())
        {
            result.tile_residual_min = *std::min_element(tile_residuals.begin(), tile_residuals.end());
            result.tile_residual_max = *std::max_element(tile_residuals.begin(), tile_residuals.end());
            result.tile_residual_p95 = percentile(std::span<const float>(tile_residuals.data(), tile_residuals.size()), 0.95f);
        }
        if (options_.collect_tile_diagnostics && !tile_drifts.empty())
        {
            result.tile_drift_min = *std::min_element(tile_drifts.begin(), tile_drifts.end());
            result.tile_drift_max = *std::max_element(tile_drifts.begin(), tile_drifts.end());
            result.tile_drift_p95 = percentile(std::span<const float>(tile_drifts.data(), tile_drifts.size()), 0.95f);
        }

        return result;
    }

private:
    static float percentile(std::span<const float> values, float p) noexcept
    {
        if (values.empty()) return 0.0f;
        if (p <= 0.0f)
        {
            return *std::min_element(values.begin(), values.end());
        }
        if (p >= 1.0f)
        {
            return *std::max_element(values.begin(), values.end());
        }
        const std::size_t idx = static_cast<std::size_t>(std::floor(p * (values.size() - 1)));
        // Make a local copy for selection without disturbing the original order
        std::vector<float> tmp(values.begin(), values.end());
        std::nth_element(tmp.begin(), tmp.begin() + static_cast<std::ptrdiff_t>(idx), tmp.end());
        return tmp[idx];
    }

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

    static MomentumSample sample_tile_momentum(const ConstraintTile& tile, const IslandState& island) noexcept
    {
        MomentumSample sample{};
        for (const TileBody& tb : tile.bodies)
        {
            if (tb.global_index >= island.bodies.size()) continue;
            const SolverBody& body = island.bodies[tb.global_index];
            if (body.inverse_mass <= 0.0f) continue;
            const float mass = 1.0f / body.inverse_mass;
            sample.mass += mass;
            sample.momentum += body.linear_velocity * mass;
        }
        if (sample.mass <= 0.0f) sample.mass = 1.0f;
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
        const ::admc::core::Vec3 effAngA = rows.ang_effect_a[row] * applied;
        const ::admc::core::Vec3 linB = rows.jacobian_linear_b[row] * applied;
        const ::admc::core::Vec3 effAngB = rows.ang_effect_b[row] * applied;

        if (bodyA.inverse_mass > 0.0f)
        {
            bodyA.linear_velocity += linA * bodyA.inverse_mass;
            bodyA.angular_velocity += effAngA;
        }
        if (bodyB.inverse_mass > 0.0f)
        {
            bodyB.linear_velocity += linB * bodyB.inverse_mass;
            bodyB.angular_velocity += effAngB;
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

    static std::size_t count_tiles(const ConstraintBatch& batch) noexcept
    {
        std::size_t tiles = 0;
        for (const ConstraintIsland& island : batch.islands)
        {
            tiles += island.tiles.size();
        }
        return tiles;
    }

    static std::size_t max_tile_bodies(const ConstraintBatch& batch) noexcept
    {
        std::size_t maxb = 0;
        for (const ConstraintIsland& island : batch.islands)
        {
            for (const ConstraintTile& tile : island.tiles)
            {
                maxb = std::max<std::size_t>(maxb, tile.bodies.size());
            }
        }
        return maxb;
    }

    void warmstart(
        ConstraintBatch& batch,
        IslandState& island,
        float island_scale,
        const std::vector<float>& residuals) const noexcept
    {
        // Ensure residual index advances once per row regardless of early continues,
        // so cached residuals remain aligned with rows across frames.
        std::size_t row_cursor = 0;
        for (ConstraintIsland& island_tiles : batch.islands)
        {
            for (ConstraintTile& tile : island_tiles.tiles)
            {
                std::vector<SolverBody*> body_views;
                fill_body_views(tile, island, body_views);
                const std::size_t rows = tile.rows.effective_mass.size();
                for (std::size_t row = 0; row < rows; ++row)
                {
                    const std::size_t residual_index = row_cursor++;
                    const float cached_residual = (residual_index < residuals.size()) ? residuals[residual_index] : std::fabs(tile.rows.bias[row]);
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
                }
            }
        }
    }

    static void fill_body_views(const ConstraintTile& tile, IslandState& island, std::vector<SolverBody*>& out_views)
    {
        out_views.assign(tile.bodies.size(), nullptr);
        for (std::size_t i = 0; i < tile.bodies.size(); ++i)
        {
            const std::uint32_t global = tile.bodies[i].global_index;
            if (global < island.bodies.size())
            {
                out_views[i] = &island.bodies[global];
            }
        }
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
