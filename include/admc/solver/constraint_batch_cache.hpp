#pragma once

#include "admc/solver/constraint_batch.hpp"
#include "admc/solver/constraint_batch_builder.hpp"
#include "admc/solver/simple_pgs.hpp"

#include <optional>
#include <unordered_map>
#include <vector>

namespace admc::solver
{
class ConstraintBatchCache
{
public:
    struct IslandCache
    {
        ConstraintBatch batch;
        std::vector<float> residuals;
    };

    explicit ConstraintBatchCache(std::size_t tile_capacity = 64) : builder_(tile_capacity) {}

    ConstraintBatch& rebuild(std::size_t island_index, const IslandState& island)
    {
        ensure_cache_size(island_index + 1);
        IslandCache& cache = caches_[island_index];

        const std::size_t new_rows = island.constraints.size();
        bool can_fast_update = false;
        if (!cache.batch.islands.empty())
        {
            // Check quick signature: same row count and first/last keys match
            std::size_t old_rows = total_rows(cache.batch);
            if (old_rows == new_rows && new_rows > 0)
            {
                const ConstraintKey& first_old = cache.batch.islands[0].tiles.front().rows.keys.front();
                const ConstraintKey& last_old = cache.batch.islands[0].tiles.back().rows.keys.back();
                const ConstraintKey& first_new = island.constraints.front().key;
                const ConstraintKey& last_new = island.constraints.back().key;
                can_fast_update = (first_old == first_new) && (last_old == last_new);
            }
        }

        if (!can_fast_update)
        {
            // Capture previous rows only when we need to rebuild structure
            RowMap previous_rows;
            if (!cache.batch.islands.empty())
            {
                collect_row_state(cache.batch, cache.residuals, previous_rows);
            }

            // Rebuild in place to reuse tile storage and minimize allocations
            builder_.build_in_place(island, cache.batch);

            const std::size_t rows = total_rows(cache.batch);
            cache.residuals.assign(rows, 0.0f);

            if (!previous_rows.empty())
            {
                std::size_t row_index = 0;
                for (ConstraintIsland& island_tiles : cache.batch.islands)
                {
                    for (ConstraintTile& tile : island_tiles.tiles)
                    {
                        const std::size_t tile_rows = tile.rows.effective_mass.size();
                        for (std::size_t r = 0; r < tile_rows; ++r, ++row_index)
                        {
                            const auto it = previous_rows.find(tile.rows.keys[r]);
                            if (it != previous_rows.end())
                            {
                                tile.rows.lambda[r] = it->second.lambda;
                                cache.residuals[row_index] = it->second.residual;
                            }
                        }
                    }
                }
            }
        }
        else
        {
            // Fast update: rewrite in-place numeric row fields and body inverse mass/inertia
            // Keep residuals vector as-is to avoid rehydration cost
            std::size_t idx = 0;
            for (ConstraintIsland& isl : cache.batch.islands)
            {
                for (ConstraintTile& tile : isl.tiles)
                {
                    // Track which local bodies changed inertia/mass
                    std::vector<char> changed(tile.bodies.size(), 0);
                    bool any_body_changed = false;
                    auto mat_equal = [](const ::admc::core::Mat3& a, const ::admc::core::Mat3& b) noexcept {
                        return (a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]);
                    };
                    for (std::size_t bi = 0; bi < tile.bodies.size(); ++bi)
                    {
                        TileBody& tb = tile.bodies[bi];
                        if (tb.global_index < island.bodies.size())
                        {
                            const float new_inv_m = island.bodies[tb.global_index].inverse_mass;
                            const ::admc::core::Mat3 new_inv_I = island.bodies[tb.global_index].inverse_inertia;
                            if (new_inv_m != tb.inverse_mass || !mat_equal(new_inv_I, tb.inverse_inertia))
                            {
                                tb.inverse_mass = new_inv_m;
                                tb.inverse_inertia = new_inv_I;
                                changed[bi] = 1;
                                any_body_changed = true;
                            }
                        }
                    }

                    // If nothing changed, keep existing rows/ang_effects
                    const std::size_t rows = tile.rows.effective_mass.size();
                    if (!any_body_changed)
                    {
                        idx += rows;
                        continue;
                    }

                    for (std::size_t r = 0; r < rows; ++r, ++idx)
                    {
                        const ConstraintInstance& c = island.constraints[idx];
                        tile.rows.jacobian_linear_a[r] = c.row.jacobian_linear_a;
                        tile.rows.jacobian_linear_b[r] = c.row.jacobian_linear_b;
                        tile.rows.jacobian_angular_a[r] = c.row.jacobian_angular_a;
                        tile.rows.jacobian_angular_b[r] = c.row.jacobian_angular_b;
                        tile.rows.effective_mass[r] = c.row.effective_mass;
                        tile.rows.bias[r] = c.row.bias;
                        tile.rows.lower_limit[r] = c.row.lower_limit;
                        tile.rows.upper_limit[r] = c.row.upper_limit;
                        tile.rows.types[r] = c.row.type;
                        // Recompute angular effects only for bodies that changed
                        const std::uint16_t la = (r < tile.rows.body_a.size()) ? tile.rows.body_a[r] : 0;
                        const std::uint16_t lb = (r < tile.rows.body_b.size()) ? tile.rows.body_b[r] : 0;
                        if (la < tile.bodies.size() && changed[la])
                        {
                            tile.rows.ang_effect_a[r] = tile.bodies[la].inverse_inertia * tile.rows.jacobian_angular_a[r];
                        }
                        if (lb < tile.bodies.size() && changed[lb])
                        {
                            tile.rows.ang_effect_b[r] = tile.bodies[lb].inverse_inertia * tile.rows.jacobian_angular_b[r];
                        }
                    }
                }
            }
        }

        return cache.batch;
    }

    IslandCache& get(std::size_t island_index)
    {
        ensure_cache_size(island_index + 1);
        return caches_[island_index];
    }

    std::vector<float>& residuals(std::size_t island_index)
    {
        ensure_cache_size(island_index + 1);
        return caches_[island_index].residuals;
    }

private:
    static std::size_t total_rows(const ConstraintBatch& batch)
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

    void ensure_cache_size(std::size_t size)
    {
        if (caches_.size() < size)
        {
            caches_.resize(size);
        }
    }

    struct RowState
    {
        float lambda{0.0f};
        float residual{0.0f};
    };
    using RowMap = std::unordered_map<ConstraintKey, RowState, ConstraintKeyHasher>;

    static void collect_row_state(
        const ConstraintBatch& batch,
        const std::vector<float>& residuals,
        RowMap& out_map)
    {
        out_map.clear();
        const std::size_t total = total_rows(batch);
        if (total > 0)
        {
            out_map.reserve(total);
        }
        std::size_t row_index = 0;
        for (const ConstraintIsland& island : batch.islands)
        {
            for (const ConstraintTile& tile : island.tiles)
            {
                const std::size_t rows = tile.rows.effective_mass.size();
                for (std::size_t r = 0; r < rows; ++r, ++row_index)
                {
                    RowState state{
                        .lambda = tile.rows.lambda[r],
                        .residual = (row_index < residuals.size()) ? residuals[row_index] : 0.0f};
                    out_map[tile.rows.keys[r]] = state;
                }
            }
        }
    }

    ConstraintBatchBuilder builder_;
    std::vector<IslandCache> caches_;
};
} // namespace admc::solver
