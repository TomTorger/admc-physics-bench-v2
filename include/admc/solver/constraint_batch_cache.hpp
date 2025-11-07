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

        RowMap previous_rows;
        collect_row_state(cache.batch, cache.residuals, previous_rows);

        builder_.build(island, cache.batch);
        const std::size_t rows = total_rows(cache.batch);
        cache.residuals.assign(rows, 0.0f);

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
