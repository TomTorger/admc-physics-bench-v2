#pragma once

#include "admc/solver/constraint_batch.hpp"
#include "admc/solver/simple_pgs.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace admc::solver
{
class ConstraintBatchBuilder
{
public:
    explicit ConstraintBatchBuilder(std::size_t tile_capacity = 64) noexcept : tile_capacity_(tile_capacity) {}

    void build(const IslandState& island, ConstraintBatch& batch) const
    {
        batch.islands.clear();
        if (island.constraints.empty())
        {
            return;
        }

        batch.islands.emplace_back();
        ConstraintIsland& island_batch = batch.islands.back();
        island_batch.tiles.clear();
        island_batch.tiles.reserve((island.constraints.size() + tile_capacity_ - 1) / tile_capacity_);

        ConstraintTile tile;
        reserve_tile(tile);

        const std::uint16_t kInvalid = std::numeric_limits<std::uint16_t>::max();
        std::vector<std::uint16_t> body_remap(island.bodies.size(), kInvalid);
        std::vector<std::uint32_t> remap_list;
        remap_list.reserve(island.bodies.size());

        auto reset_remap = [&]() {
            for (std::uint32_t idx : remap_list)
            {
                body_remap[idx] = kInvalid;
            }
            remap_list.clear();
        };

        auto flush_tile = [&]() {
            if (tile.rows.effective_mass.empty())
            {
                return;
            }
            island_batch.tiles.push_back(std::move(tile));
            tile = ConstraintTile{};
            reserve_tile(tile);
            reset_remap();
        };

        auto get_local_index = [&](std::size_t global_index) -> std::uint16_t {
            if (global_index >= body_remap.size())
            {
                return 0;
            }
            std::uint16_t local = body_remap[global_index];
            if (local != kInvalid)
            {
                return local;
            }
            local = static_cast<std::uint16_t>(tile.bodies.size());
            body_remap[global_index] = local;
            remap_list.push_back(static_cast<std::uint32_t>(global_index));
            tile.bodies.push_back(TileBody{
                static_cast<std::uint32_t>(global_index),
                island.bodies[global_index].inverse_mass,
                island.bodies[global_index].inverse_inertia});
            return local;
        };

        const auto push_row = [&](const ConstraintInstance& constraint) {
            tile.rows.jacobian_linear_a.push_back(constraint.row.jacobian_linear_a);
            tile.rows.jacobian_linear_b.push_back(constraint.row.jacobian_linear_b);
            tile.rows.jacobian_angular_a.push_back(constraint.row.jacobian_angular_a);
            tile.rows.jacobian_angular_b.push_back(constraint.row.jacobian_angular_b);
            tile.rows.effective_mass.push_back(constraint.row.effective_mass);
            tile.rows.bias.push_back(constraint.row.bias);
            tile.rows.lambda.push_back(constraint.row.lambda);
            tile.rows.lower_limit.push_back(constraint.row.lower_limit);
            tile.rows.upper_limit.push_back(constraint.row.upper_limit);
            tile.rows.types.push_back(constraint.row.type);
            tile.rows.keys.push_back(constraint.key);
        };

        for (const ConstraintInstance& constraint : island.constraints)
        {
            if (tile.rows.effective_mass.size() == tile_capacity_)
            {
                flush_tile();
            }

            const std::uint16_t local_a = get_local_index(constraint.body_a);
            const std::uint16_t local_b = get_local_index(constraint.body_b);

            push_row(constraint);
            tile.rows.body_a.push_back(local_a);
            tile.rows.body_b.push_back(local_b);
            const ::admc::core::Mat3& invIA = tile.bodies[local_a].inverse_inertia;
            const ::admc::core::Mat3& invIB = tile.bodies[local_b].inverse_inertia;
            tile.rows.ang_effect_a.push_back(invIA * constraint.row.jacobian_angular_a);
            tile.rows.ang_effect_b.push_back(invIB * constraint.row.jacobian_angular_b);
        }

        flush_tile();
    }

    // Incremental builder: reuse existing tile storage when possible by clearing in place
    void build_in_place(const IslandState& island, ConstraintBatch& batch) const
    {
        if (island.constraints.empty())
        {
            batch.islands.clear();
            return;
        }

        if (batch.islands.empty())
        {
            batch.islands.emplace_back();
        }
        ConstraintIsland& island_batch = batch.islands[0];

        const std::size_t total_constraints = island.constraints.size();
        const std::size_t needed_tiles = (total_constraints + tile_capacity_ - 1) / tile_capacity_;
        island_batch.tiles.resize(needed_tiles);

        // Prepare tiles (clear vectors, keep capacity). Row vector sizes set below per tile.
        for (ConstraintTile& t : island_batch.tiles) { clear_tile(t); reserve_tile(t); }

        const std::uint16_t kInvalid = std::numeric_limits<std::uint16_t>::max();
        std::vector<std::uint16_t> body_remap(island.bodies.size(), kInvalid);
        std::vector<std::uint32_t> remap_list;
        remap_list.reserve(island.bodies.size());

        auto reset_remap = [&]() {
            for (std::uint32_t idx : remap_list)
            {
                body_remap[idx] = kInvalid;
            }
            remap_list.clear();
        };

        auto get_local_index = [&](ConstraintTile& tile, std::size_t global_index) -> std::uint16_t {
            if (global_index >= body_remap.size())
            {
                return 0;
            }
            std::uint16_t local = body_remap[global_index];
            if (local != kInvalid)
            {
                return local;
            }
            local = static_cast<std::uint16_t>(tile.bodies.size());
            body_remap[global_index] = local;
            remap_list.push_back(static_cast<std::uint32_t>(global_index));
            tile.bodies.push_back(TileBody{
                static_cast<std::uint32_t>(global_index),
                island.bodies[global_index].inverse_mass,
                island.bodies[global_index].inverse_inertia});
            return local;
        };

        // Index-fill per tile to avoid push_back overhead in hot loops
        for (std::size_t t = 0; t < needed_tiles; ++t)
        {
            reset_remap();
            ConstraintTile& tile = island_batch.tiles[t];
            const std::size_t start = t * tile_capacity_;
            const std::size_t rows = std::min(tile_capacity_, total_constraints - start);

            // Resize row arrays once per tile
            tile.rows.jacobian_linear_a.resize(rows);
            tile.rows.jacobian_linear_b.resize(rows);
            tile.rows.jacobian_angular_a.resize(rows);
            tile.rows.jacobian_angular_b.resize(rows);
            tile.rows.effective_mass.resize(rows);
            tile.rows.bias.resize(rows);
            tile.rows.lambda.resize(rows, 0.0f);
            tile.rows.lower_limit.resize(rows);
            tile.rows.upper_limit.resize(rows);
            tile.rows.types.resize(rows);
            tile.rows.keys.resize(rows);
            tile.rows.body_a.resize(rows);
            tile.rows.body_b.resize(rows);
            tile.rows.ang_effect_a.resize(rows);
            tile.rows.ang_effect_b.resize(rows);

            for (std::size_t i = 0; i < rows; ++i)
            {
                const ConstraintInstance& constraint = island.constraints[start + i];
                const std::uint16_t local_a = get_local_index(tile, constraint.body_a);
                const std::uint16_t local_b = get_local_index(tile, constraint.body_b);

                tile.rows.jacobian_linear_a[i] = constraint.row.jacobian_linear_a;
                tile.rows.jacobian_linear_b[i] = constraint.row.jacobian_linear_b;
                tile.rows.jacobian_angular_a[i] = constraint.row.jacobian_angular_a;
                tile.rows.jacobian_angular_b[i] = constraint.row.jacobian_angular_b;
                tile.rows.effective_mass[i] = constraint.row.effective_mass;
                tile.rows.bias[i] = constraint.row.bias;
                tile.rows.lower_limit[i] = constraint.row.lower_limit;
                tile.rows.upper_limit[i] = constraint.row.upper_limit;
                tile.rows.types[i] = constraint.row.type;
                tile.rows.keys[i] = constraint.key;
                tile.rows.body_a[i] = local_a;
                tile.rows.body_b[i] = local_b;

                const ::admc::core::Mat3& invIA = tile.bodies[local_a].inverse_inertia;
                const ::admc::core::Mat3& invIB = tile.bodies[local_b].inverse_inertia;
                tile.rows.ang_effect_a[i] = invIA * constraint.row.jacobian_angular_a;
                tile.rows.ang_effect_b[i] = invIB * constraint.row.jacobian_angular_b;
            }
        }
    }

private:
    static void clear_tile(ConstraintTile& tile) noexcept
    {
        tile.bodies.clear();
        tile.rows.jacobian_linear_a.clear();
        tile.rows.jacobian_linear_b.clear();
        tile.rows.jacobian_angular_a.clear();
        tile.rows.jacobian_angular_b.clear();
        tile.rows.effective_mass.clear();
        tile.rows.bias.clear();
        tile.rows.lambda.clear();
        tile.rows.lower_limit.clear();
        tile.rows.upper_limit.clear();
        tile.rows.body_a.clear();
        tile.rows.body_b.clear();
        tile.rows.types.clear();
        tile.rows.keys.clear();
    }

    void reserve_tile(ConstraintTile& tile) const
    {
        // Worst-case each row introduces two new bodies; reserve accordingly to avoid reallocations.
        tile.bodies.reserve(tile_capacity_ * 2);
        tile.rows.jacobian_linear_a.reserve(tile_capacity_);
        tile.rows.jacobian_linear_b.reserve(tile_capacity_);
        tile.rows.jacobian_angular_a.reserve(tile_capacity_);
        tile.rows.jacobian_angular_b.reserve(tile_capacity_);
        tile.rows.effective_mass.reserve(tile_capacity_);
        tile.rows.bias.reserve(tile_capacity_);
        tile.rows.lambda.reserve(tile_capacity_);
        tile.rows.lower_limit.reserve(tile_capacity_);
        tile.rows.upper_limit.reserve(tile_capacity_);
        tile.rows.body_a.reserve(tile_capacity_);
        tile.rows.body_b.reserve(tile_capacity_);
        tile.rows.types.reserve(tile_capacity_);
        tile.rows.keys.reserve(tile_capacity_);
    }

    std::size_t tile_capacity_{32};
};
} // namespace admc::solver
