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
            island_batch.tiles.push_back(tile);
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
        }

        flush_tile();
    }

private:
    void reserve_tile(ConstraintTile& tile) const
    {
        tile.bodies.reserve(tile_capacity_);
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
