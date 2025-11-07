#pragma once

#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"
#include "admc/constraints/constraint_row.hpp"
#include "admc/solver/constraint_key.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace admc::solver
{
struct TileBody
{
    std::uint32_t global_index{0};
    float inverse_mass{0.0f};
    ::admc::core::Mat3 inverse_inertia{};
};

struct TileRowSoA
{
    std::vector<::admc::core::Vec3> jacobian_linear_a;
    std::vector<::admc::core::Vec3> jacobian_linear_b;
    std::vector<::admc::core::Vec3> jacobian_angular_a;
    std::vector<::admc::core::Vec3> jacobian_angular_b;
    std::vector<float> effective_mass;
    std::vector<float> bias;
    std::vector<float> lambda;
    std::vector<float> lower_limit;
    std::vector<float> upper_limit;
    std::vector<std::uint16_t> body_a;
    std::vector<std::uint16_t> body_b;
    std::vector<::admc::constraints::ConstraintType> types;
    std::vector<ConstraintKey> keys;
};

struct ConstraintTile
{
    std::vector<TileBody> bodies;
    TileRowSoA rows;
};

struct ConstraintIsland
{
    std::vector<ConstraintTile> tiles;
};

struct ConstraintBatch
{
    std::vector<ConstraintIsland> islands;
};
} // namespace admc::solver
