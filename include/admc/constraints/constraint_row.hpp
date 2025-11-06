#pragma once

#include "admc/core/vec3.hpp"

namespace admc::constraints
{
using Vec3 = ::admc::core::Vec3;

enum class ConstraintType
{
    ContactNormal,
    ContactFriction,
    Joint,
    Custom
};

struct ConstraintRow
{
    ConstraintType type{ConstraintType::ContactNormal};
    Vec3 jacobian_linear_a{};
    Vec3 jacobian_angular_a{};
    Vec3 jacobian_linear_b{};
    Vec3 jacobian_angular_b{};
    float effective_mass{0.0f};
    float bias{0.0f};
    float lambda{0.0f};
    float lower_limit{-1.0f};
    float upper_limit{1.0f};
};

inline void apply_impulse(
    ConstraintRow& row,
    Vec3& linear_velocity_a,
    Vec3& angular_velocity_a,
    Vec3& linear_velocity_b,
    Vec3& angular_velocity_b,
    float delta_lambda) noexcept
{
    linear_velocity_a += row.jacobian_linear_a * delta_lambda;
    angular_velocity_a += row.jacobian_angular_a * delta_lambda;
    linear_velocity_b += row.jacobian_linear_b * delta_lambda;
    angular_velocity_b += row.jacobian_angular_b * delta_lambda;
}

inline float compute_relative_velocity(
    const ConstraintRow& row,
    const Vec3& linear_velocity_a,
    const Vec3& angular_velocity_a,
    const Vec3& linear_velocity_b,
    const Vec3& angular_velocity_b) noexcept
{
    const float term_a = ::admc::core::dot(row.jacobian_linear_a, linear_velocity_a) +
                         ::admc::core::dot(row.jacobian_angular_a, angular_velocity_a);

    const float term_b = ::admc::core::dot(row.jacobian_linear_b, linear_velocity_b) +
                         ::admc::core::dot(row.jacobian_angular_b, angular_velocity_b);

    return term_a + term_b;
}
} // namespace admc::constraints
