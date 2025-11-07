#pragma once

#include "admc/constraints/constraint_row.hpp"
#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"

#include <cstddef>
#include <span>

namespace admc::solver
{
struct BodyStateView
{
    ::admc::core::Vec3 linear_velocity{};
    ::admc::core::Vec3 angular_velocity{};
};

struct BodyInertiaView
{
    float inverse_mass{0.0f};
    ::admc::core::Mat3 inverse_inertia{};
};

struct BodyImpulse
{
    ::admc::core::Vec3 linear{};
    ::admc::core::Vec3 angular{};
};

struct BodyPairImpulse
{
    BodyImpulse body_a{};
    BodyImpulse body_b{};
};

class ConstraintOperator
{
public:
    ConstraintOperator(
        std::span<const BodyStateView> states,
        std::span<const BodyInertiaView> inertia) noexcept
        : states_(states), inertia_(inertia)
    {
    }

    [[nodiscard]] float apply_jacobian(
        const ::admc::constraints::ConstraintRow& row,
        std::size_t body_a,
        std::size_t body_b) const noexcept
    {
        const BodyStateView& state_a = states_[body_a];
        const BodyStateView& state_b = states_[body_b];

        const float term_a = ::admc::core::dot(row.jacobian_linear_a, state_a.linear_velocity) +
                             ::admc::core::dot(row.jacobian_angular_a, state_a.angular_velocity);

        const float term_b = ::admc::core::dot(row.jacobian_linear_b, state_b.linear_velocity) +
                             ::admc::core::dot(row.jacobian_angular_b, state_b.angular_velocity);

        return term_a + term_b;
    }

    [[nodiscard]] BodyPairImpulse apply_jacobian_transpose(
        const ::admc::constraints::ConstraintRow& row,
        float lambda) const noexcept
    {
        BodyPairImpulse pair{};
        pair.body_a.linear = row.jacobian_linear_a * lambda;
        pair.body_a.angular = row.jacobian_angular_a * lambda;
        pair.body_b.linear = row.jacobian_linear_b * lambda;
        pair.body_b.angular = row.jacobian_angular_b * lambda;
        return pair;
    }

    [[nodiscard]] BodyImpulse apply_inverse_mass(
        std::size_t body_index,
        const BodyImpulse& impulse) const noexcept
    {
        const BodyInertiaView& inertia = inertia_[body_index];
        BodyImpulse out{};
        out.linear = impulse.linear * inertia.inverse_mass;
        out.angular = inertia.inverse_inertia * impulse.angular;
        return out;
    }

private:
    std::span<const BodyStateView> states_;
    std::span<const BodyInertiaView> inertia_;
};
} // namespace admc::solver
