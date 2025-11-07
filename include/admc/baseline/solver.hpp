#pragma once

#include "admc/baseline/contact.hpp"
#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"
#include "admc/world/body.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace admc::baseline
{
struct BaselineParams
{
    int iterations{10};
    float beta{0.2f};
    float slop{0.005f};
    float restitution{0.0f};
    float dt{1.0f / 60.0f};
};

namespace detail
{
inline ::admc::core::Vec3 compute_velocity(
    const ::admc::world::RigidBody& body,
    const ::admc::core::Vec3& offset) noexcept
{
    return body.linear_velocity + ::admc::core::cross(body.angular_velocity, offset);
}

inline void apply_impulse(
    ::admc::world::RigidBody& body,
    const ::admc::core::Vec3& impulse,
    const ::admc::core::Vec3& offset) noexcept
{
    if (body.inverse_mass > 0.0f)
    {
        body.linear_velocity += impulse * body.inverse_mass;
    }
    const ::admc::core::Vec3 angular_impulse = ::admc::core::cross(offset, impulse);
    const ::admc::core::Vec3 delta_w = body.inertia_world_inv * angular_impulse;
    body.angular_velocity += delta_w;
}

inline float compute_effective_mass(
    const ::admc::world::RigidBody& a,
    const ::admc::world::RigidBody& b,
    const ContactConstraint& contact) noexcept
{
    float term = a.inverse_mass + b.inverse_mass;
    const ::admc::core::Vec3 ang_a = a.inertia_world_inv * contact.ra_cross_n;
    const ::admc::core::Vec3 ang_b = b.inertia_world_inv * contact.rb_cross_n;
    term += ::admc::core::dot(contact.ra_cross_n, ang_a);
    term += ::admc::core::dot(contact.rb_cross_n, ang_b);
    return (term > std::numeric_limits<float>::epsilon()) ? (1.0f / term) : 0.0f;
}

inline void precompute_contact(
    ContactConstraint& contact,
    ::admc::world::RigidBody& a,
    ::admc::world::RigidBody& b,
    const BaselineParams& params) noexcept
{
    ::admc::core::Vec3 n = contact.normal;
    const float len_sq = ::admc::core::dot(n, n);
    if (len_sq <= std::numeric_limits<float>::epsilon())
    {
        n = ::admc::core::Vec3{0.0f, 1.0f, 0.0f};
    }
    else
    {
        const float inv_len = 1.0f / std::sqrt(len_sq);
        n *= inv_len;
    }
    contact.normal = n;

    contact.ra = contact.point - a.position;
    contact.rb = contact.point - b.position;
    contact.ra_cross_n = ::admc::core::cross(contact.ra, contact.normal);
    contact.rb_cross_n = ::admc::core::cross(contact.rb, contact.normal);
    contact.effective_mass = compute_effective_mass(a, b, contact);

    const float penetration_error = std::max(contact.penetration - params.slop, 0.0f);
    const float beta_dt = (params.dt > 0.0f) ? (params.beta / params.dt) : 0.0f;
    contact.bias = -beta_dt * penetration_error;
}
} // namespace detail

inline void solve_baseline(
    std::vector<::admc::world::RigidBody>& bodies,
    std::vector<ContactConstraint>& contacts,
    const BaselineParams& params) noexcept
{
    if (contacts.empty())
    {
        return;
    }

    const float restitution_clamp = std::clamp(params.restitution, 0.0f, 1.0f);

    for (ContactConstraint& contact : contacts)
    {
        if (contact.body_a < 0 || contact.body_b < 0 ||
            contact.body_a >= static_cast<int>(bodies.size()) ||
            contact.body_b >= static_cast<int>(bodies.size()))
        {
            contact.effective_mass = 0.0f;
            continue;
        }

        ::admc::world::RigidBody& body_a = bodies[contact.body_a];
        ::admc::world::RigidBody& body_b = bodies[contact.body_b];
        detail::precompute_contact(contact, body_a, body_b, params);

        if (contact.accumulated_impulse != 0.0f && contact.effective_mass > 0.0f)
        {
            const ::admc::core::Vec3 impulse = contact.normal * contact.accumulated_impulse;
            detail::apply_impulse(body_a, -impulse, contact.ra);
            detail::apply_impulse(body_b, impulse, contact.rb);
        }
    }

    for (int iteration = 0; iteration < params.iterations; ++iteration)
    {
        for (ContactConstraint& contact : contacts)
        {
            if (contact.effective_mass <= 0.0f ||
                contact.body_a < 0 || contact.body_b < 0 ||
                contact.body_a >= static_cast<int>(bodies.size()) ||
                contact.body_b >= static_cast<int>(bodies.size()))
            {
                continue;
            }

            ::admc::world::RigidBody& body_a = bodies[contact.body_a];
            ::admc::world::RigidBody& body_b = bodies[contact.body_b];

            const ::admc::core::Vec3 vel_a = detail::compute_velocity(body_a, contact.ra);
            const ::admc::core::Vec3 vel_b = detail::compute_velocity(body_b, contact.rb);
            const float relative_normal = ::admc::core::dot(contact.normal, vel_b - vel_a);

            float restitution = std::clamp(contact.restitution, 0.0f, 1.0f);
            restitution = std::max(restitution, restitution_clamp);

            float bounce = 0.0f;
            if (relative_normal < -1.0e-3f && restitution > 0.0f)
            {
                bounce = -restitution * relative_normal;
            }

            float delta_lambda = -(relative_normal + contact.bias + bounce) * contact.effective_mass;
            const float previous = contact.accumulated_impulse;
            contact.accumulated_impulse = std::max(previous + delta_lambda, 0.0f);
            delta_lambda = contact.accumulated_impulse - previous;
            if (delta_lambda == 0.0f)
            {
                continue;
            }

            const ::admc::core::Vec3 impulse = contact.normal * delta_lambda;
            detail::apply_impulse(body_a, -impulse, contact.ra);
            detail::apply_impulse(body_b, impulse, contact.rb);
        }
    }
}
} // namespace admc::baseline
