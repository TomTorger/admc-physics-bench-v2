#pragma once

#include "admc/baseline/solver.hpp"
#include "admc/core/mat3.hpp"
#include "admc/core/quat.hpp"
#include "admc/scene/scene_desc.hpp"
#include "admc/solver/simple_pgs.hpp"
#include "admc/world/body.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace admc::scene
{
struct BaselineSceneData
{
    std::vector<::admc::world::RigidBody> bodies;
    std::vector<::admc::baseline::ContactConstraint> contacts;
};

inline BaselineSceneData build_baseline_scene(
    const SceneDesc& scene,
    const ::admc::baseline::BaselineParams& params)
{
    BaselineSceneData data{};
    data.bodies.reserve(scene.bodies.size());
    for (const BodyDesc& desc : scene.bodies)
    {
        ::admc::world::RigidBody body{};
        body.position = desc.position;
        body.orientation = ::admc::core::Quat::identity();
        body.linear_velocity = desc.linear_velocity;
        body.angular_velocity = desc.angular_velocity;
        body.inertia_body = desc.inertia;
        body.update_world_inertia();
        body.set_mass(desc.mass);
        data.bodies.push_back(body);
    }

    auto compute_effective_mass = [](
                                       const ::admc::world::RigidBody& a,
                                       const ::admc::world::RigidBody& b,
                                       const ::admc::core::Vec3& ra_cross_n,
                                       const ::admc::core::Vec3& rb_cross_n) -> float {
        float term = a.inverse_mass + b.inverse_mass;
        const ::admc::core::Vec3 ang_a = a.inertia_world_inv * ra_cross_n;
        const ::admc::core::Vec3 ang_b = b.inertia_world_inv * rb_cross_n;
        term += ::admc::core::dot(ra_cross_n, ang_a);
        term += ::admc::core::dot(rb_cross_n, ang_b);
        return (term > std::numeric_limits<float>::epsilon()) ? (1.0f / term) : 0.0f;
    };

    data.contacts.reserve(scene.contacts.size());
    for (const ContactDesc& desc : scene.contacts)
    {
        if (desc.body_a < 0 || desc.body_b < 0 ||
            desc.body_a >= static_cast<int>(data.bodies.size()) ||
            desc.body_b >= static_cast<int>(data.bodies.size()))
        {
            continue;
        }

        ::admc::baseline::ContactConstraint contact{};
        contact.body_a = desc.body_a;
        contact.body_b = desc.body_b;
        contact.point = desc.point;

        ::admc::core::Vec3 n = desc.normal;
        const ::admc::core::Vec3 direction = data.bodies[contact.body_b].position - data.bodies[contact.body_a].position;
        if (::admc::core::dot(n, direction) < 0.0f)
        {
            n = -n;
        }
        const float len_sq = ::admc::core::dot(n, n);
        if (len_sq <= std::numeric_limits<float>::epsilon())
        {
            n = ::admc::core::Vec3{0.0f, 1.0f, 0.0f};
        }
        else
        {
            n *= (1.0f / std::sqrt(len_sq));
        }
        contact.normal = n;
        contact.penetration = desc.penetration;
        contact.restitution = desc.restitution;

        auto& body_a = data.bodies[contact.body_a];
        auto& body_b = data.bodies[contact.body_b];
        contact.ra = contact.point - body_a.position;
        contact.rb = contact.point - body_b.position;
        contact.ra_cross_n = ::admc::core::cross(contact.ra, contact.normal);
        contact.rb_cross_n = ::admc::core::cross(contact.rb, contact.normal);
        contact.effective_mass = compute_effective_mass(body_a, body_b, contact.ra_cross_n, contact.rb_cross_n);

        const float penetration_error = std::max(contact.penetration - params.slop, 0.0f);
        const float beta_dt = (params.dt > 0.0f) ? (params.beta / params.dt) : 0.0f;
        contact.bias = -beta_dt * penetration_error;
        contact.accumulated_impulse = 0.0f;

        data.contacts.push_back(contact);
    }
    return data;
}

inline ::admc::solver::IslandState build_simple_island(
    const SceneDesc& scene,
    const ::admc::baseline::BaselineParams& baseline_params)
{
    ::admc::solver::IslandState island;
    island.bodies.reserve(scene.bodies.size());

    std::vector<::admc::core::Mat3> inertia_world_inv(scene.bodies.size(), ::admc::core::Mat3::identity());

    for (std::size_t i = 0; i < scene.bodies.size(); ++i)
    {
        const BodyDesc& desc = scene.bodies[i];
        ::admc::solver::SolverBody body{};
        body.linear_velocity = desc.linear_velocity;
        body.angular_velocity = desc.angular_velocity;
        body.inverse_mass = (desc.mass > 0.0f) ? 1.0f / desc.mass : 0.0f;
        ::admc::core::Mat3 inv_inertia(
            ::admc::core::Vec3{},
            ::admc::core::Vec3{},
            ::admc::core::Vec3{});
        if (desc.mass > 0.0f)
        {
            ::admc::core::Mat3 inv_out;
            if (::admc::core::inverse(desc.inertia, inv_out))
            {
                inv_inertia = inv_out;
            }
        }
        body.inverse_inertia = inv_inertia;
        island.bodies.push_back(body);
        inertia_world_inv[i] = inv_inertia;
    }

    auto compute_effective_mass = [&](int a, int b, const ::admc::core::Vec3& ra_cross_n, const ::admc::core::Vec3& rb_cross_n) {
        const float inv_mass_a = island.bodies[a].inverse_mass;
        const float inv_mass_b = island.bodies[b].inverse_mass;
        float term = inv_mass_a + inv_mass_b;
        term += ::admc::core::dot(ra_cross_n, inertia_world_inv[a] * ra_cross_n);
        term += ::admc::core::dot(rb_cross_n, inertia_world_inv[b] * rb_cross_n);
        return (term > std::numeric_limits<float>::epsilon()) ? (1.0f / term) : 0.0f;
    };

    island.constraints.reserve(scene.contacts.size());
    for (const ContactDesc& desc : scene.contacts)
    {
        if (desc.body_a < 0 || desc.body_b < 0 ||
            desc.body_a >= static_cast<int>(scene.bodies.size()) ||
            desc.body_b >= static_cast<int>(scene.bodies.size()))
        {
            continue;
        }

        ::admc::solver::ConstraintInstance constraint{};
        constraint.body_a = static_cast<std::size_t>(desc.body_a);
        constraint.body_b = static_cast<std::size_t>(desc.body_b);

        ::admc::core::Vec3 n = desc.normal;
        const ::admc::core::Vec3 direction = scene.bodies[desc.body_b].position - scene.bodies[desc.body_a].position;
        if (::admc::core::dot(n, direction) < 0.0f)
        {
            n = -n;
        }
        const float len_sq = ::admc::core::dot(n, n);
        if (len_sq <= std::numeric_limits<float>::epsilon())
        {
            n = ::admc::core::Vec3{0.0f, 1.0f, 0.0f};
        }
        else
        {
            n *= (1.0f / std::sqrt(len_sq));
        }

        const ::admc::core::Vec3 ra = desc.point - scene.bodies[desc.body_a].position;
        const ::admc::core::Vec3 rb = desc.point - scene.bodies[desc.body_b].position;
        const ::admc::core::Vec3 ra_cross_n = ::admc::core::cross(ra, n);
        const ::admc::core::Vec3 rb_cross_n = ::admc::core::cross(rb, n);

        constraint.row.type = ::admc::constraints::ConstraintType::ContactNormal;
        constraint.row.jacobian_linear_a = -n;
        constraint.row.jacobian_linear_b = n;
        constraint.row.jacobian_angular_a = -ra_cross_n;
        constraint.row.jacobian_angular_b = rb_cross_n;
        constraint.row.effective_mass = compute_effective_mass(desc.body_a, desc.body_b, ra_cross_n, rb_cross_n);
        constraint.row.lower_limit = 0.0f;
        constraint.row.upper_limit = std::numeric_limits<float>::max();
        constraint.key = ::admc::solver::make_constraint_key(desc.body_a, desc.body_b, desc.point, n);

        const float penetration_error = std::max(desc.penetration - baseline_params.slop, 0.0f);
        const float beta_dt = (baseline_params.dt > 0.0f) ? (baseline_params.beta / baseline_params.dt) : 0.0f;
        constraint.row.bias = -beta_dt * penetration_error;

        island.constraints.push_back(constraint);
    }

    return island;
}
} // namespace admc::scene
