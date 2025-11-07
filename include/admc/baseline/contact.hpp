#pragma once

#include "admc/core/vec3.hpp"

namespace admc::baseline
{
struct ContactConstraint
{
    int body_a{-1};
    int body_b{-1};
    ::admc::core::Vec3 point{};
    ::admc::core::Vec3 normal{0.0f, 1.0f, 0.0f};
    float penetration{0.0f};
    float restitution{0.0f};
    float accumulated_impulse{0.0f};
    float effective_mass{0.0f};
    float bias{0.0f};
    ::admc::core::Vec3 ra{};
    ::admc::core::Vec3 rb{};
    ::admc::core::Vec3 ra_cross_n{};
    ::admc::core::Vec3 rb_cross_n{};
};
} // namespace admc::baseline
