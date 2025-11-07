#pragma once

#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"

#include <string>
#include <vector>

namespace admc::scene
{
struct BodyDesc
{
    ::admc::core::Vec3 position{};
    ::admc::core::Vec3 linear_velocity{};
    ::admc::core::Vec3 angular_velocity{};
    float mass{1.0f};
    ::admc::core::Mat3 inertia{::admc::core::Mat3::identity()};
};

struct ContactDesc
{
    int body_a{-1};
    int body_b{-1};
    ::admc::core::Vec3 point{};
    ::admc::core::Vec3 normal{0.0f, 1.0f, 0.0f};
    float penetration{0.0f};
    float restitution{0.0f};
};

struct SceneDesc
{
    std::string name{};
    std::vector<BodyDesc> bodies{};
    std::vector<ContactDesc> contacts{};
};
} // namespace admc::scene
