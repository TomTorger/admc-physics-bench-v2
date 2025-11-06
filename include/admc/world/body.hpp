#pragma once

#include "admc/core/mat3.hpp"
#include "admc/core/quat.hpp"
#include "admc/core/vec3.hpp"

namespace admc::world
{
using ::admc::core::Mat3;
using ::admc::core::Quat;
using ::admc::core::Vec3;
using ::admc::core::inverse;
using ::admc::core::mat_mul;
using ::admc::core::transpose;
using ::admc::core::to_mat3;

struct RigidBody
{
    Vec3 position{};
    Quat orientation{};
    Vec3 linear_velocity{};
    Vec3 angular_velocity{};
    float mass{1.0f};
    float inverse_mass{1.0f};
    Mat3 inertia_body{};
    Mat3 inertia_body_inv{};
    Mat3 inertia_world{};
    Mat3 inertia_world_inv{};

    void set_mass(float new_mass) noexcept
    {
        mass = new_mass;
        inverse_mass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    }

    void update_world_inertia() noexcept
    {
        const Mat3 R = to_mat3(orientation);
        const Mat3 R_T = transpose(R);
        inertia_world = mat_mul(mat_mul(R, inertia_body), R_T);
        Mat3 inv_body{};
        if (inverse(inertia_body, inv_body))
        {
            inertia_body_inv = inv_body;
            inertia_world_inv = mat_mul(mat_mul(R, inertia_body_inv), R_T);
        }
    }
};
} // namespace admc::world
