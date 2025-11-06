#pragma once

#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"

#include <cmath>

namespace admc::core
{
struct Quat
{
    float w{};
    float x{};
    float y{};
    float z{};

    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    [[nodiscard]] static constexpr Quat identity() noexcept { return Quat{1.0f, 0.0f, 0.0f, 0.0f}; }

    [[nodiscard]] constexpr Quat operator*(const Quat& other) const noexcept
    {
        return {
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w};
    }

    constexpr Quat& operator*=(const Quat& other) noexcept
    {
        *this = (*this) * other;
        return *this;
    }

    [[nodiscard]] constexpr Quat conjugate() const noexcept { return Quat{w, -x, -y, -z}; }
};

[[nodiscard]] inline Quat normalize(const Quat& q, float epsilon = 1e-6f) noexcept
{
    const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= epsilon)
    {
        return Quat::identity();
    }
    const float inv = 1.0f / norm;
    return Quat{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

[[nodiscard]] inline Quat from_axis_angle(const Vec3& axis, float radians, float epsilon = 1e-6f) noexcept
{
    Vec3 axis_norm = normalize(axis, epsilon);
    const float half = radians * 0.5f;
    const float s = std::sin(half);
    return normalize(Quat{std::cos(half), axis_norm.x * s, axis_norm.y * s, axis_norm.z * s});
}

[[nodiscard]] inline Mat3 to_mat3(const Quat& q) noexcept
{
    const float ww = q.w * q.w;
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;

    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;

    return Mat3(
        Vec3{ww + xx - yy - zz, 2.0f * (xy - wz), 2.0f * (xz + wy)},
        Vec3{2.0f * (xy + wz), ww - xx + yy - zz, 2.0f * (yz - wx)},
        Vec3{2.0f * (xz - wy), 2.0f * (yz + wx), ww - xx - yy + zz});
}

[[nodiscard]] inline Vec3 rotate_vector(const Quat& q, const Vec3& v) noexcept
{
    const Quat q_norm = normalize(q);
    const Quat p{0.0f, v.x, v.y, v.z};
    const Quat result = q_norm * p * q_norm.conjugate();
    return Vec3{result.x, result.y, result.z};
}
} // namespace admc::core
