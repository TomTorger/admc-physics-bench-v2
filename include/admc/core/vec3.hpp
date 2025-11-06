#pragma once

#include <cmath>
#include <ostream>

namespace admc::core
{
struct Vec3
{
    float x{};
    float y{};
    float z{};

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& other) const noexcept
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] constexpr Vec3 operator-(const Vec3& other) const noexcept
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] constexpr Vec3 operator*(float s) const noexcept
    {
        return {x * s, y * s, z * s};
    }

    [[nodiscard]] constexpr Vec3 operator/(float s) const noexcept
    {
        return {x / s, y / s, z / s};
    }

    constexpr Vec3& operator+=(const Vec3& other) noexcept
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& other) noexcept
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Vec3& operator*=(float s) noexcept
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    constexpr Vec3& operator/=(float s) noexcept
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }
};

[[nodiscard]] constexpr Vec3 operator*(float s, const Vec3& v) noexcept
{
    return v * s;
}

[[nodiscard]] constexpr bool operator==(const Vec3& a, const Vec3& b) noexcept
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

[[nodiscard]] constexpr bool operator!=(const Vec3& a, const Vec3& b) noexcept
{
    return !(a == b);
}

[[nodiscard]] constexpr float dot(const Vec3& a, const Vec3& b) noexcept
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x};
}

[[nodiscard]] inline float length(const Vec3& v) noexcept
{
    return std::sqrt(dot(v, v));
}

[[nodiscard]] inline float length_squared(const Vec3& v) noexcept
{
    return dot(v, v);
}

[[nodiscard]] inline Vec3 normalize(const Vec3& v, float epsilon = 1e-6f) noexcept
{
    const float len = length(v);
    if (len <= epsilon)
    {
        return Vec3{};
    }
    return v / len;
}

inline std::ostream& operator<<(std::ostream& os, const Vec3& v)
{
    os << "Vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}
} // namespace admc::core
