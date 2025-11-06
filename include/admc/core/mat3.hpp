#pragma once

#include "admc/core/vec3.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace admc::core
{
class Mat3
{
public:
    constexpr Mat3() noexcept
        : m_rows{Vec3{1.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 1.0f}}
    {
    }

    constexpr Mat3(const Vec3& r0, const Vec3& r1, const Vec3& r2) noexcept : m_rows{r0, r1, r2} {}

    [[nodiscard]] static constexpr Mat3 identity() noexcept { return Mat3{}; }

    [[nodiscard]] constexpr const Vec3& row(std::size_t index) const noexcept { return m_rows[index]; }
    [[nodiscard]] constexpr Vec3& row(std::size_t index) noexcept { return m_rows[index]; }

    [[nodiscard]] constexpr const Vec3& operator[](std::size_t index) const noexcept { return m_rows[index]; }
    [[nodiscard]] constexpr Vec3& operator[](std::size_t index) noexcept { return m_rows[index]; }

    [[nodiscard]] constexpr Mat3 operator+(const Mat3& other) const noexcept
    {
        return Mat3{m_rows[0] + other.m_rows[0], m_rows[1] + other.m_rows[1], m_rows[2] + other.m_rows[2]};
    }

    [[nodiscard]] constexpr Mat3 operator-(const Mat3& other) const noexcept
    {
        return Mat3{m_rows[0] - other.m_rows[0], m_rows[1] - other.m_rows[1], m_rows[2] - other.m_rows[2]};
    }

    constexpr Mat3& operator+=(const Mat3& other) noexcept
    {
        m_rows[0] += other.m_rows[0];
        m_rows[1] += other.m_rows[1];
        m_rows[2] += other.m_rows[2];
        return *this;
    }

    constexpr Mat3& operator-=(const Mat3& other) noexcept
    {
        m_rows[0] -= other.m_rows[0];
        m_rows[1] -= other.m_rows[1];
        m_rows[2] -= other.m_rows[2];
        return *this;
    }

    [[nodiscard]] constexpr Vec3 operator*(const Vec3& v) const noexcept
    {
        return {
            dot(m_rows[0], v),
            dot(m_rows[1], v),
            dot(m_rows[2], v)};
    }

private:
    std::array<Vec3, 3> m_rows;
};

[[nodiscard]] constexpr Mat3 mat_mul(const Mat3& a, const Mat3& b) noexcept
{
    // Build columns of b for reuse
    const Vec3 col0{b[0].x, b[1].x, b[2].x};
    const Vec3 col1{b[0].y, b[1].y, b[2].y};
    const Vec3 col2{b[0].z, b[1].z, b[2].z};

    const Vec3 r0{dot(a[0], col0), dot(a[0], col1), dot(a[0], col2)};
    const Vec3 r1{dot(a[1], col0), dot(a[1], col1), dot(a[1], col2)};
    const Vec3 r2{dot(a[2], col0), dot(a[2], col1), dot(a[2], col2)};

    return Mat3{r0, r1, r2};
}

[[nodiscard]] constexpr Mat3 transpose(const Mat3& m) noexcept
{
    return Mat3{
        Vec3{m[0].x, m[1].x, m[2].x},
        Vec3{m[0].y, m[1].y, m[2].y},
        Vec3{m[0].z, m[1].z, m[2].z}};
}

[[nodiscard]] constexpr float determinant(const Mat3& m) noexcept
{
    return m[0].x * (m[1].y * m[2].z - m[1].z * m[2].y) -
           m[0].y * (m[1].x * m[2].z - m[1].z * m[2].x) +
           m[0].z * (m[1].x * m[2].y - m[1].y * m[2].x);
}

[[nodiscard]] inline bool inverse(const Mat3& m, Mat3& out, float epsilon = 1e-6f) noexcept
{
    const float det = determinant(m);
    if (std::fabs(det) <= epsilon)
    {
        return false;
    }

    const float invDet = 1.0f / det;

    const float a00 = m[0].x;
    const float a01 = m[0].y;
    const float a02 = m[0].z;
    const float a10 = m[1].x;
    const float a11 = m[1].y;
    const float a12 = m[1].z;
    const float a20 = m[2].x;
    const float a21 = m[2].y;
    const float a22 = m[2].z;

    const float c00 =  (a11 * a22 - a12 * a21);
    const float c01 = -(a10 * a22 - a12 * a20);
    const float c02 =  (a10 * a21 - a11 * a20);

    const float c10 = -(a01 * a22 - a02 * a21);
    const float c11 =  (a00 * a22 - a02 * a20);
    const float c12 = -(a00 * a21 - a01 * a20);

    const float c20 =  (a01 * a12 - a02 * a11);
    const float c21 = -(a00 * a12 - a02 * a10);
    const float c22 =  (a00 * a11 - a01 * a10);

    const Vec3 r0{c00 * invDet, c10 * invDet, c20 * invDet};
    const Vec3 r1{c01 * invDet, c11 * invDet, c21 * invDet};
    const Vec3 r2{c02 * invDet, c12 * invDet, c22 * invDet};

    out = Mat3{r0, r1, r2};
    return true;
}
} // namespace admc::core
