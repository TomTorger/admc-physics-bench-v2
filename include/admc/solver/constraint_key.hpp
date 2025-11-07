#pragma once

#include "admc/core/vec3.hpp"

#include <cstdint>
#include <functional>
#include <cmath>

namespace admc::solver
{
struct ConstraintKey
{
    int body_a{0};
    int body_b{0};
    std::int32_t point_x{0};
    std::int32_t point_y{0};
    std::int32_t point_z{0};
    std::int32_t normal_x{0};
    std::int32_t normal_y{0};
    std::int32_t normal_z{0};

    friend bool operator==(const ConstraintKey& lhs, const ConstraintKey& rhs) noexcept = default;
};

struct ConstraintKeyHasher
{
    std::size_t operator()(const ConstraintKey& key) const noexcept
    {
        std::size_t h = std::hash<int>{}(key.body_a);
        auto mix = [&](std::size_t value) {
            h ^= value + 0x9e3779b9 + (h << 6) + (h >> 2);
        };
        mix(static_cast<std::size_t>(std::hash<int>{}(key.body_b)));
        mix(static_cast<std::size_t>(std::hash<std::int32_t>{}(key.point_x)));
        mix(static_cast<std::size_t>(std::hash<std::int32_t>{}(key.point_y)));
        mix(static_cast<std::size_t>(std::hash<std::int32_t>{}(key.point_z)));
        mix(static_cast<std::size_t>(std::hash<std::int32_t>{}(key.normal_x)));
        mix(static_cast<std::size_t>(std::hash<std::int32_t>{}(key.normal_y)));
        mix(static_cast<std::size_t>(std::hash<std::int32_t>{}(key.normal_z)));
        return h;
    }
};

inline ConstraintKey make_constraint_key(
    int body_a,
    int body_b,
    const ::admc::core::Vec3& point,
    const ::admc::core::Vec3& normal,
    float quantize_scale = 1000.0f) noexcept
{
    const auto quantize = [quantize_scale](float value) -> std::int32_t {
        return static_cast<std::int32_t>(std::lround(value * quantize_scale));
    };
    return ConstraintKey{
        body_a,
        body_b,
        quantize(point.x),
        quantize(point.y),
        quantize(point.z),
        quantize(normal.x),
        quantize(normal.y),
        quantize(normal.z)};
}
} // namespace admc::solver

