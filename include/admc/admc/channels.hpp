#pragma once

#include "admc/core/vec3.hpp"

#include <array>
#include <algorithm>
#include <cmath>

namespace admc::admc
{
using Vec3 = ::admc::core::Vec3;

struct DirectionalChannels
{
    float plus{};
    float minus{};
};

inline DirectionalChannels make_channels(float mass_energy, float directional_momentum) noexcept
{
    const float half = 0.5f * directional_momentum;
    return DirectionalChannels{mass_energy + half, mass_energy - half};
}

inline float reconstruct_mass(const DirectionalChannels& channels) noexcept
{
    return 0.5f * (channels.plus + channels.minus);
}

inline float reconstruct_momentum(const DirectionalChannels& channels) noexcept
{
    return channels.plus - channels.minus;
}

struct ADMCChannels
{
    DirectionalChannels x{};
    DirectionalChannels y{};
    DirectionalChannels z{};
};

inline ADMCChannels make_channels(float mass_energy, const Vec3& momentum) noexcept
{
    return {
        make_channels(mass_energy, momentum.x),
        make_channels(mass_energy, momentum.y),
        make_channels(mass_energy, momentum.z)};
}

inline Vec3 reconstruct_momentum(const ADMCChannels& channels) noexcept
{
    return Vec3{
        reconstruct_momentum(channels.x),
        reconstruct_momentum(channels.y),
        reconstruct_momentum(channels.z)};
}

inline float reconstruct_mass(const ADMCChannels& channels) noexcept
{
    // All channels share the same underlying mass energy; average the three
    const float mx = reconstruct_mass(channels.x);
    const float my = reconstruct_mass(channels.y);
    const float mz = reconstruct_mass(channels.z);
    return (mx + my + mz) / 3.0f;
}

class DirectionalTracker
{
public:
    void record_pre(float mass_energy, const Vec3& momentum) noexcept
    {
        pre_channels_ = make_channels(mass_energy, momentum);
        pre_set_ = true;
    }

    void record_post(float mass_energy, const Vec3& momentum) noexcept
    {
        post_channels_ = make_channels(mass_energy, momentum);
        post_set_ = true;
    }

    [[nodiscard]] Vec3 momentum_drift() const noexcept
    {
        if (!pre_set_ || !post_set_)
        {
            return Vec3{};
        }
        const Vec3 pre = reconstruct_momentum(pre_channels_);
        const Vec3 post = reconstruct_momentum(post_channels_);
        return post - pre;
    }

    [[nodiscard]] Vec3 plus_channel_drift() const noexcept
    {
        if (!pre_set_ || !post_set_)
        {
            return Vec3{};
        }
        return Vec3{
            post_channels_.x.plus - pre_channels_.x.plus,
            post_channels_.y.plus - pre_channels_.y.plus,
            post_channels_.z.plus - pre_channels_.z.plus};
    }

    [[nodiscard]] Vec3 minus_channel_drift() const noexcept
    {
        if (!pre_set_ || !post_set_)
        {
            return Vec3{};
        }
        return Vec3{
            post_channels_.x.minus - pre_channels_.x.minus,
            post_channels_.y.minus - pre_channels_.y.minus,
            post_channels_.z.minus - pre_channels_.z.minus};
    }

    [[nodiscard]] float max_abs_channel_drift() const noexcept
    {
        if (!pre_set_ || !post_set_)
        {
            return 0.0f;
        }
        const Vec3 plus = plus_channel_drift();
        const Vec3 minus = minus_channel_drift();
        const float arr[] = {
            std::fabs(plus.x), std::fabs(plus.y), std::fabs(plus.z),
            std::fabs(minus.x), std::fabs(minus.y), std::fabs(minus.z)};
        float max_value = 0.0f;
        for (float value : arr)
        {
            max_value = std::max(max_value, value);
        }
        return max_value;
    }

private:
    bool pre_set_{false};
    bool post_set_{false};
    ADMCChannels pre_channels_{};
    ADMCChannels post_channels_{};
};
} // namespace admc::admc
