#include "admc/admc/channels.hpp"
#include "admc/constraints/constraint_row.hpp"
#include "admc/core/mat3.hpp"
#include "admc/core/quat.hpp"
#include "admc/core/vec3.hpp"
#include "admc/world/body.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using admc::admc::ADMCChannels;
using admc::admc::DirectionalTracker;
using admc::admc::DirectionalChannels;
using admc::admc::make_channels;
using admc::admc::reconstruct_mass;
using admc::admc::reconstruct_momentum;
using admc::constraints::ConstraintRow;
using admc::core::Mat3;
using admc::core::Quat;
using admc::core::Vec3;
using admc::core::cross;
using admc::core::dot;
using admc::core::from_axis_angle;
using admc::core::inverse;
using admc::core::length;
using admc::core::length_squared;
using admc::core::mat_mul;
using admc::core::normalize;
using admc::core::rotate_vector;
using admc::core::to_mat3;
using admc::core::transpose;
using admc::world::RigidBody;

namespace
{
constexpr float kEpsilon = 1e-5f;
constexpr float kPi = 3.14159265358979323846f;

bool nearly_equal(float a, float b, float eps = kEpsilon)
{
    return std::fabs(a - b) <= eps;
}

bool vec_nearly_equal(const Vec3& a, const Vec3& b, float eps = kEpsilon)
{
    return nearly_equal(a.x, b.x, eps) && nearly_equal(a.y, b.y, eps) && nearly_equal(a.z, b.z, eps);
}

bool mat_nearly_equal(const Mat3& a, const Mat3& b, float eps = kEpsilon)
{
    for (int r = 0; r < 3; ++r)
    {
        if (!vec_nearly_equal(a.row(r), b.row(r), eps))
        {
            return false;
        }
    }
    return true;
}

[[noreturn]] void fail(const char* message)
{
    std::cerr << "Check failed: " << message << '\n';
    std::exit(1);
}

void expect(bool condition, const char* message)
{
    if (!condition)
    {
        fail(message);
    }
}
} // namespace

void test_vec3_basic_arithmetic()
{
    const Vec3 a{1.0f, 2.0f, 3.0f};
    const Vec3 b{-4.0f, 0.5f, 2.0f};

    expect((a + b) == Vec3{-3.0f, 2.5f, 5.0f}, "Vec3 add");
    expect((a - b) == Vec3{5.0f, 1.5f, 1.0f}, "Vec3 sub");
    expect((a * 2.0f) == Vec3{2.0f, 4.0f, 6.0f}, "Vec3 mul scalar");
    expect((2.0f * a) == Vec3{2.0f, 4.0f, 6.0f}, "scalar mul Vec3");
    expect((a / 2.0f) == Vec3{0.5f, 1.0f, 1.5f}, "Vec3 div scalar");
}

void test_vec3_dot_cross()
{
    const Vec3 a{1.0f, 0.0f, 0.0f};
    const Vec3 b{0.0f, 1.0f, 0.0f};
    const Vec3 c{0.0f, 0.0f, 1.0f};
    expect(dot(a, b) == 0.0f, "dot orthogonal");
    expect(dot(a, a) == 1.0f, "dot self");
    expect(cross(a, b) == c, "cross right-handed");
    expect(cross(b, a) == Vec3{0.0f, 0.0f, -1.0f}, "cross left-handed");
}

void test_vec3_length_normalize()
{
    const Vec3 v{3.0f, 4.0f, 0.0f};
    expect(nearly_equal(length(v), 5.0f), "length");
    expect(nearly_equal(length_squared(v), 25.0f), "length squared");
    const Vec3 unit = normalize(v);
    expect(vec_nearly_equal(unit, Vec3{0.6f, 0.8f, 0.0f}), "normalize");
    const Vec3 zero{};
    expect(vec_nearly_equal(normalize(zero), Vec3{}), "normalize zero");
}

void test_mat3_identity()
{
    const Mat3 I = Mat3::identity();
    const Vec3 v{1.0f, -2.0f, 3.5f};
    expect(vec_nearly_equal(I * v, v), "I * v = v");
    expect(mat_nearly_equal(transpose(I), I), "transpose(I) == I");
}

void test_mat3_multiplication()
{
    const Mat3 A(
        Vec3{1.0f, 2.0f, 3.0f},
        Vec3{0.0f, 1.0f, 4.0f},
        Vec3{5.0f, 6.0f, 0.0f}); // rows

    const Mat3 B(
        Vec3{-2.0f, 1.0f, 0.0f},
        Vec3{3.0f, 0.0f, 1.0f},
        Vec3{4.0f, -1.0f, 2.0f});

    const Vec3 v{1.0f, 2.0f, 3.0f};
    expect(vec_nearly_equal(A * v, Vec3{14.0f, 14.0f, 17.0f}), "A * v result");

    const Mat3 C = mat_mul(A, B);
    const Mat3 expected(
        Vec3{16.0f, -2.0f, 8.0f},
        Vec3{19.0f, -4.0f, 9.0f},
        Vec3{8.0f, 5.0f, 6.0f});
    expect(mat_nearly_equal(C, expected), "A * B matrix");
}

void test_mat3_inverse()
{
    const Mat3 A(
        Vec3{3.0f, 0.0f, 2.0f},
        Vec3{2.0f, 0.0f, -2.0f},
        Vec3{0.0f, 1.0f, 1.0f});
    Mat3 invA{};
    expect(inverse(A, invA), "inverse exists");

    const Mat3 identity = Mat3::identity();
    expect(mat_nearly_equal(mat_mul(A, invA), identity, 1e-4f), "A * invA = I");

    const Mat3 singular(
        Vec3{1.0f, 2.0f, 3.0f},
        Vec3{2.0f, 4.0f, 6.0f},
        Vec3{0.0f, 0.0f, 0.0f});
    Mat3 invSingular;
    expect(!inverse(singular, invSingular), "singular inverse fails");
}

void test_quat_basic()
{
    const Quat q1 = from_axis_angle(Vec3{0.0f, 1.0f, 0.0f}, kPi / 2.0f);
    const Vec3 v{1.0f, 0.0f, 0.0f};
    const Vec3 rotated = rotate_vector(q1, v);
    expect(vec_nearly_equal(rotated, Vec3{0.0f, 0.0f, -1.0f}), "quat rotate");

    const Quat q2 = from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, kPi / 2.0f);
    const Quat combined = q2 * q1; // apply q1 then q2
    const Vec3 rotated2 = rotate_vector(combined, v);
    expect(vec_nearly_equal(rotated2, Vec3{0.0f, 0.0f, -1.0f}), "quat multiply");

    const Mat3 m = to_mat3(q1);
    const Vec3 mv = m * v;
    expect(vec_nearly_equal(mv, rotated), "quat to mat3");
}

void test_admc_channel_roundtrip()
{
    const float M = 5.0f;
    const Vec3 momentum{1.5f, -2.0f, 0.25f};
    const ADMCChannels channels = make_channels(M, momentum);
    expect(nearly_equal(reconstruct_mass(channels), M), "ADMC mass roundtrip");
    expect(vec_nearly_equal(reconstruct_momentum(channels), momentum), "ADMC momentum roundtrip");
}

void test_admc_tracker()
{
    DirectionalTracker tracker;
    const float M = 3.0f;
    const Vec3 pre{1.0f, -1.0f, 0.5f};
    const Vec3 post{0.5f, -0.5f, 0.25f};
    tracker.record_pre(M, pre);
    tracker.record_post(M, post);
    const Vec3 drift = tracker.momentum_drift();
    expect(vec_nearly_equal(drift, post - pre), "ADMC momentum drift");
    expect(tracker.max_abs_channel_drift() > 0.0f, "ADMC channel drift non-zero");
}

void test_rigidbody_inertia()
{
    RigidBody body{};
    body.set_mass(2.0f);
    expect(nearly_equal(body.mass, 2.0f), "body mass");
    expect(nearly_equal(body.inverse_mass, 0.5f), "body inverse mass");

    body.inertia_body = Mat3(
        Vec3{2.0f, 0.0f, 0.0f},
        Vec3{0.0f, 3.0f, 0.0f},
        Vec3{0.0f, 0.0f, 4.0f});
    body.orientation = Quat::identity();
    body.update_world_inertia();
    expect(mat_nearly_equal(body.inertia_world, body.inertia_body), "world inertia identity");
    expect(mat_nearly_equal(body.inertia_world_inv, body.inertia_body_inv), "world inertia inverse identity");

    body.orientation = from_axis_angle(Vec3{0.0f, 1.0f, 0.0f}, kPi / 2.0f);
    body.update_world_inertia();
    const Mat3 R = to_mat3(body.orientation);
    const Mat3 expected = mat_mul(mat_mul(R, body.inertia_body), transpose(R));
    expect(mat_nearly_equal(body.inertia_world, expected, 1e-4f), "world inertia rotated");
}

int main()
{
    test_vec3_basic_arithmetic();
    test_vec3_dot_cross();
    test_vec3_length_normalize();
    test_mat3_identity();
    test_mat3_multiplication();
    test_mat3_inverse();
    test_quat_basic();
    test_admc_channel_roundtrip();
    test_admc_tracker();
    test_rigidbody_inertia();

    std::cout << "Core math tests passed\n";
    return 0;
}
#include "admc/constraints/constraint_row.hpp"
