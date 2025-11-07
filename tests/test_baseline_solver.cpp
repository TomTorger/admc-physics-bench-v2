#include "admc/baseline/solver.hpp"
#include "admc/scene/scene_builders.hpp"
#include "admc/scene/scene_library.hpp"
#include "admc/solver/simple_pgs.hpp"

#include "admc/core/mat3.hpp"
#include "admc/core/quat.hpp"
#include "admc/core/vec3.hpp"
#include "admc/world/body.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using admc::baseline::BaselineParams;
using admc::baseline::ContactConstraint;
using admc::baseline::solve_baseline;
using admc::core::Mat3;
using admc::core::Quat;
using admc::core::Vec3;
using admc::core::dot;
using admc::core::length;
using admc::core::normalize;
using admc::scene::SceneDesc;
using admc::solver::SimplePGSSolver;
using admc::world::RigidBody;

namespace
{
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

RigidBody make_body(const Vec3& position, const Vec3& linear_velocity, float mass)
{
    RigidBody body{};
    body.position = position;
    body.orientation = Quat::identity();
    body.linear_velocity = linear_velocity;
    body.angular_velocity = Vec3{};
    body.set_mass(mass);
    body.inertia_body = Mat3::identity();
    body.update_world_inertia();
    return body;
}

ContactConstraint make_contact(int a, int b, const Vec3& point, const Vec3& normal, float penetration)
{
    ContactConstraint contact{};
    contact.body_a = a;
    contact.body_b = b;
    contact.point = point;
    contact.normal = normal;
    contact.penetration = penetration;
    contact.restitution = 0.0f;
    contact.accumulated_impulse = 0.0f;
    return contact;
}
} // namespace

void test_dynamic_pair()
{
    std::vector<RigidBody> bodies;
    bodies.push_back(make_body(Vec3{-0.5f, 0.0f, 0.0f}, Vec3{1.0f, 0.0f, 0.0f}, 1.0f));
    bodies.push_back(make_body(Vec3{0.5f, 0.0f, 0.0f}, Vec3{-1.0f, 0.0f, 0.0f}, 1.0f));

    std::vector<ContactConstraint> contacts;
    contacts.push_back(make_contact(0, 1, Vec3{0.0f, 0.0f, 0.0f}, Vec3{1.0f, 0.0f, 0.0f}, 0.05f));

    BaselineParams params{};
    params.iterations = 20;
    params.beta = 0.3f;
    params.slop = 0.0f;
    params.restitution = 0.0f;

    const float initial_relative = std::fabs((bodies[0].linear_velocity - bodies[1].linear_velocity).x);

    (void)solve_baseline(bodies, contacts, params);

    const Vec3 relative = bodies[0].linear_velocity - bodies[1].linear_velocity;
    expect(std::fabs(relative.x) < initial_relative, "Relative normal velocity reduced");
    expect(length(bodies[0].linear_velocity) < 1.0f, "Body A slowed down");
    expect(length(bodies[1].linear_velocity) < 1.0f, "Body B slowed down");
}

void test_static_ground()
{
    std::vector<RigidBody> bodies;
    bodies.push_back(make_body(Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, -5.0f, 0.0f}, 1.0f));

    RigidBody ground{};
    ground.position = Vec3{0.0f, 0.0f, 0.0f};
    ground.orientation = Quat::identity();
    ground.linear_velocity = Vec3{};
    ground.angular_velocity = Vec3{};
    ground.set_mass(0.0f);
    ground.inertia_body = Mat3{
        Vec3{0.0f, 0.0f, 0.0f},
        Vec3{0.0f, 0.0f, 0.0f},
        Vec3{0.0f, 0.0f, 0.0f}};
    ground.inertia_body_inv = Mat3{
        Vec3{0.0f, 0.0f, 0.0f},
        Vec3{0.0f, 0.0f, 0.0f},
        Vec3{0.0f, 0.0f, 0.0f}};
    ground.inertia_world_inv = ground.inertia_body_inv;
    bodies.push_back(ground);

    std::vector<ContactConstraint> contacts;
    contacts.push_back(make_contact(0, 1, Vec3{0.0f, 0.5f, 0.0f}, Vec3{0.0f, -1.0f, 0.0f}, 0.1f));

    BaselineParams params{};
    params.iterations = 20;
    params.beta = 0.4f;
    params.slop = 0.0f;

    (void)solve_baseline(bodies, contacts, params);

    expect(bodies[1].linear_velocity.y == 0.0f, "Static ground remains static");
    expect(bodies[0].linear_velocity.y > -5.0f, "Falling body slowed by ground");
}

void test_scene_conversion()
{
    const SceneDesc scene = admc::scene::make_two_spheres_scene();
    BaselineParams params{};
    params.iterations = 10;

    auto baseline_scene = admc::scene::build_baseline_scene(scene, params);
    (void)solve_baseline(baseline_scene.bodies, baseline_scene.contacts, params);
    expect(!baseline_scene.contacts.empty(), "Scene produced contacts");

    auto island = admc::scene::build_simple_island(scene, params);
    SimplePGSSolver solver;
    const auto result = solver.solve(island);
    expect(result.iterations > 0, "Simple solver ran iterations");
}

void test_scene_library_inventory()
{
    const auto scenes = admc::scene::make_default_scene_library();
    expect(!scenes.empty(), "Scene library not empty");
    bool found_grid = false;
    for (const SceneDesc& scene : scenes)
    {
        if (scene.name.find("sphere_grid") != std::string::npos)
        {
            found_grid = true;
            expect(scene.contacts.size() >= scene.bodies.size(), "Sphere grid has ground + neighbor contacts");
        }
    }
    expect(found_grid, "Sphere grid scenes available");
}

int main()
{
    test_dynamic_pair();
    test_static_ground();
    test_scene_conversion();
    test_scene_library_inventory();
    std::cout << "Baseline solver tests passed\n";
    return 0;
}
