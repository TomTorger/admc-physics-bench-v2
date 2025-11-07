#include "admc/baseline/solver.hpp"
#include "admc/solver/simple_pgs.hpp"
#include "admc/world/body.hpp"

#include "admc/core/mat3.hpp"
#include "admc/core/quat.hpp"
#include "admc/core/vec3.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

using admc::baseline::BaselineParams;
using admc::baseline::ContactConstraint;
using admc::baseline::solve_baseline;
using admc::core::Mat3;
using admc::core::Quat;
using admc::core::Vec3;
using admc::solver::ConstraintInstance;
using admc::solver::IslandState;
using admc::solver::SimplePGSSolver;
using admc::solver::SolverBody;
using admc::world::RigidBody;

namespace
{
enum class SolverMode
{
    SimplePGS,
    Baseline
};

struct BenchConfig
{
    int iterations{8};
    float residual_threshold{1.0e-3f};
    float penetration_threshold{1.0e-3f};
    float joint_threshold{1.0e-3f};
    float admc_threshold{5.0e-4f};
    SolverMode solver{SolverMode::SimplePGS};
};

BenchConfig parse_config(int argc, char** argv)
{
    BenchConfig config{};
    for (int i = 1; i < argc; ++i)
    {
        std::string_view arg{argv[i]};
        auto parse_float = [](std::string_view value, float& out) {
            const float parsed = std::strtof(std::string(value).c_str(), nullptr);
            out = parsed;
        };
        if (arg.rfind("--iterations=", 0) == 0)
        {
            config.iterations = std::strtol(std::string(arg.substr(13)).c_str(), nullptr, 10);
        }
        else if (arg.rfind("--residual=", 0) == 0)
        {
            parse_float(arg.substr(11), config.residual_threshold);
        }
        else if (arg.rfind("--penetration=", 0) == 0)
        {
            parse_float(arg.substr(14), config.penetration_threshold);
        }
        else if (arg.rfind("--joint=", 0) == 0)
        {
            parse_float(arg.substr(8), config.joint_threshold);
        }
        else if (arg.rfind("--admc=", 0) == 0)
        {
            parse_float(arg.substr(7), config.admc_threshold);
        }
        else if (arg.rfind("--solver=", 0) == 0)
        {
            const std::string value = std::string(arg.substr(9));
            if (value == "baseline")
            {
                config.solver = SolverMode::Baseline;
            }
            else
            {
                config.solver = SolverMode::SimplePGS;
            }
        }
    }
    return config;
}

IslandState make_simple_island()
{
    IslandState island;
    island.bodies.push_back(SolverBody{
        .linear_velocity = Vec3{1.0f, -0.5f, 0.0f},
        .angular_velocity = Vec3{},
        .inverse_mass = 1.0f,
        .inverse_inertia = Mat3::identity()});
    island.bodies.push_back(SolverBody{
        .linear_velocity = Vec3{-0.5f, 0.25f, 0.0f},
        .angular_velocity = Vec3{},
        .inverse_mass = 1.0f,
        .inverse_inertia = Mat3::identity()});

    ConstraintInstance normal{};
    normal.body_a = 0;
    normal.body_b = 1;
    normal.row.type = admc::constraints::ConstraintType::ContactNormal;
    normal.row.jacobian_linear_a = Vec3{1.0f, 0.0f, 0.0f};
    normal.row.jacobian_linear_b = Vec3{-1.0f, 0.0f, 0.0f};
    normal.row.effective_mass = 0.5f;
    normal.row.lower_limit = 0.0f;
    normal.row.upper_limit = 10.0f;
    normal.row.bias = 0.0f;

    ConstraintInstance joint{};
    joint.body_a = 0;
    joint.body_b = 1;
    joint.row.type = admc::constraints::ConstraintType::Joint;
    joint.row.jacobian_linear_a = Vec3{0.0f, 1.0f, 0.0f};
    joint.row.jacobian_linear_b = Vec3{0.0f, -1.0f, 0.0f};
    joint.row.effective_mass = 0.5f;
    joint.row.lower_limit = -1.0f;
    joint.row.upper_limit = 1.0f;
    joint.row.bias = 0.1f;

    island.constraints.push_back(normal);
    island.constraints.push_back(joint);
    return island;
}

RigidBody make_rigidbody(const Vec3& position, const Vec3& velocity, float mass)
{
    RigidBody body{};
    body.position = position;
    body.orientation = Quat::identity();
    body.linear_velocity = velocity;
    body.angular_velocity = Vec3{};
    body.set_mass(mass);
    body.inertia_body = Mat3::identity();
    body.update_world_inertia();
    return body;
}

std::vector<RigidBody> make_baseline_bodies()
{
    std::vector<RigidBody> bodies;
    bodies.push_back(make_rigidbody(Vec3{-0.5f, 0.0f, 0.0f}, Vec3{1.0f, -0.5f, 0.0f}, 1.0f));
    bodies.push_back(make_rigidbody(Vec3{0.5f, 0.0f, 0.0f}, Vec3{-0.5f, 0.25f, 0.0f}, 1.0f));
    return bodies;
}

std::vector<ContactConstraint> make_baseline_contacts()
{
    ContactConstraint normal{};
    normal.body_a = 0;
    normal.body_b = 1;
    normal.point = Vec3{0.0f, 0.0f, 0.0f};
    normal.normal = Vec3{1.0f, 0.0f, 0.0f};
    normal.penetration = 0.05f;
    normal.restitution = 0.0f;
    return {normal};
}
} // namespace

int main(int argc, char** argv)
{
    const BenchConfig config = parse_config(argc, argv);

    if (config.solver == SolverMode::SimplePGS)
    {
        IslandState island = make_simple_island();

        SimplePGSSolver::Options options{};
        options.max_iterations = config.iterations;
        options.gate_settings.thresholds.residual = config.residual_threshold;
        options.gate_settings.thresholds.penetration = config.penetration_threshold;
        options.gate_settings.thresholds.joint = config.joint_threshold;
        options.gate_settings.thresholds.admc = config.admc_threshold;

        SimplePGSSolver solver{options};
        const auto result = solver.solve(island);

        std::cout << "[Simple PGS]\n";
        std::cout << "Iterations (max): " << config.iterations << '\n';
        std::cout << "Gate thresholds (residual/penetration/joint/admc): "
                  << config.residual_threshold << " / "
                  << config.penetration_threshold << " / "
                  << config.joint_threshold << " / "
                  << config.admc_threshold << '\n';
        std::cout << "Iterations: " << result.iterations << '\n';
        std::cout << "Residual: " << result.max_residual << '\n';
        std::cout << "ADMC drift: " << result.admc_drift << '\n';
        std::cout << "Body0 v: (" << island.bodies[0].linear_velocity.x << ", "
                  << island.bodies[0].linear_velocity.y << ", "
                  << island.bodies[0].linear_velocity.z << ")\n";
        std::cout << "Body1 v: (" << island.bodies[1].linear_velocity.x << ", "
                  << island.bodies[1].linear_velocity.y << ", "
                  << island.bodies[1].linear_velocity.z << ")\n";
    }
    else
    {
        std::vector<RigidBody> bodies = make_baseline_bodies();
        std::vector<ContactConstraint> contacts = make_baseline_contacts();

        BaselineParams params{};
        params.iterations = config.iterations;
        params.slop = config.penetration_threshold;
        params.beta = 0.25f;
        params.restitution = 0.0f;

        solve_baseline(bodies, contacts, params);

        const Vec3 relative = bodies[1].linear_velocity - bodies[0].linear_velocity;

        std::cout << "[Baseline AoS]\n";
        std::cout << "Iterations: " << params.iterations << '\n';
        std::cout << "Penetration slop: " << params.slop << '\n';
        std::cout << "Relative normal velocity after solve: " << relative.x << '\n';
        std::cout << "Body0 v: (" << bodies[0].linear_velocity.x << ", "
                  << bodies[0].linear_velocity.y << ", "
                  << bodies[0].linear_velocity.z << ")\n";
        std::cout << "Body1 v: (" << bodies[1].linear_velocity.x << ", "
                  << bodies[1].linear_velocity.y << ", "
                  << bodies[1].linear_velocity.z << ")\n";
    }

    return 0;
}
