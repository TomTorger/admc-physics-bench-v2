#include "admc/solver/simple_pgs.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using admc::core::Mat3;
using admc::core::Vec3;
using admc::solver::ConstraintInstance;
using admc::solver::IslandState;
using admc::solver::SimplePGSSolver;
using admc::solver::SolverBody;

namespace
{
constexpr float kTolerance = 1e-3f;

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

IslandState make_basic_contact_island()
{
    IslandState island;
    island.bodies.push_back(SolverBody{
        .linear_velocity = Vec3{-1.0f, 0.0f, 0.0f},
        .angular_velocity = Vec3{},
        .inverse_mass = 1.0f,
        .inverse_inertia = Mat3::identity()});
    island.bodies.push_back(SolverBody{
        .linear_velocity = Vec3{1.0f, 0.0f, 0.0f},
        .angular_velocity = Vec3{},
        .inverse_mass = 1.0f,
        .inverse_inertia = Mat3::identity()});

    ConstraintInstance constraint{};
    constraint.body_a = 0;
    constraint.body_b = 1;
    constraint.row.type = admc::constraints::ConstraintType::ContactNormal;
    constraint.row.jacobian_linear_a = Vec3{1.0f, 0.0f, 0.0f};
    constraint.row.jacobian_linear_b = Vec3{-1.0f, 0.0f, 0.0f};
    constraint.row.effective_mass = 0.5f;
    constraint.row.lower_limit = 0.0f;
    constraint.row.upper_limit = 10.0f;
    constraint.row.bias = 0.0f;
    island.constraints.push_back(constraint);

    return island;
}
} // namespace

void test_contact_converges()
{
    IslandState island = make_basic_contact_island();
    SimplePGSSolver solver;
    const auto result = solver.solve(island);

    expect(result.iterations > 0, "Solver performed iterations");
    expect(result.max_residual < 1e-2f, "Residual shrinks");

    const Vec3 rel = island.bodies[0].linear_velocity - island.bodies[1].linear_velocity;
    expect(std::fabs(rel.x) < kTolerance, "Relative velocity damped");
}

void test_impulse_limits()
{
    IslandState island = make_basic_contact_island();
    island.constraints[0].row.upper_limit = 0.25f;
    island.constraints[0].row.bias = -2.0f;

    SimplePGSSolver solver;
    solver.solve(island);

    expect(std::fabs(island.constraints[0].row.lambda) <= 0.25f + kTolerance, "Impulse limit enforced");
}

int main()
{
    test_contact_converges();
    test_impulse_limits();
    std::cout << "Simple PGS tests passed\n";
    return 0;
}
