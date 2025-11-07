#include "admc/solver/constraint_batch_cache.hpp"
#include "admc/solver/tile_pgs.hpp"

#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using admc::core::Mat3;
using admc::core::Vec3;
using admc::solver::ConstraintBatch;
using admc::solver::ConstraintBatchCache;
using admc::solver::ConstraintInstance;
using admc::solver::IslandState;
using admc::solver::TilePGSSolver;

namespace {
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

bool nearly_equal(float a, float b, float eps = 1e-6f)
{
    return std::fabs(a - b) <= eps;
}
} // namespace

// Construct a small island with 2 bodies and 3 constraints. The second
// constraint has lambda=0 so warm-start will skip applying an impulse.
// We verify residual indexing advances per-row so row 3 uses its own residual.
int main()
{
    IslandState island;
    island.bodies.resize(2);
    island.bodies[0].linear_velocity = Vec3{};
    island.bodies[0].angular_velocity = Vec3{};
    island.bodies[0].inverse_mass = 1.0f;
    island.bodies[0].inverse_inertia = Mat3::identity();
    island.bodies[1].linear_velocity = Vec3{};
    island.bodies[1].angular_velocity = Vec3{};
    island.bodies[1].inverse_mass = 1.0f;
    island.bodies[1].inverse_inertia = Mat3::identity();

    auto make_row = [](float lambda) {
        admc::constraints::ConstraintRow row{};
        row.type = admc::constraints::ConstraintType::ContactNormal;
        // Simple opposing linear Jacobians
        row.jacobian_linear_a = Vec3{-1.0f, 0.0f, 0.0f};
        row.jacobian_linear_b = Vec3{1.0f, 0.0f, 0.0f};
        row.jacobian_angular_a = Vec3{};
        row.jacobian_angular_b = Vec3{};
        row.effective_mass = 1.0f;
        row.bias = 0.0f;
        row.lower_limit = 0.0f;
        row.upper_limit = 10.0f;
        row.lambda = lambda;
        return row;
    };

    // Three constraints with distinct keys and lambdas [1.0, 0.0, 1.0]
    for (int i = 0; i < 3; ++i)
    {
        ConstraintInstance c{};
        c.body_a = 0;
        c.body_b = 1;
        c.row = make_row((i == 1) ? 0.0f : 1.0f);
        // Unique key via varying contact point x
        Vec3 pt{static_cast<float>(i), 0.0f, 0.0f};
        Vec3 n{1.0f, 0.0f, 0.0f};
        c.key = admc::solver::make_constraint_key(0, 1, pt, n);
        island.constraints.push_back(c);
    }

    ConstraintBatchCache cache;
    ConstraintBatch& batch = cache.rebuild(0, island);
    std::vector<float>& residuals = cache.residuals(0);

    // Set distinct residuals so misalignment is observable.
    // Using default warm-start scaler thresholds (residual_threshold = 1e-3):
    // scale = 1 - 0.75 * clamp(residual/1e-3, 0..1) when |lambda| > sleeping_impulse
    // r0=0.0005 => scale0=0.625; r1=0.0009 => scale1=0.325; r2=0.0002 => scale2=0.85
    if (residuals.size() < 3) residuals.resize(3, 0.0f);
    residuals[0] = 0.0005f;
    residuals[1] = 0.0009f; // will be skipped by warm-start (lambda=0) but must still advance index
    residuals[2] = 0.0002f;

    TilePGSSolver::Options opts{};
    opts.max_iterations = 0; // run warm-start only
    TilePGSSolver solver(opts);
    (void)solver.solve(island, batch, residuals, nullptr);

    // Inspect lambdas post warm-start. Row 0 and 2 should be scaled according to
    // residuals[0] and residuals[2], respectively. Row 1 stays zero.
    const auto& rows = batch.islands[0].tiles[0].rows;
    const float expected0 = 1.0f * 0.625f;
    const float expected2 = 1.0f * 0.85f;

    expect(nearly_equal(rows.lambda[0], expected0, 1e-5f), "Row 0 scaled by residual[0]");
    expect(nearly_equal(rows.lambda[1], 0.0f, 1e-8f), "Row 1 remains zero");
    expect(nearly_equal(rows.lambda[2], expected2, 1e-5f), "Row 2 scaled by residual[2]");

    std::cout << "Tile warm-start residual index test passed\n";
    return 0;
}

