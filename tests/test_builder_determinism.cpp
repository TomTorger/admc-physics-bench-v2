#include "admc/solver/constraint_batch_cache.hpp"
#include "admc/solver/constraint_key.hpp"
#include "admc/solver/simple_pgs.hpp"

#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"

#include <cstdlib>
#include <iostream>

using admc::core::Mat3;
using admc::core::Vec3;
using admc::solver::ConstraintBatch;
using admc::solver::ConstraintBatchCache;
using admc::solver::ConstraintInstance;
using admc::solver::IslandState;

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
} // namespace

static IslandState make_island()
{
    IslandState island;
    // 4 bodies
    for (int i = 0; i < 4; ++i)
    {
        admc::solver::SolverBody b{};
        b.linear_velocity = Vec3{};
        b.angular_velocity = Vec3{};
        b.inverse_mass = 1.0f;
        b.inverse_inertia = Mat3::identity();
        island.bodies.push_back(b);
    }
    // 10 constraints between varying pairs; unique points ensure unique keys.
    for (int i = 0; i < 10; ++i)
    {
        const int a = i % 3;
        const int b = (i + 1) % 4;
        ConstraintInstance c{};
        c.body_a = a;
        c.body_b = b;
        c.row.type = admc::constraints::ConstraintType::ContactNormal;
        c.row.jacobian_linear_a = Vec3{-1.0f, 0.0f, 0.0f};
        c.row.jacobian_linear_b = Vec3{1.0f, 0.0f, 0.0f};
        c.row.jacobian_angular_a = Vec3{};
        c.row.jacobian_angular_b = Vec3{};
        c.row.effective_mass = 1.0f;
        c.row.lower_limit = 0.0f;
        c.row.upper_limit = 10.0f;
        c.row.bias = 0.0f;
        Vec3 pt{static_cast<float>(i), 0.0f, 0.0f};
        Vec3 n{1.0f, 0.0f, 0.0f};
        c.key = admc::solver::make_constraint_key(a, b, pt, n);
        island.constraints.push_back(c);
    }
    return island;
}

int main()
{
    IslandState island = make_island();

    ConstraintBatchCache cache(4); // tile capacity small to create multiple tiles deterministically
    ConstraintBatch& batch1 = cache.rebuild(0, island);
    // Save keys
    std::vector<admc::solver::ConstraintKey> keys1;
    for (const auto& isl : batch1.islands)
        for (const auto& tile : isl.tiles)
            for (const auto& k : tile.rows.keys)
                keys1.push_back(k);

    // Rebuild again without changes
    ConstraintBatch& batch2 = cache.rebuild(0, island);
    std::vector<admc::solver::ConstraintKey> keys2;
    for (const auto& isl : batch2.islands)
        for (const auto& tile : isl.tiles)
            for (const auto& k : tile.rows.keys)
                keys2.push_back(k);

    expect(keys1.size() == keys2.size(), "Key counts match");
    for (std::size_t i = 0; i < keys1.size(); ++i)
    {
        expect(keys1[i] == keys2[i], "Key order stable across rebuilds");
    }

    std::cout << "Builder determinism test passed\n";
    return 0;
}

