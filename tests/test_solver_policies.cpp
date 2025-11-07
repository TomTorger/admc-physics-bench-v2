#include "admc/solver/iteration_control.hpp"
#include "admc/solver/operators.hpp"
#include "admc/solver/warmstart.hpp"

#include "admc/constraints/constraint_row.hpp"
#include "admc/core/mat3.hpp"
#include "admc/core/vec3.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using admc::admc::DirectionalTracker;
using admc::core::Mat3;
using admc::core::Vec3;
using admc::solver::BodyImpulse;
using admc::solver::BodyInertiaView;
using admc::solver::BodyPairImpulse;
using admc::solver::BodyStateView;
using admc::solver::CompositeIterationGate;
using admc::solver::ConstraintOperator;
using admc::solver::DriftSample;
using admc::solver::IterationPolicy;
using admc::solver::IterationSignals;
using admc::solver::ManifoldWarmstartSample;
using admc::solver::ManifoldWarmstartScaler;
using admc::solver::make_drift_sample;

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

bool nearly_equal(float a, float b, float eps = 1e-5f)
{
    return std::fabs(a - b) <= eps;
}
} // namespace

void test_iteration_gate()
{
    CompositeIterationGate gate{
        CompositeIterationGate::Settings{
            .thresholds = {.residual = 1e-3f, .penetration = 1e-3f, .joint = 1e-3f, .admc = 5e-4f},
            .focus_ratio = 3.0f}};

    IterationSignals settled{
        .constraint_residual = 2e-4f,
        .penetration_error = 2e-4f,
        .joint_error = 2e-4f,
        .admc_drift = 1e-4f};
    IterationPolicy policy = gate.evaluate(settled);
    expect(!policy.should_continue, "Gate stops when all signals are quiet");

    IterationSignals residual_issue{
        .constraint_residual = 2e-3f,
        .penetration_error = 2e-4f,
        .joint_error = 2e-4f,
        .admc_drift = 1e-4f};
    policy = gate.evaluate(residual_issue);
    expect(policy.should_continue && !policy.focus_island, "Residuals alone do not focus island");

    IterationSignals drift_issue{
        .constraint_residual = 2e-3f,
        .penetration_error = 2e-4f,
        .joint_error = 2e-4f,
        .admc_drift = 2.0e-3f};
    policy = gate.evaluate(drift_issue);
    expect(policy.should_continue && policy.focus_island, "Drift + residual triggers focus");
}

void test_iteration_gate_penetration_guard()
{
    CompositeIterationGate gate{
        CompositeIterationGate::Settings{
            .thresholds = {.residual = 1e-3f, .penetration = 1e-4f, .joint = 1e-3f, .admc = 5e-4f},
            .focus_ratio = 2.0f}};

    IterationSignals penetration_issue{
        .constraint_residual = 5e-5f,
        .penetration_error = 5e-3f,
        .joint_error = 1e-5f,
        .admc_drift = 1e-5f};
    const IterationPolicy policy = gate.evaluate(penetration_issue);
    expect(policy.should_continue, "Penetration alone must keep iterating");
}

void test_warmstart_scaler()
{
    const ManifoldWarmstartScaler::Settings settings{};
    ManifoldWarmstartScaler scaler{settings};
    const ManifoldWarmstartSample sleeping{
        .residual_norm = 5e-5f,
        .admc_drift = 1e-5f,
        .impulse_magnitude = 1e-4f};
    expect(nearly_equal(scaler.scale_for(sleeping), 1.0f), "Sleeping manifolds remain fully warm-started");

    const ManifoldWarmstartSample stressed{
        .residual_norm = 5e-3f,
        .admc_drift = 1e-3f,
        .impulse_magnitude = 0.05f};
    const float scale = scaler.scale_for(stressed);
    expect(scale < 1.0f && scale > 0.2f, "Stressed manifold is damped but clamped");

    const ManifoldWarmstartSample samples[] = {sleeping, stressed};
    const std::vector<float> table = scaler.scale_all(samples);
    expect(table.size() == 2, "scale_all preserves size");
    expect(nearly_equal(table[0], 1.0f), "scale_all matches first value");
    expect(nearly_equal(table[1], scale), "scale_all matches second value");

    const ManifoldWarmstartSample saturated{
        .residual_norm = 1.0f,
        .admc_drift = 1.0f,
        .impulse_magnitude = 1.0f};
    expect(nearly_equal(scaler.scale_for(saturated), settings.min_scale), "Scale clamps to minimum");
}

void test_drift_sample()
{
    DirectionalTracker tracker;
    const float mass = 3.0f;
    tracker.record_pre(mass, Vec3{1.0f, -2.0f, 0.5f});
    tracker.record_post(mass, Vec3{0.0f, -1.0f, 0.0f});
    const DriftSample sample = make_drift_sample(tracker);
    expect(sample.max_channel_drift > 0.0f, "Channel drift is positive");
    const float expected_norm = std::sqrt(1.0f * 1.0f + 1.0f * 1.0f + 0.5f * 0.5f);
    expect(nearly_equal(sample.momentum_norm, expected_norm), "Momentum norm matches Euclidean drift");
}

void test_constraint_operator()
{
    BodyStateView states[] = {
        {.linear_velocity = Vec3{1.0f, 0.0f, 0.0f}, .angular_velocity = Vec3{}},
        {.linear_velocity = Vec3{-1.0f, 0.0f, 0.0f}, .angular_velocity = Vec3{}}};

    BodyInertiaView inertia[] = {
        {.inverse_mass = 0.5f, .inverse_inertia = Mat3::identity()},
        {.inverse_mass = 1.0f, .inverse_inertia = Mat3::identity()}};

    ConstraintOperator op(states, inertia);

    admc::constraints::ConstraintRow row{};
    row.jacobian_linear_a = Vec3{1.0f, 0.0f, 0.0f};
    row.jacobian_linear_b = Vec3{-1.0f, 0.0f, 0.0f};
    row.jacobian_angular_a = Vec3{0.0f, 1.0f, 0.0f};
    row.jacobian_angular_b = Vec3{0.0f, -1.0f, 0.0f};

    const float rel = op.apply_jacobian(row, 0, 1);
    expect(nearly_equal(rel, 2.0f), "Jacobian operator matches relative velocity");

    const BodyPairImpulse pair = op.apply_jacobian_transpose(row, 0.5f);
    expect(nearly_equal(pair.body_a.linear.x, 0.5f), "Transpose linear A");
    expect(nearly_equal(pair.body_b.linear.x, -0.5f), "Transpose linear B");

    const BodyImpulse applied = {.linear = Vec3{2.0f, 0.0f, 0.0f}, .angular = Vec3{0.0f, 1.0f, 0.0f}};
    const BodyImpulse response = op.apply_inverse_mass(0, applied);
    expect(nearly_equal(response.linear.x, 1.0f), "Inverse mass scales linear impulses");
    expect(nearly_equal(response.angular.y, 1.0f), "Inverse inertia identity");
}

void test_constraint_operator_anisotropic_inertia()
{
    BodyStateView states[] = {
        {.linear_velocity = Vec3{}, .angular_velocity = Vec3{}},
        {.linear_velocity = Vec3{}, .angular_velocity = Vec3{}}};

    Mat3 inv_inertia(
        Vec3{2.0f, 0.0f, 0.0f},
        Vec3{0.0f, 0.5f, 0.0f},
        Vec3{0.0f, 0.0f, 1.0f});
    BodyInertiaView inertia[] = {
        {.inverse_mass = 1.0f, .inverse_inertia = inv_inertia},
        {.inverse_mass = 1.0f, .inverse_inertia = Mat3::identity()}};

    ConstraintOperator op(states, inertia);
    const BodyImpulse impulse = {.linear = Vec3{}, .angular = Vec3{1.0f, 1.0f, 1.0f}};
    const BodyImpulse response = op.apply_inverse_mass(0, impulse);
    expect(nearly_equal(response.angular.x, 2.0f), "Anisotropic inertia scales X");
    expect(nearly_equal(response.angular.y, 0.5f), "Anisotropic inertia scales Y");
    expect(nearly_equal(response.angular.z, 1.0f), "Anisotropic inertia scales Z");
}

int main()
{
    test_iteration_gate();
    test_iteration_gate_penetration_guard();
    test_warmstart_scaler();
    test_drift_sample();
    test_constraint_operator();
    test_constraint_operator_anisotropic_inertia();
    std::cout << "Solver policy tests passed\n";
    return 0;
}
