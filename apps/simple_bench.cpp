#include "admc/baseline/solver.hpp"
#include "admc/scene/scene_builders.hpp"
#include "admc/scene/scene_library.hpp"
#include "admc/solver/conservation_tracker.hpp"
#include "admc/solver/constraint_batch_cache.hpp"
#include "admc/solver/simple_pgs.hpp"
#include "admc/solver/tile_pgs.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

using admc::baseline::BaselineParams;
using admc::baseline::solve_baseline;
using admc::scene::SceneDesc;
using admc::solver::SimplePGSSolver;

namespace
{
enum class SolverMask : unsigned
{
    Simple = 1u << 0,
    Baseline = 1u << 1,
    Both = Simple | Baseline
};

struct BenchConfig
{
    int iterations{8};
    int steps{1};
    float residual_threshold{1.0e-3f};
    float penetration_threshold{1.0e-3f};
    float joint_threshold{1.0e-3f};
    float admc_threshold{5.0e-4f};
    bool tile_diagnostics{true};
    float sor{1.0f};
    int tile_capacity{64};
    SolverMask solvers{SolverMask::Both};
    std::string scene_filter{};
    std::string csv_path{};
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
        else if (arg.rfind("--steps=", 0) == 0)
        {
            config.steps = std::strtol(std::string(arg.substr(8)).c_str(), nullptr, 10);
            if (config.steps < 1) config.steps = 1;
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
        else if (arg.rfind("--sor=", 0) == 0)
        {
            parse_float(arg.substr(6), config.sor);
        }
        else if (arg.rfind("--solver=", 0) == 0)
        {
            const std::string value = std::string(arg.substr(9));
            if (value == "baseline")
            {
                config.solvers = SolverMask::Baseline;
            }
            else if (value == "simple")
            {
                config.solvers = SolverMask::Simple;
            }
            else
            {
                config.solvers = SolverMask::Both;
            }
        }
        else if (arg.rfind("--tile-diagnostics=", 0) == 0)
        {
            const std::string value = std::string(arg.substr(19));
            if (value == "off" || value == "false" || value == "0")
            {
                config.tile_diagnostics = false;
            }
            else if (value == "on" || value == "true" || value == "1")
            {
                config.tile_diagnostics = true;
            }
        }
        else if (arg.rfind("--tile-capacity=", 0) == 0)
        {
            config.tile_capacity = std::strtol(std::string(arg.substr(16)).c_str(), nullptr, 10);
            if (config.tile_capacity <= 0) config.tile_capacity = 64;
        }
        else if (arg.rfind("--scene=", 0) == 0)
        {
            config.scene_filter = std::string(arg.substr(8));
        }
        else if (arg.rfind("--csv=", 0) == 0)
        {
            config.csv_path = std::string(arg.substr(6));
        }
    }
    return config;
}

struct BenchMeasurement
{
    std::string scene;
    std::string solver;
    std::size_t contacts{0};
    double assembly_ms{0.0};
    double total_ms{0.0};
    double warm_ms{0.0};
    double iteration_ms{0.0};
    int iterations{0};
    float residual{0.0f};
    float max_penetration{0.0f};
    float max_joint_error{0.0f};
    float admc_drift{0.0f};
    float tile_residual_min{0.0f};
    float tile_residual_max{0.0f};
    float tile_residual_p95{0.0f};
    float tile_drift_min{0.0f};
    float tile_drift_max{0.0f};
    float tile_drift_p95{0.0f};
};

bool has_solver(SolverMask mask, SolverMask flag)
{
    return (static_cast<unsigned>(mask) & static_cast<unsigned>(flag)) != 0;
}
} // namespace

int main(int argc, char** argv)
{
    const BenchConfig config = parse_config(argc, argv);
    const auto scenes = admc::scene::make_default_scene_library();

    BaselineParams baseline_params{};
    baseline_params.iterations = config.iterations;
    baseline_params.slop = config.penetration_threshold;
    baseline_params.beta = 0.25f;
    baseline_params.restitution = 0.0f;
    const float drift_vscale = 9.81f * baseline_params.dt;

    SimplePGSSolver::Options simple_options{};
    simple_options.max_iterations = config.iterations;
    simple_options.gate_settings.thresholds.residual = config.residual_threshold;
    simple_options.gate_settings.thresholds.penetration = config.penetration_threshold;
    simple_options.gate_settings.thresholds.joint = config.joint_threshold;
    simple_options.gate_settings.thresholds.admc = config.admc_threshold;
    simple_options.drift_velocity_scale = drift_vscale;
    admc::solver::AdaptiveConservationTracker simple_tracker(simple_options.gate_settings);
    admc::solver::ConstraintBatchCache batch_cache(static_cast<std::size_t>(config.tile_capacity));
    admc::solver::TilePGSSolver tile_solver(admc::solver::TilePGSSolver::Options{
        .max_iterations = simple_options.max_iterations,
        .gate_settings = simple_options.gate_settings,
        .warmstart_settings = simple_options.warmstart_settings,
        .collect_tile_diagnostics = config.tile_diagnostics,
        .tile_diagnostics_last_iteration_only = true,
        .sor = config.sor,
        .drift_velocity_scale = drift_vscale});

    std::vector<BenchMeasurement> measurements;

    for (std::size_t scene_index = 0; scene_index < scenes.size(); ++scene_index)
    {
        const SceneDesc& scene = scenes[scene_index];
        if (!config.scene_filter.empty() && scene.name != config.scene_filter)
        {
            continue;
        }

        if (has_solver(config.solvers, SolverMask::Baseline))
        {
            auto baseline_scene = admc::scene::build_baseline_scene(scene, baseline_params);
            const auto metrics = solve_baseline(baseline_scene.bodies, baseline_scene.contacts, baseline_params);

            measurements.push_back(BenchMeasurement{
                scene.name,
                "baseline",
                baseline_scene.contacts.size(),
                0.0,
                metrics.warmstart_ms + metrics.iteration_ms,
                metrics.warmstart_ms,
                metrics.iteration_ms,
                metrics.iterations,
                metrics.max_residual,
                /*max_penetration*/ 0.0f,
                /*max_joint_error*/ 0.0f,
                /*admc_drift*/ 0.0f,
                /*tile_residual_min*/ 0.0f,
                /*tile_residual_max*/ 0.0f,
                /*tile_residual_p95*/ 0.0f,
                /*tile_drift_min*/ 0.0f,
                /*tile_drift_max*/ 0.0f,
                /*tile_drift_p95*/ 0.0f});
        }

        if (has_solver(config.solvers, SolverMask::Simple))
        {
            auto island = admc::scene::build_simple_island(scene, baseline_params);
            std::vector<float>& residuals = batch_cache.residuals(scene_index);

            double assembly_accum_ms = 0.0;
            admc::solver::TilePGSSolver::SolverResult last_result{};
            for (int step = 0; step < config.steps; ++step)
            {
                if (step == 0)
                {
                    // Cold build measured as assembly
                    const auto assembly_start = std::chrono::steady_clock::now();
                    (void)batch_cache.rebuild(scene_index, island);
                    const auto assembly_end = std::chrono::steady_clock::now();
                    assembly_accum_ms += std::chrono::duration<double, std::milli>(assembly_end - assembly_start).count();
                }
                else
                {
                    // Reuse batch across frames; no assembly work
                }
                // Solve using the cached batch
                admc::solver::ConstraintBatch& batch = batch_cache.get(scene_index).batch;
                last_result = tile_solver.solve(island, batch, residuals, &simple_tracker);
            }
            const double assembly_ms = (config.steps > 0) ? (assembly_accum_ms / static_cast<double>(config.steps)) : 0.0;

            measurements.push_back(BenchMeasurement{
                scene.name,
                "simple_pgs",
                island.constraints.size(),
                assembly_ms,
                last_result.total_ms,
                last_result.warmstart_ms,
                last_result.iteration_ms,
                last_result.iterations,
                last_result.max_residual,
                last_result.max_penetration_error,
                last_result.max_joint_error,
                last_result.admc_drift,
                last_result.tile_residual_min,
                last_result.tile_residual_max,
                last_result.tile_residual_p95,
                last_result.tile_drift_min,
                last_result.tile_drift_max,
                last_result.tile_drift_p95});
        }
    }

    if (measurements.empty())
    {
        std::cout << "No scenes matched the provided filters.\n";
        return 0;
    }

    std::cout << "Scene           Solver        Contacts  Assembly  Total(ms)  Warm(ms)  Iter(ms)  Iterations  Residual   Penetr.   JointErr   ADMC     tResMin   tResP95   tResMax   tDrMin    tDrP95    tDrMax\n";
    for (const BenchMeasurement& m : measurements)
    {
        std::cout << std::left << std::setw(15) << m.scene << ' '
                  << std::setw(12) << m.solver
                  << std::right << std::setw(9) << m.contacts << ' '
                  << std::fixed << std::setprecision(3)
                  << std::setw(9) << m.assembly_ms << ' '
                  << std::fixed << std::setprecision(3)
                  << std::setw(10) << m.total_ms << ' '
                  << std::setw(9) << m.warm_ms << ' '
                  << std::setw(9) << m.iteration_ms << ' '
                  << std::defaultfloat
                  << std::setw(11) << m.iterations << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.residual << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.max_penetration << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.max_joint_error << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.admc_drift << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.tile_residual_min << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.tile_residual_p95 << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.tile_residual_max << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.tile_drift_min << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.tile_drift_p95 << ' '
                  << std::fixed << std::setprecision(4) << std::setw(9) << m.tile_drift_max << '\n'
                  << std::defaultfloat;
    }

    if (!config.csv_path.empty())
    {
        std::filesystem::path path = config.csv_path;
        if (path.has_parent_path())
        {
            std::filesystem::create_directories(path.parent_path());
        }
        std::ofstream csv(path);
        if (csv.is_open())
        {
            csv << "scene,contacts,solver,total_ms,warm_ms,iteration_ms,assembly_ms,iterations,residual,max_penetration,max_joint_error,admc_drift,tile_residual_min,tile_residual_p95,tile_residual_max,tile_drift_min,tile_drift_p95,tile_drift_max\n";
            for (const BenchMeasurement& m : measurements)
            {
                csv << m.scene << ','
                    << m.contacts << ','
                    << m.solver << ','
                    << std::fixed << std::setprecision(6) << m.total_ms << ','
                    << m.warm_ms << ','
                    << m.iteration_ms << ','
                    << m.assembly_ms << ','
                    << m.iterations << ','
                    << m.residual << ','
                    << m.max_penetration << ','
                    << m.max_joint_error << ','
                    << m.admc_drift << ','
                    << m.tile_residual_min << ','
                    << m.tile_residual_p95 << ','
                    << m.tile_residual_max << ','
                    << m.tile_drift_min << ','
                    << m.tile_drift_p95 << ','
                    << m.tile_drift_max << '\n';
            }
            std::cout << "CSV written to " << path << '\n';
        }
        else
        {
            std::cerr << "Failed to open CSV path: " << path << '\n';
            return 1;
        }
    }

    return 0;
}
