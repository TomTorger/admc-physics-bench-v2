# Benchmarks

> This document describes the **benchmark methodology** for the next-gen ADMC solver repo: what we measure, which scenes we use, how the harness is structured, and how to interpret results. It builds on the theory docs (for metrics) and the architecture docs (for solvers/layouts).

---

## 1. Purpose & Scope

Benchmarks in this repo serve three main purposes:

1. **Performance evaluation**  
   - Compare solver families (baseline AoS, SoA tile PGS, block solvers, etc.).
   - Compare backends (single-thread, multi-thread, GPU when available).
   - Study scaling with contact count, bodies, and timestep.

2. **Quality evaluation**  
   - Measure constraint satisfaction (penetration, joint error).
   - Measure conservation properties (ADMC-style directional momentum, energy drift).
   - Check friction cone consistency and other invariants.

3. **Regression testing & tuning**  
   - Detect performance regressions.
   - Ensure new solver features don’t silently degrade stability or invariants.
   - Provide guidance on parameter defaults.

The benchmark suite is **not** a full game/production metric; it is a **controlled environment** designed to stress specific aspects of the solver.

---

## 2. What We Measure

### 2.1 Performance Metrics

For each solver / backend / scene configuration we record:

- **Timings (ms/step)**:
  - Total solver step time.
  - Sub-phases when available:
    - Contact / joint generation (if included).
    - Row build / tiling.
    - Warm-start.
    - Iteration loop.
    - Integration.
- **Throughput**:
  - Contacts per second.
  - Bodies per second.
  - Iterations per second.

We generally run multiple steps (or repetitions) and compute:

- Min / median / max time.
- Optional trimmed mean.

### 2.2 Quality Metrics

From `metrics/` and `admc/` modules:

- **Constraint metrics**:
  - Max / RMS penetration (contacts).
  - Max / RMS joint positional error.
- **ADMC/invariant metrics**:
  - Max / RMS directional momentum drift across a fixed set of directions.
  - Optional energy drift (kinetic).
- **Friction consistency**:
  - Max / RMS cone violation (how far tangential impulses leave the Coulomb cone).

We may also log:

- Number of clamped rows (normals, friction, joints).
- Final residual norms (if exposed by the solver).

### 2.3 Solver-Internal Metrics

Optional, but useful when tuning:

- Number of iterations actually used (per run).
- Iteration-by-iteration residual decay (sampled).
- Per-tile / per-island residuals (for adaptive schemes).

---

## 3. Benchmark Scenes

Benchmark scenes are defined under `include/admc/scenes/` and implemented in `src/scenes/`. They are designed to cover:

- Different **contact counts** (sparse → dense).
- Different **structure** (stacks, clouds, chains, joints).
- Different **constraint types** (contacts, friction, joints).

### 3.1 Core Scene Categories

#### 3.1.1 Tiny / Micro Scenes

Examples:

- `two_spheres`  
  - Two spheres colliding and separating / resting.  
  - Purpose: correctness, timing noise floor, sanity check.

- `box_stack_4`  
  - Small stack of 4 boxes.  
  - Purpose: basic stacking stability, jointless but interesting.

These scenes are useful for:

- Seeing baseline vs advanced solvers with almost zero overhead.
- Verifying that optimization changes don’t break the simplest setups.

#### 3.1.2 Medium Scenes (Clouds, Small Stacks)

Examples:

- `spheres_cloud_1024`  
  - ~1024 spheres in a loose cloud, colliding under gravity.
- `box_stack_medium`  
  - ~20–50 boxes in a taller stack.

Purpose:

- Check scaling behavior where contact counts are significant but manageable.
- Compare AoS vs SoA vs tile solvers on more realistic workloads.

#### 3.1.3 Large Scenes (High-Contact Stress)

Examples:

- `spheres_cloud_10k`  
  - ~10,000 spheres in a dense region.  
- `spheres_cloud_50k`  
  - ~50,000 contacts or more.  
- `random_pile`  
  - Mixed shapes scrambled into a pile.

Purpose:

- Stress memory hierarchy and solver scaling.
- See where SoA/tile solvers shine compared to baseline.

#### 3.1.4 Joint & Articulation Scenes

Examples:

- `simple_pendulum`  
  - Chain of 5–10 bodies with hinge joints.
- `ragdoll_cluster`  
  - Several ragdoll skeletons with standard joint limits.
- `bridge_or_chain`  
  - Chain/bridge of boxes with joints.

Purpose:

- Test joint constraint quality:
  - Joint error drift.
  - Stability at various timesteps and iteration counts.

#### 3.1.5 Specialty / Edge Case Scenes

Examples:

- `bouncy_contacts` (high restitution).
- `high_friction_pile` (large μ).
- `zero_gravity_cluster` (conservation emphasis).

Purpose:

- Stress specific aspects of the solver (restitution, friction, conservation).

---

## 4. Solvers & Backends Under Test

The benchmark harness treats each **solver + backend** combination as a named “solver config”. Typical examples:

- `baseline_aos`  
  - Legacy AoS PGS solver, single-thread.

- `pgs_scalar_soa`  
  - Scalar PGS over a flat row array, SoA row layout, single-thread.

- `pgs_tile_soa_st`  
  - SoA tile-based PGS, single-thread backend.

- `pgs_tile_soa_mt`  
  - SoA tile-based PGS, multi-thread backend.

- `block_pgs_tile_soa`  
  - Block PGS (manifolds / joint blocks) on top of tile-based layout.

- `xpbd_tile` (optional)  
  - Position-based solver variant for compatible scenes.

The bench app can:

- Enumerate available solvers.
- Allow selection of one or more solvers for comparison in a single run.

---

## 5. Benchmark Harness Design

### 5.1 Command-Line Interface (CLI)

The `apps/bench/main_bench.cpp` program exposes a CLI like:

```bash
bench \
  --scene spheres_cloud_10k \
  --solver pgs_tile_soa_mt \
  --steps 200 \
  --warmup-steps 50 \
  --threads 8 \
  --dt 0.0166667 \
  --output results.json
````

Common flags:

* `--scene <name>`
  Selects the scene generator.

* `--solver <name>` (repeatable)
  One or more solver configs to run.

* `--steps N`
  Number of simulated steps for metrics.

* `--warmup-steps N`
  Steps to discard for timing (steady-state warmup).

* `--dt <seconds>`
  Timestep.

* `--threads N`
  Hint for multi-thread backends.

* `--repetitions R`
  Repeat full run R times (for statistical robustness).

* `--output <file>`
  JSON or CSV file for results.

* `--seed <int>`
  Seed for randomized scenes to make runs reproducible.

### 5.2 Output Formats

The harness prints:

* **Human-readable tables** to stdout:

  * Times per solver.
  * Key quality metrics.

* **Machine-readable output** (JSON/CSV) to file:

  * Full breakdown of timings, metrics, and solver stats.

JSON sketch:

```json
{
  "scene": "spheres_cloud_10k",
  "dt": 0.0166667,
  "steps": 200,
  "solvers": [
    {
      "name": "pgs_tile_soa_mt",
      "params": { "max_iterations": 10, "threads": 8 },
      "timings": {
        "total_ms_per_step": 10.47,
        "row_build_ms": 2.3,
        "solve_ms": 7.5,
        "integration_ms": 0.6
      },
      "metrics": {
        "max_penetration": 0.0025,
        "rms_penetration": 0.0003,
        "max_joint_error": 0.0,
        "max_dir_momentum_drift": 0.0012,
        "rms_dir_momentum_drift": 0.0001,
        "max_cone_violation": 0.05
      }
    }
  ]
}
```

---

## 6. Measurement Protocol

To minimize noise and improve comparability:

1. **Warmup phase**

   * Run `warmup-steps` without recording timings or metrics:

     * JIT / caches settle.
     * Warm-start impulses stabilize.
2. **Measurement phase**

   * Measure `steps` steps:

     * Timer per step or per block of steps.
   * Log metrics at:

     * Every step, or
     * Selected checkpoints (e.g. at the end).
3. **Repetitions**

   * Optionally repeat entire run `R` times:

     * Use min/median for timing.
     * Take max/mean for metrics (e.g., worst penetration across repetitions).
4. **System noise considerations**

   * Benchmarks are typically run with:

     * Fixed CPU frequency (no turbo) if possible.
     * Minimal background load.
   * These are guidelines; the harness does not enforce system configuration.

---

## 7. Interpreting Results

### 7.1 Performance Comparisons

When comparing solvers:

* Look at **ms/step** and **contacts per second** for each scene.
* Note where solver algorithms trade speed vs quality:

  * Some may be faster but allow more penetration/joint error.
* For multi-thread backends:

  * Check scaling: speedup vs thread count.
  * Watch for plateaus or regressions (contention, overhead).

### 7.2 Quality vs Performance Trade-offs

Use quality metrics alongside timings:

* If a solver is faster but has:

  * Larger penetration.
  * Higher momentum drift.
  * More cone violations.
* Decide whether the quality is acceptable for the intended use case:

  * Benchmarks don’t dictate policy; they give data.

This repo aims to make such trade-offs explicit.

### 7.3 Regression Detection

In CI or manual checks:

* Maintain **baseline result snapshots**:

  * JSON files for key scenes and solvers.
* On changes:

  * Run the same configs.
  * Compare:

    * Timings (with tolerance).
    * Quality metrics (with tolerance).
* Highlight regressions where:

  * Quality worsens beyond allowed thresholds.
  * Performance drops significantly.

---

## 8. Extending the Benchmark Suite

Adding a new scene:

1. Implement a generator in `src/scenes/`.
2. Expose a factory in `include/admc/scenes/scene_factory.hpp`.
3. Document it in this file (short description + purpose).

Adding a new solver config:

1. Implement a solver that conforms to `ISolver`.
2. Register it in the bench app’s solver registry.
3. Optionally define a “preset name” and default parameters.

Adding new metrics:

1. Implement computations in `metrics/` and/or `admc/`.
2. Plumb them into the harness:

   * Update `SolverStats` or a separate metrics struct.
3. Include them in JSON/CSV output and tables.

---

## 9. Summary

* The benchmark suite evaluates **speed**, **stability**, and **conservation** across scenes, solvers, and backends.
* It is built around:

  * A flexible CLI harness.
  * A consistent set of scenes.
  * ADMC-inspired metrics and constraint quality indicators.
* Results feed directly into:

  * Solver design decisions.
  * Architecture refinements.
  * Regression tests and documentation.

By keeping benchmarks integrated and well-documented, we ensure that solver and architecture experiments are always grounded in **measurable, comparable outcomes**.
