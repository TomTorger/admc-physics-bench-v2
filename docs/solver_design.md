# Solver Layer Cheatsheet

> Contracts, not prose. All solvers must follow the same rules so we can swap them mid-bench.

---

## Position in the Stack

```
AoS world → assembly (ConstraintBatch) → solver_core / parallel → diagnostics
```

Solvers only see:

- `WorldSoA` view (velocities, inv mass, orientations).
- `ConstraintBatch` (tiles + local bodies).
- `IConservationTracker` (ADMC feedback).
- `SolverSettings`.

No solver touches AoS state directly.

---

## Key Interfaces

```cpp
struct SolverSettings {
    int   max_iterations;
    float dt;
    float baumgarte_beta;
    bool  enable_friction;
    bool  enable_joints;
    bool  deterministic;
};

struct SolverStats {
    int   islands;
    int   tiles;
    int   iterations;
    float ms_total;
    float ms_assembly;
    float ms_iterations;
    float max_penetration;
    float max_joint_error;
    float max_admc_drift;
};

void solve_pgs_single_thread(
    WorldSoA& world,
    ConstraintBatch& batch,
    const SolverSettings& settings,
    IConservationTracker* tracker);
```

Rules:

- Solver owns no global state; everything lives in the batch or tracker.
- Stats must be filled even when early exit occurs.
- Tracker hooks decide warm-start scaling and per-island iteration counts.

---

## Iteration Loop (reference)

```cpp
for (Island& island : batch.islands) {
    float warm = tracker ? tracker->warmstart_scale(island.id) : 1.0f;
    apply_warmstart(island, warm);

    for (int iter = 0; iter < settings.max_iterations; ++iter) {
        float residual = solve_tiles_once(island);
        if (!tracker) {
            if (residual < settings.convergence) break;
            continue;
        }
        IterationPolicy policy = tracker->on_iteration_end(island.id, residual);
        if (!policy.shouldContinue) break;
        if (!policy.focusThisIsland) break_when_everyone_else_done();
    }
}
```

Important:

- `solve_tiles_once` walks tiles in GS order (colors if provided).
- Trackers may request extra passes on “difficult” islands.
- Parallel schedulers wrap this loop but must honor the same policies.

---

## Solver Families (current + future)

| Family | Purpose | Notes |
|--------|---------|-------|
| `baseline::AoS` | Regression truth | Works on AoS rows, no tiles; kept for tests only. |
| `pgs::scalar` | Simple reference | Operates on row arrays; useful for debugging assembly without tiles. |
| `pgs::tile` | Default high-perf path | Uses SoA tiles, SIMD kernels, ADMC tracker for warm-start + iterations. |
| `simple::pgs` | Header-only toy solver | `admc/solver/simple_pgs.hpp` exercises the composite gate + manifold warm-start scaler. |
| `block::manifold` | Boost manifolds/joints | Small dense solves per manifold; plugs into tile pipeline. |
| `krylov::anderson` | Experimental acceleration | Wraps `pgs::tile` iterations with Anderson / Krylov steps. |
| `admm::prototype` | Research slot | Uses the same `ConstraintBatch`; proves we can reuse layout. |

Every new solver must:

1. Consume `WorldSoA`, `ConstraintBatch`, `SolverSettings`, `IConservationTracker`.
2. Emit `SolverStats`.
3. Leave AoS state untouched except through the provided interfaces.

---

## Testing + Bench Expectations

- Baseline vs tile solver parity checked on small deterministic scenes.
- Each solver publishes:
  - max penetration/joint error,
  - ADMC drift,
  - timings (warm-start, iterations, scatter).
- Bench harness can flip between solvers via CLI flag; no bespoke wiring allowed.
- Unit targets:
  - `core_tests` keep Vec3/Mat3/RigidBody math honest.
  - `solver_policy_tests` lock down iteration gates, warm-start scaling, and the constraint-operator surface.
  - `simple_pgs_tests` ensure the toy solver reduces relative velocities and respects impulse bounds.
  - `baseline_tests` keep the AoS sequential impulse reference solver aligned with the original repo.
- `scene_builders` + `simple_bench` + Python bench CLI:
  - `scene_library` exposes canned scenes (`two_spheres`, `box_stack_N`, `sphere_grid_4x4`, `sphere_grid_8x8`, `sphere_cloud_16x16/32x32/64x64`) for both solvers.
  - CLI flags: `--solver=simple|baseline|both`, `--scene=name`, `--csv path/to/results.csv`.
  - `python -m bench.cli --config <yaml>` orchestrates multi-scene sweeps and aggregates CSVs; CSV columns now include `total_ms`, `warm_ms`, `iteration_ms`, `iterations`, and `residual` (all sourced from solver instrumentation). Default configs set a high iteration cap (64) and lean on the adaptive gate thresholds to stop early.

Fail any of these checks and the solver does not land.
