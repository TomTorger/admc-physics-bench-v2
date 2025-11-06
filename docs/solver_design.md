# Solver Design

> This document describes the **solver layer** in the next-gen ADMC engine: interfaces, families of solvers, how they interact with layout/backends, and how they expose metrics. It assumes familiarity with `architecture_overview.md` and the theory docs.

---

## 1. Scope & Position in the Stack

The solver layer sits between:

- **Inputs**
  - World state (bodies, materials).
  - Constraint data (contacts, joints → constraint rows).
  - Layout views (AoS, SoA, tiles, islands).
  - Execution backend (single/multi-thread, GPU).
- **Outputs**
  - Updated velocities (and optionally positions).
  - Solver stats + physical metrics (drift, penetration, etc.).

The layer’s responsibilities:

- Implement different **solver algorithms** (PGS, block PGS, etc.).
- Use constraint data produced by `constraints` + `layout`.
- Be callable through a **uniform interface** so the bench app and tests can swap solvers easily.

---

## 2. Solver Interfaces

### 2.1 Solver Parameters

We define a shared parameter struct with family-specific extensions:

```cpp
struct SolverParams {
    int    max_iterations       = 10;
    float  dt                   = 1.0f / 60.0f;
    float  baumgarte_beta       = 0.1f;
    float  restitution_scale    = 1.0f;
    float  warmstart_scale      = 1.0f;
    float  convergence_threshold = 1e-3f;

    bool   enable_friction      = true;
    bool   enable_joints        = true;
    bool   enable_warmstart     = true;

    // Backend hints:
    int    thread_count         = 0;      // 0 = "auto"
    int    tile_size            = 64;
    bool   prefer_determinism   = false;
};
````

Solver families can derive from or wrap `SolverParams` with additional fields:

* Block solvers: block size options, manifold aggregation thresholds.
* Krylov / ADMM: number of inner iterations, history depth.
* XPBD: compliance/stiffness parameters.

### 2.2 Solver Stats & Debug Info

Solvers should report:

```cpp
struct SolverStats {
    int   iterations_used = 0;
    int   tiles_processed = 0;
    int   islands_processed = 0;

    float total_ms            = 0.0f;
    float warmstart_ms        = 0.0f;
    float iteration_ms        = 0.0f;
    float integration_ms      = 0.0f;

    // Aggregated metrics:
    float max_penetration     = 0.0f;
    float rms_penetration     = 0.0f;
    float max_joint_error     = 0.0f;
    float rms_joint_error     = 0.0f;

    float max_dir_momentum_drift = 0.0f;
    float rms_dir_momentum_drift = 0.0f;
};
```

More detailed per-solver debug info can be exposed via an optional struct, e.g.:

```cpp
struct SolverDebugInfo {
    int   num_clamped_normals      = 0;
    int   num_clamped_friction     = 0;
    int   num_cone_projections     = 0;
    int   num_singular_rows        = 0;

    float max_row_residual         = 0.0f;
    float rms_row_residual         = 0.0f;
};
```

### 2.3 Abstract Solver Interface

All solver families implement a common interface:

```cpp
class ISolver {
public:
    virtual ~ISolver() = default;

    virtual const char* name() const = 0;

    virtual void solve(
        WorldView&                 world,   // AoS or SoA view
        ConstraintView&            constraints, // contacts + joints → rows or tiles
        const SolverParams&        params,
        SolverStats&               out_stats,
        SolverDebugInfo*           out_debug = nullptr
    ) = 0;
};
```

Notes:

* `WorldView` and `ConstraintView` are **layout abstractions**:

  * For baseline: AoS view over bodies & contacts.
  * For SoA/tile PGS: island/tile views.
* Backends (single/multi-thread, GPU) are passed to or configured inside `WorldView` / `ConstraintView`, or via additional parameters if needed.

---

## 3. Data Model Seen by Solvers

### 3.1 World Views

Solvers never own the world; they see:

* `WorldView`:

  * Access to body velocities and mass properties:

    * `get_linear_velocity(BodyId)`,
    * `get_angular_velocity(BodyId)`,
    * `apply_impulse(BodyId, Vec3 linear, Vec3 angular)`, etc.
  * Layout-specific implementations:

    * `AoSWorldView`, `SoAWorldView`, `TileWorldView`.

### 3.2 Constraint Views

Depending on solver family, we have:

* `RowConstraintView`:

  * Global array of `ConstraintRow`:

    * Normal, friction, joint rows.
  * Used by simple scalar PGS implementations.

* `TileConstraintView`:

  * A collection of `Tile` objects:

    * Each tile has a local body table + SoA arrays for rows.
  * Used by SoA/tile-based solvers.

* `IslandView`:

  * Groups of bodies + constraints forming independent islands.
  * Tiles live inside islands.

Solvers are written against **abstract views**, so we can reuse code across layouts where reasonable (e.g. scalar row PGS over AoS or SoA).

---

## 4. Solver Families

This section describes the main solver families we plan to support.

### 4.1 Baseline AoS PGS (Legacy Reference)

**Purpose**

* Serve as a clear, minimal implementation matching common engine patterns.
* Provide a regression baseline for correctness.

**Data**

* AoS world:

  * `std::vector<RigidBody>` and `std::vector<Contact>`.
* On-the-fly computation of Jacobians, effective masses, and impulses.

**Algorithm (sketch)**

```cpp
for each step:
    // optional: apply warmstart impulses
    if (params.enable_warmstart) {
        apply_cached_impulses(bodies, contacts);
    }

    for (int iter = 0; iter < params.max_iterations; ++iter) {
        for each contact in contacts:
            solve_normal_row(bodies, contact);
            if (params.enable_friction)
                solve_friction_rows(bodies, contact);
        for each joint in joints:
            solve_joint_rows(bodies, joint);
    }

    integrate_bodies(bodies, params.dt);
```

Characteristics:

* Single-threaded.
* No SoA or tiling.
* Easy to understand; performance not the primary target.

---

### 4.2 Scalar PGS over Row Arrays

**Purpose**

* Provide a generic PGS solver operating on a **flat list of `ConstraintRow`**.
* Useful for:

  * Direct comparison with other layouts.
  * Simpler debugging than full tile-based solvers.

**Data**

* `ConstraintRow[]`:

  * Each row contains:

    * Body indices (A/B).
    * Direction (normal or tangent).
    * Effective mass, bias, and limits.
    * Cached impulse.
* World view:

  * AoS or SoA, doesn’t matter as long as we can apply impulses.

**Algorithm**

```cpp
// Warm-start once at the start
if (params.enable_warmstart) {
    for each row in rows:
        apply_impulse(world, row, row.cached_lambda);
}

// PGS iterations
for (int iter = 0; iter < params.max_iterations; ++iter) {
    float max_residual = 0.0f;

    for each row in rows:
        float v_rel = row.compute_relative_velocity(world);
        float c_dot = v_rel + row.bias; // J v + b

        float delta_lambda = -row.effective_mass * c_dot;

        float old_lambda = row.lambda;
        row.lambda = project(row.lambda + delta_lambda, row.bounds); // clamping or cone projection

        float applied = row.lambda - old_lambda;
        apply_impulse(world, row, applied);

        max_residual = max(max_residual, fabs(applied));
    }

    if (max_residual < params.convergence_threshold)
        break;
}
```

Notes:

* **Projection** handles inequalities (normal (\ge 0), friction cone, joint limits).
* This solver is still fundamentally scalar, but the row abstraction isolates it from where the rows came from.

---

### 4.3 SoA Tile-Based PGS

**Purpose**

* High-performance CPU solver for large scenes:

  * Exploit SoA and AoSoA layouts.
  * Use tiles and islands as primary units of work.
* Serve as the main “fast” backend.

**Data**

* `Island`:

  * Contains a list of tiles and local body data for that island.
* `Tile`:

  * Local body table (mapping tile indices → global bodies).
  * SoA arrays for normal and friction rows:

    * Directions, offsets, effective masses, impulses, biases.
  * Optional joint rows.

**Algorithm (per island, single-threaded version)**

```cpp
// 1. Warm-start per tile
if (params.enable_warmstart) {
    for each tile in island:
        warmstart_tile(tile, world);
}

// 2. Iterations
for (int iter = 0; iter < params.max_iterations; ++iter) {
    float max_tile_residual = 0.0f;

    for each tile in island:
        float tile_residual = solve_tile_pgs(tile, world, params);
        max_tile_residual = max(max_tile_residual, tile_residual);
    }

    if (max_tile_residual < params.convergence_threshold)
        break;
}

// 3. Integrate bodies for this island (handled by world or layout layer)
integrate_island_bodies(island, world, params.dt);
```

**Tile solver (core kernel)**

```cpp
float solve_tile_pgs(Tile& tile, WorldView& world, const SolverParams& params) {
    float max_residual = 0.0f;

    // Normals first
    for each normal row in tile.normals:
        float v_rel = normal_row.compute_relative_velocity(tile, world);
        float c_dot = v_rel + normal_row.bias;
        float delta_j = -normal_row.effective_mass * c_dot;

        float old_j = normal_row.lambda;
        normal_row.lambda = max(0.0f, old_j + delta_j); // clamp
        float applied = normal_row.lambda - old_j;

        apply_tile_impulse(tile, world, normal_row, applied);
        max_residual = max(max_residual, fabs(applied));
    }

    // Friction per contact, using updated normal
    if (params.enable_friction) {
        for each friction triplet in tile.friction:
            solve_friction_block(tile, world, friction_triplet);
            // friction_block may update max_residual internally
    }

    return max_residual;
}
```

**SIMD considerations**

* Within `solve_tile_pgs`, rows are processed in groups of width `W` (SIMD width).
* The SoA layout (arrays of scalars) allows vector loads/stores of directions, offsets, and effective masses.
* Per-lane clamping and projection are applied as mask operations.

---

### 4.4 Block PGS (Manifolds & Joints)

**Purpose**

* Improve convergence for tightly coupled local constraints:

  * Multi-point contact manifolds.
  * Joints with multiple scalar constraints.

**Data**

* `ConstraintBlock`:

  * Small set of rows belonging to a single manifold or joint.
  * Precomputed dense block matrix `A_block` and RHS `b_block` (optional).
  * Bounds and cone constraints.

**Algorithm (per block)**

1. Gather local velocities and impulses for the bodies in the block.
2. Form a small system:
   [
   A_\text{block} \Delta \boldsymbol\lambda = \mathbf b_\text{block}
   ]
   with bounds on (\boldsymbol\lambda).
3. Solve either:

   * Directly (if purely linear, no inequalities), or
   * With a small internal PGS or specialized solver that respects inequalities.
4. Apply resulting impulses to bodies.

Integration into main PGS loop:

* Blocks can replace per-row updates for those rows:

  * For each manifold block:

    * Perform block solve.
  * For remaining constraints:

    * Run scalar PGS.
* Alternatively, blocks act as preconditioners, used periodically rather than every iteration.

---

### 4.5 Advanced Families (Sketch-Level)

The architecture should allow adding new families with minimal friction:

1. **Krylov-accelerated PGS**

   * Treat PGS as a fixed-point map (F(\lambda)).
   * Use Anderson-style acceleration occasionally:

     * Maintain a short history of (\lambda) and residuals.
     * Compute an improved next iterate via a small LSQ problem.
   * Keep projection step intact.
   * Implemented as a decorator around tile-based PGS.

2. **ADMM-based solver**

   * Split constraints and velocities into local and global variables.
   * Alternate:

     * Local updates (prox operators for contact, friction, joints).
     * Global updates (velocity consistency).
   * Good candidate for GPU.

3. **XPBD / compliance solvers**

   * Work at the **position level** with compliance.
   * Reuse the constraint graph and layout, but with different update formulas.
   * Useful for soft bodies, cloth, or robust stacks at larger timesteps.

Each of these should:

* Implement `ISolver`.
* Reuse `WorldView`, `ConstraintView`, and `Tile` where possible.
* Emit `SolverStats` and metrics via the same mechanisms.

---

## 5. Iteration & Convergence Control

### 5.1 Global vs Local Convergence

We want control at both levels:

* **Global**:

  * Hard cap: `max_iterations`.
  * Optional global residual threshold.

* **Local (per tile / per island)**:

  * Per-tile residual thresholds:

    * Stop updating tiles that are already “good enough”.
  * Adaptive iteration counts:

    * Hard tiles get more attention.

### 5.2 Residual Definitions

For PGS-like solvers, residuals can be:

* Per-row impulse change:
  [
  r_j = \lambda_j^\text{new} - \lambda_j^\text{old}.
  ]
* Per-row constraint velocity:
  [
  s_j = J_j v + b_j.
  ]

We can use either or both:

* Tile residual:
  [
  r_\text{tile} = \max_j |r_j| \quad \text{or} \quad \max_j |s_j|.
  ]
* Global residual:
  [
  r_\text{global} = \max_\text{tiles} r_\text{tile}.
  ]

Thresholds are configured via `SolverParams`.

### 5.3 Early Exit

Basic pattern:

```cpp
for (int iter = 0; iter < max_iterations; ++iter) {
    float global_residual = 0.0f;

    // For single-thread: just loop tiles.
    // For multi-thread: aggregate per-task residuals.
    for each tile:
        float r = solve_tile_step(tile);
        global_residual = max(global_residual, r);

    if (global_residual < convergence_threshold)
        break;
}
```

Per-tile adaptive variant:

* Maintain per-tile residual state.
* Mark tiles “inactive” once below a stricter local threshold.
* Skip inactive tiles in subsequent iterations.

---

## 6. Integration with Execution Backends

### 6.1 Single-Thread Executor

Simplest implementation:

* `ISolver::solve` runs in the calling thread.
* Islands are processed sequentially.
* Tiles in an island are processed in a fixed order.
* Deterministic by design if the world and constraint view are deterministic.

This backend is ideal for:

* Debugging and regression testing.
* Baseline comparisons.

### 6.2 Multi-Thread Executor

Responsibilities of `multi_thread_executor`:

* Build a job list:

  * Typically one job per island, or per tile for large islands.
* Use a work-stealing or thread-pool model:

  * Threads pull jobs and call solver kernels for those jobs.

Constraints for tile solvers:

* **No shared bodies across jobs**:

  * Jobs must be defined so that tiles in the same job never act on the same body simultaneously.
* Or, if shared bodies are unavoidable:

  * Use coloring to ensure conflicting tiles are processed in different sub-passes.
  * Or adopt Jacobi-like schemes that allow parallel updates but require extra reads/writes.

Determinism:

* Multi-thread runs are generally **not deterministic** by default.
* If required:

  * Use a fixed scheduling strategy (e.g., deterministic per-color passes).
  * At the cost of some performance.

### 6.3 GPU Backend (Future)

The solver design should anticipate a GPU backend:

* Tiles map naturally to GPU workgroups:

  * Each workgroup handles one tile or group of tiles.
* Kernels:

  * Per-tile PGS iterations.
  * Optional block solvers for manifolds.
* Data:

  * SoA arrays already match GPU-friendly layouts.

The key is to structure solver kernels around **tile-local operations** that can be executed with minimal synchronization and without relying on host-side ordering.

---

## 7. Warm-Starting Strategy

Warm-start impulses are usually stored:

* In constraint manifolds and joints:

  * Per-contact normal and friction impulses.
  * Per-joint scalar impulses.

At solve time:

1. Constraint builder copies these cached impulses into rows/tiles.
2. Solver applies warm-start once before iterations:

   * Ensures velocities reflect last frame’s solution.
3. Updates to impulses during the solve are written back to caches.

Reset and scaling:

* New contacts: start with zero impulses.
* Contacts that disappear: drop their caches.
* Large changes in orientation or large dt jumps:

  * Optionally scale down warm-start or reset to zero.

Warm-start must be **consistent** across solver families:

* When swapping solvers, caches should reuse existing impulses where possible.
* Strict equivalence is not required, but we should avoid pathological behavior due to incompatible caches.

---

## 8. Determinism & Reproducibility

Determinism is important for:

* Regression tests.
* Debugging.
* Some game workflows.

Guidelines:

* **Single-thread mode** is deterministic as long as:

  * The world/constraints are deterministic.
  * The iteration and tile order are fixed.
* **Multi-thread mode** is typically not deterministic:

  * Floating point summation order changes.
  * Task scheduling is order-dependent.

Options for deterministic multi-thread mode (optional):

* Use a fixed two-phase scheme:

  * Phase 1: compute impulses for tiles in parallel in a Jacobi-like way.
  * Phase 2: apply impulses in a fixed order.
* Or run multi-thread only for certain phases (e.g., row build), and keep solver itself single-threaded.

We can expose this via `SolverParams::prefer_determinism`.

---

## 9. Debugging & Diagnostics Hooks

The solver layer should make it easy to turn on diagnostics:

* **Assertions**:

  * Valid denominators (no zero effective mass).
  * Impulses within expected bounds.
  * Proper cone projection.

* **Logging**:

  * Per-iteration residuals.
  * Number of clamped rows and cone projections.
  * Statistics on iterations and convergence.

* **Metrics integration**:

  * Tie `SolverStats` to higher-level `metrics` module:

    * ADMC directional drift.
    * Penetration and joint error.
    * Cone consistency.

Debug modes can be enabled via:

* Compile-time flags (e.g. `ADMC_DEBUG_SOLVER`).
* Runtime flags in `SolverParams`.

---

## 10. Extensibility Guidelines

When adding a new solver family:

1. **Implement `ISolver`**

   * Provide a name.
   * Implement `solve()` using the existing `WorldView` and `ConstraintView`.
   * Emit `SolverStats` and optionally `SolverDebugInfo`.

2. **Reuse shared infrastructure**

   * Use existing:

     * Row and tile builders.
     * Island detection and coloring.
     * Metrics and timing hooks.
   * Do not duplicate constraint math.

3. **Add tests and bench entries**

   * Add unit tests for basic stability and correctness.
   * Add entries to the bench CLI so the solver can be compared in standard scenes.

4. **Document solver-specific behavior**

   * Describe:

     * Any special parameters.
     * Convergence properties.
     * Limitations or known trade-offs.
   * Place this in `docs/solver_design.md` (this file) and/or a dedicated `docs/solver_*.md` if needed.

---

## 11. Summary

* The solver layer provides a **common interface** (`ISolver`) and shared parameter/stats structures.
* It operates over abstract **world and constraint views**, allowing multiple data layouts and backends.
* Core families include:

  * AoS baseline PGS (legacy reference).
  * Scalar PGS over row arrays.
  * SoA tile-based PGS (high-performance CPU).
  * Block, Krylov, ADMM, and XPBD variants (future/experimental).
* Iteration, convergence, warm-starting, and parallelism are handled in a way that keeps:

  * The **math** clear and close to the theory docs.
  * The **implementation** optimized for real-world workloads.
  * The **metrics** first-class and easy to analyze.

This design aims to give us a solid, flexible foundation for exploring solver architectures, guided by ADMC-based metrics and modern data-oriented practices.
