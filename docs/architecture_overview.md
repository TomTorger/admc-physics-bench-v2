# ADMC Solver Architecture Overview

> Sparse words. High signal. This is the “map” of the system.

---

## 1. Purpose

- Define a **clear mental model** for the solver stack.
- Keep **ADMC** at the core as *the* differentiator.
- Guide **implementation, optimization, extension**, including **ADMC-informed convergence**.

---

## 2. Design Goals

- **ADMC-first**  
  Directional invariants visible, measurable, and able to **influence solver behaviour**.

- **Data-oriented**  
  SoA / AoSoA, cache-friendly, minimal pointer chasing.

- **Single-core strong**  
  SoA-native PGS as the **golden reference path**.

- **Parallel-friendly**  
  Clean scheduling, no hidden shared state, safe fallbacks.

- **Modular & composable**  
  Clear boundaries, small interfaces, explicit dependencies.

---

## 3. Core Concepts

- **World (AoS)**  
  Engine-facing: bodies, colliders, joints, forces.

- **World (SoA / AoSoA)**  
  Solver-facing: contiguous arrays of velocities, masses, indices.

- **Constraint**  
  Contact, joint, limit, motor; defines relationships between bodies.

- **Row**  
  Scalar constraint: Jacobian row + limits + effective mass + bias.

- **Tile**  
  Small SoA block of rows + local body table (hot unit of work).

- **Island**  
  Connected component of bodies/constraints.

- **ADMC State**  
  Directional momentum invariants (p\_{x±}, p\_{y±}, p\_{z±}) per world (and optionally per island/tile).

---

## 4. System Layers (Modularization Overview)

From bottom to top:

1. **core/**  
   Math + basic rigid body types.

2. **admc/**  
   Invariants, mapping, tracking, **feedback signals**.

3. **constraints/**  
   Manifolds, joints, row descriptors.

4. **backend/**  
   SoA / AoSoA buffers, SIMD abstraction, allocators.

5. **assembly/**  
   AoS world + manifolds → `ConstraintBatch` (tiles + local body tables).

6. **solver_core/**  
   Single-thread SoA-native PGS on `ConstraintBatch`.

7. **parallel/**  
   Islands, coloring (optional), schedulers using `solver_core`.

8. **diagnostics/**  
   ADMC trackers, residual metrics, logging.

9. **api/**  
   Engine-facing `PhysicsWorld`, `step(dt)` orchestration.

**Dependency direction:**

- `core` ← `admc`, `constraints`, `backend`
- `constraints` ← `assembly`
- `backend` ← `assembly`, `solver_core`, `parallel`
- `admc` ← `solver_core`, `diagnostics`
- `assembly` ← `solver_core`, `parallel` (interfaces only)
- `solver_core` ← `parallel`, `api`
- `parallel` ← `api`
- `diagnostics` ← `api`

No module may depend “upwards”.

---

## 5. High-Level Pipeline (Per Step)

1. **Engine update (AoS)**  
   External forces, input, animation.

2. **Broad phase**  
   Overlapping pairs of bodies.

3. **Narrow phase & manifolds**  
   Build/maintain contact manifolds and joints.

4. **Row assembly (AoS → Tiles)**  
   Caches, archetypes, tiling → `ConstraintBatch`.

5. **Solve (SoA-native)**  
   - Single-thread PGS (`solver_core`), or  
   - Parallel scheduling (`parallel` + `solver_core`), with **ADMC-informed iteration control**.

6. **Integrate & scatter**  
   Update positions/orientations in AoS world.

7. **Diagnostics & feedback**  
   ADMC drift, residuals, timing → **feeds back into next step’s policies**.

---

## 6. Modules & Responsibilities

### 6.1 `core/`

- Math primitives: `Vec3`, `Mat3`, `Quat`.
- `Transform`, `RigidBody` (AoS).
- Numerics: clamping, interpolation, small linear algebra.

Depends on: nothing.  
Used by: all other modules.

---

### 6.2 `admc/`

**What:**

- `ADMCState` for body / island / world.
- Mapping between velocities/impulses and directional channels p\_{x±}, p\_{y±}, p\_{z±}.
- `ADMCTracker`:
  - Pre-/post-step invariants.
  - Drift per world/island/tile.
- **Feedback hooks**:
  - **Iteration policy** per island (adaptive iterations).
  - **Warm-start scaling** per island/frame.
  - Optional global correction hints.

**Interface sketch:**

```cpp
struct IterationPolicy {
    bool shouldContinue;     // per-iteration decision
    bool focusThisIsland;    // may get extra iterations vs others
};

class IConservationTracker {
public:
    virtual void on_pre_step(const WorldSoA&) = 0;
    virtual void on_post_step(const WorldSoA&) = 0;

    virtual IterationPolicy on_iteration_end(
        IslandId island,
        float residualEstimate) = 0;   // ADMC + residual

    virtual float warmstart_scale(IslandId island) const = 0;

    virtual ~IConservationTracker() = default;
};
````

Depends on: `core`.
Used by: `solver_core`, `parallel`, `diagnostics`, `api`.

---

### 6.3 `constraints/`

* Types:

  * `Contact`, `Manifold`, `Joint`, `ConstraintRowDesc`.
* Functions:

  * Build Jacobian rows from manifolds/joints.
  * Evaluate constraint error/residual.
* Archetypes:

  * Sphere–sphere, box–box, generic poly.
  * Distance joints, hinges, etc.
* ADMC metadata as **data only**:

  * Row direction vector for mapping impulses → ADMC channels.

Depends on: `core`, optional `admc` types for labels.
Used by: `assembly`, `diagnostics`.

---

### 6.4 `backend/`

* SoA / AoSoA buffer types:

  * `WorldSoA`, `Tile`, `ConstraintBatch`.
* SIMD abstraction layer.
* Memory pools / allocators.

Depends on: `core`.
Used by: `assembly`, `solver_core`, `parallel`.

---

### 6.5 `assembly/`

* AoS world + manifolds/joints → `ConstraintBatch`:

  * Row construction via `constraints/`.
  * Tiling (rows + local body tables).
* Caching:

  * Geometry, denominators (per manifold/contact).
* Archetypes:

  * Sphere clouds fast path.
  * Generic contact path.

Design: no solving, no threads.

Depends on: `core`, `constraints`, `backend`.
Used by: `solver_core`, `parallel`, `api`.

---

### 6.6 `solver_core/`

* Single-thread SoA-native PGS:

  * Operates on `WorldSoA` + `ConstraintBatch`.
  * Tight tile kernels (normals, friction, joints).
* Uses `IConservationTracker` to:

  * Announce pre/post step.
  * Obtain **iteration policy per island**.
  * Obtain **warm-start scale per island**.
* Operator surface:

  * `admc/solver/operators.hpp` exposes `ConstraintOperator`, so higher-order solvers (block/ADMM/Krylov) can reuse the same Jacobian + mass-operator hooks without touching AoS data.
* Prototype:

  * `admc/solver/simple_pgs.hpp` wires the composite iteration gate + manifold warm-start scaler into a header-only solver, used by tests and the tiny bench app.

---

### 6.6a `baseline/`

* AoS sequential-impulse solver ported from the original repo.
* `admc/baseline/contact.hpp` + `admc/baseline/solver.hpp` keep the scalar reference path alive:

  * `ContactConstraint` stores AoS contact state and cached denominators.
  * `solve_baseline(...)` runs Baumgarte-stabilized sequential impulses directly on `world::RigidBody`.
* Used for regression parity (`baseline_tests`) and bench comparisons (`simple_bench --solver=baseline`).


**Entry:**

```cpp
void solve_pgs_single_thread(
    WorldSoA& world,
    ConstraintBatch& batch,
    const SolverSettings& settings,
    IConservationTracker* tracker);
```

Inside:

* Per island:

  * Apply `warmstart_scale(island)` when seeding cached impulses.
  * Loop iterations until:

    * iteration count limit, or
    * `tracker->on_iteration_end(island, residual)` says stop.

Design: no graph coloring, no threads.

Depends on: `core`, `backend`, `admc` (via interface).
Used by: `parallel`, `api`, tests, benchmarks.

---

### 6.7 `parallel/`

* Island builder:

  * Connected components over bodies/constraints.
* Optional graph & coloring:

  * Build constraint/tile graph per island.
  * Assign colors → conflict-free sets.
* Schedulers:

  * `SerialScheduler` (debug / reference).
  * `ThreadPoolScheduler` (production).

Schedulers:

* Drive `solver_core` on:

  * Individual islands (serial mode), or
  * Color groups / tiles (parallel mode).
* Honor per-island `IterationPolicy` from `IConservationTracker`.

Depends on: `core`, `backend`, `solver_core`.
Used by: `api`.

---

### 6.8 `diagnostics/`

* Implementations of `IConservationTracker`:

  * `ADMCTracker`: full invariants + iteration policy + warm-start scale.
  * `NoopTracker`: minimal, constant policies, zero cost.
* Constraint residual metrics:

  * Max penetration, joint error, friction cone violation.
* Time breakdowns:

  * Assembly vs solve vs integration vs ADMC.

Observation-only.

Depends on: `core`, `admc`, `constraints`.
Used by: `api`, tests, benchmarks.

---

### 6.9 `api/`

* Engine-facing interface:

  * `PhysicsWorld`, handles, configuration.
* Orchestrates:

  * Broad-phase → manifolds → `assembly` → `parallel`/`solver_core` → integrate → `diagnostics`.
* Exposes knobs for:

  * Solver variant, scheduler, ADMC tracker mode.

Depends on: all other modules.
Used by: users, examples, engines.

---

## 7. Data Layout Strategy

* **AoS (World side)**

  * Bodies, colliders, joints as simple structs.
  * Friendly to tools and serialization.

* **SoA / AoSoA (Solver side)**

  * `WorldSoA` for velocities, inverse masses, etc.
  * `Tile` for constraint rows (normals, tangents, effMass, impulses).
  * `ConstraintBatch` for the full set of tiles per frame.

* **Tiling**

  * Fixed-size tiles (e.g. 32–128 rows).
  * Each tile has a **local body table** → small working set.
  * Row creation is **fused** with tiling during `assembly`.

---

## 8. Solver Variants

All variants consume `WorldSoA` + `ConstraintBatch` and may use `IConservationTracker`.

* **PGS SoA-native (default)**

  * Single-thread, deterministic.
  * Iteration loop driven by **ADMC-aware `IterationPolicy`**.

* **PGS SoA + Parallel Scheduler**

  * Same kernels, scheduled per island / color group.
  * Parallel loop still uses `IterationPolicy` for:

    * per-island iteration counts,
    * possible focus on “difficult” islands.

* **Block Solver (optional)**

  * Per-manifold / patch blocks on top of `ConstraintBatch`.
  * Can share ADMC iteration policy & warm-start scaling.

* **Experimental (optional)**

  * Krylov-accelerated, hierarchical, GPU backends.
  * Must preserve:

    * `ConstraintBatch`, `WorldSoA`, `IConservationTracker`.

---

## 9. Parallelism Model

* **Unit of work**

  * Island → Color group → Tile.

* **Within a color group:**

  * Tiles are body-disjoint → no shared writes.
  * Tiles processed in parallel.

* **Across colors:**

  * Colors processed in sequence → GS-like update ordering.

* **Schedulers:**

  * `SerialScheduler`:

    * Single-thread, no colors required.
  * `ThreadPoolScheduler`:

    * Distributes color groups/tiles across workers.
    * Can prioritize islands flagged by `IterationPolicy.focusThisIsland`.

* **Fallbacks:**

  * Parallel scheduling can be disabled (pure `solver_core`).
  * Coloring optional for small / trivial islands.

---

## 10. Row Building Strategy

**Goal:** minimize per-frame work for **unchanged** contacts, remain SoA-friendly.

**Key tactics:**

* **Split geometry vs state:**

  * Geometry (frames, rA/rB, denominators) cached per manifold/contact.
  * State (penetration, relative velocity, bias) recomputed each step.

* **Archetypes:**

  * Sphere–sphere fast path (cloud scenes).
  * General path for boxes / polyhedra / joints.

* **Fused tiling:**

  * Allocate rows directly into tiles as they’re built.
  * No global “intermediate RowSoA” pass.

* **Incremental rebuild:**

  * Rebuild geometry only if:

    * Orientation/position changed beyond thresholds,
    * Or contact topology changed.

* **Tile metrics:**

  * Track per-tile residuals and inexpensive ADMC contrib stats:

    * Used by `IterationPolicy` to deactivate converged tiles in future iterations (optional extension).

---

## 11. ADMC Integration & Feedback

**Representation**

* `ADMCState` per world (and optionally per island).
* Channels p_{x±}, p_{y±}, p_{z±}.

**Measurement**

* `IConservationTracker` is notified:

  * `on_pre_step(world)`
  * `on_post_step(world)`

**Feedback (most promising hooks)**

1. **Adaptive iteration counts (per island)**

   * After each iteration:

     * Tracker sees:

       * Constraint residual estimate (from solver),
       * Current ADMC drift per island.
     * Returns `IterationPolicy`:

       * `shouldContinue` = false when residual **and** ADMC drift are below thresholds.
       * `focusThisIsland` = true for “difficult” islands → scheduler can spend more iterations there.
   * `CompositeIterationGate` (`admc/solver/iteration_control.hpp`) is the canonical reducer:

     * Bundles residual, penetration, joint, and ADMC drift signals.
     * Sets `focusThisIsland` when ADMC drift dominates the residual budget.

2. **Warm-start scaling (per island, per frame)**

   * Before solving:

     * Tracker returns `warmstart_scale(island) ∈ (0,1]` based on previous-frame ADMC drift.
   * Solver seeds cached impulses as:

     * `j_warm = warmstart_scale * j_cached`.
   * High drift last frame → smaller scale, avoiding overshoot and improving stability.
   * Tile/manifold override: `ManifoldWarmstartScaler` (`admc/solver/warmstart.hpp`) provides per-manifold scalars driven by residual + ADMC drift + cached impulse magnitude.

3. **Tile-level focus (optional)**

   * Tiles can report light-weight residual + ADMC-contribution metrics.
   * Tracker (or a simple heuristic) can:

     * Mark tiles as “converged” → skip in later iterations.
   * Reduces work on “cold” tiles in large clouds.

4. **Global ADMC correction (optional, small pass)**

   * After PGS:

     * Tracker may propose a small, bounded correction step:

       * Adjust velocities to reduce ADMC drift further,
       * Without re-introducing large constraint errors.
   * Implemented as an optional `post_solve_global_correction(...)` pass.

**Usage modes**

* **Debug / research mode**:

  * Full ADMC tracking + iteration feedback + logs.

* **Production mode**:

  * Lightweight tracker:

    * Still provides conservative iteration policy & warm-start scale,
    * Minimal overhead, no heavy logging.

---

## 12. Extensibility & Safety Rails

* **Always keep:**

  * Baseline AoS solver in `examples/legacy_baseline` for comparison.
  * Single-thread SoA-native PGS as reference.

* **Rules for new features:**

  * Must plug in via existing module boundaries.
  * Must not break:

    * `ConstraintBatch`, `WorldSoA`, `IConservationTracker`.

* **Config knobs:**

  * Select solver variant, scheduler, ADMC tracker implementation.
  * Enable/disable:

    * Parallel scheduling,
    * Graph coloring,
    * ADMC feedback (adaptive iterations / warm-start scale),
    * Archetype optimizations.

* **Testing:**

  * All solver variants:

    * Same scenes,
    * Same ADMC thresholds,
    * Same residual targets.
  * Verify ADMC-aware policies **improve convergence or cost** vs fixed-iteration baselines.
  * `apps/simple_bench` provides a smoke-test harness for `simple::pgs` (and the AoS baseline via `--solver=baseline`) so solver hooks can be exercised without the full assembly stack.

* **Debug paths:**

  * “Safe mode” build:

    * Serial scheduler, no colors,
    * Full diagnostics and ADMC tracking,
    * Deterministic step order.

---
