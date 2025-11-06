````markdown
# ADMC Next-Gen Solver Architecture

## 1. Introduction

This document describes a new, clean architecture for an ADMC-based rigid body solver library.

We want to:

- Preserve the **existing AoS baseline** as a canonical reference.
- Use what we learned from `admc-physics-bench` to design a **data-oriented, parallel-first** architecture.
- Support a **family of solvers** (PGS variants, block solvers, ADMM, XPBD/compliance) on top of shared world, constraint, and layout layers.
- Make it straightforward to **benchmark**, **inspect**, and **evolve** solver implementations over time.

### 1.1 Scope

- Rigid bodies with contact and joints.
- Velocity-/impulse-based solvers with ADMC invariants as diagnostics.
- Position-based/compliant methods as extensions.
- Desktop CPU as primary target, with GPU-friendly layout from day one.

### 1.2 Audience

- Engine developers using or integrating the solver.
- Researchers experimenting with new algorithms and layouts.
- Contributors extending or maintaining the codebase.

### 1.3 Relationship to `admc-physics-bench`

The new repository:

- Reuses the **baseline AoS solver** from `admc-physics-bench` (in a `legacy/` module).
- Encodes lessons learned from SoA, tiling, and benchmarks into a cleaner architecture.
- Adds first-class **parallel execution** and more room for experimental solvers.

---

## 2. Motivation & Goals

### 2.1 Why a Fresh Repository

The current bench repo grew organically around experimentation. For long-term evolution we want:

- Clear layering and ownership of responsibilities.
- Strong separation between **API**, **data layout**, **solver algorithms**, and **execution backends**.
- A modular design that doesn’t force every feature into the same code paths.

### 2.2 Design Goals

- **Keep AoS baseline as a canonical reference**
  - Simple, didactic, and testable.
- **Parallel-first**
  - Islands and tiles as **parallel units** from day one.
  - A clean backend abstraction for single-thread, multi-thread, and GPU execution.
- **Performance-oriented**
  - SoA/AoSoA layouts, cache-friendly tiles, SIMD-friendly kernels.
  - Focus on row/constraint assembly cost, not just solver math.
- **Experimentation-friendly**
  - Add new solver families (PGS variants, block, Krylov, ADMM, XPBD) without reworking the whole codebase.
- **Diagnostics-first**
  - Built-in timing breakdowns, ADMC invariants, and physics metrics.

### 2.3 Non-Goals (Initial)

- Soft bodies, fluids, or highly deformable systems.
- Networking, persistence, or engine integration beyond basic demos.
- Highly specialized platforms (mobile, consoles) in the first iteration.

---

## 3. Current Learnings (from `admc-physics-bench`)

### 3.1 Baseline AoS Solver

**Characteristics**

- AoS storage (`std::vector<RigidBody>`, `std::vector<Contact>`).
- Sequential impulses / PGS:
  - Normal rows, ERP/Baumgarte, restitution, friction.
- Minimal caching.

**Learnings**

- Extremely **clear and easy to debug**.
- Very low overhead for tiny scenes.
- **Does not scale** to many contacts due to poor cache behavior and lack of vectorization.

---

### 3.2 Scalar Cached Solver (AoS + Cached Rows)

**Characteristics**

- Still AoS bodies/contacts.
- Per-contact scalar rows (normal + tangents).
- Caches effective mass and impulses for warm-start.

**Learnings**

- Warm-starting and cached denominators help convergence and reduce per-iteration cost.
- Overhead can offset gains in highly dynamic situations.
- It’s a good “classic engine” optimization, but not a structural fix.

---

### 3.3 Scalar SoA Solver

**Characteristics**

- AoS → SoA row build for constraints.
- Scalar PGS over SoA arrays.
- Timing breakdown: contact prep, row build, solver, integration, scatter.

**Learnings**

- SoA improves locality and solver throughput.
- **Row build and contact prep often dominate** the frame at scale.
- Shows that the main performance target is **constraint assembly and data movement**, not just the solver loop.

---

### 3.4 SoA-Native SIMD Solver

**Characteristics**

- SoA world representation, contact tiles with local body tables.
- SIMD-aware kernels over tiles, especially for simple shapes.

**Learnings**

- For 10k–50k contacts, SoA-native is the **fastest CPU solver**.
- After SoA-native, solver time becomes a relatively small fraction of the frame; assembly dominates.
- Confirms the importance of **tiling** and **SIMD-ready SoA**.

---

### 3.5 SoA-Parallel Solver

**Characteristics**

- Island-based and tile-based parallel wrapper over SoA-native.
- Multi-threading via islands and tiles.

**Learnings**

- Island/tile parallelism is the right conceptual model.
- Actual speedups depend heavily on:
  - Task granularity.
  - Synchronization strategy.
  - How conflicts between constraints are avoided.

---

### 3.6 Vec-SoA / Experimental SIMD Paths

**Characteristics**

- SoA layout intended for SIMD, currently forwarding to mostly scalar kernels.

**Learnings**

- Data layout alone isn’t enough; kernels must be **explicitly designed** for SIMD.
- Useful as an experimental path, but not yet a production backend.

---

### 3.7 Summary of Learnings

- **Constraint assembly dominates** large scenes.
- **SoA-native + tiling** is the best performance path.
- **AoS baseline** is still ideal for tiny scenes and correctness.
- **Parallelism needs its own abstraction** (backends), rather than ad-hoc threading.
- Detailed **instrumentation** (timings, invariants) is essential.

---

## 4. Physics & Constraint Model

### 4.1 ADMC & NR Bridging

- ADMC invariants use scalar quantities per direction (`p_{k±}`) that are equivalent to conserving energy + momentum.
- In the non-relativistic rigid-body context, they reduce to:
  - Directional momentum conservation.
  - Kinetic (and potential) energy behavior.
- In this repo, ADMC invariants will be used primarily as **diagnostic metrics**:
  - Check directional momentum drift along multiple directions.
  - Compare solvers’ conservation behavior.

### 4.2 Constraint Types

- **Contacts**
  - Normal constraints with penetration correction and restitution.
  - Friction constraints in tangential directions.
- **Joints**
  - Distance, hinge, ball-and-socket, etc.
  - Optional motors and limits.
- **Optional extras**
  - Rolling friction.
  - Custom “scripted” constraints.

### 4.3 Friction Models

- Coulomb **friction cone** vs **pyramid approximation**:
  - Cone gives better modeling; pyramid is solver-friendly.
- Regularization/compliance for friction:
  - Avoids hard non-differentiability.
- Rolling resistance / rolling friction:
  - Optional constraints or terms for more realistic rolling behavior.

### 4.4 Compliance & XPBD-style Aspects

- Some constraints benefit from **softness** (compliance) rather than pure rigidity.
- We may combine:
  - Velocity-based solvers for contact and friction.
  - Position-based / XPBD-style solvers for some joints or “soft” constraints.
- The architecture should allow for mixed methods without changing the world representation.

### 4.5 Metrics & Correctness Criteria

- ADMC directional drift per frame or per N frames.
- Penetration error (L2 / L∞ across contacts).
- Joint errors (distance/angle).
- Energy and momentum drift.
- Constraint drift for position-based/XPBD parts.

---

## 5. Design Principles & Requirements

### 5.1 Layered Architecture

- **Physics & constraints**: bodies, materials, contacts, joints.
- **Layout**: AoS vs SoA/AoSoA, tiles, islands.
- **Solvers**: algorithms that operate on constraints and velocities.
- **Execution backends**: single-thread, multi-thread, GPU.
- **Metrics & tooling**: integrated diagnostics.

### 5.2 API vs Implementation Separation

- External / public API:
  - Row-centric: worlds, contacts/joints, constraint rows, solver parameters.
- Internal implementation:
  - Tile- and island-centric: small SoA blocks as the unit of work.

### 5.3 Data-Oriented Design

- Use **SoA/AoSoA** for hot loops.
- Tiles sized to cache lines and SIMD width.
- Local body tables in tiles to minimize pointer chasing.

### 5.4 Extensibility

- Solvers organized in modules (`pgs_scalar`, `pgs_soa`, `block`, `krylov`, `admm`, `xpbd`).
- Easy to add a new solver by:
  - Implementing the common solver interface.
  - Optionally adding new kernels or layouts.

### 5.5 Parallelism from the Start

- Islands and tiles as **fundamental design elements**.
- A dedicated `backend` module for:
  - Single-thread execution (for debugging).
  - Multi-thread execution using task-based scheduling.
  - GPU execution as a future extension.
- Solvers remain mostly **back-end agnostic**:
  - They know how to process a tile or island.
  - They do not hard-code threading or GPU details.

### 5.6 Testability

- Unit tests for core math and layout.
- Regression tests for solvers vs baseline AoS.
- Benchmarks as repeatable tests (with tolerances).

---

## 6. Architectural Views

### 6.1 External API View

Users interact with:

- `World` and `Body`:
  - Add bodies, set materials, apply forces.
- `Contact` and `Joint` descriptions:
  - Provided by collision detection or scene setup.
- `SolverParams` and solver selection:
  - Choose solver family and backend (e.g., PGS-SoA + multi-thread).

API examples (conceptual):

- `World world;`
- `SolverParams params = SolverPresets::FastPGS();`
- `auto solver = SolverFactory::Create("pgs_soa", params);`
- `solver->step(world, dt);`

### 6.2 Internal Data Layout View

Internally:

- `World` → decomposed into **islands** (connected components).
- Each island is decomposed into **tiles**:
  - Tiles hold:
    - Local body tables.
    - SoA arrays for constraint rows.
    - Cached geometric and effective-mass data.

- AoS views:
  - Used for user-level scenes and the legacy baseline.
- SoA/AoSoA views:
  - Used by high-performance solvers and GPU backends.

### 6.3 Execution Backend View

Backends handle **how** tiles and islands are processed:

- Single-thread:
  - Iterate islands and tiles sequentially.
- Multi-thread:
  - Assign islands/tiles as tasks to a thread pool.
  - Use work-stealing for load balancing.
- GPU:
  - Map tiles to thread blocks or workgroups.
  - Use shared memory for tile-local data.

Solvers expose tile/island operations; backends orchestrate them.

### 6.4 Metrics & Logging View

- Central metrics module:
  - Scoped timers for each phase (prep, build, solve, integrate, scatter).
  - Physics metrics (penetration, drift).
  - ADMC invariants.
- Backends and solvers report into this module.
- Bench tools export metrics as text and CSV/JSON.

---

## 7. Solver Families

### 7.1 Common Solver Interface

All solvers implement a shared interface:

- `SolverParams`:
  - Iteration count, tolerances, warm-start, friction options, etc.
- `SolverStats`:
  - Timing breakdown, iterations used, convergence flags.
- API:
  - `prepare(World&)`
  - `solve(World&, dt)`
  - `get_stats()`

### 7.2 PGS / Sequential Impulse Family

- **AoS baseline PGS**:
  - Direct AoS loops; used in `legacy/`.
- **Row-centric scalar PGS**:
  - Works over global row arrays.
- **Tile-based PGS (SoA)**:
  - Operates on tiles, using SoA arrays and SIMD-friendly kernels.
  - Designed to work naturally with multi-thread and GPU backends.

### 7.3 Per-Contact & Robust GS Variants

- **Per-contact iteration**:
  - Solve each contact more accurately (e.g., bisection) with SOR coupling.
- **Robust GS**:
  - State-aware variants to better handle simultaneous collisions and shock propagation.
- **Modulus-based / accelerated GS**:
  - Advanced splitting schemes for hard contact problems.

These map cleanly onto tiles, with each tile running its local variant.

### 7.4 Block Solvers

- Solve small blocks corresponding to:
  - A contact manifold (one pair with multiple contact points).
  - Multi-DOF joints.
- Acts as:
  - Preconditioners for global PGS/ADMM.
  - Standalone “block PGS” for stacks and stiff structures.

### 7.5 Krylov-Accelerated Solvers

- Use residuals from PGS to apply:
  - Anderson acceleration.
  - Other lightweight Krylov-like steps.
- Maintain projections to inequality constraints (e.g., impulses ≥ 0, friction cones).

### 7.6 Global ADMM-Based Solvers

- Global view of contact + friction:
  - Split into local subproblems and global updates.
- Can handle:
  - Exact friction cones.
  - Compliance between rigid/compliant regimes.
- Targets:
  - High-quality real-time modes.
  - Offline or “simulation-as-ground-truth” use.

### 7.7 XPBD / Compliant Solvers

- Position-level constraint solvers with compliance.
- Can be used for:
  - Joints.
  - Soft/elastic structures.
- Coexist with velocity-based contact solvers.

### 7.8 Comparison & Trade-Offs

Each solver family has documented trade-offs:

- Accuracy vs performance.
- Robustness for stacking/contact-rich scenes.
- Parallelization friendliness (CPU/GPU).
- Complexity and code maintenance cost.

---

## 8. Parallelism & Scheduling

Parallelism is treated as a **core concern**, not an afterthought.

### 8.1 Island Decomposition

- Build a constraint graph (bodies as nodes, constraints as edges).
- Connected components become **islands**.
- Each island can be processed in parallel with others (subject to backend).

### 8.2 Graph Coloring & Conflict-Free Sets

- Within an island, constraints may conflict (share bodies).
- Two approaches:
  - **Graph coloring**:
    - Colors represent sets of constraints that can be solved in parallel.
  - **Mutual exclusion blocks**:
    - Partition constraints into conflict-free batches.

Solvers can choose between:

- Fully GS-style (serial per island).
- Colored / partially parallel PGS.

### 8.3 Tile-Based Task System

- Tiles are the main **unit of work**.
- Each tile contains:
  - A small set of bodies.
  - A local set of rows.
- Tasks:
  - `solve_tile_normals(tile)`
  - `solve_tile_friction(tile)`
  - `integrate_tile_bodies(tile)`

Backends schedule these tasks across threads or GPU kernels.

### 8.4 Hierarchical Parallelism for GPU

We design tiles with GPU mapping in mind:

- Scene → islands → tiles → thread blocks / workgroups.
- Each tile fits in shared memory; local body table ensures compact indexing.
- Same data layout used on CPU and GPU, avoiding separate pipelines.

### 8.5 Determinism vs Throughput

- Single-thread backend:
  - Maximum determinism, used for tests.
- Multi-thread / GPU backends:
  - May sacrifice strict determinism for throughput.
  - We document and parameterize this trade-off.

---

## 9. Data Layout & Row/Tile Build

### 9.1 AoS vs SoA vs AoSoA

- AoS:
  - User-facing API and legacy baseline.
- SoA:
  - High-performance solvers and GPU.
- AoSoA:
  - Tiles as “structures of arrays” blocks with fixed or bounded size.

### 9.2 Tile & Island Structures

- `Island`:
  - Set of bodies and constraints forming a connected graph.
- `Tile`:
  - Local body table (mapping to global IDs).
  - SoA arrays for:
    - Normals, tangents.
    - Effective masses.
    - Impulses.
    - Flags and meta-data.

### 9.3 Constraint/Row Build Pipeline

- Input:
  - World bodies (AoS).
  - Contact/joint descriptions.
- Pipeline:
  1. Build islands from constraint graph.
  2. For each island, build tiles:
     - Assign constraints to tiles.
     - Directly build rows into tile SoA arrays.
  3. Cache data cross-frame when possible.

Row build and tiling are **fused**: we do not build a huge global RowSOA and then retile.

### 9.4 Cross-Frame Caching

- Cache:
  - Contact frames (normals, tangents).
  - Relative offsets.
  - Effective masses.
- Invalidate when:
  - Body orientations change beyond a threshold.
  - Contact topology changes.

### 9.5 Contact Archetypes

- Special paths for:
  - Sphere–sphere.
  - Sphere–plane.
- Use analytic formulas and avoid general inertia transforms where possible.

---

## 10. Diagnostics, Benchmarks & Tooling

### 10.1 Metrics Module

- Timing:
  - Contact prep.
  - Island/tiling.
  - Row build.
  - Solver phases (warm, iterations, friction, joints).
  - Integration and scatter.
- Physics:
  - Penetration and joint errors.
  - Energy and momentum drift.
  - ADMC directional drift.

### 10.2 Benchmark Harness

- Configurable via CLI:
  - Scene selection.
  - Solver family and backend.
  - Iteration counts, tolerances.
- Output:
  - Human-readable tables.
  - CSV/JSON dumps for analysis.

### 10.3 Regression Testing

- Compare new solvers vs AoS baseline:
  - Kinematics within tolerances.
  - Metrics within tolerances.
- Include:
  - Small, medium, and large scenes.
  - “Nasty” scenarios (stacks, dense clouds, joints).

### 10.4 Visualization & Debugging

- Optional interactive viewer:
  - Render scenes and constraints.
  - Visualize contact normals, impulse magnitudes.
  - Inspect tiles and islands.

---

## 11. Modularization & Directory Structure

### 11.1 Logical Modules

- `core` – math utilities and low-level types.
- `admc` – invariants and ADMC-specific metrics.
- `world` – bodies, materials, world state.
- `constraints` – contacts, joints, constraint rows, friction models.
- `layout` – AoS/SoA views, tiles, islands.
- `solver` – solver families (PGS, block, Krylov, ADMM, XPBD).
- `backend` – execution backends (single, multi-thread, GPU).
- `metrics` – timing and physics metrics.
- `scenes` – benchmark and test scenes.
- `apps` – bench, viewer, tools.
- `tests` – unit and regression tests.
- `legacy` – original AoS baseline solver.

### 11.2 Proposed Directory Layout

```text
admc-next/
├─ CMakeLists.txt
├─ README.md
├─ docs/
│  ├─ architecture_overview.md
│  ├─ solver_design.md
│  ├─ layout_and_tiling.md
│  ├─ admc_invariants.md
│  └─ benchmarks.md
├─ include/
│  └─ admc/
│     ├─ core/
│     │  ├─ math_types.hpp
│     │  ├─ math_utils.hpp
│     │  ├─ aligned_allocator.hpp
│     │  └─ small_array.hpp
│     ├─ admc/
│     │  ├─ invariants.hpp
│     │  ├─ metrics.hpp
│     │  └─ nr_bridge.hpp
│     ├─ world/
│     │  ├─ body.hpp
│     │  ├─ material.hpp
│     │  ├─ world_state.hpp
│     │  └─ transforms.hpp
│     ├─ constraints/
│     │  ├─ contact.hpp
│     │  ├─ joint.hpp
│     │  ├─ constraint_row.hpp
│     │  ├─ constraint_builder.hpp
│     │  ├─ constraint_graph.hpp
│     │  ├─ friction_models.hpp
│     │  └─ rolling_friction.hpp
│     ├─ layout/
│     │  ├─ aos_world_view.hpp
│     │  ├─ soa_world.hpp
│     │  ├─ tile.hpp
│     │  ├─ island.hpp
│     │  └─ layout_utils.hpp
│     ├─ solver/
│     │  ├─ solver_common.hpp
│     │  ├─ solver_interface.hpp
│     │  ├─ baseline/
│     │  │  ├─ baseline_solver.hpp
│     │  ├─ pgs_scalar/
│     │  │  ├─ scalar_solver.hpp
│     │  ├─ pgs_soa/
│     │  │  ├─ soa_solver.hpp
│     │  │  ├─ soa_kernels.hpp
│     │  ├─ pgs_per_contact/
│     │  │  ├─ per_contact_solver.hpp
│     │  ├─ pgs_robust/
│     │  │  ├─ robust_gs_solver.hpp
│     │  ├─ block/
│     │  │  ├─ block_solver.hpp
│     │  ├─ krylov/
│     │  │  ├─ krylov_solver.hpp
│     │  ├─ admm/
│     │  │  ├─ admm_solver.hpp
│     │  │  ├─ admm_kernels.hpp
│     │  ├─ xpbd/
│     │  │  ├─ xpbd_solver.hpp
│     │  └─ config/
│     │     ├─ solver_presets.hpp
│     ├─ backend/
│     │  ├─ executor.hpp
│     │  ├─ single_thread_executor.hpp
│     │  ├─ multi_thread_executor.hpp
│     │  └─ gpu_executor.hpp
│     ├─ metrics/
│     │  ├─ timing.hpp
│     │  ├─ physics_metrics.hpp
│     │  └─ logging.hpp
│     └─ scenes/
│        ├─ scene_factory.hpp
│        ├─ spheres_cloud.hpp
│        ├─ box_stacks.hpp
│        ├─ joint_scenes.hpp
│        └─ randomized_scenes.hpp
├─ src/
│  ├─ core/
│  │  ├─ math_types.cpp
│  │  └─ math_utils.cpp
│  ├─ world/
│  │  ├─ world_state.cpp
│  │  └─ transforms.cpp
│  ├─ constraints/
│  │  ├─ constraint_builder.cpp
│  │  ├─ constraint_graph.cpp
│  │  ├─ friction_models.cpp
│  │  └─ rolling_friction.cpp
│  ├─ layout/
│  │  ├─ soa_world.cpp
│  │  ├─ tile.cpp
│  │  └─ island.cpp
│  ├─ solver/
│  │  ├─ baseline/
│  │  │  └─ baseline_solver.cpp
│  │  ├─ pgs_scalar/
│  │  │  └─ scalar_solver.cpp
│  │  ├─ pgs_soa/
│  │  │  ├─ soa_solver.cpp
│  │  │  └─ soa_kernels.cpp
│  │  ├─ pgs_per_contact/
│  │  │  └─ per_contact_solver.cpp
│  │  ├─ pgs_robust/
│  │  │  └─ robust_gs_solver.cpp
│  │  ├─ block/
│  │  │  └─ block_solver.cpp
│  │  ├─ krylov/
│  │  │  └─ krylov_solver.cpp
│  │  ├─ admm/
│  │  │  ├─ admm_solver.cpp
│  │  │  └─ admm_kernels.cpp
│  │  └─ xpbd/
│  │     └─ xpbd_solver.cpp
│  ├─ backend/
│  │  ├─ single_thread_executor.cpp
│  │  ├─ multi_thread_executor.cpp
│  │  └─ gpu_executor.cpp
│  ├─ metrics/
│  │  ├─ timing.cpp
│  │  └─ physics_metrics.cpp
│  └─ scenes/
│     ├─ spheres_cloud.cpp
│     ├─ box_stacks.cpp
│     ├─ joint_scenes.cpp
│     └─ randomized_scenes.cpp
├─ apps/
│  ├─ bench/
│  │  ├─ CMakeLists.txt
│  │  └─ main_bench.cpp
│  ├─ viewer/
│  │  ├─ CMakeLists.txt
│  │  └─ main_viewer.cpp
│  └─ tools/
│     ├─ dump_metrics.cpp
│     └─ convert_results_to_csv.cpp
├─ tests/
│  ├─ CMakeLists.txt
│  ├─ test_math.cpp
│  ├─ test_admc_invariants.cpp
│  ├─ test_baseline_vs_soa.cpp
│  ├─ test_constraint_builder.cpp
│  ├─ test_solvers_regression.cpp
│  └─ test_scenes.cpp
└─ legacy/
   ├─ README.md
   ├─ baseline_aos_solver.hpp
   └─ baseline_aos_solver.cpp
````

This structure encodes both **modularization** and **parallelism** explicitly:

* Solvers are modular subdirectories.
* Backends are separated from solver logic.
* Layout and constraints are reusable across CPU and GPU.

---

## 12. Baseline & Legacy Strategy

### 12.1 Purpose

The legacy AoS solver remains:

* A canonical reference for correctness.
* A minimal, readable example of the physics.

### 12.2 Integration

* Wrapper to expose it via the common solver interface.
* Included as a solver option in the bench app.
* Used in tests:

  * Compare positions, velocities, and metrics against other solvers.

### 12.3 Maintenance

* Only minimal changes for compatibility and bug fixes.
* No heavy optimization—its value is **stability and simplicity**.

---

## 13. Roadmap

### 13.1 Phase 1: Core & Baseline

* Implement `core`, `world`, `constraints` basics.
* Port legacy AoS baseline into `legacy/`.
* Create the benchmark app with a couple of simple scenes.

### 13.2 Phase 2: SoA Tiles & PGS

* Implement `layout::soa_world`, `layout::tile`, and `layout::island`.
* Implement `solver::pgs_soa` (single-thread backend).
* Reproduce and ideally exceed existing benchmark performance.

### 13.3 Phase 3: Parallel Backends & Metrics

* Implement `backend::multi_thread_executor` using tile and island tasks.
* Flesh out `metrics` (timing + physics + ADMC).
* Add regression tests vs baseline and sanity checks for metrics.

### 13.4 Phase 4: Advanced PGS & Block Solvers

* Add `pgs_per_contact` and `pgs_robust` variants.
* Implement `solver::block` for manifolds/joints.
* Explore over-relaxed PGS and residual-based adaptive iterations.

### 13.5 Phase 5: Global & XPBD Extensions

* Prototype `solver::admm` for global contact/friction.
* Implement `solver::xpbd` for compliant/position-based constraints.
* Add mixed-mode examples (velocity-based contacts + XPBD joints).

### 13.6 Phase 6: GPU & Tooling

* Implement `backend::gpu_executor` using the same tile layout.
* Add initial GPU kernels for PGS and tile operations.
* Improve viewer and add more analysis tools.

### 13.7 Long-Term Directions

* Enhanced shock propagation and stacking robustness.
* Hybrid solvers combining PGS and ADMM.
* Integration with control/optimization pipelines (e.g., robotics).

---

## 14. References

This section will list the key papers and resources that inform:

* ADMC / momentum-first framework.
* Classical PGS / sequential impulses.
* Warm-starting and convergence improvements.
* ADMM-based rigid contact & friction solvers.
* XPBD and compliant constraint methods.
* GPU-friendly and hierarchical parallel rigid body simulation.

(Concrete citations to be added in the actual implementation docs.)

```
::contentReference[oaicite:0]{index=0}
```
