# ADMC Next-Gen Solver Architecture

## 1. Motivation & Goals

We want a **fresh repository** that:

- Keeps the existing **baseline AoS solver** as a clean reference.
- Encodes what we’ve learned from `admc-physics-bench` about performance, data layout, and ADMC invariants.
- Provides a **modular, extensible architecture** for future solver research:
  - SIMD & SoA variants.
  - Block and Krylov-accelerated solvers.
  - Island/tile-based parallelization.
  - GPU-ready backends.
- Makes it easy to **compare solver variants** on shared scenes with consistent metrics (ADMC invariants, penetration, energy & momentum drift, etc.).

This document:

1. Summarizes **current learnings** from the existing repo.
2. States **design principles** for the next-gen solver.
3. Outlines **ideas & improvement opportunities** arising from those learnings.
4. Presents **candidate architectures** and their trade-offs.
5. Proposes a **concrete modularization** and directory layout.
6. Sketches a **high-level roadmap** for building the new repo.

The separate `docs/theory/*.md` documents cover ADMC and constraint math in more detail; this document focuses on **architecture and implementation**.

---

## 2. Learnings from the Current Repo

This section distills what we’ve learned from `admc-physics-bench`—what worked, what didn’t, and where the main bottlenecks are.

### 2.1 Baseline AoS Solver

**Characteristics**

- AoS layout:
  - `std::vector<RigidBody>` and `std::vector<Contact>`.
- Simple PGS / sequential impulses:
  - Per-contact normal row.
  - Optional friction rows.
- No SoA tiling or SIMD; minimal abstraction.

**What we’ve learned**

- Excellent **clarity**:
  - Easy to reason about, debug, and use as a reference.
- For small scenes and simple stacks, the baseline is surprisingly competitive:
  - Very low overhead, no row-build step.
- Doesn’t scale well to **tens of thousands of contacts**:
  - Poor cache behavior.
  - No comprehensive SIMD utilization.
  - Limited parallelism.

**Conclusion**

- The baseline is ideal as a **canonical reference** and for **didactic purposes**.
- It should be preserved in the new repo as a “truth” implementation and regression target, not as the main high-performance engine.

---

### 2.2 Scalar Cached Solver (AoS + Cached Rows)

**Characteristics**

- Still AoS storage for world and contacts.
- Adds:
  - Scalar “rows” per contact (normal + friction).
  - Cached effective masses and impulses.
  - Warm-starting across frames.

**What we’ve learned**

- **Warm-starting** reduces required iterations in stable contact patterns.
- Caching denominators (effective masses) is beneficial when the contact graph changes **slowly**.
- For rapidly changing contact patterns (e.g., exploding clouds), the overhead of building and maintaining caches can partially negate the benefits.
- Still constrained by AoS layout and limited SIMD/parallelism.

**Conclusion**

- Good “classic engine” optimization.
- Still not the right **structural** solution for very large scenes or heavy SIMD.

---

### 2.3 Scalar SoA Solver

**Characteristics**

- Per-step **AoS → SoA conversion**:
  - A `RowSOA` structure holds constraint scalars in separate arrays (normals, tangents, effective masses, impulses, etc.).
- Scalar PGS iterations:
  - Operate over SoA arrays.
- Detailed timing breakdown:
  - Contact prep.
  - Row build.
  - Solver (warm + iterations).
  - Integration.
  - Scatter.

**What we’ve learned**

- SoA improves cache locality and solver throughput compared to pure AoS.
- For high-contact scenes, **row build and contact prep dominate runtime**:
  - Often > 50% of frame time.
  - Solver iterations themselves can be relatively cheap.
- Data movement and assembly cost can overshadow solver math when not carefully managed.

**Conclusion**

- SoA is the right direction for performance.
- However, a global RowSOA plus separate tiling is not enough—row build becomes the main bottleneck.
- Next-gen architecture must focus on **fusing assembly and layout**, not just optimizing the solver loop.

---

### 2.4 SoA-Native SIMD Solver

**Characteristics**

- SoA-native world and constraints:
  - Rigid bodies and contacts stored in SoA-friendly arrays.
- Tiling:
  - Constraints grouped into tiles with local body tables.
  - Tiles sized to fit SIMD width × small factor.
- SIMD-friendly kernels:
  - Tight loops that operate on tiles’ SoA arrays.

**What we’ve learned**

- For medium and large scenes (10k–50k contacts), SoA-Native is often the **fastest CPU path**.
- Once SoA + tiling are in place, solver time becomes a smaller portion of the frame:
  - Row/constraint assembly and contact prep are still major contributors.
- The tiling abstraction is a good **unit of work** for SIMD and future backends.

**Conclusion**

- SoA-Native confirms that **layout + tiling** are critical for high performance.
- The solver math can remain relatively simple (PGS, block PGS) and still perform very well when fed by good data.

---

### 2.5 SoA-Parallel Solver

**Characteristics**

- Island-based, tile-based parallel wrapper around SoA-Native.
- Idea:
  - Decompose into independent islands.
  - Run tiles or islands on multiple threads.

**What we’ve learned**

- Conceptually correct:
  - Islands and tiles are the right granularity for parallelism.
- Implementation complexity:
  - If the separation between solver kernel and execution backend is not clean, parallel overhead can destroy performance.
- In early experiments, parallel execution sometimes made things slower than single-thread runs:
  - Scheduling overhead.
  - Synchronization or false sharing.
  - Nested parallelism or confusing thread counts.

**Conclusion**

- Parallelism must be a **first-class concern** in the architecture:
  - Clear kernel vs. backend roles.
  - Task-based execution and work stealing.
- SoA-Parallel is a strong proof that the underlying **tiling abstraction** is suitable for parallel backends, but the design must be “backend-aware” from the start.

---

### 2.6 Vec-SoA Path

**Characteristics**

- Intermediate SoA layout meant to host more advanced vectorized kernels.
- Currently forwards many operations to scalar kernels.

**What we’ve learned**

- Layout changes alone are not enough:
  - Kernels themselves must be designed to exploit SIMD lanes.
- Useful as an experimental path to plug in alternative kernels, but not yet a production-quality backend.

**Conclusion**

- Vec-SoA is a candidate “playground” backend for further SIMD experimentation in the new repo.
- It should have a clear contract and be one of several backends under a common interface.

---

### 2.7 High-Level Takeaways

From all the above, key conclusions:

- **Constraint assembly is the main bottleneck** at scale:
  - Especially AoS → SoA conversion and row build.
- Pure AoS is still best for **tiny scenes and clarity**.
- **SoA + tiling** plus per-tile SIMD kernels are the best path for large scenes.
- Parallelism requires a clean division between:
  - **Local kernels** operating on self-contained data (tiles, islands).
  - **Execution backends** responsible for scheduling and threading.
- Detailed instrumentation (timings, metrics, ADMC invariants) is invaluable and should be a core part of the design.

---

## 3. Design Principles for the Next-Gen Solver

Based on these learnings, we adopt the following principles:

1. **Keep the AoS baseline**  
   - Preserve a simple, AoS, sequential impulse solver as a canonical reference.  
   - Use it for correctness checks, regression tests, and small-scene examples.

2. **Data-oriented design**  
   - Separate conceptual entities (world, constraints, solver) from their **data layouts** (AoS, SoA, AoSoA, tiles).
   - Allow multiple layouts behind stable interfaces where it makes sense.

3. **Row-level abstraction**  
   - Retain a **row-centric view** for constraints (normal, friction, joints):
     - Clear mapping to theory.
     - Easy to swap solvers and preconditioners.
   - Implementation can remap these rows into tiles internally.

4. **Tile- and island-centric execution**  
   - Treat tiles and islands as primary units of work for performance-sensitive solvers.
   - Ensure tile execution is as self-contained as possible.

5. **Backend separation**  
   - Cleanly separate solver **kernels** from **execution backends**:
     - Single-thread, multi-thread, GPU.
   - No solver should be tightly coupled to a specific threading model.

6. **Metrics-first**  
   - Treat ADMC invariants and constraint metrics as first-class outputs:
     - Built-in measurement of drift, penetration, joint error, cone consistency, etc.
   - Make benchmarks and tests depend on these metrics.

7. **Extensibility**  
   - Make it easy to plug in new solver families:
     - Block solvers.
     - Krylov-accelerated PGS.
     - ADMM / XPBD variants.
   - Solvers should reuse shared world, constraint, and layout code.

---

## 4. Ideas & Improvement Opportunities

This section outlines concrete directions the new architecture should support, based on performance bottlenecks and design goals.

### 4.1 Data Layout & Row Build

**Goals**

- Reduce the cost of constraint assembly.
- Improve cache locality.
- Make SoA and tiles natural, not bolted-on.

**Ideas**

1. **SoA-all-the-way for hot paths**  
   - For high-performance solvers, maintain bodies and constraints in SoA/AoSoA layouts from the start.
   - Avoid repetitive AoS → SoA conversions each frame.

2. **Fuse row build and tiling**  
   - Instead of:
     - AoS → global RowSOA → tile partition.
   - Do:
     - AoS/SoA → tiles directly:
       - As contacts are generated or updated, they are assigned to tiles and stored in tile-local SoA arrays.

3. **Cross-frame caching for stable contacts**  
   - Cache per-contact:
     - World-space frames (normal + tangents).
     - Offsets and effective masses.
   - Recompute only when:
     - Body orientations change beyond a threshold.
     - Contact topology changes.

4. **Shape-specific fast paths**  
   - Separate special-case pipelines for:
     - Sphere–sphere.
     - Sphere–plane.
   - Precompute or simplify effective mass terms where possible.

5. **AoSoA (array-of-structures-of-arrays)**  
   - Use tiles themselves as AoSoA blocks:
     - A tile is an SoA block of rows with a fixed capacity.
     - Provides excellent cache locality and straightforward SIMD mapping.

---

### 4.2 Solver Algorithms

**Goals**

- Faster convergence without sacrificing stability.
- Reuse the same constraint data for multiple solver families.

**Ideas**

1. **Residual-based adaptive iterations**  
   - Track per-tile residual norms.
   - Stop iterating tiles that have converged while continuing on hard ones.
   - Reduces wasted iterations on easy regions.

2. **Block solvers for manifolds and joints**  
   - Solve small multi-row blocks exactly or more accurately:
     - Contact manifold: 1 normal + 2 tangents, or 4-contact blocks.
     - Joint: 3–6 scalar constraints.
   - Use small dense factorizations as:
     - Standalone solvers for these blocks.
     - Preconditioners inside PGS.

3. **Over-relaxed PGS (SOR)**  
   - Introduce a relaxation factor ω:
     - ω > 1 for faster convergence (with caution).
   - Tune globally or per tile.

4. **Krylov-accelerated PGS**  
   - Use residuals to compute improved updates (e.g., Anderson acceleration).
   - Maintain projections (clamping) on top.

5. **ADMM-inspired solvers (long-term)**  
   - Splitting methods with:
     - Local constraint updates.
     - Global consistency step.
   - Particularly interesting for dense contact blocks and GPU implementations.

6. **XPBD / compliance variants**  
   - Position-level constraints with compliance.
   - Use similar graph and row structures, different solver loops.
   - Useful for soft constraints and robust stacking.

---

### 4.3 Parallelism & Scheduling

**Goals**

- Make multi-threading a net win, not a regression.
- Prepare for GPU backends without forcing them into CPU-centric patterns.

**Ideas**

1. **Island decomposition**  
   - Build contact/joint graphs and identify connected components.
   - Treat islands as units of work:
     - Suitable for threading across cores.
     - Also match natural synchronization boundaries.

2. **Graph coloring for conflict-free batches**  
   - Within each island, color the constraint graph:
     - Each color is a set of constraints that do not share bodies.
   - Process one color at a time in parallel:
     - Jacobi-like updates within a color, GS across colors.

3. **Tile-based tasks**  
   - Represent each tile (or group of tiles) as a task.
   - Use a work-stealing scheduler:
     - Threads pull tiles as they become idle.
   - Avoid per-row locks; keep write conflicts out of the tile kernel.

4. **Backend-agnostic kernels**  
   - Solver kernels must:
     - Accept tile/island contexts.
     - Not assume a particular threading model.
   - Execution backends decide:
     - How many threads.
     - How tasks are scheduled.
     - How to map tiles to GPU workgroups, etc.

---

### 4.4 Diagnostics & Tooling

**Goals**

- Make it easy to see *why* performance or stability improved or regressed.
- Ensure theory and implementation stay aligned.

**Ideas**

1. **Instrumentation hooks**  
   - Timings per phase:
     - Contact gen, row build, solver warm, solver iterations, integration, scatter.
   - Metrics:
     - ADMC directional drift.
     - Penetration and joint error.
     - Cone consistency.
     - Residuals and iteration counts.

2. **Bench harness**  
   - Command-line tool to:
     - Run scenes.
     - Select solvers and backends.
     - Emit human-readable tables + machine-readable JSON/CSV.

3. **Regression tests**  
   - Small suite of standard scenes:
     - Clouds, stacks, joints, random.
   - Compare:
     - Different solvers.
     - Baseline vs new architectures.
   - Use tolerance-based comparison of metrics.

4. **(Optional) Visual viewer**  
   - Lightweight tool to inspect dynamics and stability visually.
   - Not required for the core library but helpful for development.

---

## 5. Candidate Architectures & Trade-offs

We consider three main architectural shapes.

### 5.1 Architecture A — Row-Centric Pipeline

**Concept**

- Everything revolves around explicit **constraint rows**:
  - AoS world → row builder → solver → integration.
- Layout is abstracted away but essentially the solver sees the world as a list of rows.

**Strengths**

- Conceptually simple and close to theory.
- Easy to plug in various solvers:
  - PGS, block solvers, Krylov, ADMM.
- Good for building prototypes and test harnesses.

**Weaknesses**

- Tends to accumulate a large global row array.
- Harder to express tile-based SoA optimizations without extra layers.
- Doesn’t naturally express island/tile parallelism.

---

### 5.2 Architecture B — Tile/Island-Centric SoA Engine

**Concept**

- Primary entities are:
  - **Islands** (connected components).
  - **Tiles** (AoSoA blocks with local body tables).
- Constraints are built directly into tiles; solvers operate on tiles.

**Strengths**

- Excellent data locality and SIMD story in high-performance paths.
- Naturally maps to:
  - Multi-threading: tiles as tasks.
  - GPU: tiles as workgroups.
- Fusing build + tiling reduces assembly overhead.

**Weaknesses**

- More complex mental model:
  - Users need to understand islands and tiles.
- Harder to expose a simple “list of rows” API externally.
- Might encourage overly specialized paths if not balanced with a more abstract view.

---

### 5.3 Architecture C — Hybrid (Row API, Tile Engine, Multi-Backend)

**Concept**

- **External API is row-centric** (like Architecture A):
  - World + constraints + rows as conceptual objects.
- **Internal engine is tile/island-centric** (like Architecture B):
  - High-performance solvers use tiles and SoA layouts.
- **Multiple backends** share common abstractions:
  - AoS baseline.
  - SoA tile-based CPU solver.
  - Experimental GPU / advanced solvers.

**Strengths**

- Best of both worlds:
  - Simple conceptual API aligned with theory and tests.
  - High-performance internals for large scenes.
- Solvers remain pluggable:
  - All operate on the same conceptual data.
- Users can ignore tiling details unless they care about performance internals.

**Weaknesses**

- More layers → potential for boilerplate if not kept in check.
- Requires discipline to avoid duplication of logic between layers.

---

### 5.4 Recommended Architecture

We propose to adopt **Architecture C**:

- **Externally**:
  - A row-centric API:
    - Worlds, bodies, materials.
    - Contacts, joints, constraint rows.
    - Solver parameters and metrics.
- **Internally**:
  - Island/tile-centric SoA engine for high-performance backends.
- **Backends**:
  - AoS baseline solver.
  - SoA tile-based PGS / block PGS.
  - Optional advanced backends (Krylov, ADMM, GPU, XPBD).

This preserves the existing **intellectual model** (rows, PGS, ADMC invariants) while aggressively optimizing the **implementation** for real-world performance.

---

## 6. Modularization & Components

We now map the architecture to modules and responsibilities.

### 6.1 Core Modules

1. **`core`**
   - Math types (vec, mat, quat).
   - Small containers, aligned allocators.
   - Utilities shared across the library.

2. **`admc`**
   - Definitions of ADMC-style directional invariants.
   - Newtonian bridge and directional metrics.
   - Helpers to compute drift and related statistics.

3. **`world`**
   - Body definitions (mass, inertia, state).
   - Materials (friction, restitution, etc.).
   - Global world state and high-level management.

4. **`constraints`**
   - Contact manifolds.
   - Joints (distance, hinge, etc.).
   - Constraint rows and builders.
   - Constraint graphs (islands, graph coloring).

5. **`layout`**
   - AoS world views (for baseline).
   - SoA/AoSoA world representations.
   - Tiles and islands as data structures.
   - Conversion and staging utilities.

6. **`solver`**
   - Common solver interface (params, stats).
   - Solver families:
     - Baseline AoS PGS.
     - Scalar SoA PGS.
     - SoA tile-based PGS / block PGS.
     - Experimental solvers (Krylov, ADMM, XPBD).
   - Configuration presets (e.g., “fast”, “stable”, “reference”).

7. **`backend`**
   - Execution backends:
     - Single-thread executor.
     - Multi-thread executor.
     - (Optional) GPU executor.
   - Task and job definitions for tiles/islands.

8. **`metrics`**
   - Timing utilities.
   - Physics metrics (penetration, drift, cone consistency).
   - Logging and reporting helpers.

9. **`scenes`**
   - Scene generators:
     - Spheres clouds.
     - Box stacks.
     - Joint tests.
     - Randomized stress tests.

10. **`apps` & `tests`**
    - Bench CLI.
    - Optional viewer.
    - Unit and regression tests.

11. **`legacy`**
    - The old baseline AoS solver, lightly wrapped to fit into the new interfaces.

---

## 7. Proposed Directory Layout

A possible directory structure:

```text
admc-next/
├─ CMakeLists.txt
├─ README.md
├─ docs/
│  ├─ architecture_overview.md       # This document
│  ├─ solver_design.md               # Detailed solver internals
│  ├─ layout_and_tiling.md           # SoA/AoSoA, tiling, islands
│  ├─ theory/
│  │  ├─ 01_admc_overview.md
│  │  ├─ 02_relativistic_admc.md
│  │  ├─ 03_nr_mechanics_bridge.md
│  │  ├─ 04_constraint_rows_math.md
│  │  ├─ 05_pgs_and_variants_math.md
│  │  └─ 06_invariants_and_metrics.md
│  └─ benchmarks.md                  # Methodology & results
├─ include/
│  └─ admc/
│     ├─ core/
│     │  ├─ math_types.hpp
│     │  ├─ math_utils.hpp
│     │  ├─ aligned_allocator.hpp
│     │  └─ small_array.hpp
│     ├─ admc/
│     │  ├─ invariants.hpp            # Directional invariants helpers
│     │  ├─ metrics.hpp               # ADMC-based drift checks
│     │  └─ nr_bridge.hpp             # Non-relativistic mappings
│     ├─ world/
│     │  ├─ body.hpp                  # RigidBody, BodyId, etc.
│     │  ├─ material.hpp              # Material, friction, restitution
│     │  ├─ world_state.hpp           # World, islands, global IDs
│     │  └─ transforms.hpp
│     ├─ constraints/
│     │  ├─ contact.hpp               # ContactPoint, ContactManifold
│     │  ├─ joint.hpp                 # Joint definitions
│     │  ├─ constraint_row.hpp        # Row representation
│     │  ├─ constraint_builder.hpp    # Builds rows from contacts/joints
│     │  └─ constraint_graph.hpp      # Islands, graph coloring
│     ├─ layout/
│     │  ├─ aos_world_view.hpp        # Views over AoS body/contact arrays
│     │  ├─ soa_world.hpp             # SoA representation of bodies
│     │  ├─ tile.hpp                  # Tile struct + local body tables
│     │  ├─ island.hpp                # Island containers
│     │  └─ layout_utils.hpp          # Conversion & staging helpers
│     ├─ solver/
│     │  ├─ solver_common.hpp         # SolverParams, SolverStats
│     │  ├─ solver_interface.hpp      # Abstract solver API
│     │  ├─ baseline/
│     │  │  ├─ baseline_solver.hpp    # AoS reference solver
│     │  ├─ pgs_scalar/
│     │  │  ├─ scalar_solver.hpp      # AoS or SoA scalar PGS
│     │  ├─ pgs_soa/
│     │  │  ├─ soa_solver.hpp         # SoA/tile PGS solver
│     │  │  ├─ soa_kernels.hpp        # Tile kernels (SIMD-aware)
│     │  ├─ block/
│     │  │  ├─ block_solver.hpp       # Block PGS / manifold solver
│     │  ├─ krylov/
│     │  │  ├─ krylov_solver.hpp      # Accelerated solver variants
│     │  ├─ admm/
│     │  │  ├─ admm_solver.hpp        # ADMM-inspired solver (optional)
│     │  ├─ xpbd/
│     │  │  ├─ xpbd_solver.hpp        # Position-based solver (optional)
│     │  └─ config/
│     │     ├─ solver_presets.hpp     # “Fast”, “Stable”, “Reference”, etc.
│     ├─ backend/
│     │  ├─ executor.hpp              # Common executor interface
│     │  ├─ single_thread_executor.hpp
│     │  ├─ multi_thread_executor.hpp
│     │  └─ gpu_executor.hpp          # Or gpu_executor_stub.hpp initially
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
│  ├─ world/
│  ├─ constraints/
│  ├─ layout/
│  ├─ solver/
│  ├─ backend/
│  ├─ metrics/
│  └─ scenes/
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

Notes:

* `include/admc` is the **public API surface**.
* `src/` contains implementations; some hot code may remain header-only.
* `legacy/` preserves the original baseline AoS solver implementation.

---

## 8. Using the Baseline in the New Repo

We want to keep the baseline solver as a **reference** but not entangle it with the new architecture.

**Plan**

* Place baseline code in `legacy/` with minimal changes.
* Implement a thin adapter that lets the bench harness call it through the same `SolverInterface` used by other solvers:

  * Example: `LegacyBaselineSolver` implementing `ISolver`.

**Regression & comparison**

In tests and benchmarks, we can:

* Compare baseline vs new solvers on selected scenes:

  * Spheres clouds, stacks, joints, randomized.
* Check:

  * Positions and velocities (within tolerances).
  * ADMC directional invariants.
  * Penetration and joint error.
  * Energy and momentum drift.

This ensures that improvements in performance or new solver features do not silently degrade physical behavior.

---

## 9. Key Literature & Links

The new repo is grounded in:

* ADMC and the Momentum-First framework (theoretical grounding).
* Classic rigid-body contact and joint modeling:

  * Constraint rows, effective masses, ERP/Baumgarte, restitution.
* PGS / sequential impulses and warm-starting:

  * Practical solver design for real-time engines.
* SoA and AoSoA data layout best practices:

  * SIMD-friendly structures, tiling, and cache-aware design.
* Advanced methods:

  * Block PGS, Krylov acceleration, ADMM, XPBD.

The `docs/theory/*.md` files document these aspects in more detail. The architecture described here is designed to sit on top of that theory and make it **practical and extensible**.

---

## 10. Roadmap (High-Level)

A possible rollout plan for the new repo:

1. **Bring up core & baseline**

   * Implement `core`, `world`, `constraints` minimal set.
   * Import baseline AoS solver into `legacy/`.
   * Provide a simple bench app and basic scenes.

2. **Introduce SoA world & tiles**

   * Implement `layout/soa_world.hpp`, `tile.hpp`, `island.hpp`.
   * Add a tile-based PGS solver (`pgs_soa`) using single-thread executor.
   * Verify parity with baseline on small scenes.

3. **Instrumentation & metrics**

   * Add timing and physics metrics in `metrics/`.
   * Extend bench app to output tables and JSON/CSV.

4. **Parallel backends**

   * Implement `multi_thread_executor`.
   * Introduce island-based and tile-based tasking.
   * Tune thread counts and tile sizes.

5. **Advanced solvers**

   * Add block PGS for manifolds.
   * Experiment with over-relaxed PGS and residual-based adaptive iterations.
   * Prototype Krylov/ADMM-inspired variants and XPBD, using the same constraint graph and tile abstractions.

6. **Optional extras**

   * GPU-backed solver using tiles as workgroups.
   * A simple viewer app to visualize scenes and stability.

Following this roadmap should take us from:

> “Clean baseline + fresh architecture”

to

> “A robust, research-friendly solver stack with ADMC-aware metrics and high-performance backends.”
