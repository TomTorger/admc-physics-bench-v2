```markdown
# ADMC Next-Gen Solver – Literature Review

## 1. Scope and Purpose

This document surveys key literature on rigid-body contact solvers, iterative methods, and parallel implementations that are directly relevant to the ADMC next-gen solver. The goal is to:

- Capture the **design space** of modern contact solvers (PGS, per-contact iterations, ADMM, XPBD, etc.).
- Understand **convergence, stability, and robustness** trade-offs.
- Extract ideas for:
  - Data layout and batching.
  - Solver algorithms (scalar, block, Krylov, ADMM-like).
  - Parallelization (CPU and GPU).
- Anchor the **ADMC / directional-momentum** viewpoint in the broader literature.

---

## 2. Classical Game / DEM Contact Solvers

### 2.1 Catto’s Sequential Impulses and Constraint View

**Key ideas**

- Introduces an **iterative constraint solver for rigid body dynamics with contact** (essentially a projected Gauss–Seidel / sequential impulses scheme) designed for real-time games.
- Emphasizes:
  - Linear-time, linear-memory contact solving.
  - **Temporal coherence** via contact caching: reusing previous impulses as warm-starts.
  - Explicit construction of **Jacobian rows** and constraint mass (“effective mass”).
- Later work on “Understanding Constraints” expands the conceptual toolkit:
  - Clear distinction between **position**, **velocity**, and **acceleration** constraints.
  - Discussion of constraint stabilization, mass ratios, and convergence issues in stacks.
  - Practical tricks: Baumgarte vs. position correction, soft constraints, block solvers.

**Relevance to ADMC**

- Validates the **row-centric, sequential impulse** approach as a baseline.
- Justifies **warm-starting and caching** in our architecture (e.g., persistent manifold/row data).
- Motivates our emphasis on:
  - Per-row effective masses.
  - Constraint graphs and island decomposition.
  - Block solvers for contact manifolds and joints.

---

### 2.2 Warm-Started Projected Gauss–Seidel (PGS) for DEM / Granular Matter

**Key ideas**

- Studies **warm-starting strategies** for PGS in nonsmooth Discrete Element Method (DEM) simulations of granular media.
- Shows that reusing previous impulses as initial guesses can reduce iteration counts by **2–5×** without sacrificing accuracy.
- Highlights:
  - Importance of **temporal coherence** in large contact graphs.
  - Sensitivity of convergence to contact topology changes.

**Relevance to ADMC**

- Justifies our plan to:
  - Maintain **persistent impulse caches** at the manifold/row level.
  - Treat warm starting as a **first-class feature** in solver APIs and data layout.
- Suggests adding metrics and tooling to:
  - Measure convergence vs. warm-start strategy.
  - Compare “cold” vs. “cached” vs. “ADMC-guided” warm starts.

---

### 2.3 Accelerated Modulus-Based Gauss–Seidel (MBGS) for Interactive Rigid Bodies

**Key ideas**

- Applies **modulus-based matrix splitting** and acceleration techniques to large LCPs arising from rigid-body contact.
- Focuses on exploiting **sparsity** and **block structure** of the Delassus matrix to improve convergence over vanilla PGS.
- Demonstrates:
  - Better per-iteration error reduction than PGS.
  - Practical techniques to avoid the full dense matrix while still benefiting from advanced splitting schemes.

**Relevance to ADMC**

- Motivates a solver layer that can:
  - Use richer **block/splitting preconditioners** over tile- or manifold-level blocks.
  - Experiment with **modulus-based updates** as an alternative to vanilla PGS inside each tile.
- Suggests that our architecture should:
  - Expose a **matrix-free “apply Delassus”** or “apply Jacobian / mass-inverse / Jacobian^T” primitive to support such methods.
  - Support **multiple iterations modes**: classic PGS vs. MBGS vs. block preconditioners.

---

### 2.4 Per-Contact Iteration Methods

**Key ideas**

- Introduces a **per-contact iteration method** for solving contact dynamics, primarily for legged robots.
- Rather than updating scalar rows independently, the method:
  - Solves **single-contact subproblems** (normal + friction) more exactly (bisection-based search).
  - Uses successive over-relaxation at the level of **contacts** (blocks), not individual scalar rows.
- Shows significantly better performance than PGS in challenging articulated contact scenarios.

**Relevance to ADMC**

- Strongly supports a **manifold/block-centric design**:
  - Treat each contact manifold as a **small coupled block** (1 normal + 2 tangents) and solve it more accurately.
  - View PGS as the outer iteration, with per-manifold inner solves.
- Aligns with our plan for:
  - Block solvers per manifold (3×3, 6×6) as preconditioners or stand-alone methods.
  - ADMC-aware block solvers that explicitly manage directional momentum within a manifold.

---

## 3. ADMM and Convex Optimization for Contact

### 3.1 Unified ADMM Frameworks for Rigid and Compliant Contact

**Key ideas**

- Casts rigid and compliant contact as **Nonlinear Complementarity Problems (NCPs)**, then solves them using **ADMM + proximal operators**.
- Unifies:
  - Rigid contacts (hard unilateral constraints).
  - Compliant contacts (penalty/spring-like models).
- Introduces:
  - Automatic tuning of ADMM hyperparameters.
  - Efficient proximal operators for contact/friction.
- Demonstrates improved robustness and scalability vs. classic game-engine solvers, especially in **ill-conditioned or highly constrained** setups (hands, granular media).

**Relevance to ADMC**

- Confirms that ADMM is a viable **“high-end” contact solver** when we can afford global or semi-global iterations.
- Suggests a long-term direction for ADMC:
  - ADMC invariants could be encoded as **proximal penalties or constraints** in an ADMM scheme.
  - Our row/tile layout must be flexible enough to support:
    - PGS-like local methods.
    - ADMM-like global methods.
- At architecture level:
  - We should design the constraint graph and tile structures as a **generic factorization** of the contact problem, not hard-wired to PGS.

---

### 3.2 ADMM for Dynamics, Deformables, and Projective Dynamics

**Key ideas**

- “ADMM ⊇ Projective Dynamics” shows that **projective dynamics** (PD) can be interpreted as an instance of ADMM:
  - Local constraint projections ↔ proximal steps.
  - Global solve ↔ dual/primal update.
- Further ADMM work on dynamics and control:
  - ADMM as a backbone for **contact-aware optimal control**.
  - Use of Anderson acceleration and other techniques to improve convergence.

**Relevance to ADMC**

- Reinforces the **conceptual bridge**:
  - PGS / XPBD / PD and ADMM all sit on a spectrum of **splitting methods**.
- Design implications:
  - By expressing our constraints and data flow in terms of “local projections” + “global updates,” we keep the door open for:
    - ADMC-aware PD/ADMM solvers.
    - Hybrid methods (e.g., PGS with occasional global ADMM smoothing or residual projection).
- Encourages a **matrix-free, operator-centric design**: treat solvers as orchestrators of repeated applications of primitive operators (J, Jᵀ, M⁻¹, proximal contact/friction, etc.).

---

### 3.3 ADMM in Robotics and Optimal Control

**Key ideas**

- ADMM used for:
  - Multi-contact model predictive control.
  - Robust optimization with frictional contacts.
- Focus on:
  - Reliable convergence guarantees.
  - Handling of friction cones via cone programs or proximal operators.
- Shows that real-time performance with ADMM is possible for **reduced or structured problems**.

**Relevance to ADMC**

- Indicates that the ADMC solver codebase could later serve:
  - Not just forward simulation, but **control and identification** tasks.
- Supports the idea of:
  - Designing solvers that can operate both as forward simulation engines and as **differentiable layers** or optimization primitives.

---

## 4. Position-Based Dynamics and XPBD

### 4.1 XPBD Fundamentals

**Key ideas**

- XPBD generalizes Position-Based Dynamics by introducing **constraint compliance** and a **Lagrange multiplier** per constraint.
- Allows:
  - Stable simulation of stiff constraints without tiny time steps.
  - Clean handling of compliance and energy conservation compared to classic PBD.
- XPBD has been extended and reinterpreted as:
  - A form of **nonlinear Gauss–Seidel** on a constraint potential.
  - A special case in broader splitting/ADMM frameworks.

**Relevance to ADMC**

- For our solver, XPBD suggests:
  - A **position-level, constraint-centric** alternative to pure impulse/velocity methods.
  - A way to add **softness/compliance** while still using a row-centric interface.
- Architecture implications:
  - Our constraint representation should be rich enough to store:
    - Compliance.
    - Per-constraint Lagrange multipliers.
  - The world/row/tile layout should not assume exclusively “impulse at velocity level”; it should allow position-level solvers to reuse the same contact graph and tiling.

---

### 4.2 XPBD for Rigid Bodies and High-Resolution Systems

**Key ideas**

- Extensions of XPBD to rigid bodies:
  - Detailed rigid-body simulations using XPBD for complex stacking and contact scenarios.
- Recent works:
  - Multigrid and algebraic multigrid (AMG) preconditioners wrapped around XPBD for high-resolution meshes.
  - Differentiable XPBD (DiffXPBD) for gradient-based parameter fitting and control.

**Relevance to ADMC**

- Suggests that:
  - Our tile-centric architecture can be reused by XPBD-like solvers, with tiles acting as **regions** for multigrid coarsening or block preconditioners.
  - A future ADMC backend could be:
    - Hybrid: XPBD-like for soft or compliant parts, PGS/ADMM for rigid parts.
- Also reinforces the value of a **clean differentiation-ready design**:
  - ADMC metrics and invariants can be integrated into differentiable pipelines.

---

## 5. Parallel and GPU Contact Solvers

### 5.1 Early GPU Constraint Solvers and Batch Construction

**Key ideas**

- Harada’s GPU work and related OpenCL notes show:
  - A **parallel constraint solver** that batches constraints into groups with no body overlap, allowing parallel PGS on GPU.
  - A two-level batch construction:
    - Global batching into independent groups.
    - Local SIMD-friendly batches per group.
- Highlights the challenges of:
  - Creating independent constraint batches in parallel.
  - Dealing with extremely dense cells (many contacts in one region).

**Relevance to ADMC**

- Validates our focus on:
  - **Graph coloring and batching** at the tile/island level.
  - A separation between:
    - Solver kernels (operate on local, independent batches).
    - Scheduling (decides how to partition constraints into batches).
- Informs our parallel API:
  - A concept of “batch” or “color” should be explicit in the data structures.

---

### 5.2 Hierarchical Parallelization with Soft Blocking on GPU

**Key ideas**

- Hierarchical parallelization for rigid-body simulations:
  - Introduces a **soft blocking** method to manage conflicts between tasks on GPU.
  - Organizes work hierarchically (coarse → fine) to keep many cores busy while avoiding race conditions.
- Combines:
  - Contact graph analysis.
  - Hierarchical task decomposition.
  - GPU scheduling strategies.

**Relevance to ADMC**

- Suggests that our solver/executor layer should:
  - Be **hierarchical from the start**: islands → tiles → batches.
  - Expose enough metadata (body ranges, constraint scopes, etc.) for advanced blocking strategies.
- Encourages us to:
  - Keep the GPU backend in mind even if we start on CPU:
    - Prefer regular, fixed-size tiles.
    - Avoid heavy per-tile dynamic allocations.

---

### 5.3 GPU-Optimized Collision Detection and Response

**Key ideas**

- Recent GPU work combines:
  - Octree/AABB hierarchies for broad-phase.
  - GPU-parallel narrow-phase and simple response.
- Emphasis on:
  - Handling **densely populated 3D scenes**.
  - Balancing collision detection vs. response work.

**Relevance to ADMC**

- Confirms that collision detection and contact generation can be:
  - Major performance bottlenecks.
  - Good targets for GPU offload.
- Our architecture should:
  - Separate **collision/contact generation** from **solver** in a way that:
    - Allows both to be parallelized independently.
    - Uses compatible data layouts (e.g., SoA contacts flowing directly into SoA tiles).

---

## 6. High-Order Iterative and Jacobi/Gauss–Seidel Variants

### 6.1 Near Second-Order Jacobi/Gauss–Seidel (JGS2)

**Key ideas**

- JGS2 proposes a **Jacobi/Gauss–Seidel-like GPU method** for elastodynamics with:
  - Convergence rates close to full Newton’s method.
  - Parallelism similar to Jacobi.
- Key observation:
  - **Local overshoot** is the main reason parallel Jacobi/GS converges slowly or stagnates.
- Solution:
  - Precompute reduced models that make local updates globally aware.
  - Retain Jacobi-level parallelism but with near second-order convergence.

**Relevance to ADMC**

- Encourages us to:
  - Instrument our solvers to detect **overshoot-like behavior** (e.g., residual norms increasing).
  - Explore **precomputed or adaptive correction terms** at tile/manifold level to mitigate overshoot.
- Conceptual lesson:
  - It is possible to **bridge convergence and parallelism** by enriching local updates, not just by adding more global iterations.

---

### 6.2 Other GS Acceleration and Block Methods

**Key ideas**

- Numerous works on:
  - Block coordinate descent for complementarity and inequality-constrained problems.
  - Symmetric and multi-sweep GS variants acting as **preconditioners**.
  - Heuristic GS acceleration in game physics engines.

**Relevance to ADMC**

- Supports our aim to:
  - Treat “PGS” not as a fixed algorithm, but as a **family of Gauss–Seidel-like schemes**.
  - Implement block and accelerated variants as pluggable modules in the solver stack.

---

## 7. Summary of Design Implications for ADMC Next-Gen Architecture

### 7.1 Data Layout and Row/Tile Design

From Catto, warm-start PGS, MBGS, and GPU batching:

- Emphasize **SoA-native, tile-centric layouts** where:
  - Each tile holds local bodies and constraint rows.
  - Rows are grouped into batchable sets (colors) with no overlapping bodies.
- Make **warm-starting** part of the design:
  - Persistent row/manifold IDs.
  - Stable indexing across frames where possible.
- Support **block rows**:
  - Per-manifold blocks (normal+friction) as first-class citizens.
  - Optional expansion into scalar rows for low-end paths.

### 7.2 Solver Interfaces

From per-contact iterations, ADMM, and XPBD:

- Define solver interfaces in terms of:
  - **Local projections / subproblem solves** (contact manifolds, joints).
  - **Global coupling operators** (mass inverse, Jacobian/Jᵀ).
- Ensure the interface is flexible enough to host:
  - Classic PGS (row-by-row).
  - Per-contact/per-manifold iterations with more accurate local solves.
  - ADMM-like splitting (local prox + global solve).
  - XPBD/position-based variants.

### 7.3 Parallelization

From Harada, hierarchical GPU work, and JGS2:

- Design **parallelism in from the start**:
  - Islands are independent.
  - Within islands, tiles are small, cache-resident, and parallelizable.
  - Within tiles, rows/manifolds are colored or batched for lock-free updates.
- Executors should support:
  - Single-thread, multi-thread CPU, and GPU backends with the **same logical API**.
- Use concurrency-friendly data structures:
  - Fixed-size or slab allocators for tiles.
  - Read-only views for world/constraint data during solve; write-back in well-defined phases.

### 7.4 ADMC / Directional Momentum Integration

Cross-cutting ideas from all literature:

- ADMC (directional momentum conservation) can be layered on top of:
  - PGS/XPBD (as metrics and soft constraints).
  - ADMM (as proximal terms or equality constraints).
- The architecture should:
  - Let us **evaluate ADMC invariants cheaply** per tile or per manifold.
  - Permit **ADMC-guided heuristics** (e.g., adjust iteration counts or relaxation where directional momentum violations are highest).
  - Support both **forward simulation** and **offline analysis/control** workflows.

---

## 8. Takeaways

1. **Sequential impulses and PGS** remain the practical workhorse for game physics and DEM; we should keep a clear, AoS baseline and a highly optimized SoA tile-based PGS core.
2. **Warm-starting and caching** are proven, essential techniques for large scenes and must be designed into the data layout and scene update logic.
3. **Block and per-contact methods** deliver better convergence on articulated systems and complex contact graphs; our solver should treat manifolds as first-class blocks.
4. **ADMM and convex optimization** offer a robust, unified view of contact and friction; the architecture should be ADMM-ready even if we start with PGS-like solvers.
5. **XPBD and PD-like approaches** show how to handle compliance and large time steps; we should allow position-level solvers to reuse the same constraint graph and tiling.
6. **Parallelization is structural**:
   - Islands, tiles, batches must be explicit concepts.
   - GPU-friendly layouts and hierarchical scheduling should be designed from the outset.
7. **Convergence vs. parallelism is not a zero-sum game**: techniques like JGS2 illustrate how enriched local models can reconcile high parallelism with good convergence—an inspiring direction for future ADMC solvers.

This literature collectively justifies the next-gen architecture: a **modular, row/tile-centric, parallel-first solver stack** with clear separation of concerns (world, constraints, layout, solvers, executors), and enough flexibility to support everything from classic PGS to ADMC-aware ADMM and XPBD-style methods.
```

**Key references used for this review**

* Erin Catto, *“Iterative Dynamics with Temporal Coherence”*, GDC 2005; and *“Understanding Constraints”*, GDC 2014. ([box2d.org][1])
* Da Wang, Martin Servin, Tomas Berglund, *“Warm starting the projected Gauss–Seidel algorithm for granular matter simulation”*, Computational Particle Mechanics, 2016. ([umit.cs.umu.se][2])
* Shugo Miyamoto, Makoto Yamashita, *“Sparsity Exploitation of Accelerated Modulus-Based Gauss-Seidel Method for Interactive Rigid Body Simulations”*, 2019. ([arXiv][3])
* Jemin Hwangbo, Joonho Lee, Marco Hutter, *“Per-Contact Iteration Method for Solving Contact Dynamics”*, IEEE Robotics and Automation Letters, 2018. ([SciSpace][4])
* Justin Carpentier et al., *“From Compliant to Rigid Contact Simulation: a Unified and Efficient Approach”*, RSS 2024. ([arXiv][5])
* Overby et al., *“ADMM ⊇ Projective Dynamics: Fast Simulation of Hyperelastic Models with Dynamic Constraints”*, 2017. ([Semantic Scholar][6])
* Macklin et al., XPBD and follow-up work on position-based and unified particle physics. ([arXiv][7])
* Harada, *“A Parallel Constraint Solver for a Rigid Body Simulation”*, SIGGRAPH Asia 2011, and related GPU rigid-body pipeline notes. ([ACM Digital Library][8])
* Rikuya Tomii, Tetsu Narumi, *“Hierarchical Parallelization of Rigid Body Simulation with Soft Blocking Method on GPU”*, Computation, 2025. ([MDPI][9])
* Nak-Jun Sung, Min Hong, *“Toward Real-Time Scalable Rigid-Body Simulation Using GPU-Optimized Collision Detection and Response”*, Mathematics, 2025. ([MDPI][10])
* Lei Lan et al., *“JGS2: Near Second-order Converging Jacobi/Gauss–Seidel for GPU Elastodynamics”*, TOG/SIGGRAPH 2025. ([arXiv][11])

[1]: https://box2d.org/files/ErinCatto_IterativeDynamics_GDC2005.pdf?utm_source=chatgpt.com "Iterative Dynamics with Temporal Coherence"
[2]: https://umit.cs.umu.se/modsimcomplmech/docs/warm_start.pdf?utm_source=chatgpt.com "Warm starting the projected Gauss-Seidel algorithm for ..."
[3]: https://arxiv.org/abs/1910.09873?utm_source=chatgpt.com "Sparsity Exploitation of Accelerated Modulus-Based Gauss-Seidel Method for Interactive Rigid Body Simulations"
[4]: https://scispace.com/pdf/per-contact-iteration-method-for-solving-contact-dynamics-2xznkg3mx1.pdf?utm_source=chatgpt.com "Per-Contact Iteration Method for Solving Contact Dynamics"
[5]: https://arxiv.org/abs/2405.17020?utm_source=chatgpt.com "[2405.17020] From Compliant to Rigid Contact Simulation"
[6]: https://www.semanticscholar.org/paper/ADMM-%E2%8A%87-Projective-Dynamics%3A-Fast-Simulation-of-with-Overby-Brown/102a8ef3562918a629cc22190c0f21600be2f258?utm_source=chatgpt.com "[PDF] ADMM ⊇ Projective Dynamics: Fast Simulation of ..."
[7]: https://arxiv.org/pdf/2302.14344?utm_source=chatgpt.com "Modular and Parallelizable Multibody Physics Simulation ..."
[8]: https://dl.acm.org/doi/10.1145/2077378.2077406?utm_source=chatgpt.com "A parallel constraint solver for a rigid body simulation"
[9]: https://www.mdpi.com/2079-3197/13/11/250?utm_source=chatgpt.com "Hierarchical Parallelization of Rigid Body Simulation with ..."
[10]: https://www.mdpi.com/2227-7390/13/19/3230?utm_source=chatgpt.com "Toward Real-Time Scalable Rigid-Body Simulation Using ..."
[11]: https://arxiv.org/abs/2506.06494?utm_source=chatgpt.com "JGS2: Near Second-order Converging Jacobi/Gauss ..."
