# ADMC Solver — Literature Signals

> Short notes on the papers that shape the architecture. Use this as a map, not a survey.

---

## PGS / DEM Roots

- **Catto (2005, 2014)** – Sequential impulses, constraint framing, warm-starting. Validates row-centric thinking and motivates cached impulses + clear ERP knobs.
- **Wang & Servin (2016)** – Quantifies warm-start gains (2–5× fewer iterations) in DEM. Justifies persistent manifold caches and ADMC-informed warm-start scaling.
- **Modulus-Based GS (MBGS)** – Shows block-aware splittings cut residuals faster. Supports tile/block solvers and Delassus-operator abstractions.
- **Per-contact iteration (robotics)** – Treat each manifold as a coupled block. Inspires the block-solver hooks that sit next to scalar PGS.

## ADMM / Convex Variants

- **Unified ADMM for rigid+compliant contact (2023–2024)** – Proves that proximal splits can outperform classic PGS on hard stacks. Keeps the door open for ADMC-as-constraint penalties.
- **Projective dynamics as ADMM** – Frames PD / XPBD as another point on the splitting spectrum. Motivates keeping solver interfaces operator-centric (apply J, Jᵀ, M⁻¹).
- **ADMM in MPC (Aydinoglu & Posa)** – Shows real-time contact control with ADMM; hints that our solver stack can double as a differentiable layer.

## XPBD / Compliance

- **XPBD fundamentals** – Compliance via per-row multipliers; stable stiff constraints. Encourages storing row metadata (compliance, lambda) even for velocity-level solvers.
- **Diff/Multigrid XPBD** – Demonstrates that tile hierarchies make sense for multigrid and differentiation—matching our tile-first layout.

## Parallel & GPU Lessons

- **Harada / early GPU SI** – Batch constraints into body-disjoint groups before running parallel PGS. Justifies island/color scheduling and tile-based conflict checks.
- **Hierarchical soft blocking** – Keep multiple granularities of work (island → tile) to feed wide GPUs. Reinforces our scheduler abstractions.
- **GPU collision + response** – Reminds us that contact generation can become the bottleneck; reason to keep assembly separated and SoA-friendly.

## High-Order GS / Acceleration

- **Near-second-order Jacobi/GS** – Introduces overshoot-aware corrections to regain convergence while staying parallel. Motives ADMC-driven iteration policies.
- **Anderson / Krylov accelerations** – Treat PGS as a preconditioner and occasionally accelerate with Anderson or Krylov steps; matches the “experimental solver” slot.

---

## How to Use This Doc

1. Pick the solver idea you want to explore.
2. Skim the bullets above to recall the key paper + lesson.
3. Port that lesson into the architecture via the relevant module (assembly, solver, parallel, diagnostics).

Less reading, more building.
