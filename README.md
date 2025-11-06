# admc-physics-bench-v2
A benchmark of physics engines based on the math of "Additive Directional Momentum Conservation" laws.
# ADMC Next-Gen Physics Solver

This repo is a **next-generation physics solver playground** for the ADMC (Additive Directional Momentum Conservation) framework.

It is built to:

- Keep a **clean AoS baseline solver** as a reference.
- Provide a **modular, data-oriented architecture** for experimenting with solvers and layouts.
- Expose detailed **metrics and benchmarks** (including ADMC-style invariants) so we can quantify performance and physical quality.

Think of it as a **research lab** for contact/joint solvers and data layouts, not a full game engine.

---

## High-Level Overview

### What’s in here?

- A **baseline AoS PGS solver** (legacy-style sequential impulses).
- A **SoA/tile-based solver architecture** designed for:
  - SIMD-friendly kernels,
  - Island/tile parallelism,
  - Future GPU backends.
- A growing set of **solver families**:
  - Scalar PGS, tile-based PGS, block PGS, and experimental variants.
- **ADMC theory & metrics**:
  - Directional momentum invariants.
  - Drift, penetration, joint error, cone consistency.
- A CLI **benchmark app** for running scenes across solvers/backends.

### What this project is *not*

- A general-purpose game engine.
- A full-featured scene editor or asset pipeline.
- A “drop-in” replacement for existing engines (yet) — it’s primarily a **bench + research repo**.

---

## Directory Layout (Short Version)

```text
admc-next/
├─ docs/
│  ├─ architecture_overview.md   # Overall design
│  ├─ solver_design.md           # Solver interfaces & families
│  ├─ layout_and_tiling.md       # SoA/AoSoA, islands, tiles
│  ├─ benchmarks.md              # Benchmark methodology
│  └─ theory/                    # ADMC & constraint math
│     ├─ 01_admc_overview.md
│     ├─ 02_relativistic_admc.md
│     ├─ 03_nr_mechanics_bridge.md
│     ├─ 04_constraint_rows_math.md
│     ├─ 05_pgs_and_variants_math.md
│     └─ 06_invariants_and_metrics.md
├─ include/admc/                 # Public API headers
│  ├─ core/                      # Math & utilities
│  ├─ admc/                      # Invariants & metrics helpers
│  ├─ world/                     # Bodies, materials, world state
│  ├─ constraints/               # Contacts, joints, rows, graphs
│  ├─ layout/                    # AoS/SoA/tile/island views
│  ├─ solver/                    # Solver interfaces & families
│  ├─ backend/                   # Single-/multi-thread executors
│  ├─ metrics/                   # Timing & physics metrics
│  └─ scenes/                    # Scene generators
├─ src/                          # Implementations
├─ apps/
│  ├─ bench/                     # CLI benchmark tool
│  └─ viewer/                    # (Optional) simple visual viewer
├─ tests/                        # Unit & regression tests
└─ legacy/
   └─ baseline_aos_solver.*      # Original AoS baseline solver
