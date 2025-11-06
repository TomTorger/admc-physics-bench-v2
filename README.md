# ADMC Next-Gen Physics Solver

Minimal lab for experimenting with Additive Directional Momentum Conservation (ADMC) solvers.

---

## Intent

- Keep a **clean AoS baseline** for truth checks.
- Push a **SoA/tile architecture** for cache efficiency and GPU-ready kernels.
- Treat **ADMC invariants** as the primary quality signal.
- Ship **benchmarks + metrics** that make solver comparisons trivial.

Not a full engine—this is a solver playground.

---

## Contents

- `docs/` – architecture map, solver notes, tiling guide, benchmarks, theory primers.
- `include/admc/` – public headers: core math, ADMC helpers, world/constraints/layout/solver/backends/metrics/scenes.
- `src/` – implementations mirroring the headers.
- `apps/bench` – CLI bench driver.
- `apps/viewer` – optional lightweight viewer.
- `tests/` – unit + regression suites.
- `legacy/` – frozen AoS baseline solver.

---

## Quick Start

1. Read `docs/architecture_overview.md` to understand the stack.
2. Build the bench app.
3. Run a scene with both the baseline and SoA solvers; compare ADMC drift + timings.

Everything else is optional experimentation.
