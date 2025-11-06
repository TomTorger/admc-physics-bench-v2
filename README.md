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

---

## Building (Linux/macOS)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Requirements: CMake ≥ 3.20, a C++20 compiler (Clang/GCC), and a recent libc++/libstdc++.

## Building (Windows / MSVC)

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build --output-on-failure -C Release
```

Requirements: Visual Studio 2022 with C++ workload and CMake ≥ 3.20 (comes with VS).
