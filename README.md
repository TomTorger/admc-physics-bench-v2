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
2. Build the bench app (`simple_bench`) or the unit suites.
3. Run the lightweight bench to explore iteration thresholds or export CSV snapshots:
   ```bash
   ./build/simple_bench --iterations=12 --admc=0.0002 --csv results/latest.csv
   ```
   Add `--solver=baseline` to compare against the AoS baseline derived from the original repo.
   Use `--scene=two_spheres` to target a specific scene; omit to process the default library.
4. Run a scene with both the baseline and SoA solvers; compare ADMC drift + timings.

## Built-in scenes

`simple_bench` and the tests share the small scene library in `admc/scene/scene_library.hpp`:

- `two_spheres` – opposing impulses, single contact pair.
- `box_stack_N` – small vertical stacks resting on a static ground.
- `sphere_grid_4x4`, `sphere_grid_8x8` – ground-supported sphere arrays with lateral contacts.
- `sphere_cloud_16x16`, `sphere_cloud_32x32`, `sphere_cloud_64x64` – larger grids approximating the legacy “sphere cloud” scenes.

Each scene can be filtered via `--scene=<name>` and every solver run can be exported to CSV via `--csv path/to/results.csv`.

## Python bench orchestrator

For more complex sweeps (multiple scenes/solvers, repeated runs, CSV aggregation) use the Python helper:

```bash
python -m bench.cli --config bench/templates/default.yaml
```

Edit the YAML to point at your `simple_bench` binary (Debug/Release), add scenes (`two_spheres`, `box_stack_3`, `sphere_grid_4x4`, …), and customize solver arguments. The script runs every combination, aggregates medians, and writes a summary CSV plus a concise console table.

Each CSV row now contains timing breakdowns (`total_ms`, `warm_ms`, `iteration_ms`), the iteration count actually used (adaptive gate), and the final residual for that solver/scene. In the default config both solvers run with a high iteration cap (64) and rely on the adaptive gate / residual thresholds to exit early, mimicking the legacy benchmark workflow.

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
