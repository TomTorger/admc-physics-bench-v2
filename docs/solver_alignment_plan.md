# ADMC Tile Solver Alignment Plan

> Long-term roadmap for turning the current prototype into a competitive ADMC-first solver.

## 1. Architecture Pillars

1. **Persistent tile pipeline**
   - Cache `ConstraintBatch` objects per island; only rebuild tiles when topology or archetype state changes.
   - Maintain per-tile warm-start data (`lambda`, ADMC deltas, residual estimates).
   - Expose `TileHandle` API for schedulers (coloring, parallelism) and diagnostics.

2. **SIMD-grade kernels**
   - Convert tile rows to fixed-width SoA/AoSoA chunks (e.g., 8 normals, 8 tangents).
   - Implement SIMD-friendly `apply_jacobian`, `apply_jacobian_transpose`, and inverse-mass updates.
   - Separate normal/tangent/joint spans to enable specialized kernels.

3. **Conservation-aware iteration control**
   - Track ADMC per island + per tile; record drift vectors and channel deltas each iteration.
   - Allow trackers to focus specific tiles or islands, not just whole worlds.
   - Emit diagnostics (residual decay, ADMC drift, warm-start scaling) per tile for bench tooling.

4. **Geometry/state caches**
   - Split archetype builders into geometry (cached until pose drift) and state passes.
   - Share cached effective masses, rA/rB, contact frames between baseline and tile solvers.
   - Instrument cache hit/miss ratios to identify cost centers.

5. **Solver matrix**
   - Implement `pgs_tile`, `pgs_block`, and at least one accelerator (Anderson/Krylov) on top of the shared batch.
   - Keep AoS baseline and scalar PGS for reference/regression.
   - Ensure bench harness can flip between solvers with identical configs.

6. **Testing + instrumentation**
   - Add golden tests for tile builder determinism, warm-start reuse, ADMC tracker hooks, and SIMD kernels (unit + perf microbenchmarks).
   - Extend `bench_smoke_check` to validate assembly time, iteration counts, and ADMC drift thresholds.
   - Wire per-scene/per-solver metrics into CI to detect regressions early.

## 2. Execution Stages

### Stage A – Tile Data & Builder Foundations
1. Define `ConstraintBatch`/`Tile` data layout (local body tables, SoA spans).
2. Build a fused island→tile builder with slab allocators, archetype hooks, and cached geometry.
3. Persist batches between frames; rebuild incrementally upon topology/pose changes.

### Stage B – Tile Solver Core
1. Implement `pgs_tile` operating directly on `ConstraintBatch` without extra copies.
2. Integrate warm-start cache per tile; eliminate unordered_map hot paths.
3. Embed ADMC/residual sampling per tile; feed signals to `IConservationTracker`.
4. Optimize iteration loop (SIMD kernels, batched dot products, vectorized inverse mass).

### Stage C – Advanced Policies & Variants
1. Extend `IConservationTracker` to report per-tile focus hints and warm-start scaling.
2. Implement `block::manifold` solver using same batch layout (dense 3×3/4×4 solves).
3. Add Anderson/Krylov acceleration hooks layered on `pgs_tile`.

### Stage D – Diagnostics, Bench, CI
1. Update bench tooling to record assembly, iteration, scatter, and drift times separately.
2. Add perf regression tests (microbench + representative scenes) to CI.
3. Document tuning knobs for thresholds, tile sizes, and trackers.

## 3. Immediate Next Steps (Stage A)

1. Redesign `ConstraintBatchBuilder`:
   - Use a pool (slab allocator) to reuse tile storage per frame.
   - Maintain local body tables referencing AoS body indices and storing inverse mass/inertia snapshots.
   - Partition rows into fixed-capacity tiles (e.g., 64) with type spans.
   - Provide delta rebuild API (update tiles in place when manifolds change).

2. Refactor warm-start caching:
   - Move `lambda` storage into tiles; persist across frames via batch cache.
   - Record residual/ADMC metrics per tile instead of global unordered_map.

3. Update solver interface:
   - `TilePGSSolver::solve(WorldSoA&, ConstraintBatch&, const SolverSettings&, IConservationTracker&)`.
   - Return detailed `SolverStats` (assembly, warm, iterations, scatter, ADMC drift).

Once Stage A foundations are in place, we can iterate on core performance work (Stage B) with confidence.
