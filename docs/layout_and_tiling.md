# Layout & Tiling Cheatsheet

> How data flows from AoS worlds to SoA tiles. Keep it tight; every cache miss hurts.

---

## Goals

- Separate **authoring-friendly AoS** from **solver-friendly SoA/AoSoA**.
- Fuse **row build + tiling** to avoid giant global buffers.
- Keep tiles **self-contained** so SIMD kernels and schedulers stay trivial.
- Make it easy to gather **ADMC + residual stats** per tile/island.

---

## Data Modes

| Mode | Used by | Notes |
|------|---------|-------|
| AoS  | API, baseline | `std::vector<RigidBody>`, manifolds, joints. Simple, cache-ugly. |
| SoA  | World views | Contiguous arrays for positions, velocities, inertia, impulses. Built per frame or stored natively. |
| AoSoA (tiles) | Hot solver path | Fixed-capacity blocks (32–128 rows) with local body tables + SoA row arrays. |

Rule: AoS for clarity, tiles for speed. No other hidden layouts.

---

## Pipeline (per frame)

1. **AoS world update** – integrate forces, refresh manifolds/joints.
2. **Island detection** – connected components over bodies/constraints.
3. **Tile build (fused)**  
   - Walk manifolds/joints once.  
   - Build row data + tile assignment in the same pass.  
   - Fill local body tables via `get_or_add_local(bodyId)`.  
   - Copy cached impulses for warm-start.
4. **Solve** – SoA kernels iterate tiles per island (serial or scheduled).
5. **Scatter** – apply impulses back to AoS world; update caches.

Everything outside step 3 should treat tiles as opaque work units.

---

## Tile Anatomy

- **Header**: capacity, active row count, type spans (normals, tangents, joints).
- **Local bodies**: dense array of `(bodyId, invMass, inertia, velocity refs)`.
- **Row SoA**:
  - directions, rA/rB, effective mass, bias, limits, lambda.
- **Bookkeeping**:
  - Warm-start lambda slots.
  - Optional lightweight metrics (max residual, ADMC delta).

Guarantees:

- Rows referencing the same bodies live in the same tile whenever practical.
- Tiles never exceed SIMD-friendly multiples (e.g., 64 rows, 8-lane chunks).
- No pointer chasing inside tiles; only contiguous arrays + local indices.

---

## Caching Rules

- Split **geometry vs state**:
  - Geometry data (contact frames, offsets, denominators) cached per manifold until pose drift exceeds thresholds.
  - State data (penetration, relative velocity, bias) recomputed every step.
- Archetypes:
  - Sphere/sphere + sphere/plane get bespoke builders (no inertia math).
  - Everything else falls back to the generic builder.
- Cache hits should bypass expensive math entirely; misses are instrumented.

---

## Parallel & Metrics Hooks

- Tiles expose:
  - `body_mask` (bitset or small table) so schedulers can detect conflicts.
  - Optional `residual_estimate` + `admc_delta`; used by iteration policies.
- Islands own vectors of tiles → schedulers run them color-by-color.
- Diagnostics can sum ADMC contributions per tile without touching solver code.

---

## Implementation Checklist

- [ ] AoS structs stay simple (no SoA leaks).
- [ ] `WorldSoA` / `ConstraintBatch` built via explicit builder APIs.
- [ ] Tile builder never allocates per row (use slab allocators).
- [ ] SIMD width is compile-time constant inside kernels.
- [ ] Tile creation and scatter paths are fully deterministic (needed for tests).

If any of these boxes fail, fix layout before touching solver math.
