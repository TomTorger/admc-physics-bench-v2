# Layout and Tiling

> This document describes the **data layout** and **tiling strategy** for the next-gen ADMC solver stack. It explains how bodies and constraints are stored in memory, how AoS and SoA coexist, how islands and tiles are formed, and how this feeds the solver backends.

---

## 1. Goals & Scope

The layout layer’s job is to bridge between:

- **High-level world representation**  
  (bodies, contacts, joints in intuitive AoS structures), and
- **Solver-friendly data**  
  (SoA / AoSoA arrays, tiles, islands, SIMD-ready buffers).

Key goals:

- Make the **baseline AoS solver** easy to keep and reason about.
- Give high-performance solvers a **cache-friendly, SIMD-aware** layout.
- Make **tiling and island decomposition** explicit and reusable across solvers and backends.
- Keep the layout layer **modular**: constraints/solvers shouldn’t care about whether they see AoS or SoA, as long as they have the right views.

---

## 2. Layout Modes Overview

We support three main layout “modes”:

1. **AoS (Array-of-Structures)**  
   - Used by:
     - Baseline solver.
     - High-level world management / user-facing API.
   - Structure:
     - `std::vector<RigidBody>`.
     - `std::vector<ContactManifold>`.
     - `std::vector<Joint>`.
   - Pros:
     - Simple and intuitive; great for clarity and debugging.
   - Cons:
     - Poor cache behavior for large scenes.
     - Hard to exploit SIMD.

2. **SoA (Structure-of-Arrays)**  
   - Used inside high-performance solvers and tiles.
   - Bodies and rows are split into arrays per field:
     - `pos_x[]`, `pos_y[]`, `pos_z[]`, …
     - `vel_x[]`, `vel_y[]`, `vel_z[]`, …
   - Pros:
     - Excellent for SIMD and cache-friendly iteration.
   - Cons:
     - Less intuitive; more boilerplate.

3. **AoSoA (Array-of-Structures-of-Arrays)**  
   - Realized as **tiles**:
     - A tile is a small SoA block of fixed capacity (e.g. 64 rows).
   - Pros:
     - Keep SoA benefits while keeping **locality** and **bounded size**.
     - Natural unit for tasks and GPU workgroups.

High-level pattern:

- **API & baseline**: AoS.
- **Performance-sensitive solver**: islands + tiles (AoSoA built from AoS / SoA sources).

---

## 3. World Representation

### 3.1 Bodies

At the API / high-level world layer, bodies are stored in AoS:

```cpp
struct RigidBody {
    Vec3  position;
    Quat  orientation;

    Vec3  linear_velocity;
    Vec3  angular_velocity;

    float mass;
    float inv_mass;
    Mat3  inertia_body;
    Mat3  inertia_inv_body;

    // Cached world-space inertia (optional)
    Mat3  inertia_world;
    Mat3  inertia_inv_world;

    MaterialId material;
    // flags, user data, etc.
};
````

Bodies are identified via a handle:

```cpp
struct BodyId {
    uint32_t index;
    uint32_t generation;
};
```

The world keeps:

```cpp
struct WorldState {
    std::vector<RigidBody> bodies;
    // contacts, joints, broadphase data, etc.
};
```

### 3.2 SoA Views for Bodies

For SoA/tile solvers we don’t duplicate entire bodies; instead, we create **views** with SoA-type accessors:

```cpp
struct SoABodyData {
    float* pos_x;
    float* pos_y;
    float* pos_z;

    float* vel_x;
    float* vel_y;
    float* vel_z;

    float* inv_mass;
    // orientation, angular velocity, inertia, etc.
};
```

These pointers refer either to:

* A SoA buffer built once per frame, or
* Existing world arrays (if we store world bodies in SoA internally).

The `WorldView` abstraction will hide whether it’s AoS or SoA under the hood.

---

## 4. Constraint & Row Representation

### 4.1 Contacts and Joints (AoS)

Contacts and joints are created and managed as AoS objects:

```cpp
struct ContactPoint {
    Vec3  position;
    float penetration;
};

struct ContactManifold {
    BodyId bodyA;
    BodyId bodyB;

    Vec3 normal;
    ContactPoint points[MaxPoints];
    int  point_count;

    float friction;
    float restitution;

    // Warm-start impulses
    float normal_impulse[MaxPoints];
    Vec2  tangent_impulse[MaxPoints];
};

struct Joint {
    BodyId bodyA;
    BodyId bodyB;
    // joint-local data, anchor points, limits, etc.
    // warm-start impulses per scalar DOF
};
```

These are **high-level**: easy to construct, debug, and persist between frames.

### 4.2 Constraint Rows (Conceptual)

From contacts and joints we derive **constraint rows**:

```cpp
struct ConstraintRow {
    BodyId bodyA;
    BodyId bodyB;

    Vec3  jacobian_linear_A;
    Vec3  jacobian_angular_A;
    Vec3  jacobian_linear_B;
    Vec3  jacobian_angular_B;

    float effective_mass;
    float bias;
    float lambda;        // accumulated impulse

    float lower_limit;   // e.g. 0 for normal
    float upper_limit;   // e.g. +∞ or friction cone bound

    // meta: type (normal, friction, joint), contact index, etc.
};
```

These exist as an **abstract concept**. For AoS/scalar solvers we can store them directly as AoS. For tile solvers, we store them in SoA form per tile.

---

## 5. Islands

An **island** is a connected component of the body–constraint graph:

* Nodes: bodies.
* Edges: contacts and joints.

Algorithm (high-level):

1. Build adjacency:

   * For each contact/joint between bodies A and B, add edges.
2. Run a union-find or DFS/BFS to find connected components.
3. For each component, create an `Island` object:

```cpp
struct Island {
    std::vector<BodyId>         bodies;
    std::vector<ContactManifold*> contacts;
    std::vector<Joint*>         joints;
    // later: tiles, local maps, etc.
};
```

Reasons for islands:

* Independent islands can be processed **in parallel**.
* Smaller matrices / graphs → better cache usage.
* Matches physical separations (e.g., different groups of objects).

---

## 6. Tiles (AoSoA Blocks)

### 6.1 What is a Tile?

A **tile** is a small SoA block of constraints belonging to an island, with a **local body table**.

Example:

```cpp
constexpr int TILE_CAPACITY = 64;

struct TileBodies {
    // Map local tile indices -> global bodies
    BodyId global_body_ids[MaxTileBodies];
    int    body_count;

    // Map global body indices -> local tile index
    // (dense or hash-based, depending on size)
};

struct TileRows {
    // Example for normal rows:
    float n_x[TILE_CAPACITY];
    float n_y[TILE_CAPACITY];
    float n_z[TILE_CAPACITY];

    float rA_x[TILE_CAPACITY], rA_y[TILE_CAPACITY], rA_z[TILE_CAPACITY];
    float rB_x[TILE_CAPACITY], rB_y[TILE_CAPACITY], rB_z[TILE_CAPACITY];

    float eff_mass[TILE_CAPACITY];
    float bias[TILE_CAPACITY];
    float lambda[TILE_CAPACITY];

    uint16_t bodyA_idx[TILE_CAPACITY];
    uint16_t bodyB_idx[TILE_CAPACITY];

    // similar arrays for friction/joint rows
};

struct Tile {
    TileBodies bodies;
    TileRows   normals;
    TileRows   friction;   // or a separate struct specialized for tangents
    // optional: joint rows
    int        normal_count;
    int        friction_count;
};
```

**Key properties:**

* All arrays have the same **capacity** (e.g. 64 or 128).
* A **tile is SoA** internally.
* Tiles are kept small to:

  * Fit well in caches.
  * Be processed by a single core or GPU workgroup.

### 6.2 Mapping Global Bodies to Tile Bodies

We maintain a local mapping:

* `global_body_ids[local_index]` tells which global body a local index refers to.
* A small table or hash from `BodyId` → local index is used when building rows:

```cpp
uint16_t get_or_add_local_body(TileBodies& tb, BodyId id);
```

This allows tile solver kernels to operate on **compact local indices**.

---

## 7. AoS → Islands → Tiles Pipeline

This section outlines the per-frame pipeline for high-performance solvers.

### 7.1 Baseline Solver Path (AoS Only)

For the baseline AoS solver we skip SoA and tiles:

1. Broadphase & contact generation produce `ContactManifold`s in AoS.
2. Joints are stored as AoS.
3. Solver iterates directly over `WorldState.bodies`, `contacts`, and `joints`.

No islands or tiles needed (though we may still build islands for metrics or debugging if desired).

### 7.2 SoA/Tiled Solver Path

For the SoA/tile solver, per frame we do:

1. **Broadphase & contact generation**

   * Produce `ContactManifold`s (AoS).
2. **Island building**

   * Build islands from bodies + contacts + joints.
3. **Tile building (fused with row build)**
   For each island:

   * Create empty `Tile`s.
   * Iterate contacts and joints:

     * For each contact manifold:

       * Compute world-space normal and tangents.
       * For each contact point:

         * Compute offsets `rA`, `rB`.
         * Compute or update effective mass, bias, etc.
         * Choose a tile for this contact:

           * E.g., round-robin or based on body grouping.
         * Add a row to the tile:

           * Fill SoA arrays.
           * Assign local body indices via `get_or_add_local_body`.
       * Copy warm-start impulses from manifold into tile arrays.
     * For each joint:

       * Similarly build joint rows and place them in tiles.
4. **Solver**

   * The solver iterates islands and tiles, invoking tile-level PGS kernels.
5. **Scatter back to AoS world**

   * After solving, the world view:

     * Holds updated velocities directly (if SoA world).
     * Or velocities are copied back to AoS structures.

This pipeline ensures the **heavy assembly** is done once per frame and produces **tile-ready data**.

---

## 8. Fusing Row Build & Tiling

A critical optimization is to **avoid building a massive global row array** and re-partitioning it into tiles. Instead:

* As we process each contact manifold or joint:

  * We immediately decide which tile it belongs to.
  * We build the rows directly into that tile’s SoA arrays.

Benefits:

* Less data movement (no global row arrays to partition).
* Better temporal locality:

  * We touch contact/joint AoS data once.
  * We write SoA data once into the tile.
* Easier to parallelize row build:

  * Different islands (or tiles in different islands) can be built on multiple threads.

We still keep conceptual `ConstraintRow` structs in the docs and tests, but the actual hot path uses fused build + tiling.

---

## 9. Alignment & SIMD Considerations

### 9.1 Alignment

For SIMD-friendly loads:

* Align key arrays in tiles to at least 16/32 bytes:

  * Implementation-level detail, e.g. via custom allocators or alignment attributes.
* Tile capacity should be a **multiple of SIMD width** (e.g. 64 rows with width 8).

### 9.2 Handling Partial Tiles

Not all tiles are full:

* For a tile with `N` rows and SIMD width `W`:

  * Full SIMD chunks: `N / W`.
  * Remainder: `N % W`.
* We can:

  * Mask off lanes for the remainder, or
  * Use scalar fallback for the tail.

The layout keeps rows for a tile contiguous so we can iterate in steps of `W` easily.

### 9.3 Vector-friendly Access Patterns

Within a tile, we design loops such that:

* All data needed for a constraint is accessed via **strided loads** across SoA arrays.
* We can keep body data in registers or in separate SoA arrays for a group of bodies.

Example (conceptual):

```cpp
for (int i = 0; i < normal_count; i += W) {
    // Load normals n_x[i..i+W-1], etc.
    // Load local indices for bodies A and B.
    // Gather velocities for bodies A and B.
    // Compute v_rel, bias, delta_j in SIMD.
    // Scatter impulses back to velocities in registers or SoA body arrays.
}
```

Exact implementation details depend on the compiler and SIMD intrinsics but the tile layout supports this style.

---

## 10. Warm-Starting & Per-Frame Caches

Warm-start data is naturally stored on **persistent AoS objects**:

* In `ContactManifold`:

  * `normal_impulse[MaxPoints]`.
  * `tangent_impulse[MaxPoints]`.
* In `Joint`:

  * Impulses per scalar DOF.

Per frame:

1. **Build tiles from manifolds/joints**:

   * Copy cached impulses into tile `lambda` arrays.
2. **Solver**:

   * Uses these values as initial impulses.
   * Updates tile `lambda` arrays each iteration.
3. **Write back**:

   * After solving:

     * Tile builder or world view writes updated impulses back to the owning manifold/joint.

This keeps caches **stable across layout changes**:

* Solvers can change (AoS → SoA, PGS → block) without losing warm-start context.
* Manifolds and joints remain the canonical owners of warm-start state.

---

## 11. Parallelism & Tiles

Tiles and islands are the core units for parallelism:

* **Islands**:

  * Independent: no body belongs to two islands.
  * Ideal coarse-grain tasks for threads.

* **Tiles** within islands:

  * For large islands, we can treat tiles as individual tasks.
  * Need to ensure tiles that share bodies are scheduled carefully (e.g. by coloring or in GS fashion).

Basic patterns:

1. **Island-level parallelism**

   * Each island is a task.
   * Within an island, tiles processed sequentially (simpler, still effective).

2. **Tile-level parallelism with coloring**

   * Inside each island, color the constraint graph or tile graph so that:

     * Tiles in the same color do not share bodies.
   * Process each color in parallel, then move to the next color.

3. **Jacobi-style updates (experimental)**

   * Each tile computes impulses on a read-only snapshot of velocities.
   * A second pass applies impulses (reduces conflicts but more memory traffic).

The layout supports these patterns by:

* Keeping each tile’s data **self-contained**.
* Providing a **local body table** so we can reason about dependencies at tile granularity.

---

## 12. Debug vs Release Layout Choices

To keep life sane:

* **Debug builds**:

  * May use AoS views more heavily.
  * Keep extra metadata in tiles (e.g., indices back to contact manifolds).
  * Enable bounds checks and assertions on indices.

* **Release builds**:

  * Strip most debug metadata from tiles.
  * Use more aggressive alignment and vectorization.
  * Possibly use separate build flags for GPU/CPU-specific layouts.

We keep the same conceptual API (`WorldView`, `ConstraintView`, etc.), but underlying structures may differ slightly in debug vs release.

---

## 13. Extensibility

The layout/tiling design should accommodate:

* **New solver families**:

  * Block solvers may want extra block-level metadata per tile.
  * Krylov/ADMM may need additional arrays for dual variables or history.
* **New hardware targets**:

  * GPU backends:

    * Tiles map directly to workgroups.
    * SoA arrays can be copied or shared in GPU buffers.
* **New constraint types**:

  * Add new SoA fields per tile (or separate tile sections) without changing existing solver logic, as long as we plug them into appropriate kernels.

Guidelines:

* Keep tile structs **additive**:

  * New features add fields.
  * Avoid breaking existing layouts unless strictly necessary.
* Add new views / adapters as needed:

  * E.g., `GpuTileView` for GPU kernels built from the same core tile data.

---

## 14. Summary

* **AoS** is used for:

  * The main world representation and the baseline solver.
  * Persistent data (bodies, manifolds, joints, caches).

* **SoA / AoSoA (tiles)** are used for:

  * High-performance solvers and backends.
  * SIMD-friendly, cache-aware kernels.

* **Islands and tiles** provide:

  * Logical and physical decomposition of the world.
  * Natural task boundaries for threading and GPU.

* The **pipeline**:

  * AoS contacts/joints → islands → tiles (fused row build) → solver → metrics and write-back.

This layout and tiling design underpins the solver architecture, making it possible to iterate on solver algorithms and backends without constantly reworking data structures.
