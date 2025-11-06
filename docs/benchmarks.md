# Benchmark Playbook

> Measure speed, stability, and ADMC drift with as little ceremony as possible.

---

## What We Measure

- **Timings**: total step, assembly, warm-start, iterations, scatter.
- **Throughput**: contacts/s, bodies/s.
- **Quality**: max/RMS penetration, joint error, friction cone violation.
- **ADMC**: max/RMS drift per channel; tracker decides if iterations keep going.
- **Diagnostics** (optional): residual decay per iteration, per-tile stats.

Everything is reported per solver/back-end/scene triple.

---

## Scene Suite

| Category | Example | Purpose |
|----------|---------|---------|
| Tiny sanity | `two_spheres`, `box_stack_4` | Validate correctness; catch regressions instantly. |
| Medium clouds/stacks | `spheres_cloud_1024`, `box_stack_medium` | Show AoS vs SoA speed gaps without massive runtimes. |
| Large stress | `spheres_cloud_10k+`, `random_pile` | Saturate memory + parallel paths; measure scaling. |
| Joints / chains | `pendulum`, `bridge_chain`, `ragdoll_cluster` | Track joint drift and ADMC response to articulated loads. |
| Edge cases | `bouncy_contacts`, `high_friction_pile`, `zero_g` | Stress restitution, friction, and conservation. |

Scenes live under `include/admc/scenes/` with deterministic seeds.

---

## Solvers Under Test

- `baseline_aos` – legacy reference, single-thread.
- `pgs_scalar_soa` – flat SoA rows, scalar kernels.
- `pgs_tile_soa_st` – SoA tiles, serial backend.
- `pgs_tile_soa_mt` – SoA tiles, multi-thread scheduler.
- `block_pgs_tile` – manifold/joint blocks layered onto tiles.
- `exp_*` – slots for Anderson/Krylov/ADMM/XPBD experiments.

Each solver reports stats via the shared `SolverStats` struct; benchmarks never special-case implementations.

---

## Harness Basics

```
bench --scene spheres_cloud_1024 --steps 256 \
      --solvers baseline_aos,pgs_tile_soa_mt \
      --out results.json
```

Harness responsibilities:

1. Instantiate the scene.
2. Run warm-up steps (optional).
3. Execute N measured steps per solver config.
4. Collect metrics + ADMC drift.
5. Emit table + machine-readable (JSON/CSV) output.

All results include git SHA + build flags for reproducibility.

---

## Policies

- Compare solvers at the **same timestep, iterations, and tracker settings** unless explicitly experimenting.
- Flag a run if:
  - ADMC drift > configured threshold,
  - Penetration/joint error exceed limits,
  - Runtime regresses beyond tolerance vs baseline history.
- Staple the bench script into CI to stop stability or performance regressions before landing.
