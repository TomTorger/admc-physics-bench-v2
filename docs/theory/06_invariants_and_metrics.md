# Theory 06 – Invariants & Metrics Cheat Sheet

> What we track, how we compute it, and when to panic.

---

## ADMC Channels

- Choose directions (world axes, gravity, contact normals).
- Before solve:
  \[
  P_{k^\pm}^\text{pre} = \sum_i \left(M_i \pm \tfrac{1}{2} p_{k,i}\right)
  \]
- After solve:
  \[
  \Delta P_{k^\pm} = P_{k^\pm}^\text{post} - P_{k^\pm}^\text{pre}
  \]
- Report max and RMS drift across channels.
- Tracker thresholds:
  - `drift_ok` – below this, allow early exit.
  - `drift_bad` – above this, reduce warm-start scale or request extra iterations.

---

## Constraint Metrics

| Metric | Formula | Notes |
|--------|---------|-------|
| Penetration | max / RMS of positional error per contact | Derived from manifold depth after solver. |
| Joint error | max / RMS constraint violation | Use joint-space positional error. |
| Cone violation | \(\max(0, \|\lambda_t\| - \mu \lambda_n)\) | Evaluate per friction pair. |
| Residual | \(|J v + b|\) | Stored per row/tile; feeds convergence heuristics. |

---

## Energy / Momentum (classic view)

- Total linear momentum: \(\mathbf P = \sum_i m_i \mathbf v_i\).
- Kinetic energy: \(T = \sum_i \tfrac{1}{2} m_i \|\mathbf v_i\|^2 + \tfrac{1}{2} \boldsymbol\omega_i^T \mathbf I_i \boldsymbol\omega_i\).
- Optional to log, but ADMC already captures the essence direction-by-direction.

---

## Reporting Guidelines

- Bench display vs CSV output:
  - Display: solvers print aggregate timings; ADMC drift is used for iteration gating but is not currently printed.
  - CSV (current): `scene`, `contacts`, `solver`, `total_ms`, `warm_ms`, `iteration_ms`, `assembly_ms`, `iterations`, `residual`.
  - CSV (planned): add `max_penetration`, `max_joint_error`, `max_admc_drift` and optional per-tile breakdowns.

---

## Alarm Thresholds (defaults — tune per project)

- Penetration > 1e-3 m → fail test.
- Joint error > 1e-3 rad/m → fail test.
- ADMC drift > 5e-3 (normalized) → clamp warm-start next frame and raise warning.
- Runtime regression > 5% vs previous commit → alert.

---

## Implementation Hooks

- Metrics module pulls data from:
  - `ConstraintBatch` (for row residuals, lambda).
  - `WorldSoA` (for velocities).
  - `IConservationTracker` (for ADMC drift & policies).
- Keep metrics read-only: never mutate solver state from the metrics path.

High-signal metrics make debugging easy and keep solver experiments honest.
