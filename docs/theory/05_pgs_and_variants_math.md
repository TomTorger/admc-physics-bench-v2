# Theory 05 – PGS & Friends (Quick Math)

> Sequential impulses, block solves, and accelerators share one loop. These are the knobs.

---

## Plain PGS (per row)

```
for iter in 0..N:
  for row in rows (GS order):
    v_rel = J · v + b
    delta = -m_eff * v_rel
    lambda_new = clamp(lambda + delta, [min, max])
    applyImpulse(row, lambda_new - lambda)
    lambda = lambda_new
```

- GS order: normals → friction pairs → joints (per manifold).
- Deterministic ordering matters for reproducibility.

---

## Warm-Start + ADMC Feedback

- Cached `lambda` from previous frame seeds the iteration.
- ADMC tracker scales warm-start per island: `lambda *= warmstart_scale`.
- Tracker may request early exit once residual + ADMC drift drop below thresholds.

---

## Block Variants

### Contact Manifold Block

- Rows: one normal + two tangents (3×3).
- Solve small LCP analytically or via buffered GS with 2–3 micro-iterations.
- Use as:
  - Replacement for per-row loop on that manifold, or
  - Preconditioner (block solve first, then global GS sweep).

### Joint Block

- Gather rows belonging to the same joint.
- Build dense matrix \(A = J M^{-1} J^T\); solve using LDLᵀ or cached inverse.
- Enforce limits by clamping Lagrange multipliers after the solve.

---

## Over-Relaxation

- Successive over-relaxation (SOR): `lambda += ω * delta`.
- ω ∈ (0, 2); >1 accelerates, but risks overshoot. Trackers can back off ω when ADMC drift explodes.

---

## Krylov / Anderson Acceleration

Treat PGS as fixed-point iteration \(x_{k+1} = F(x_k)\).

- Store history of `(x_i, F(x_i))`.
- Solve least squares to find better combination → new iterate.
- Use sparingly (every M iterations) to avoid cost blow-up.
- Works best when residual norms stagnate but constraints are already nearly satisfied.

---

## XPBD / Compliance Note

- XPBD updates Lagrange multipliers at the position level:
  \[
  \Delta\lambda = -\frac{C(q) + \alpha \lambda}{\Delta t^2 / m_\text{eff} + \alpha}
  \]
  where \(\alpha = 1/\text{compliance}\).
- Fits into the same row structure; only the update formula differs.

---

## Practical Tips

- Keep `m_eff`, `lambda`, limits, bias in SoA arrays for tiles.
- Always emit residual estimates so the tracker can steer iterations.
- If experimenting with new methods, wire them through the same `ConstraintBatch`; layout stays identical.
