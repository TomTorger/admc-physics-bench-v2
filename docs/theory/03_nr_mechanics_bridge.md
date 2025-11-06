# Theory 03 – Bridging ADMC ↔ Newtonian Rigid Bodies

> Use this when you need to explain why a “relativistic” vocabulary shows up in a classical engine.

---

## Non-Relativistic Limit

- Start from \(P^\mu = (M, \mathbf p)\).
- For \(v \ll c\):
  - \(M = m c + \frac{1}{2} m \frac{v^2}{c} + \cdots\)
  - \(\mathbf p = m \mathbf v + \mathcal O(v^3/c^2)\)
- The constant \(mc\) term cancels out when we take differences, so ADMC scalars effectively encode:
  - Linear momentum \(m\mathbf v\)
  - Kinetic energy \(\tfrac{1}{2}m v^2\)

Thus ADMC ≈ Newtonian energy + directional momentum, just packaged differently.

---

## From Rigid-Body State to ADMC Channels

Given a rigid body with \((\mathbf v, \boldsymbol\omega, m, \mathbf I)\):

1. Compute linear momentum \(\mathbf p = m \mathbf v\).
2. Pick directions \(k\) of interest (world axes, contact normals, joint axes).
3. Build scalars:
   \[
   p_{k^\pm} = M_k + \tfrac{1}{2} p_k, \qquad
   M_k = \frac{T_\text{linear} + T_\text{angular}}{c_k}
   \]
   (Use a consistent scaling constant \(c_k\); same choice everywhere.)
4. Store \(\sum p_{k^\pm}\) per island/world before and after solving.

Angular motion matters because it feeds linear velocity at contact points; ADMC does not need a separate angular channel.

---

## Contacts/Joints in ADMC Terms

- Each contact row already has a direction (normal or tangent). Use that direction when attributing drift.
- For joints, use the joint axis or constraint Jacobian row direction.
- When a row applies an impulse \(\lambda\) along direction \(\hat{\mathbf d}\):
  - Update velocities via standard impulse math.
  - Optionally compute the implied change in \(p_{d^\pm}\) for diagnostics.

This keeps ADMC accounting local to the row/tile.

---

## Practical Cheat Sheet

- **Before solve**: accumulate \(p_{k^\pm}\) per world/island from AoS velocities.
- **During solve**: tiles touch only SoA data; if needed, feed approximate drift to trackers using tile metadata.
- **After solve**: recompute \(p_{k^\pm}\); tracker reports drift and suggests iteration/warm-start tweaks.

No relativistic numerics are required—the constants cancel. ADMC is a bookkeeping convention layered over familiar rigid-body math.
