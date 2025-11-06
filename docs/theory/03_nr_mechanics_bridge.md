# ADMC Theory – 03: Bridge to Newtonian Rigid-Body Mechanics

> This document explains how the relativistic ADMC framework connects to standard Newtonian rigid-body mechanics, and how we interpret and use ADMC-style invariants in a non-relativistic engine.

---

## 1. Non-Relativistic Limit of 4-Momentum

We begin from relativistic 4-momentum:

\[
P^\mu = (M, \mathbf p), \quad M = \frac{E}{c}, \quad \mathbf p = \gamma m_0 \mathbf v.
\]

In the non-relativistic regime, speeds satisfy \(v \ll c\). The Lorentz factor \(\gamma\) can be expanded as:

\[
\gamma \approx 1 + \frac{1}{2}\frac{v^2}{c^2} + \mathcal{O}\left(\frac{v^4}{c^4}\right).
\]

Then:

- The energy:
  \[
  E = \gamma m_0 c^2 \approx m_0 c^2 + \frac{1}{2} m_0 v^2 + \mathcal{O}\left(\frac{v^4}{c^2}\right).
  \]
- The rescaled energy:
  \[
  M = \frac{E}{c} \approx m_0 c + \frac{1}{2} m_0 \frac{v^2}{c} + \cdots.
  \]

The spatial momentum remains:

\[
\mathbf p = m_0 \gamma \mathbf v \approx m_0 \mathbf v + \mathcal{O}\left(\frac{v^3}{c^2}\right).
\]

In this limit:

- The rest term \(m_0 c\) in \(M\) is dominant but **constant** for a given particle.
- The interesting dynamics come from:
  - The **Newtonian momentum** \(m_0 \mathbf v\).
  - The **kinetic energy** \(\frac{1}{2} m_0 v^2\).

The ADMC scalars, built from \(M\) and directional \(p_k\), reduce to expressions dominated by the Newtonian momentum and kinetic energy, up to additive constants and \(1/c\) factors.

---

## 2. Newtonian Rigid-Body State

In the engine we simulate rigid bodies with the following state:

- **Configuration**
  - Position: \(\mathbf x \in \mathbb R^3\).
  - Orientation: unit quaternion \(q\) (or rotation matrix \(\mathbf R\)).
- **Velocities**
  - Linear velocity: \(\mathbf v\).
  - Angular velocity: \(\boldsymbol\omega\).
- **Mass & inertia**
  - Mass: \(m\).
  - Body-space inertia tensor: \(\mathbf I_\text{body}\).
  - World-space inertia tensor: \(\mathbf I = \mathbf R\,\mathbf I_\text{body}\,\mathbf R^T\).
  - Inverse inertia: \(\mathbf I^{-1}\).

From these we define canonical Newtonian quantities:

- **Linear momentum**
  \[
  \mathbf p = m \mathbf v.
  \]
- **Angular momentum**
  \[
  \mathbf L = \mathbf I \boldsymbol\omega.
  \]
- **Kinetic energy**
  \[
  T = \frac{1}{2} m \|\mathbf v\|^2 + \frac{1}{2} \boldsymbol\omega^T \mathbf I \boldsymbol\omega.
  \]

Potential energies depend on gravity and other external fields and are not directly part of ADMC, which is framed in kinematic terms.

---

## 3. Contacts and Relative Motion

At a contact between body A and body B:

- Let \(\mathbf x_A, \mathbf x_B\) be body centers.
- Let \(\mathbf r_A, \mathbf r_B\) be offsets from centers to the contact point.
- Let the contact frame be defined by:
  - Normal \(\hat{\mathbf n}\) (pointing from A to B).
  - Two orthogonal tangents \(\hat{\mathbf t}_1, \hat{\mathbf t}_2\).

Linear velocities at the contact point are:

\[
\mathbf v_A^\text{(pt)} = \mathbf v_A + \boldsymbol\omega_A \times \mathbf r_A,
\]
\[
\mathbf v_B^\text{(pt)} = \mathbf v_B + \boldsymbol\omega_B \times \mathbf r_B.
\]

The **relative velocity** at the contact point is:

\[
\mathbf v_\text{rel} = \mathbf v_B^\text{(pt)} - \mathbf v_A^\text{(pt)}.
\]

We often work with its projections:

- Normal component: \(v_n = \mathbf v_\text{rel} \cdot \hat{\mathbf n}\).
- Tangent components:
  \[
  v_{t_1} = \mathbf v_\text{rel} \cdot \hat{\mathbf t}_1, \quad
  v_{t_2} = \mathbf v_\text{rel} \cdot \hat{\mathbf t}_2.
  \]

These are the building blocks for contact and friction constraint rows.

---

## 4. Newtonian Analogues of ADMC Scalars

In the relativistic formulation, ADMC uses \(M\) and \(p_k\) to define directional invariants:

\[
p_{k^+} = M + \frac12 p_k, \quad p_{k^-} = M - \frac12 p_k.
\]

In the Newtonian limit and rigid-body context:

- We are typically interested in **directional linear momentum**, e.g.,
  \[
  P_d = \sum_i m_i \mathbf v_i \cdot \hat{\mathbf d},
  \]
  for some direction \(\hat{\mathbf d}\).
- And **directional kinetic energy contributions**, e.g.,
  \[
  T_d = \sum_i \frac12 m_i (\mathbf v_i \cdot \hat{\mathbf d})^2,
  \]
  possibly plus rotational contributions projected along \(\hat{\mathbf d}\).

We can choose to define Newtonian ADMC-style scalars at the system level:

\[
S_{d^+} = T_d + \alpha P_d, \quad S_{d^-} = T_d - \alpha P_d
\]

for some scaling \(\alpha\). This mirrors the structure of \(M \pm \frac12 p_k\) in the relativistic theory, though note:

- In the strict NR limit, rest energy \(m_0 c^2\) is huge and constant, so we usually **drop it** for engine purposes.
- We track **changes** in energy and momentum, not absolute values including rest mass.

Rather than insist on a specific numeric mapping, we focus on preserving the **structural features**:

- Two scalars per direction.
- Additivity over bodies.
- Conservation (up to solver errors and external work).

---

## 5. Impulses and Momentum Update

In rigid-body simulation, constraint solvers apply **impulses** \(\mathbf J\) at contact points or joint anchors.

For a contact impulse \(\mathbf J\) applied at offset \(\mathbf r\) on body A:

- Linear momentum update:
  \[
  \mathbf p'_A = \mathbf p_A + \mathbf J.
  \]
- Angular momentum update:
  \[
  \mathbf L'_A = \mathbf L_A + \mathbf r \times \mathbf J.
  \]

For body B, the impulse is \(-\mathbf J\) (equal and opposite), producing opposite updates.

In an ideal, closed system:

- The **sum of linear momenta** over all bodies is unchanged by internal impulses, i.e., internal contact impulses only rearrange momentum.
- Similarly, the total angular momentum about an inertial origin is preserved, modulo numerical error.

In an ADMC-style view, each impulse step:

- Preserves the **sum of directional scalars** in the absence of bias and external forces.
- Redistributes directional contribution between bodies and directions tied to the contact/joint.

When we use bias terms (for stabilization) or gravity, we deliberately violate some invariants to enforce constraints or model external work. ADMC metrics then tell us **how much** we deviated.

---

## 6. Where ADMC Fits in the Newtonian Engine

In practice, we do not carry around relativistic scalars; we:

1. Simulate rigid bodies with Newtonian dynamics.
2. Use standard impulse-based constraint solvers (PGS or variants).
3. Post-process solver output to compute **metrics** inspired by ADMC.

For example, per frame we might:

- Choose a set of directions:
  - The global axes ±X, ±Y, ±Z.
  - A few additional directions (e.g., diagonals, typical contact normals).
- For each direction \(\hat{\mathbf d}\), compute:
  - Directional momentum before and after the solver step.
  - Directional kinetic energy contributions before and after.
- Build ADMC-like scalar combinations and report:
  - Directional momentum drift.
  - Directional energy drift.

This lets us compare **different solver architectures** (baseline AoS, SoA PGS, block solvers, etc.) on a common footing, and see:

- Which conserve directional invariants best.
- How drift behaves as we vary timestep, stiffness, solver iterations, and so on.

---

## 7. Angular Momentum and Rotational Effects

So far we focused on linear momentum. Real rigid bodies also carry angular momentum \(\mathbf L\) and rotational kinetic energy.

While the original ADMC derivation is framed in terms of linear momentum, in the Newtonian rigid-body context we can extend our metrics by:

- Choosing an origin (e.g., world origin or center of mass).
- Computing total angular momentum \(\mathbf L_\text{total}\) about that origin.
- Building **directional angular momentum** components:
  \[
  L_d = \mathbf L_\text{total} \cdot \hat{\mathbf d}.
  \]

We can then define similar drift metrics for angular contributions, or treat them together with linear contributions when evaluating solver quality. In many practical physics benchmarks:

- Linear momentum drift is the primary concern.
- Angular drift is still useful to monitor, especially for joints and tall stacks.

---

## 8. Summary

- Relativistic 4-momentum reduces, in the NR limit, to the familiar Newtonian momentum and kinetic energy plus a large constant rest term.
- ADMC’s directional scalars become, in spirit, combinations of **directional momentum** and **directional kinetic energy**.
- In a rigid-body engine, we:
  - Track velocities and momenta.
  - Apply impulses from constraints and external forces.
  - Use ADMC as a **conceptual guide** and as a basis for **directional metrics**, rather than as a literal runtime data representation.
- Impulses in an ideal closed system only redistribute momentum; real solvers with bias and damping deliberately break invariants in controlled ways. ADMC-style metrics measure that breakage.

With the bridge to Newtonian mechanics established, we can now turn to the **constraint math** and the **solver algorithms** that operate on these quantities in the engine:

- **“ADMC Theory – 04: Constraint Rows and Contact/Joints Math.”**
- **“ADMC Theory – 05: PGS and Solver Variants (Math Perspective).”**
