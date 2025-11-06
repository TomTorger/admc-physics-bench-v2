# ADMC Theory – 02: Relativistic ADMC

> This document gives a more formal derivation of ADMC from relativistic 4-momentum, and explains why the “two scalars per direction” structure is essentially unique under natural assumptions.

---

## 1. Relativistic Setup

We work in flat Minkowski spacetime with metric signature \((- + + +)\) or \((+ - - -)\) — the precise sign convention is not critical for our purposes.

For a particle of rest mass \(m_0\):

- 4-velocity: \(U^\mu = \gamma(c, \mathbf v)\), where \(\gamma = 1/\sqrt{1 - v^2/c^2}\).
- 4-momentum: \(P^\mu = m_0 U^\mu = (\frac{E}{c}, \mathbf p)\).

We define:

- \(M = E/c\), which is just the time component of \(P^\mu\) in convenient units.
- \(\mathbf p = (p_x, p_y, p_z)\), the spatial components.

The invariant mass relation is:

\[
P^\mu P_\mu = -m_0^2 c^2,
\]

which in components (in a common sign convention) reads:

\[
-M^2 + \|\mathbf p\|^2 = -m_0^2 c^2.
\]

This implies:

\[
M^2 = m_0^2 c^2 + \|\mathbf p\|^2.
\]

---

## 2. Desired Properties for Directional Scalars

We want to define **directional scalar quantities** that capture the content of \(M\) and directional momentum \(p_k\) (for some axis \(k\)) in a way that is:

1. **Additive** over particles:
   - If we have particles labeled by \(i\), then the system’s scalar is the sum over \(i\).
   - This matches the way total energy and momentum are additive.

2. **Conserved** for isolated systems:
   - If the total 4-momentum is conserved, the new scalars should also be conserved.

3. **Isotropic**:
   - Different spatial directions are treated symmetrically, modulo our choice of axis \(k\).

4. **Parity-respecting**:
   - Flipping the sign of \(p_k\) (reversing direction along the axis) should induce well-defined behavior in the scalars.

5. **Locally invertible**:
   - From the two scalar invariants per direction, we should be able to reconstruct \(M\) and \(p_k\) (up to trivial degeneracies).

These conditions are enough to strongly constrain the possible functional form of the scalars.

---

## 3. Form of Directional Scalars

Fix a spatial direction \(k\) (e.g., the x-axis). We look for scalar functions of the form:

\[
f_{k^+}(M, p_k), \qquad f_{k^-}(M, p_k),
\]

such that:

- The map \((M, p_k) \mapsto (f_{k^+}, f_{k^-})\) is locally invertible.
- The scalars are linear (or at least affine) in \(M\) and \(p_k\), to preserve additivity.

Given additivity and conservation arguments, **linearity** is the natural choice:

\[
f_{k^+}(M, p_k) = \alpha_+ M + \beta_+ p_k, \quad
f_{k^-}(M, p_k) = \alpha_- M + \beta_- p_k.
\]

If we allow only affine transformations (constant offsets), those can be absorbed into a redefinition of zero and do not affect conservation. So we focus on linear forms.

---

## 4. Basis Transformation View

It is helpful to think of \((M, p_k)\) as a 2-component column vector:

\[
\mathbf u_k = 
\begin{pmatrix}
M \\
p_k
\end{pmatrix}.
\]

Then our pair of scalars \((f_{k^+}, f_{k^-})\) is just a linear transformation:

\[
\begin{pmatrix}
f_{k^+} \\
f_{k^-}
\end{pmatrix}
=
A_k
\begin{pmatrix}
M \\
p_k
\end{pmatrix}
\]

for some invertible \(2 \times 2\) matrix \(A_k\). Invertibility ensures we can reconstruct \(M\) and \(p_k\) from the new scalars.

Isotropy and parity give constraints on the form of \(A_k\):

- Parity (flipping \(p_k \to -p_k\)) should exchange the role of “plus” and “minus” scalars in a simple way.
- This suggests the matrix should be symmetric in some sense under \(p_k \to -p_k\).

A particularly simple and symmetric choice is:

\[
A_k =
\begin{pmatrix}
1 & \frac{1}{2} \\
1 & -\frac{1}{2}
\end{pmatrix},
\]

leading to the definitions:

\[
p_{k^+} = M + \frac{1}{2} p_k, \qquad
p_{k^-} = M - \frac{1}{2} p_k.
\]

This matrix is clearly invertible, and its inverse is:

\[
A_k^{-1} =
\begin{pmatrix}
\frac{1}{2} & \frac{1}{2} \\
1 & -1
\end{pmatrix},
\]

so that:

\[
M = \frac{1}{2} (p_{k^+} + p_{k^-}), \qquad
p_k = p_{k^+} - p_{k^-}.
\]

Thus our ADMC scalars form a legitimate basis transformation.

---

## 5. Conservation in ADMC Form

Suppose we have an isolated system (no external work or impulses) with total 4-momentum conserved:

\[
\sum_i M_i^\text{(after)} = \sum_i M_i^\text{(before)}, \qquad
\sum_i \mathbf{p}_i^\text{(after)} = \sum_i \mathbf{p}_i^\text{(before)}.
\]

Focusing on a single direction \(k\), this means:

\[
\sum_i M_i^\text{(after)} = \sum_i M_i^\text{(before)}, \qquad
\sum_i p_{k,i}^\text{(after)} = \sum_i p_{k,i}^\text{(before)}.
\]

Now compute total ADMC scalars:

\[
\sum_i p_{k^+,i} 
= \sum_i \left( M_i + \frac{1}{2} p_{k,i} \right)
= \sum_i M_i + \frac{1}{2} \sum_i p_{k,i},
\]

and similarly for \(p_{k^-}\). Since both \(\sum_i M_i\) and \(\sum_i p_{k,i}\) are conserved, it follows that both

\[
\sum_i p_{k^+,i} \quad\text{and}\quad \sum_i p_{k^-,i}
\]

are conserved.

Conversely, if for each direction \(k\) we assume that both total \(p_{k^+}\) and \(p_{k^-}\) are conserved, the inverse relations imply:

\[
\sum_i M_i^\text{(after)} = \sum_i \frac{1}{2} (p_{k^+,i}^\text{(after)} + p_{k^-,i}^\text{(after)})
= \sum_i \frac{1}{2} (p_{k^+,i}^\text{(before)} + p_{k^-,i}^\text{(before)})
= \sum_i M_i^\text{(before)},
\]

and similarly for \(p_k\). Doing this for three linearly independent directions \(k\) recovers full 4-momentum conservation.

Therefore:

> **Conserving the ADMC scalars for each direction is equivalent to conserving the usual energy and momentum.**

---

## 6. Multi-Particle and Subsystem Structure

Because the ADMC scalars are defined linearly in \(M\) and \(p_k\), and because those are additive over particles:

- Each scalar \(p_{k^\pm}\) is **additive over particles**.
- For any subsystem (subset of particles), the subsystem’s total \(p_{k^\pm}\) is just the sum over that subset.

This mirrors the way we can talk about momentum and energy for subsystems. ADMC therefore behaves naturally under:

- Partitioning a system into parts.
- Combining subsystems.
- Tracking invariants for a specific region or group of bodies.

---

## 7. Change of Direction and Isotropy

So far we fixed a direction \(k\). What happens if we pick a different direction \(\hat{\mathbf{d}}\) instead?

- We can define \(p_d = \mathbf p \cdot \hat{\mathbf d}\).
- Then we introduce
  \[
  p_{d^+} = M + \frac12 p_d, \qquad p_{d^-} = M - \frac12 p_d.
  \]

This is just the same construction in a rotated basis. Under spatial rotations:

- \(\mathbf p\) transforms as a vector.
- \(M\) stays invariant.
- The ADMC scalars associated with one direction map into linear combinations associated with rotated directions.

The key point is: ADMC respects **isotropy**—no direction is singled out by the definition itself. The choice of direction comes from the problem (e.g., gravitational axis, contact normals), not from the theory.

---

## 8. Relation to Engine Usage

In a simulation engine we typically:

- Work in the **Newtonian limit**.
- Track velocities and positions, not 4-vectors.
- Implement constraint solvers that operate on linear and angular velocities.

So why keep the relativistic derivation around?

1. It gives a **clean theoretical grounding**: ADMC is not an ad-hoc trick, but a result of taking a simple, natural change of basis in relativistic mechanics and applying some structural requirements (linearity, isotropy, invertibility).
2. It guarantees that, as long as we’re consistent with the underlying momentum conservation principles, the ADMC scalars reflect the same physical content.
3. It makes it straightforward to design **Newtonian approximations** that retain the same structural qualities.

The Newtonian limit and rigid-body mapping are handled in detail in the companion document:

- **“ADMC Theory – 03: Bridge to Newtonian Rigid-Body Mechanics.”**

---

## 9. Summary

- We started from relativistic 4-momentum \(P^\mu = (M, \mathbf p)\).
- Under mild and natural assumptions, linear directional scalars that are additive and invertible must be related to \((M, p_k)\) by an invertible linear transformation.
- The symmetric choice
  \[
  p_{k^+} = M + \frac12 p_k, \quad p_{k^-} = M - \frac12 p_k
  \]
  gives nicely behaved scalars that are conserved whenever 4-momentum is conserved.
- Conservation of all ADMC scalars for three independent directions is equivalent to conservation of 4-momentum.
- ADMC thus provides a **directional scalar encoding** of energy and momentum that is structurally well-motivated and convenient for engine metrics and analysis.

For practical engine work we typically drop back to the Newtonian limit and rigid bodies, which is the subject of the next theory document.
