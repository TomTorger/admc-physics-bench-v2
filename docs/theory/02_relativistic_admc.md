# Theory 02 – Relativistic Roots (Cliff Notes)

> ADMC drops straight out of 4-momentum algebra. Here’s the short derivation so you can defend it on a whiteboard.

---

## Setup

- Work in flat spacetime, \(P^\mu = (M, \mathbf p)\) with \(M = E/c\).
- Conservation of 4-momentum ⇒ conservation of \(M\) and each \(p_k\).
- We want two additive scalars per direction \(k\) that:
  1. Are linear in \((M, p_k)\).
  2. Survive parity flips (switching direction should swap roles).
  3. Form an invertible change of basis.

---

## Only Solution That Works

Let \(\mathbf u_k = (M, p_k)^T\). Any additive scalar pair is

\[
\mathbf f_k = A_k \mathbf u_k, \quad
A_k =
\begin{pmatrix}
\alpha_+ & \beta_+ \\
\alpha_- & \beta_-
\end{pmatrix}.
\]

Parity demands that exchanging \(p_k \to -p_k\) simply swaps the scalars, so:

\[
\alpha_+ = \alpha_- = 1, \qquad
\beta_+ = -\beta_- = \tfrac{1}{2}.
\]

Hence:

\[
p_{k^+} = M + \tfrac{1}{2} p_k,\qquad
p_{k^-} = M - \tfrac{1}{2} p_k.
\]

Up to constant scaling, this transformation is unique.

---

## Interpretation

- Geometrically, we just rotated the \((M, p_k)\) plane by \(45^\circ\) and scaled the axes.
- Because the transformation is linear and invertible, additivity and conservation are inherited automatically.
- Direction choice is arbitrary: pick world axes, gravity-aligned axes, or per-contact normals. The math stays the same.

---

## Takeaway

ADMC is not “extra physics”. It is the only linear, parity-respecting, additive way to package energy + directional momentum into scalar channels. That’s why we trust it as an invariant and as a solver steering signal.
