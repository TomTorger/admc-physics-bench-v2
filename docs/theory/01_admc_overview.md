# Theory 01 – ADMC in 5 Minutes

> ADMC (Additive Directional Momentum Conservation) is just a change of basis that makes conservation bookkeeping trivial.

---

## Why Bother?

- Game solvers violate energy/momentum slightly every step.
- We care about *directional* drift (stack normals, joint axes, contact normals) more than global vectors.
- ADMC expresses energy + directional momentum as two additive scalars per direction → easy to track, easy to test.

---

## Core Definition

For any direction \(k\):

\[
p_{k^+} = M + \tfrac{1}{2} p_k,\qquad
p_{k^-} = M - \tfrac{1}{2} p_k
\]

Where:

- \(M\) – energy-like term (relativistic mass or a compatible NR quantity).
- \(p_k\) – component of linear momentum along \(k\).

Properties:

- Both \(p_{k^+}\) and \(p_{k^-}\) are additive across bodies.
- Conserving both is equivalent to conserving \(M\) and \(p_k\).
- Recover original values via:
  \[
  M = \tfrac{1}{2}(p_{k^+} + p_{k^-}),\quad
  p_k = p_{k^+} - p_{k^-}
  \]

---

## How We Use It

- Choose axes of interest (world X/Y/Z, gravity direction, contact normals, joint axes).
- For each body:
  - Compute \(p_{k^\pm}\) from velocity + mass properties.
  - Store snapshots before and after the solver step.
- Report drift:
  \[
  \Delta p_{k^\pm} = \sum_i p_{k^\pm}^{(i)}\_\text{after} - \sum_i p_{k^\pm}^{(i)}\_\text{before}
  \]
- Hook the drift into `IConservationTracker` to:
  - Scale warm-start (large drift → smaller reuse).
  - Request more iterations on bad islands.
  - Trigger optional post-solve corrections.

---

## Mental Model

- ADMC does **not** invent new physics; it just reframes conservation in scalars that are easy to sum and compare.
- Because everything is additive, you can track ADMC per tile, per island, per world, or per “region” without extra math.
- Solver developers see fast feedback: “+Y drift exploded after iteration 3” is far clearer than “total momentum changed by ε”.

That’s it. Treat ADMC as the lingua franca for conservation diagnostics and solver control.
