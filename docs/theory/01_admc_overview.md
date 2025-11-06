# ADMC Theory – 01: Overview

> This document gives a conceptual overview of ADMC (Additive Directional Momentum Conservation) and how we use it in the engine. It is intentionally “engine-developer friendly” and light on heavy relativity; detailed derivations live in the other theory documents.

---

## 1. Why ADMC?

In classical mechanics and game physics we usually talk about:

- **Linear momentum**: \(\mathbf{p} = m\mathbf{v}\)
- **Angular momentum**: \(\mathbf{L} = \mathbf{I}\boldsymbol\omega\)
- **Energy**: kinetic + potential

At a system level we know that, in the absence of external forces and torques, **total momentum and energy are conserved**. In practice, game-style solvers:

- Approximate constraints iteratively.
- Introduce bias terms (for stabilization).
- Use discrete timesteps and finite precision.

So we never get *perfect* conservation. Instead we ask, “How badly did we violate the conservation laws?” and use that as a quality signal.

**Problem**: the natural invariants—total energy and total momentum—are **vector/scalar combinations** and are **not always convenient to reason about locally**. For example:

- You may be interested in how well momentum is preserved along specific **directions**, e.g., stack normal directions or contact normals.
- You may want metrics that are easily tracked per-region or per-feature, not just global scalar/vector totals.
- In a relativistic formulation, different coordinate choices can obscure what is “really” being conserved.

ADMC gives us a way to re-express energy and directional momentum as **simple additive scalars** that are easier to bookkeep and to use as metrics.

---

## 2. Basic Idea of ADMC

ADMC rests on a simple but powerful trick:

> Instead of storing “energy + directional momentum” as \(M\) and \(p_k\), store them as **two scalars per direction**, \(p_{k^+}\) and \(p_{k^-}\), which are individually additive and conserved.

Here:

- \(M\) is a rescaled energy (e.g., \(M = E/c\) in relativity, or a compatible quantity in a chosen unit system).
- \(p_k\) is the component of linear momentum along some direction \(k\) (e.g., the \(x\)-axis or a contact normal).

We define:

\[
p_{k^+} = M + \frac{1}{2} p_k, \qquad
p_{k^-} = M - \frac{1}{2} p_k.
\]

Key properties:

- Each of \(p_{k^+}\) and \(p_{k^-}\) is **additive** over particles.
- If the system is closed (no external work or impulse along direction \(k\)), then both \(p_{k^+}\) and \(p_{k^-}\) are **conserved**.
- From \(p_{k^+}\) and \(p_{k^-}\) you can reconstruct \(M\) and \(p_k\) via:
  \[
  M = \frac{1}{2}(p_{k^+} + p_{k^-}), \qquad
  p_k = p_{k^+} - p_{k^-}.
  \]

So, at the level of information content, ADMC is just a **change of basis** in the space spanned by energy and directional momentum. The physics is unchanged; we just rewrote the same conservation law in a scalar-friendly form.

---

## 3. Why “Directional”?

Momentum is a vector. But many phenomena in rigid bodies are strongly **directional**:

- A stack of boxes has a distinguished **stacking normal** (gravity direction).
- A contact manifold has a **contact normal** and **tangent directions**.
- Joint chains often have specific axes where stability is most important.

Because ADMC is defined **per direction**, e.g., along an axis \(k\) or a contact normal \(\hat{\mathbf{n}}\), we can specialize our conservation and error metrics:

- “How much directional momentum drifted along +Y?”
- “How much did the solver violate directional momentum conservation along each contact normal?”
- “Are we injecting energy along certain directions because of bias or numerical issues?”

By tracking conserved **directional scalars** we gain finer control and better diagnostics than with just a global \(\mathbf{P}\) and \(E\).

---

## 4. ADMC vs Standard Conservation Laws

Let’s compare the two pictures.

### 4.1 Standard Picture

For a closed system (no external forces), we usually say:

- Total 4-momentum \(P^\mu = (M, \mathbf{p})\) is conserved.
- In Newtonian terms, total linear momentum and (in many cases) energy are conserved.

In components:

- \(M_\text{total}^\text{(after)} = M_\text{total}^\text{(before)}\)
- \(p_{x,\text{total}}^\text{(after)} = p_{x,\text{total}}^\text{(before)}\)
- \(p_{y,\text{total}}^\text{(after)} = p_{y,\text{total}}^\text{(before)}\)
- \(p_{z,\text{total}}^\text{(after)} = p_{z,\text{total}}^\text{(before)}\)

### 4.2 ADMC Picture

For each direction \(k \in \{x,y,z\}\) we have:

\[
p_{k^+} = M + \frac{1}{2} p_k, \qquad
p_{k^-} = M - \frac{1}{2} p_k.
\]

We require that **both** sums

\[
\sum_i p_{k^+}^{(i)} \quad\text{and}\quad \sum_i p_{k^-}^{(i)}
\]

are conserved across the interaction for each direction \(k\).

Given the linear relations, this is *equivalent* to conservation of \(M\) and \(p_k\) for each direction. So:

- ADMC introduces no new physical assumptions.
- It is a **coordinate choice** for the conserved quantities, optimized for additivity and directional bookkeeping.

---

## 5. ADMC in the Engine Context

Our engine is **Newtonian**, not relativistic. So why bring in ADMC at all?

Because:

1. The derivation (in the relativistic view) gives a **clean theoretical justification** that these directional scalars are deeply tied to fundamental conservation laws.
2. We can take a **non-relativistic limit** and recover the familiar rigid-body quantities (handled in another document).
3. Even in the Newtonian context, the ADMC form is useful for:
   - Designing **metrics** (directional momentum drift).
   - Designing and validating **solvers** (e.g., checking how much each solver disturbs directional invariants).
   - Structuring **debug visualizations** and regression tests.

### 5.1 What We Actually Use

In practice we do not run a relativistic simulation. Instead:

- We use the ADMC story as a **conceptual backbone**.
- We define engine-level **directional metrics** that mirror the \(p_{k^\pm}\) invariants.
- We sum these metrics over particles/bodies and compare before vs after each solver step or frame.

For example, for a set of bodies and a chosen direction \(\hat{\mathbf{d}}\), we might define:

- Directional momentum:
  \[
  P_d = \sum_i m_i \mathbf{v}_i \cdot \hat{\mathbf{d}}.
  \]
- Directional energy proxy:
  \[
  E_d = \sum_i \frac{1}{2} m_i (\mathbf{v}_i \cdot \hat{\mathbf{d}})^2
  \quad\text{(or related directional kinetic term)}.
  \]

We can then define ADMC-inspired scalar combinations of these and check their drift across a solver step.

---

## 6. ADMC as a Metric Framework

A major use of ADMC in this project is as a **metric framework**:

1. Pick a set of **directions** to monitor:
   - Global axes ±X, ±Y, ±Z.
   - A few additional directions (e.g., diagonals, typical contact normals).
2. For each direction \(k\), compute directional scalars (adapted to the Newtonian setting).
3. Track these scalars over time and across solver configurations.
4. Report **drift** per direction:
   - Max absolute drift.
   - Relative drift (% of initial).
   - RMS drift over multiple directions.

Compared to a single global “total energy drift” number, this gives a much **richer view**:

- Some solvers may introduce directional bias (e.g., more drift along stack normals).
- Some scenes are sensitive to particular directions (columns, ropes, articulated chains).

ADMC-style directional invariants make these issues easier to **quantify** and **compare**.

---

## 7. Relationship to Constraint Solvers

Our constraint solvers (PGS, block solvers, etc.) operate by applying impulses that:

- Enforce contact/joint constraints approximately.
- Exchange momentum between bodies.
- Often inject or remove a bit of energy via bias or damping terms.

Each impulse, in the ADMC view, is a **local redistribution** of directional momentum scalars. If the solver were exact and bias-free, total directional scalars would match perfectly before and after.

In reality:

- We can’t expect perfect conservation because:
  - Constraints are inconsistent or over-constrained.
  - We use damping, stabilization, and discrete timesteps.
- But we can expect **drift to stay small and controlled**, and to behave sensibly across solver variants and parameter changes.

Therefore, ADMC provides:

- A **reference ideal** (exact directional invariants).
- A set of **practical, engine-friendly metrics** telling us how close we are to that ideal.

---

## 8. Summary & Pointers

- ADMC is a **directional scalar representation** of standard momentum/energy conservation.
- It introduces **no new physics**; it is a structured way of encoding invariants that’s useful for both theory and implementation.
- In the engine, we use ADMC primarily as:
  - A conceptual foundation for invariants.
  - A framework for **directional conservation metrics**.
- The detailed derivations and uniqueness results live in the **relativistic document**.
- The mapping to standard rigid-body Newtonian mechanics is covered in the **NR bridge document**.
- Constraint rows and solvers are described in companion theory docs and wired into actual code in the architecture documents.

If you want the full derivation and the uniqueness theorems, continue with:

- **“ADMC Theory – 02: Relativistic ADMC”**  
- followed by **“ADMC Theory – 03: Bridge to Newtonian Rigid-Body Mechanics.”**
