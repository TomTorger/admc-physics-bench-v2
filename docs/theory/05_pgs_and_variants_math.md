# ADMC Theory – 05: PGS and Solver Variants (Math Perspective)

> This document describes the mathematical side of the solver algorithms we use: PGS (Projected Gauss–Seidel) / sequential impulses, warm-starting, over-relaxation, block variants, and higher-level ideas like Krylov acceleration and ADMM.

---

## 1. Problem Formulation

From the constraint row derivation, we end up with a system of scalar constraints:

\[
\dot{C}_j(\mathbf v) = J_j \mathbf v + b_j = 0,
\]

for rows \(j = 1, \dots, m\). Collecting them:

\[
J \mathbf v + \mathbf b = \mathbf 0,
\]

with \(\mathbf v\) the stacked generalized velocities and \(J\) the full Jacobian.

We seek impulses \(\boldsymbol\lambda\) such that:

\[
\mathbf v' = \mathbf v + M^{-1} J^T \boldsymbol\lambda,
\]

and

\[
J \mathbf v' + \mathbf b = 0
\]

subject to **inequality constraints**, e.g.:

- \(j_n \ge 0\) for normal impulses.
- Friction cone \(\|\mathbf j_t\| \le \mu j_n\).
- Joint limits, motors, etc.

Formally this becomes a **variational inequality** or **linear complementarity problem (LCP)**. Solving the full LCP exactly is expensive, so real-time engines rely on **iterative methods**, especially PGS / sequential impulses.

---

## 2. PGS / Sequential Impulses

### 2.1 Gauss–Seidel on the Linear System

Ignore inequalities for a moment. If we define:

\[
A = J M^{-1} J^T, \quad \mathbf x = \boldsymbol\lambda, \quad \mathbf b' = - (J\mathbf v + \mathbf b),
\]

then the exact solution of the linearized system is the solution to:

\[
A \mathbf x = \mathbf b'.
\]

In PGS, we treat this as a system of scalar equations and update **one component at a time**, using the latest information as we sweep.

For row \(j\), we have:

\[
a_{jj} x_j + \sum_{k \ne j} a_{jk} x_k = b'_j,
\]

so the scalar update is:

\[
x_j \leftarrow \frac{b'_j - \sum_{k \ne j} a_{jk} x_k}{a_{jj}}.
\]

We perform such updates in sequence over all rows.

### 2.2 Impulse View

In the engine we avoid forming \(A\) explicitly. Instead:

- We store per-row **effective mass** \(k_j = (J_j M^{-1} J_j^T)^{-1} = 1/a_{jj}\).
- We compute **constraint velocity**:

  \[
  \dot{C}_j = J_j \mathbf v + b_j.
  \]

- For a candidate update, we compute:

  \[
  \Delta \lambda_j = -k_j \dot{C}_j.
  \]

- We apply this increment to \(\mathbf v\) via:

  \[
  \Delta \mathbf v = M^{-1} J_j^T \Delta \lambda_j,
  \quad \mathbf v \leftarrow \mathbf v + \Delta \mathbf v.
  \]

This effectively performs the Gauss–Seidel update for row \(j\) using the current velocities, which already include contributions from earlier rows in the sweep.

Because we update \(\mathbf v\) *in place*, later rows see the effects of earlier impulses within the same iteration.

### 2.3 Projection to Handle Inequalities

To handle bounds like \(j_n \ge 0\), friction cone, etc., we adopt **projected Gauss–Seidel**:

1. Maintain **accumulated impulses** \(\lambda_j^\text{acc}\).
2. Compute a candidate increment \(\Delta \lambda_j\).
3. Propose:
   \[
   \lambda_j^\text{acc,new} = \lambda_j^\text{acc} + \Delta \lambda_j.
   \]
4. **Project** \(\lambda_j^\text{acc,new}\) onto the feasible set:
   - For normal: clamp to \([0, \infty)\).
   - For friction: projected onto the Coulomb cone.
5. Compute the actual increment to apply:
   \[
   \Delta \lambda_j^\text{actual} = \lambda_j^\text{acc,new} - \lambda_j^\text{acc}.
   \]
6. Apply \(\Delta \lambda_j^\text{actual}\) to \(\mathbf v\).

This yields PGS: Gauss–Seidel iteration combined with per-row projection.

---

## 3. Warm-Starting

Warm-starting uses **previous step’s impulses** as an initial guess for \(\boldsymbol\lambda\).

### 3.1 Motivation

If the constraint graph changes slowly between frames:

- The solution at the new frame is close to the solution at the previous frame.
- Starting from previous impulses reduces the number of iterations needed to converge.

Warm-starting is especially helpful in:

- Stable stacks.
- Joints that remain active for long periods.
- Contacts with persistent manifolds.

### 3.2 Implementation

For each row \(j\):

1. Store last frame’s accumulated impulse \(\lambda_j^{\text{prev}}\).
2. At the start of the solver:
   - Set \(\lambda_j^\text{acc} \leftarrow \lambda_j^{\text{prev}}\).
   - Apply its effect to velocities once:
     \[
     \mathbf v \leftarrow \mathbf v + M^{-1} J_j^T \lambda_j^\text{acc}.
     \]

3. Then run PGS iterations as usual.

We may scale warm-start impulses or reset them in case of:

- Severe topology changes.
- Bodies separating entirely.

---

## 4. Over-Relaxation (SOR)

Over-relaxation modifies the basic Gauss–Seidel update to accelerate convergence.

### 4.1 SOR Update

Let \(\lambda_j^\text{acc}\) be the current accumulated impulse for row \(j\), and let \(\lambda_j^\text{PGS}\) be the value obtained by a standard PGS update (before projection). Then the SOR update is:

\[
\lambda_j^\text{new} = \lambda_j^\text{acc} + \omega (\lambda_j^\text{PGS} - \lambda_j^\text{acc}),
\]

where \(\omega \in (0, 2)\) is the **over-relaxation factor**.

In practice we often work with increments:

- Compute \(\Delta \lambda_j^\text{PGS}\).
- Scale it by \(\omega\) before projection:
  \[
  \Delta \lambda_j^\text{SOR} = \omega \Delta \lambda_j^\text{PGS}.
  \]

Then apply:

1. Tentative:
   \[
   \lambda_j^\text{acc,new} = \lambda_j^\text{acc} + \Delta \lambda_j^\text{SOR}.
   \]
2. Project to feasible set.
3. Apply difference as usual.

### 4.2 Choosing ω

- \(\omega = 1\): ordinary PGS.
- \(1 < \omega < 2\): over-relaxation; can accelerate convergence but risks instability if too large.
- \(\omega < 1\): under-relaxation; more stable but slower.

Common strategies:

- Use a **global ω** (e.g. 1.2–1.8) tuned empirically.
- Potential future work: per-row or per-tile ω based on heuristic estimates of stiffness or local convergence behavior.

---

## 5. Block PGS

Instead of treating each scalar row independently, we can **group related rows** and solve small blocks exactly (or more accurately).

### 5.1 Motivation

Many constraints naturally form small blocks:

- A contact manifold with 1 normal + 2 tangents: 3 rows.
- A joint with multiple scalar constraints.

Solving these rows simultaneously as a block can:

- Improve convergence near stiff, tightly coupled constraints.
- Reduce the number of outer iterations.

### 5.2 Block System

For a block of constraints indexed by set \(B\):

\[
A_B \boldsymbol\lambda_B = \mathbf b_B,
\]

where:

- \(\boldsymbol\lambda_B\) is the vector of impulses in the block.
- \(A_B = J_B M^{-1} J_B^T\) is a small dense matrix (e.g. 3×3, 4×4, etc.).
- \(\mathbf b_B\) is the corresponding right-hand side.

We can:

- Form \(A_B\) explicitly (just for the block).
- Factorize it (e.g., Cholesky or LDL).
- Solve for \(\Delta \boldsymbol\lambda_B\) exactly in one shot.

### 5.3 Inequalities and Projection

When blocks include inequality constraints (e.g., friction cones), we must:

- Either approximate them (e.g., box friction),
- Or perform a small local iterative solve inside the block that respects the constraints.

Even with approximations, block solvers can significantly improve behavior in stacks and complex joints.

---

## 6. Krylov / Accelerated Iterations (High-Level)

PGS can be seen as a fixed-point iteration:

\[
\boldsymbol\lambda^{(k+1)} = F(\boldsymbol\lambda^{(k)}),
\]

where \(F\) is the composition of all per-row PGS updates.

**Krylov-type accelerators** (like Anderson acceleration) attempt to speed up convergence by:

- Using several past iterates and residuals.
- Forming an improved next iterate via a small least-squares problem.

### 6.1 Matrix-Free Nature

Since we never form \(A\) explicitly, we need **matrix-free** accelerators:

- We only require evaluations of \(F(\boldsymbol\lambda)\) and residuals:
  \[
  \mathbf r^{(k)} = F(\boldsymbol\lambda^{(k)}) - \boldsymbol\lambda^{(k)}.
  \]

These can be computed from:

- Constraint rows.
- Current velocities and impulses.

Acceleration is especially attractive for:

- High-contact scenes where vanilla PGS requires many iterations.
- Large timesteps.

### 6.2 Combining with Projection

We can:

- Perform acceleration on the **unprojected** update and then project.
- Or treat projection as part of the fixed-point map and accelerate the whole \(F\).

Details are subtle; this is a promising but more experimental direction.

---

## 7. ADMM / Global Splitting (Overview)

**ADMM (Alternating Direction Method of Multipliers)** and related splitting schemes can treat constraints more globally:

- Split variables into local and global parts.
- Alternate between enforcing constraints locally and solving a global consistency step.

In our context:

- Local updates might solve contact and joint constraints (prox operators).
- Global updates might enforce consistency of velocities and momentum across the system.

ADMM is interesting for:

- Dense contact networks.
- GPU-style parallel implementations (local updates are highly parallel).

However, it is more complex and usually has higher per-iteration cost than basic PGS. In a real-time engine we would likely deploy it selectively (e.g., as a special mode for dense, high-accuracy tasks).

---

## 8. XPBD and Compliance (Optional)

**XPBD (Extended Position-Based Dynamics)** introduces **compliance** directly at the position level:

- Constraints are formulated on positions, with compliance controlling how “soft” they are.
- Updates operate primarily on positions, with velocities derived or corrected afterward.

From a mathematical perspective:

- XPBD is related to implicit integration of compliant constraints.
- It can be tied back to an effective mass/compliance picture similar to the one we use at the velocity level.

In our project, XPBD-style methods could:

- Co-exist with traditional PGS.
- Use the same constraint graph and row data, but with different solver loops.

They are particularly attractive for:

- Soft constraints,
- Cloth and rope, or
- Cases where robustness is more important than strict energy/momentum conservation.

---

## 9. Convergence and Quality Metrics

We measure solver quality via:

- **Convergence metrics**:
  - Residual norms \(\|\mathbf r\|\).
  - Changes in impulses \(\|\Delta \boldsymbol\lambda\|\).
  - Number of iterations to reach a tolerance.

- **Physics metrics** (ties back to ADMC and constraint theory):
  - Maximum penetration and joint error.
  - Directional momentum and energy drift.
  - Cone consistency for friction.
  - Stability over many timesteps.

Different solver variants can be compared on:

- Speed (ms per step).
- Required iterations.
- Quality metrics (drift, errors).

This informs which solver/backends are suitable for a given application (e.g., high-speed benchmark vs interactive game).

---

## 10. Summary

- PGS / sequential impulses is our **baseline iterative method**:
  - Scalar per-row updates.
  - Projection to handle inequalities.
- Warm-starting and over-relaxation significantly improve convergence in typical engine scenarios.
- Block PGS improves behavior for tightly coupled constraints (manifolds, joints).
- Krylov acceleration and ADMM are promising directions for more global, high-accuracy solvers.
- XPBD/compliance offers a complementary position-based view.

All of these methods operate over the **constraint row structures** derived in the previous theory document and implemented in the engine’s solver architecture. They are evaluated and tuned using the **invariants and metrics** described in the next theory document:

- **“ADMC Theory – 06: Invariants and Metrics.”**
