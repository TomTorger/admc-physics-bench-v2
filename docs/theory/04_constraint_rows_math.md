# ADMC Theory – 04: Constraint Rows and Contact/Joints Math

> This document derives the scalar constraint rows used by the solver: contact normals, friction, and basic joints. It focuses on the math we implement in the engine.

---

## 1. General Constraint Form

We describe constraints in terms of:

- Configuration \( \mathbf q \) (positions and orientations).
- Velocities \( \mathbf v \) (linear and angular).
- A scalar or vector constraint function \( C(\mathbf q, t) \).

### 1.1 Position- and Velocity-Level Constraints

- **Position-level constraint**:
  \[
  C(\mathbf q, t) = 0.
  \]
  For example, a distance joint requiring bodies A and B be separated by a fixed length.

- **Velocity-level constraint**:
  \[
  \dot{C}(\mathbf q, \mathbf v, t) = 0.
  \]
  Often derived from the time derivative of the position-level constraint.

We typically enforce constraints at the **velocity level** using impulses, with optional bias terms that push the position-level constraint towards zero over time.

### 1.2 Jacobian Representation

For rigid bodies we can stack generalized velocities into a vector:

\[
\mathbf v = 
\begin{pmatrix}
\mathbf v_1 \\
\boldsymbol\omega_1 \\
\vdots \\
\mathbf v_N \\
\boldsymbol\omega_N
\end{pmatrix}.
\]

A linearized velocity-level constraint can be written as:

\[
\dot{C}(\mathbf q, \mathbf v, t) = J \mathbf v + b = 0,
\]

where:

- \(J\) is the constraint Jacobian.
- \(b\) is a bias term (for Baumgarte, restitution, etc.).

For a **scalar** constraint, \(J\) is a row vector; for a vector-valued constraint, \(J\) has one row per scalar component.

---

## 2. Impulses, Mass Matrix, and Effective Mass

We denote the **generalized mass matrix** by \(M\), which is block-diagonal for rigid bodies:

- Each body \(i\) contributes:
  \[
  M_i = 
  \begin{pmatrix}
  m_i \mathbf I_3 & 0 \\
  0 & \mathbf I_i
  \end{pmatrix},
  \]
  where \(\mathbf I_i\) is the world-space inertia tensor.

Collecting all bodies, \(M\) is a large block-diagonal matrix.

### 2.1 Impulse Update

Given a scalar impulse \(\Delta \lambda\) associated with a constraint row \(J\), the generalized velocity update is:

\[
\Delta \mathbf v = M^{-1} J^T \Delta \lambda.
\]

The new velocities are:

\[
\mathbf v' = \mathbf v + \Delta \mathbf v.
\]

### 2.2 Effective Mass for a Scalar Row

We define the **effective mass** (or inverse effective mass) of a scalar row as:

\[
k = (J M^{-1} J^T)^{-1}.
\]

Intuitively, \(k\) describes how much a scalar impulse along this constraint direction changes the constraint velocity.

Given a desired change in the constraint velocity \(\Delta \dot{C}\), the scalar impulse that achieves this (ignoring inequality constraints) is:

\[
\Delta \lambda = \frac{\Delta \dot{C}}{J M^{-1} J^T} = k \, \Delta \dot{C}.
\]

This is the core of most iterative solvers in this project.

---

## 3. Contact Normal Constraints

Consider a contact between body A and body B:

- Contact point position: \(\mathbf x_c\).
- Offset from centers:
  \[
  \mathbf r_A = \mathbf x_c - \mathbf x_A, \quad \mathbf r_B = \mathbf x_c - \mathbf x_B.
  \]
- Contact normal: \(\hat{\mathbf n}\) (pointing from A to B).

### 3.1 Relative Velocity at the Contact

Velocities at the contact point:

\[
\mathbf v_A^\text{(pt)} = \mathbf v_A + \boldsymbol\omega_A \times \mathbf r_A,
\]
\[
\mathbf v_B^\text{(pt)} = \mathbf v_B + \boldsymbol\omega_B \times \mathbf r_B.
\]

Relative velocity:

\[
\mathbf v_\text{rel} = \mathbf v_B^\text{(pt)} - \mathbf v_A^\text{(pt)}.
\]

Normal component:

\[
v_n = \mathbf v_\text{rel} \cdot \hat{\mathbf n}.
\]

We typically want \(v_n\) to be **non-negative**, i.e., bodies are separating or resting, not approaching.

### 3.2 Jacobian Row

We want a velocity-level constraint \(\dot{C} = 0\) related to \(v_n\). A standard choice for the scalar constraint is:

\[
\dot{C}_n = v_n + \text{(bias / restitution)}.
\]

The Jacobian row \(J_n\) is constructed so that:

\[
J_n \mathbf v = v_n.
\]

Expanding \(v_n\):

\[
v_n = \hat{\mathbf n} \cdot (\mathbf v_B - \mathbf v_A)
+ \hat{\mathbf n} \cdot (\boldsymbol\omega_B \times \mathbf r_B - \boldsymbol\omega_A \times \mathbf r_A).
\]

This can be written as contributions from each body’s linear and angular velocity:

- For A:
  \[
  -\hat{\mathbf n} \cdot \mathbf v_A - (\mathbf r_A \times \hat{\mathbf n}) \cdot \boldsymbol\omega_A.
  \]
- For B:
  \[
  +\hat{\mathbf n} \cdot \mathbf v_B + (\mathbf r_B \times \hat{\mathbf n}) \cdot \boldsymbol\omega_B.
  \]

So the Jacobian row has blocks:

- For A:
  - Linear: \(-\hat{\mathbf n}^T\).
  - Angular: \(-(\mathbf r_A \times \hat{\mathbf n})^T\).
- For B:
  - Linear: \(+\hat{\mathbf n}^T\).
  - Angular: \(+(\mathbf r_B \times \hat{\mathbf n})^T\).

All other bodies have zero contribution in this row.

### 3.3 Effective Mass for the Normal Row

Let the inverse mass and inverse inertia for A and B be:

- Linear: \(m_A^{-1}, m_B^{-1}\).
- Angular: \(\mathbf I_A^{-1}, \mathbf I_B^{-1}\).

Then the effective mass for the normal row is:

\[
k_n = \left(
m_A^{-1} + m_B^{-1}
+ (\mathbf r_A \times \hat{\mathbf n})^T \mathbf I_A^{-1} (\mathbf r_A \times \hat{\mathbf n})
+ (\mathbf r_B \times \hat{\mathbf n})^T \mathbf I_B^{-1} (\mathbf r_B \times \hat{\mathbf n})
\right)^{-1}.
\]

The solver usually stores \(k_n\) or its reciprocal.

### 3.4 Target Velocity and Bias

The raw constraint is:

\[
\dot{C}_n = v_n^\text{(after)} - v_n^\star = 0,
\]

where \(v_n^\star\) is the **target normal velocity**, often defined as:

\[
v_n^\star = -\text{restitution} \cdot v_n^\text{(before)} - \beta \frac{\phi}{\Delta t},
\]

where:

- \(v_n^\text{(before)}\) is the pre-solve normal velocity.
- \(\phi\) is the contact penetration depth.
- \(\beta\) is a Baumgarte coefficient.
- \(\Delta t\) is the timestep.

We can rewrite this in the standard PGS form:

\[
v_n^\text{(after)} = v_n^\text{(before)} + \Delta v_n,
\]
\[
\Delta v_n = v_n^\star - v_n^\text{(before)}.
\]

The update is:

\[
\Delta j_n = \frac{v_n^\star - v_n}{1 / k_n} = k_n (v_n^\star - v_n).
\]

### 3.5 Inequality Constraint

The contact normal impulse must satisfy:

\[
j_n \ge 0.
\]

In practice, we:

1. Maintain an accumulated impulse \(j_n^\text{acc}\).
2. Compute a candidate update \(\Delta j_n\).
3. Clamp the new accumulated impulse \(j_n^\text{acc}'\) to \([0,\infty)\).
4. Use the difference \(\Delta j_n' = j_n^\text{acc}' - j_n^\text{acc}\) as the actual applied impulse in this iteration.

This is the standard “projected Gauss–Seidel” step for contact normals.

---

## 4. Friction Constraints

For friction we choose two orthonormal tangents \(\hat{\mathbf t}_1, \hat{\mathbf t}_2\) perpendicular to \(\hat{\mathbf n}\).

### 4.1 Tangent Velocities

Tangent components of the relative velocity:

\[
v_{t_1} = \mathbf v_\text{rel} \cdot \hat{\mathbf t}_1, \quad
v_{t_2} = \mathbf v_\text{rel} \cdot \hat{\mathbf t}_2.
\]

We want to drive \(v_{t_1}, v_{t_2}\) towards zero (stick) or keep them limited (slip).

### 4.2 Tangent Jacobian Rows

Similar to the normal case, define rows \(J_{t_1}, J_{t_2}\) such that:

\[
J_{t_1} \mathbf v = v_{t_1}, \quad
J_{t_2} \mathbf v = v_{t_2}.
\]

The pattern is identical to the normal, replacing \(\hat{\mathbf n}\) with \(\hat{\mathbf t}_1\) or \(\hat{\mathbf t}_2\):

- For A:
  - Linear: \(-\hat{\mathbf t}_i^T\).
  - Angular: \(-(\mathbf r_A \times \hat{\mathbf t}_i)^T\).
- For B:
  - Linear: \(+\hat{\mathbf t}_i^T\).
  - Angular: \(+(\mathbf r_B \times \hat{\mathbf t}_i)^T\).

### 4.3 Tangent Effective Masses

Effective masses are:

\[
k_{t_i} = \left(
m_A^{-1} + m_B^{-1}
+ (\mathbf r_A \times \hat{\mathbf t}_i)^T \mathbf I_A^{-1} (\mathbf r_A \times \hat{\mathbf t}_i)
+ (\mathbf r_B \times \hat{\mathbf t}_i)^T \mathbf I_B^{-1} (\mathbf r_B \times \hat{\mathbf t}_i)
\right)^{-1}.
\]

### 4.4 Coulomb Cone and Projection

Let \(\mu\) be the friction coefficient and \(j_n\) the current normal impulse. The **Coulomb cone** constraint is:

\[
\|\mathbf j_t\| \le \mu j_n,
\]
where
\[
\mathbf j_t = (j_{t_1}, j_{t_2}).
\]

Implementation:

1. Maintain accumulated tangent impulses \(j_{t_1}^\text{acc}, j_{t_2}^\text{acc}\).
2. For each tangent \(i\), compute a candidate update:
   \[
   \Delta j_{t_i} = k_{t_i} (v_{t_i}^\star - v_{t_i}),
   \]
   usually with \(v_{t_i}^\star = 0\).

3. Tentatively update:
   \[
   \mathbf j_t^\text{acc,new} = \mathbf j_t^\text{acc} + (\Delta j_{t_1}, \Delta j_{t_2}).
   \]

4. If \(\|\mathbf j_t^\text{acc,new}\| \le \mu j_n\) (inside cone), accept; else:
   - Project onto the cone:
     \[
     \mathbf j_t^\text{acc,new} = \mu j_n \cdot \frac{\mathbf j_t^\text{acc,new}}{\|\mathbf j_t^\text{acc,new}\|}.
     \]
   - Use the difference between old and new accumulated impulses as the applied impulse in this iteration.

This enforces an approximate Coulomb friction model.

---

## 5. Joint Constraints (Example: Distance Joint)

Consider a distance joint between two anchor points:

- Anchor on A: \(\mathbf a_A\) in local space.
- Anchor on B: \(\mathbf a_B\) in local space.
- Desired distance: \(L\).

World-space positions:

\[
\mathbf x_A^\text{(anchor)} = \mathbf x_A + \mathbf R_A \mathbf a_A,
\]
\[
\mathbf x_B^\text{(anchor)} = \mathbf x_B + \mathbf R_B \mathbf a_B.
\]

Constraint:

\[
C(\mathbf q) = \|\mathbf x_B^\text{(anchor)} - \mathbf x_A^\text{(anchor)}\| - L = 0.
\]

### 5.1 Velocity-Level Form

Let \(\mathbf d = \mathbf x_B^\text{(anchor)} - \mathbf x_A^\text{(anchor)}\), and \(\hat{\mathbf d} = \mathbf d / \|\mathbf d\|\) when \(\|\mathbf d\| > 0\).

The velocity of the anchor points:

\[
\mathbf v_A^\text{(anchor)} = \mathbf v_A + \boldsymbol\omega_A \times (\mathbf R_A \mathbf a_A),
\]
\[
\mathbf v_B^\text{(anchor)} = \mathbf v_B + \boldsymbol\omega_B \times (\mathbf R_B \mathbf a_B).
\]

Relative velocity:

\[
\mathbf v_\text{rel} = \mathbf v_B^\text{(anchor)} - \mathbf v_A^\text{(anchor)}.
\]

We compute:

\[
\dot{C} = \hat{\mathbf d} \cdot \mathbf v_\text{rel}.
\]

And we want \(\dot{C} = 0\) (velocity-level constraint).

### 5.2 Jacobian Row for Distance Joint

This is analogous to a normal contact constraint where the direction is \(\hat{\mathbf d}\), and the offsets are \(\mathbf R_A \mathbf a_A\) and \(\mathbf R_B \mathbf a_B\). We get:

- For A:
  - Linear: \(-\hat{\mathbf d}^T\).
  - Angular: \(-(\mathbf R_A \mathbf a_A \times \hat{\mathbf d})^T\).
- For B:
  - Linear: \(+\hat{\mathbf d}^T\).
  - Angular: \(+(\mathbf R_B \mathbf a_B \times \hat{\mathbf d})^T\).

Effective mass is computed using the same formula as contact normals, with these offsets.

### 5.3 Stabilization

To prevent drift in \(C(\mathbf q)\), we add a bias term:

\[
\dot{C} + \beta \frac{C}{\Delta t} = 0,
\]

which yields a target normal velocity for the joint:

\[
v_\text{joint}^\star = - \beta \frac{C}{\Delta t}.
\]

This is directly analogous to bias in contact constraints.

---

## 6. Bias, Restitution, and Softness

Bias and restitution appear in the scalar constraint equation:

\[
\dot{C} = J \mathbf v + b = 0.
\]

We can interpret \(b\) as:

\[
b = \text{bias} + \text{restitution contribution} + \text{(other softness/damping terms)}.
\]

- **Baumgarte bias**:
  \[
  \text{bias} = \beta \frac{C}{\Delta t},
  \]
  where \(\beta\) controls how aggressively we correct positional error.

- **Restitution** (for contacts):
  \[
  \text{rest. term} = e \max(-v_n^\text{(before)}, 0),
  \]
  with restitution coefficient \(e \in [0, 1]\).

- **Softness / compliance**:
  - Sometimes incorporated by modifying effective mass or adding damping.

These terms directly influence the target velocities used to compute impulses.

---

## 7. ADMC Interpretation of Rows

Each scalar row operates along a **specific direction** in velocity space:

- Contact normals: along contact normal.
- Friction: along tangents.
- Joints: along some constraint direction (which may or may not align with global axes).

From the ADMC perspective:

- Each update to \(\lambda\) (the scalar impulse) corresponds to a **redistribution of directional momentum** among the affected bodies.
- If we restricted ourselves to perfectly elastic constraints with no bias, and we solved the system exactly, the sum of ADMC-like directional invariants would remain constant for the closed system.
- In practice, with approximate PGS iterations and bias, we expect small but nonzero drift.

This motivates:

- **Metric definitions** that measure directional momentum/energy drift per step.
- **Comparisons** between solvers on how well they preserve these invariants.

---

## 8. Mapping to Engine Structures

In code, the above derivations correspond to:

- **Constraint row structs**:
  - Store per-row Jacobian data in a reduced form (e.g., contact direction, offsets, effective mass, bias, limits).
- **Constraint builders**:
  - Take bodies and contacts/joints, compute Jacobians, effective masses, biases, and cache them into rows.
- **Solvers**:
  - Use the per-row data to run PGS or other schemes, updating scalar impulses and applying them to body velocities.

The engine architecture documents describe:

- How these rows are organized in AoS / SoA / AoSoA layouts.
- How they are grouped into tiles and islands.
- How solver backends iterate and apply impulses.

This theory document is the “math side” of those implementation choices. For how we actually iterate on these rows, see:

- **“ADMC Theory – 05: PGS and Solver Variants (Math Perspective).”**
