# Theory 04 – Constraint Rows in One Page

> Every solver row is “J · v + b = 0 with limits”. Keep these formulas handy.

---

## Row Anatomy

For bodies A and B with offsets \(\mathbf r_A, \mathbf r_B\):

- Jacobian pieces:
  - Linear A:  \(\mathbf J_{vA} = -\hat{\mathbf d}\)
  - Angular A: \(\mathbf J_{\omega A} = -(\mathbf r_A \times \hat{\mathbf d})\)
  - Linear B:  \(\mathbf J_{vB} = \hat{\mathbf d}\)
  - Angular B: \(\mathbf J_{\omega B} = (\mathbf r_B \times \hat{\mathbf d})\)
- Relative velocity along direction \(\hat{\mathbf d}\):
  \[
  v_\text{rel} = \mathbf J_{vA}\cdot \mathbf v_A + \mathbf J_{\omega A}\cdot \boldsymbol\omega_A
               + \mathbf J_{vB}\cdot \mathbf v_B + \mathbf J_{\omega B}\cdot \boldsymbol\omega_B
  \]
- Effective mass:
  \[
  m_\text{eff} = \left(
     \frac{1}{m_A} + \frac{1}{m_B}
     + (\mathbf J_{\omega A}\cdot \mathbf I_A^{-1} \mathbf J_{\omega A})
     + (\mathbf J_{\omega B}\cdot \mathbf I_B^{-1} \mathbf J_{\omega B})
  \right)^{-1}
  \]
- Bias \(b\):
  - Contacts: positional correction (Baumgarte/ERP) + restitution term.
  - Joints: error reduction along the joint axis.

Row solve (projected GS):

\[
\Delta \lambda = -m_\text{eff} (v_\text{rel} + b),\quad
\lambda \leftarrow \text{clamp}(\lambda_\text{old} + \Delta\lambda, [\lambda_\text{min}, \lambda_\text{max}])
\]

Apply impulses:

\[
\mathbf v_A \mathrel{+}= \frac{\Delta\lambda}{m_A} \mathbf J_{vA}, \quad
\boldsymbol\omega_A \mathrel{+}= \mathbf I_A^{-1} (\mathbf J_{\omega A} \Delta\lambda),
\]
and similarly for B with opposite signs.

---

## Contact Specializations

- **Normal row**
  - Direction: contact normal \(\hat{\mathbf n}\).
  - Limits: \([0, +\infty)\).
  - Bias: penetration correction + restitution.
- **Friction rows**
  - Directions: tangents \(\hat{\mathbf t}_1, \hat{\mathbf t}_2\).
  - Limits: Coulomb cone via `lambda_t ∈ [-μ λ_n, μ λ_n]`.
  - Bias: usually zero unless modeling friction stabilization.
- **Rolling / twisting** rows (optional)
  - Work the same way; directions align with rotational axes.

---

## Joint Rows

- Build from joint frames + constraint axes.
- Bias often targets positional error: \(b = \beta \frac{\text{error}}{\Delta t}\).
- Limits depend on joint type (equality vs inequality).
- Use the same Jacobian + effective-mass math; only the direction vector changes.

---

## Implementation Notes

- Precompute `m_eff` and store alongside each row (warm-start uses it too).
- Keep `lambda` cached per row; warm-start simply reapplies `lambda` scaled by tracker policy.
- For SIMD tiles:
  - Store Jacobian pieces as SoA arrays of floats.
  - Keep local body indices to avoid gathers.
- Diagnostics:
  - Residual = `|v_rel + b|`.
  - ADMC attribution uses the row direction for its channel.

That’s all you need to implement or audit a constraint row.
