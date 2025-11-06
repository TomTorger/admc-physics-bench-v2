# ADMC Theory – 06: Invariants and Metrics

> This document defines the invariants and metrics we use to evaluate solver quality: momentum and energy drift, constraint violations, friction consistency, and various stability indicators. These metrics are intended to be computed in benchmarks and optionally during development builds.

---

## 1. Why Metrics?

No real-time solver is exact. We:

- Use discrete timesteps.
- Approximate constraints.
- Introduce bias and damping for stability.

Without metrics, it is hard to tell:

- Whether a change in solver architecture is an improvement.
- How sensitive a solver is to timestep, stacking depth, or stiffness.
- Whether a regression has occurred between versions.

We therefore define a set of **invariants** (ideal conservation laws and constraints) and associated **metrics** (how much we deviate from them). These are used in:

- Benchmark runs.
- Regression tests.
- Solver tuning and comparison.

---

## 2. Global Linear Momentum and Energy

### 2.1 Linear Momentum

For a collection of rigid bodies indexed by \(i\):

- Linear momentum of body \(i\):
  \[
  \mathbf p_i = m_i \mathbf v_i.
  \]
- Total linear momentum:
  \[
  \mathbf P = \sum_i \mathbf p_i.
  \]

In a closed system with no external impulses, \(\mathbf P\) should be constant. In practice we compute:

- **Initial momentum** \(\mathbf P_\text{init}\) at the start of a scenario.
- Momentum \(\mathbf P_t\) at later times \(t\).
- Metrics:
  - Absolute drift:
    \[
    \Delta \mathbf P_t = \mathbf P_t - \mathbf P_\text{init}.
    \]
  - Norms:
    \[
    \|\Delta \mathbf P_t\|, \quad \frac{\|\Delta \mathbf P_t\|}{\max(\|\mathbf P_\text{init}\|, \epsilon)}.
    \]
    Here \(\epsilon\) is a small number to avoid division by zero in cases where total momentum is nominally zero.

### 2.2 Angular Momentum

Choose a reference point, e.g., the world origin or center of mass. Angular momentum of body \(i\) about this point:

\[
\mathbf L_i = \mathbf r_i \times \mathbf p_i + \mathbf I_i \boldsymbol\omega_i,
\]

where:

- \(\mathbf r_i\) is position of body \(i\)’s center relative to the reference.
- \(\mathbf I_i\) is world-space inertia.

Total angular momentum:

\[
\mathbf L = \sum_i \mathbf L_i.
\]

Metrics analogous to linear momentum:

- Absolute and relative drift:
  \[
  \Delta \mathbf L_t = \mathbf L_t - \mathbf L_\text{init},
  \]
  with norms and normalized error.

Angular momentum metrics are especially useful for scenes involving free-floating bodies and joints.

### 2.3 Kinetic Energy

Kinetic energy per body:

\[
T_i = \frac{1}{2} m_i \|\mathbf v_i\|^2 + \frac{1}{2} \boldsymbol\omega_i^T \mathbf I_i \boldsymbol\omega_i.
\]

Total kinetic energy:

\[
T = \sum_i T_i.
\]

In the absence of external work, perfect constraints, and no damping, kinetic energy should be conserved for ideally elastic interactions; with restitution and friction, energy may intentionally change. Nevertheless, tracking kinetic energy is useful for:

- Detecting unexpected energy injection or excessive damping.
- Comparing solvers at fixed settings.

Metrics:

- Absolute and relative energy drift:
  \[
  \Delta T_t = T_t - T_\text{init}, \quad
  \frac{\Delta T_t}{\max(|T_\text{init}|, \epsilon)}.
  \]

---

## 3. Directional ADMC-Inspired Metrics

ADMC suggests working with **directional** invariants. In the Newtonian context, we define directional metrics that approximate this idea.

### 3.1 Choice of Directions

We select a set of unit directions \(\{\hat{\mathbf d}_k\}\). Typical choices include:

- Global axes: \(\pm \hat{\mathbf x}, \pm \hat{\mathbf y}, \pm \hat{\mathbf z}\).
- Additional directions: diagonals or directions aligned with gravity, stack normals, etc.

This gives a finite set of directions for which we track directional quantities.

### 3.2 Directional Momentum

For direction \(\hat{\mathbf d}\), define the **directional linear momentum**:

\[
P_d = \sum_i m_i (\mathbf v_i \cdot \hat{\mathbf d}).
\]

We then track:

- Initial \(P_{d,\text{init}}\).
- \(P_{d,t}\) over time.
- Drift:
  \[
  \Delta P_{d,t} = P_{d,t} - P_{d,\text{init}}.
  \]

We may record:

- Max absolute drift over all directions:
  \[
  \max_k |\Delta P_{d_k, t}|.
  \]
- RMS drift:
  \[
  \sqrt{ \frac{1}{K} \sum_k (\Delta P_{d_k, t})^2 }.
  \]

### 3.3 Directional Kinetic Contributions

For direction \(\hat{\mathbf d}\), a simple directional kinetic measure is:

\[
T_d = \sum_i \frac12 m_i (\mathbf v_i \cdot \hat{\mathbf d})^2.
\]

We can also consider rotational contributions projected onto \(\hat{\mathbf d}\) if desired. Then:

- Track \(T_{d,\text{init}}\) and \(T_{d,t}\).
- Compute drift analogous to linear momentum.

### 3.4 Summary Metrics

For each frame or at the end of a run, we can report:

- **Directional momentum drift**:
  - Max over directions.
  - RMS over directions.
- **Directional kinetic drift**:
  - Max over directions.
  - RMS over directions.

These metrics highlight directional biases that may not appear clearly in global scalar/vector summaries.

---

## 4. Constraint Violation Metrics

Constraints (contacts, joints) define conditions we would like to hold approximately. We quantify **how badly** they are violated.

### 4.1 Contacts: Penetration Depth

For each contact \(c\):

- Penetration depth \(\phi_c\) (positive if bodies overlap).
- We collect:
  - Max penetration:
    \[
    \phi_\text{max} = \max_c \phi_c.
    \]
  - Average penetration:
    \[
    \bar\phi = \frac{1}{N_c} \sum_c \phi_c.
    \]
  - RMS penetration:
    \[
    \phi_\text{rms} = \sqrt{ \frac{1}{N_c} \sum_c \phi_c^2 }.
    \]

These metrics reflect how well the solver keeps bodies from interpenetrating. Particularly useful in stacking and pile scenarios.

### 4.2 Joints: Positional and Angular Errors

For each joint \(j\) with scalar constraints \(C_{j,\ell}\):

- Positional error:
  \[
  C_{j,\ell}(\mathbf q).
  \]

We record:

- Max joint error (\(\ell_\infty\) norm):
  \[
  C_\text{joint,max} = \max_{j,\ell} |C_{j,\ell}|.
  \]
- RMS joint error:
  \[
  C_\text{joint,rms} = \sqrt{ \frac{1}{N_\text{joint}} \sum_{j,\ell} C_{j,\ell}^2 }.
  \]

These measure drift in joint constraints, useful for articulated characters, chains, and mechanisms.

---

## 5. Friction and Cone Consistency

Friction constraints should approximately satisfy the **Coulomb cone**:

\[
\|\mathbf j_t\| \le \mu j_n
\]

for each contact with friction coefficient \(\mu\), where \(j_n\) is the normal impulse and \(\mathbf j_t\) is the tangential impulse vector.

### 5.1 Cone Violation Measure

For each contact, define the cone violation:

\[
\delta_c = \max(0, \|\mathbf j_{t,c}\| - \mu_c j_{n,c}).
\]

This is zero if the impulse lies within or on the cone, positive if outside.

We can track:

- Max cone violation:
  \[
  \delta_\text{max} = \max_c \delta_c.
  \]
- RMS cone violation:
  \[
  \delta_\text{rms} = \sqrt{ \frac{1}{N_c} \sum_c \delta_c^2 }.
  \]

While small violations can occur due to numerical error, large or systematic violations suggest solver or parameter issues.

---

## 6. Time-Stepping Stability

Beyond per-step invariants, we care about **long-term behavior** over many timesteps.

### 6.1 Long-Run Momentum and Energy Drift

We can track:

- Drifts in \(\mathbf P\), \(\mathbf L\), and \(T\) over long runs (many seconds of simulated time).
- Their growth rate:
  - Linear, sublinear, or explosive behavior.

This helps detect:

- Energy pumping from integrator or solver.
- Excess damping.

### 6.2 Scenario-Specific Stability

Some scenarios have **expected qualitative behavior**:

- A simple pendulum:
  - Should swing back and forth, with amplitude decaying slowly if damping is present, or staying roughly constant otherwise.
- A tower of boxes:
  - Should remain standing if physically reasonable, or collapse in a controlled way if perturbed.
- A spinning top:
  - Should maintain spin with predictable precession.

For such scenarios we can define scenario-specific metrics, e.g.:

- Decay rate of amplitude.
- Time until collapse or failure.
- Deviation from analytic trajectories when available.

These complement the more generic invariants.

---

## 7. Solver-Internal Metrics

While invariants are formulated in terms of physical quantities, we also track **solver-internal** metrics.

### 7.1 Iteration Counts and Residuals

For each solver run:

- Number of iterations taken:
  - Total iterations.
  - Per-tile or per-island iterations if adaptive.
- Residuals:
  - For each iteration \(k\), residual norm:
    \[
    \|\mathbf r^{(k)}\| = \|J \mathbf v^{(k)} + \mathbf b\|.
    \]
  - Optionally per-row residuals.

We can log:

- Final residual norm.
- Residual norm decay over iterations.
- Convergence rates when comparing solvers.

### 7.2 Projection and Clamping Statistics

We also record:

- How often and how strongly impulses are **clamped**:
  - Count of rows where normal impulses hit zero or maximum.
  - Frequency of friction cone projections.
- Distribution of impulse magnitudes:
  - Histograms of \(j_n\), \(\|\mathbf j_t\|\), joint impulses.

These metrics give insight into how “active” constraints are, and whether we are regularly hitting limits.

---

## 8. Reporting and Visualization

In practice we want metrics to be:

- **Machine-readable**:
  - Stored as JSON/CSV for automatic comparison and plotting.
- **Human-readable**:
  - Summarized in tables in benchmark output.
  - Visualized in plots (time series, histograms).

Typical outputs for each benchmark run:

- Per-step or per-frame metrics:
  - Timings (per phase).
  - Max/RMS penetration, joint error.
  - Directional drift metrics.
- Aggregate metrics over the run:
  - Max over time.
  - Final values.
  - Time-averaged values.

These can be used to:

- Populate tables in documentation.
- Drive CI regression tests:
  - E.g., require that penetration and drift not exceed specified thresholds.

---

## 9. Connecting Back to ADMC and Theory

The metrics defined here are grounded in the earlier theory:

- Global momentum and energy drift reflect **standard conservation laws**.
- Directional metrics mirror the **ADMC scalars**—they essentially sample directional combinations of momentum and energy.
- Constraint violation metrics measure how well we enforce the scalar constraints derived from rigid-body mechanics.
- Cone consistency and residuals reflect how well we approximate the formal mathematical conditions of the LCP / variational problem.

In particular:

- ADMC gives us a principled way to think about **directional** invariants.
- The constraint math explains how impulses should, in principle, preserve those invariants (modulo restitution, friction, and bias).
- The solver theory describes how iterative methods approximate the solution.

The metrics are where all of these pieces come together in a **quantitative, implementation-friendly** way.

---

## 10. Summary

- We defined a set of metrics across several categories:
  - Global momentum and energy.
  - Directional ADMC-inspired invariants.
  - Constraint violations (contacts and joints).
  - Friction cone consistency.
  - Time-stepping and scenario-specific stability.
  - Solver-internal quantities (iterations, residuals, clamping).
- These metrics are essential for:
  - Comparing solver architectures.
  - Tuning solver parameters.
  - Detecting regressions.
- They provide the practical link between:
  - Theoretical invariants (ADMC and classical mechanics).
  - Concrete engine behavior across a wide range of scenes.

In the new solver architecture, these metrics should be **first-class citizens**—easy to compute, log, and visualize in our benchmarking and development workflows.
