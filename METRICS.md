# 📊 Metric Definitions & Mathematical Interpretations

> Formal definitions of every column emitted by **`benchmark_results.csv`**.
> All symbols are defined per-run (one fixed random seed).  Aggregate across
> seeds for statistical significance.

---

## 1 Success rate `success_rate%`

| Symbol | Meaning |
|--------|---------|
| *N*    | Number of start–goal test cases (default **6**) |
| *S*    | Number of cases solved collision‑free |

$$
\widehat p = \frac{S}{N},\qquad
\text{success\_rate} = 100\widehat p\,\%.
$$

**Confidence interval (Clopper–Pearson, two‑sided):**

$$
CI_{1-\alpha} = \left[\;\mathrm{B}^{-1}\!\Bigl(\tfrac{\alpha}{2};\,S, N-S+1\Bigr),\;
                         \mathrm{B}^{-1}\!\Bigl(1-\tfrac{\alpha}{2};\,S+1, N-S\Bigr)\right].
$$

*Unit:* percent [%]

---

## 2 Planning time `plan_time_s`

$$
T_{\text{plan}}\;=\;t_{\text{after}}-t_{\text{before}},\qquad
[t_{\text{before}},\,t_{\text{after}}] \text{ placed around the *planner* call.}
$$

*Excludes:* world setup, IK, PyBullet stepping, logging.  
*Unit:* seconds [s]

---

## 3 Execution time `exec_time_s`

PyBullet fixed‑step Δt = 1⁄240 s. A path with *L* way‑points yields

$$
T_{\text{exec}}\;=\;L\,\Delta t.
$$

*Upper bound*—controllers may insert intermediate micro‑steps.  
*Unit:* seconds [s]

---

## 4 Smoothness `smoothness`

Discrete second difference in joint space  
\(\kappa_i = q_{i+1}-2q_i+q_{i-1}\), \(q_i\in\mathbb R^d\),

$$
S = \sum_{i=1}^{L-2}\lVert\kappa_i\rVert_2.
$$

As the discretisation \(\Delta s\to0\) the sum approaches the integral of squared curvature—lower *S* ⇒ smoother path.  
*Unit:* radians (dimensionless)

---

## 5 Clearance `clearance_m`

For pose *q* and link–obstacle pair (ℓ,o) PyBullet returns signed distance  
*d*<sub>ℓ,o</sub>(q). Minimum over the entire path:

$$
C = \min_{q\in\text{path}}\;\min_{\ell,o} d_{\ell,o}(q),\qquad d_{\max}=1\,\text m.
$$

*Negative (≈ −0.001 m)* ⇒ robot entered 1 mm tolerance shell yet remained collision‑free.  
*Unit:* metres [m]

---

## 6 Memory overhead `mem_MB`

$$
\Delta m\;=\;\frac{\text{RSS}_{\text{after}}-\text{RSS}_{\text{before}}}{1024^2}\;\text{MB}
$$

RSS obtained via **psutil**. Reported value is the mean across test cases.  
*Unit:* megabytes [MB]

---

## 7 Reporting recommendations
* Use **median ± IQR** for timing metrics.  
* Provide Clopper–Pearson intervals for success rate.  
* Scale clearance & smoothness to [0,1] before radar plots.
