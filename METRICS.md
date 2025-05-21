
# 📊 Metric definitions & derivations

This page gives the precise mathematical background for every column written to **`benchmark_results.csv`**.

---

## 1 Success rate `success_rate%`

If *N* start/goal pairs are attempted ( *N = 6* in the default dataset) and *S* of those are solved,

\[
\text{success\_rate} \,=\, \frac{S}{N}\times100\;\%.
\]

No variance is reported for a single benchmark pass; run multiple seeds to obtain an empirical distribution.

---

## 2 Planning time `plan_time_s`

Wall‑clock time (Python `time.perf_counter`) is sampled **immediately before** and **immediately after** the planner call:

\[
T_{\text{plan}} = t_{\text{after}} - t_{\text{before}}.
\]

This excludes:

* PyBullet start‑up and world loading  
* any IK used to generate configurations  
* path execution or visual playback  

---

## 3 Execution time `exec_time_s`

PyBullet steps at a fixed rate of  
\(\Delta t = \tfrac{1}{240}\;\text{s}\).

If a returned path has *L* way‑points,

\[
T_{\text{exec}} = L\,\Delta t.
\]

This is an **upper bound** because controllers may interpolate internally.

---

## 4 Smoothness (curvature heuristic) `smoothness`

Given a discrete joint path \(\{q_0,\dots,q_{L-1}\}\),

\[
\kappa_i = q_{i+1} \;−\; 2q_i \; + \; q_{i-1}\qquad (1\le i\le L-2).
\]

We sum Euclidean norms:

\[
\text{smoothness}=\sum_{i=1}^{L-2}\lVert\kappa_i\rVert_2.
\]

* **0** → perfectly straight (only two segments)  
* larger → jerkier, more “wiggles”

---

## 5 Clearance `clearance_m`

For each configuration the robot is **teleported** (no dynamics) and PyBullet  
`getClosestPoints(robot, obstacle, d_max)` with *d_max = 1 m* returns separation \(d\).

\[
\text{clearance}=\min_{q\in\text{path}}\;\min_{\ell,o}\; d_{\ell,o}(q).
\]

* negative ≤ −0.001 m means the robot grazed the obstacle within the 1 mm tolerance shell  
* higher positive values are safer but may correlate with longer paths

---

## 6 Memory overhead `mem_MB`

Using `psutil.Process(...).memory_info().rss` before and after planning:

\[
\Delta m = \frac{\text{RSS}_{\text{after}} - \text{RSS}_{\text{before}}}{1024^2}\;\text{MB}.
\]

Averaged across the six cases for each planner row.

---

## 7 Statistical usage

* **Fix random seeds** for deterministic benchmarking **or** average metrics over ≥20 seeds.  
* For heavy‑tailed timings use median ± IQR.  
* Represent timing distributions with violin or box‑and‑whisker plots instead of single bars.

