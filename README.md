
# 🐼 PyBullet-Panda Planner Benchmark

A **deterministic, reproducible benchmark suite** that compares five classical motion-planning algorithms on a Franka Panda robot in PyBullet.

Algorithms compared
-------------------
* **Straight-line** (baseline)
* **RRT** (single-tree)
* **RRT-Connect** (bi-directional)
* **PRM** (eager roadmap)
* **Lazy-PRM** (deferred edge checks)

The benchmark measures *solver-only* runtime, success-rate, path smoothness, obstacle clearance and memory overhead. Every run appends to a **CSV log** for longitudinal tracking.

---

## 📁 Repository layout & file roles

| File | Purpose |
|------|---------|
| `motion_planning.py` | Implements the five planners **and** holds global hyper-parameters (`DEFAULT_MAX_ITER`, `DEFAULT_STEP_SIZE`, `DEFAULT_NUM_SAMPLES`, `DEFAULT_K`). Provides helpers `execute_motion_plan`, `get_movable_joints`. |
| `simEnv.py` | Builds the shelf scene, registers obstacles, and exposes a pin-hole depth camera (unused in this benchmark but ready for future perception tasks). |
| `panda_sim_grasp_arm.py` | Loads the Franka URDF, synchronises gripper joints, supplies inverse-kinematics helpers and Cartesian motion primitives for interactive tests. |
| `motion_test.py` | Interactive script to **generate six start/goal joint pairs** via collision-aware IK (`--regen`) and preview any single planner in a GUI. Serialises the pairs to `benchmark_pairs.json`. |
| `benchmark_runner.py` | **Batch benchmark driver** – loops over all five planners and all six pairs, captures metrics, updates `benchmark_results.csv`. GUI/playback toggle via the `VISUALISE` flag. |
| `benchmark_pairs.json` | Six pre-computed collision-free start/goal joint lists (auto-generated). |
| `benchmark_results.csv` | Growing log of every benchmark invocation (timestamped rows). |

---

## 🔧 Benchmark parameters

The **hyper-parameters below are hard-coded once** in `motion_planning.py` so *every* planner abides by the same computational budget.  Feel free to sweep them, but do so **globally** to preserve comparability.

| Constant | Applies to | Intuition | Default |
|----------|-----------|-----------|---------|
| `DEFAULT_MAX_ITER` | RRT & RRT-Connect | Upper bound on how many vertices a search tree may grow.  Fewer iterations → faster but less reliable; more iterations → slower but higher success. | **5 000** |
| `DEFAULT_STEP_SIZE` | All planners | Radial distance (in joint radians) between successive samples / interpolations.  Smaller steps sample the workspace finely (better for narrow gaps) at the cost of more collision checks. | **0.05 rad** (≈ 2.9°) |
| `DEFAULT_NUM_SAMPLES` | PRM & Lazy-PRM | Number of random joint states placed before connecting neighbours.  Sets roadmap density: raise for cluttered scenes; lower for speed. | **500** |
| `DEFAULT_K` | PRM flavours | Each node is connected to its *k* nearest neighbours.  Higher *k* improves connectivity but adds edge checks. | **10** |

> **Why equal values?** If one planner were allowed, say, 10 000 iterations while another got only 2 000, runtime numbers would not reflect algorithmics but merely allotted compute. Normalising these ceilings makes the benchmark *fair*.

### Runtime metrics logged by `benchmark_runner.py`

| Column | Formula | Insight |
|--------|---------|---------|
| `success_rate%` | `(successful pairs / 6) × 100` | Reliability of the planner under identical limits. |
| `plan_time_s` | `t_after – t_before` (around **planner function only**) | Raw solver speed, independent of setup or playback. |
| `exec_time_s` | `len(path) × 1/240 s` | Real-time duration to execute the joint trajectory.  Shorter often = more direct motion. |
| `smoothness` | `Σ‖qᵢ₊₁ − 2qᵢ + qᵢ₋₁‖` | Heuristic curvature measure; lower → fewer jerks. |
| `clearance_m` | Minimum of `getClosestPoints` distances along the path | Safety margin; negative means the robot skimmed within 1 mm of an obstacle (query tolerance). |
| `mem_MB` | Δ in resident set size (`psutil`) during planning | Extra RAM burden; usually negligible but tracked. |

---

## 🖥️  Requirements

| Package | Tested version |
|---------|---------------|
| Python | 3.9 – 3.11 |
| `pybullet` | ≥ 3.4 (Jan-2025 build) |
| `pybullet_planning` | ≥ 0.7.2 |
| `numpy` | ≥ 1.24 |
| `psutil` | ≥ 5.9 *(optional – memory metric)* |
| `matplotlib` | ≥ 3.8 *(optional – plotting)* |

```bash
python -m venv venv
source venv/bin/activate
pip install -U pip
pip install pybullet pybullet_planning numpy psutil matplotlib
```

---

## ⚙️  Installation & first run

```bash
# 1 Clone repo
git clone https://github.com/<you>/<repo>.git
cd <repo>

# 2 Generate (or regenerate) start/goal dataset
python motion_test.py --regen    # writes benchmark_pairs.json

# 3 Run benchmark head-less (fast CI)
python benchmark_runner.py       # appends 5 rows to CSV

#   Or watch every trajectory in a GUI window
VISUALISE=True python benchmark_runner.py
```

`benchmark_results.csv` schema
```
timestamp,planner,cases,success_rate%,plan_time_s,exec_time_s,
          smoothness,clearance_m,mem_MB
```

---

## 🔄  Reproducibility tips

```python
# Place near the top of benchmark_runner.py
import random, numpy as np
random.seed(42); np.random.seed(42)
import pybullet as p
p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
```
Each subsequent run appends a new timestamped block – no data is overwritten.

---

## 📈  Plotting example

```python
import pandas as pd, matplotlib.pyplot as plt

results = pd.read_csv('benchmark_results.csv')
(results.pivot(columns='planner', values='plan_time_s')
        .plot(kind='bar', rot=0, ylabel='planning time (s)'))
plt.tight_layout(); plt.show()
```

---

## 🛠️  Extending the benchmark

* **Parameter sweeps** – automate loops over `DEFAULT_*` constants.
* **Path post-processing** – add shortcutting/smoothing passes then recompute metrics.
* **Vision-in-the-loop** – use the depth camera in `simEnv.py` for perception-aware planning.

Contributions are welcome – open an issue or PR.

---

## 📜 License

Apache 2.0 – see `LICENSE`.

---

## 🔖 Citation

```bibtex
@misc{panda_planner_benchmark2025,
  author       = {Your Name},
  title        = {A Reproducible Benchmark of Classical Motion–Planning Algorithms on a Franka Panda},
  year         = 2025,
  howpublished = {GitHub},
  url          = {https://github.com/<you>/<repo>}
}
```
