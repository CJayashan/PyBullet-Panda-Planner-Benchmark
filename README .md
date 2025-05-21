
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

## 🔄  Project flow / pipeline

```text
1. Scene build             2. Pair generation          3. Batch benchmark          4. Analysis
┌───────────────┐          ┌──────────────────────┐     ┌────────────────────┐     ┌────────────────┐
│ simEnv.py     │  ─▶───── │ motion_test.py        │     │ benchmark_runner.py │     │  Jupyter / Excel │
│  • shelf URDF │  saves   │  • collision-free IK  │ --> │  • five planners   │ --> │  visual charts  │
│  • markers    │  env     │  • 6 start/goal pairs │     │  • metrics → CSV  │     │  & reports      │
└───────────────┘          └──────────────────────┘     └────────────────────┘     └────────────────┘
```

1. **Scene build** – `simEnv.py` constructs the two-tier shelf and sets gravity/lighting.  
2. **Pair generation** – `motion_test.py --regen` solves IK for six hand-picked shelf poses and serialises them to **`benchmark_pairs.json`**.  
3. **Batch benchmark** – `benchmark_runner.py` loads those pairs, spawns the Panda, runs each of the five planners, logs metrics to **`benchmark_results.csv`** (GUI optional).  
4. **Analysis** – import the CSV in Python/Excel to graph performance.

➡️ *See* **`METRICS.md`** *for the mathematical definitions of each logged metric.*

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

## ➕ Additional scripts & future work
| File | Folder | What it does | Present use |
|------|--------|-------------|-------------|
| `test_rrt_plan.py` | root | Manual sandbox that spawns the Panda + shelf, runs **only the RRT** planner over a 6×6 grid, and visualises each attempt step-by-step. Great for demos and for checking new start/goal pairs. | Experimental |
| `vision_based_grasping_old.py` | root | Legacy perception pipeline: depth camera → GG-CNN grasp detection → Panda pick-and-place loop. Depends on utils `camera.py` & `tool.py`. | Future vision research |
| `pytourch_verify.py`, misc. | root | One-off experiments (network checks, mesh viewers). | Scratch / ignore for paper |
| `camera.py` | utils/ | Maths helpers to convert depth pixels ⇄ 3‑D rays, intrinsic matrix builder. | Only used by vision scripts. |
| `tool.py` | utils/ | Small point‑cloud & image utilities (`depth2Gray3`, line drawing, etc.). | Only used by vision scripts. |

> **Essential benchmark files:** `motion_planning.py`, `simEnv.py`, `panda_sim_grasp_arm.py`, `motion_test.py`, `benchmark_runner.py`.

---

## 🔧 Benchmark parameters
The **hyper‑parameters** below live in `motion_planning.py` and are applied uniformly to every planner.

| Constant | Applies to | Intuition | Default |
|----------|-----------|-----------|---------|
| `DEFAULT_MAX_ITER` | RRT & RRT‑Connect | Max vertices in the search tree; more ⇒ slower yet more reliable. | **5 000** |
| `DEFAULT_STEP_SIZE` | All planners | Joint‑space interpolation step; smaller samples tight spaces but increases collision checks. | **0.05 rad** |
| `DEFAULT_NUM_SAMPLES` | PRM & Lazy‑PRM | Roadmap density. | **500** |
| `DEFAULT_K` | PRM flavours | k‑nearest neighbour edges. | **10** |

### Runtime metrics (see `METRICS.md`)
`success_rate%`, `plan_time_s`, `exec_time_s`, `smoothness`, `clearance_m`, `mem_MB`.

---

## 🕸️  Sample radar plot (walled shelf)
![Spider plot of planners](docs/spider_walls.png)

Code to regenerate lives in README above.

---

## 🖥️ Requirements
| Package | Tested version |
|---------|----------------|
| Python | 3.9 – 3.11 |
| pybullet | ≥ 3.4 |
| pybullet_planning | ≥ 0.7.2 |
| numpy | ≥ 1.24 |
| psutil | ≥ 5.9 (optional) |
| matplotlib | ≥ 3.8 (optional) |

```bash
python -m venv venv
source venv/bin/activate
pip install -U pip pybullet pybullet_planning numpy psutil matplotlib
```

---

## ⚙️ Installation & first run
```bash
git clone https://github.com/<you>/<repo>.git
cd <repo>

python motion_test.py --regen      # build start/goal pairs
python benchmark_runner.py         # append 5 rows to CSV
# VISUALISE=True python benchmark_runner.py  # watch in GUI
```

---

## 🔄 Reproducibility
```python
random.seed(42); np.random.seed(42)
p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
```

---

## 🛠️ Extending
Parameter sweeps, path post‑processing, vision‑in‑the‑loop.

---

## 📜 License
Apache 2.0

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
