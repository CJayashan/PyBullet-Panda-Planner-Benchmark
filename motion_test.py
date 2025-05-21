# test_rrt_plan.py — Updated to test 6 reachability points

import pybullet as p
import pybullet_data
import time
import sys
import math
import numpy as np
from pathlib import Path
from pybullet_planning import get_collision_fn
import argparse, json, os, time
from typing import List

# ── Project helpers
ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from utils.simEnv import SimEnv
from utils.motion_planning import *
from utils.panda_sim_grasp_arm import PandaSim

# --------------------------------------------------------------------
parser = argparse.ArgumentParser(
        description="Benchmark Panda planners on fixed start–goal pairs")
parser.add_argument(
        "--regen", action="store_true",
        help="force regeneration of benchmark_pairs.json")
ARGS = parser.parse_args()
# --------------------------------------------------------------------

visual_speed=0.01

def choose_planner():
    menu = """
Select planner:
  [1] RRT
  [2] Lazy-PRM
  [3] PRM
  [4] RRT-Connect
  [5] Straight-line 
→ """
    while True:
        choice = input(menu).strip()
        if choice == "1":
            print("You selected RRT.")
            return plan_motion_rrt
        elif choice == "2":
            print("You selected Lazy-PRM.")
            return plan_motion_lazy_prm
        elif choice == "3":
            print("You selected classic PRM.")
            return plan_motion_prm
        elif choice == "4":
            print("You selected RRT-Connect.")
            return plan_motion_rrt_connect
        elif choice == "5":
            print("You selected Straight-line .")
            return plan_motion_straight
        else:
            print("Invalid input. Choose 1-5.")

def visualize_plan_step_by_step(robot_id, joints, plan, steps_per_conf=30, sleep_time=0.01):
    for i in range(len(plan) - 1):
        current = np.array(plan[i])
        next_conf = np.array(plan[i + 1])

        for step in range(steps_per_conf):
            interp = current + (next_conf - current) * (step / steps_per_conf)
            for j, joint_id in enumerate(joints):
                p.resetJointState(robot_id, joint_id, interp[j])
            p.stepSimulation()
            time.sleep(sleep_time)

def safe_ik(robot_id, ee_link, pos, orn, obstacles,
            yaw_seeds: List[float] = None):
    """Return a joint list that is free of *any* collision; else None."""
    joints  = get_movable_joints(robot_id)
    coll_fn = get_collision_fn(robot_id, joints,
                               obstacles=obstacles, self_collisions=True)

    # default: try current shoulder-yaw only
    if yaw_seeds is None:
        yaw_seeds = [p.getJointState(robot_id, 0)[0]]

    for yaw in yaw_seeds:
        p.resetJointState(robot_id, 0, yaw)        # seed
        q = compute_ik(robot_id, ee_link, pos, orn)
        if q and not coll_fn(q):
            return q
    return None

def get_neutral_conf(robot_id, obstacles):
    """Choose one collision-free pose in free space to use as IK seed."""
    target_pos = [0.0, 0.30, 0.60]                              # 30 cm in front
    target_orn = p.getQuaternionFromEuler([math.pi, 0, 0])     # gripper down
    yaw_list   = np.deg2rad([-30, 0, 30])                      # 3 quick tries
    q = safe_ik(robot_id, 11, target_pos, target_orn,
                obstacles, yaw_list)
    if q:      # found one
        return q
    # fallback – factory demo pose
    return [0, -0.6, 0, -1.8, 0, 2.2, 0.8]


def run_benchmark(p, panda, planner_fun, obstacles,
                  start_confs, goal_confs):
    joints = get_movable_joints(panda.pandaId)
    coll_fn = get_collision_fn(panda.pandaId, joints,obstacles=obstacles, self_collisions=True)

    for i, (qs, qg) in enumerate(zip(start_confs, goal_confs)):
    # ── 0) wait for user ───────────────────────────────────────────────
        input(f"\n[TEST {i}]  — press ENTER to run this case…")

        # ── 1) show defined start & goal xyz ───────────────────────────────
        start_xyz_defined = start_grid[i]            # your grid list
        goal_xyz_defined  = env.PLACE_POINTS[i]      # your shelf list
        print(f"    Start xyz (defined) : {start_xyz_defined}")
        print(f"    Goal  xyz (defined) : {goal_xyz_defined}")

        # ── 2) move wrist to goal_conf *temporarily* to read IK xyz ────────
        joints = get_movable_joints(panda.pandaId)
        for jid, q in zip(joints, qg):
            p.resetJointState(panda.pandaId, jid, q)
        p.stepSimulation()
        wrist_xyz = p.getLinkState(panda.pandaId, 11)[4]

        # displacement
        disp = np.linalg.norm(np.array(wrist_xyz) - np.array(goal_xyz_defined))
        print(f"    Wrist xyz (IK)      : {np.round(wrist_xyz,5).tolist()}")
        print(f"    Δpos (goal-IK)      : {disp:.4f} m")

        # ── 3) collision check on IK pose ──────────────────────────────────
        print(f"    Collision-free IK ? : {'YES' if not coll_fn(qg) else 'NO'}")

        # ── 4) reset to start_conf and run planner (silent) ────────────────
        for jid, q in zip(joints, qs):
            p.resetJointState(panda.pandaId, jid, q)
        p.stepSimulation()

        plan = planner_fun(panda.pandaId, joints, qs, qg, obstacles)
        if plan:
            visualize_plan_step_by_step(panda.pandaId, joints, plan,
                                        sleep_time=visual_speed)
            print("    Planner found the path.")
        else:
            print("    Planner could not find a path.")



if __name__ == '__main__':
    # Connect to PyBullet
    cid = p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    panda = PandaSim(p, [0, -0.6, 0])
    env = SimEnv(p, path="", gripperId=panda.pandaId)

    # add this line ⬇⬇⬇
    obstacles = env.shelf_ids + [env.planeId]

    # ------------- definition of the six pick-up grid points --------------
    start_grid = [
        [-0.1, -0.1, 0.2], [-0.1, 0.0, 0.2], [-0.1, 0.1, 0.2],
        [ 0.1, -0.1, 0.2], [ 0.1, 0.0, 0.2], [ 0.1, 0.1, 0.2],
        ]
# ----------------------------------------------------------------------
    
    # ---------- (Re)build or load benchmark start/goal lists -------------
    PAIR_FILE = "benchmark_pairs.json"
    need_regen = ARGS.regen or not os.path.exists(PAIR_FILE)

    # 1) quick sanity-check: if file exists, does it have the right length?
    if not need_regen:
        with open(PAIR_FILE) as f:
            data = json.load(f)
        expected = min(len(start_grid), len(env.PLACE_POINTS))
        need_regen = (len(data.get("start", [])) != expected)

    if need_regen:
        print("  Generating fresh start/goal joint lists …")
        obstacles  = env.shelf_ids + [env.planeId]
        NEUTRAL    = get_neutral_conf(panda.pandaId, obstacles)
        YAW_LIST   = np.deg2rad([-60,-45,-30,-15,0,15,30,45,60])

        start_confs, goal_confs = [], []
        for sp, gp in zip(start_grid, env.PLACE_POINTS):
            # always reset to neutral before each IK
            for jid, q in zip(get_movable_joints(panda.pandaId), NEUTRAL):
                p.resetJointState(panda.pandaId, jid, q)

            start_q = safe_ik(panda.pandaId, 11, sp,
                              p.getQuaternionFromEuler([math.pi,0,0]),
                              obstacles)
            goal_q  = safe_ik(panda.pandaId, 11, gp,
                              p.getQuaternionFromEuler([0,math.pi/2,0]),
                              obstacles, YAW_LIST)
            if start_q and goal_q:
                start_confs.append(start_q)
                goal_confs.append(goal_q)

        # dump to JSON
        with open(PAIR_FILE, "w") as f:
            json.dump({"start": start_confs, "goal": goal_confs}, f, indent=2)
        print(f"   ✓ Saved {len(start_confs)} pairs to {PAIR_FILE}")
    else:
        with open(PAIR_FILE) as f:
            data = json.load(f)
        start_confs = data["start"]
        goal_confs  = data["goal"]
        print(f"file  Loaded {len(start_confs)} pre-computed pairs from {PAIR_FILE}")
# --------------------------------------------------------------------

    motion_planner = choose_planner()
    run_benchmark(p, panda, motion_planner, obstacles,
              start_confs, goal_confs)
    