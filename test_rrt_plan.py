# test_rrt_plan.py — Updated to test 6 reachability points

import pybullet as p
import pybullet_data
import time
import sys
import math
import numpy as np
from pathlib import Path

# ── Project helpers
ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from utils.simEnv import SimEnv
from utils.motion_planning import *
from utils.panda_sim_grasp_arm import PandaSim

visual_speed=0.01

"""
if __name__ == '__main__':
    # Connect to PyBullet
    cid = p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load robot and environment
    panda = PandaSim(p, [0, -0.6, 0])
    env = SimEnv(p, path="", gripperId=panda.pandaId)
    obstacles = env.shelf_ids + [env.planeId]

    reachable_points = []

    for label, target_pos in test_points:
        print(f"\n[INFO] Testing {label} point at {target_pos}")

        joints, current_conf = get_current_joint_positions(panda.pandaId)

        # Define goal pose for testing
        pos, orn = define_goal_pose(target_pos, euler_angles=[0, math.pi / 2, 0])
        goal_conf = compute_ik(panda.pandaId, 11, pos, orn)

        if goal_conf is None:
            print("[ERROR] No IK solution found.")
            continue

        # Check for collision
        collision_fn = get_collision_fn(panda.pandaId, joints, obstacles=obstacles)
        if collision_fn(goal_conf):
            print("[WARNING] IK solution is in collision.")
            continue

        # Plan motion using RRT
        plan = plan_motion_rrt(panda.pandaId, joints, goal_conf, obstacles=obstacles)

        if plan:
            print("[SUCCESS] Motion plan found.")
            reachable_points.append((label, target_pos))
        else:
            print("[FAIL] Motion planning failed.")

    # Print summary of reachable points
    print("\n========== Reachable Points Summary ==========")
    for label, pt in reachable_points:
        print(f"{label} → {pt}") """

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

def test_rrt_from_grid_to_shelf(p, panda, env):
    obstacles = env.shelf_ids + [env.planeId]

    start_grid = [
        [-0.1, -0.1, 0.2], [-0.1, 0.0, 0.2], [-0.1, 0.1, 0.2],
        [ 0.1, -0.1, 0.2], [ 0.1, 0.0, 0.2], [ 0.1, 0.1, 0.2],
    ]
    goal_points = env.PLACE_POINTS.copy()

    start_idx = 0
    goal_idx = 0

    while start_idx < len(start_grid) and goal_idx < len(goal_points):
        start_pos = start_grid[start_idx]
        goal_pos = goal_points[goal_idx]
        start_orn = p.getQuaternionFromEuler([math.pi, 0, 0])            # Gripper facing down
        goal_orn = p.getQuaternionFromEuler([0, math.pi / 2, 0])         # Gripper parallel to shelf

        print(f"\n[TEST] Start {start_idx}: {start_pos} → Goal {goal_idx}: {goal_pos}")

        # Set robot to start pose using IK
        start_conf = compute_ik(panda.pandaId, 11, start_pos, start_orn )
        if start_conf is None:
            print(f"[SKIP] Invalid IK for start pose {start_idx}")
            start_idx += 1
            continue

        joint_ids = get_movable_joints(panda.pandaId)
        for i, j in enumerate(joint_ids):
            p.resetJointState(panda.pandaId, j, start_conf[i])
        p.stepSimulation()
        time.sleep(0.2)

        # IK for goal
        goal_conf = compute_ik(panda.pandaId, 11, goal_pos, goal_orn)
        for i, j in enumerate(joint_ids):
            p.resetJointState(panda.pandaId, j, goal_conf[i])
        p.stepSimulation()
        ee_pos, ee_orn = p.getLinkState(panda.pandaId, 11)[4:6]
        print("IK Target Pose:", goal_pos)
        print("Actual Wrist Pose:", ee_pos)
        if goal_conf is None:
            print(f"[SKIP] Invalid IK for goal pose {goal_idx}")
            goal_idx += 1
            continue

        # Check EE deviation
        joint_ids = get_movable_joints(panda.pandaId)
        for i, j in enumerate(joint_ids):
            p.resetJointState(panda.pandaId, j, goal_conf[i])
        p.stepSimulation()

        ee_pos, _ = p.getLinkState(panda.pandaId, 11)[4:6]
        ee_error = np.linalg.norm(np.array(ee_pos) - np.array(goal_pos))
        # Blue sphere at desired goal position
        goal_marker = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[0, 0, 1, 1])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_marker, basePosition=goal_pos)

        # Yellow sphere at actual EE position
        ee_marker = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 1, 0, 1])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=ee_marker, basePosition=ee_pos)

        # Red line between goal and EE position
        p.addUserDebugLine(goal_pos, ee_pos, lineColorRGB=[1, 0, 0], lineWidth=2.0, lifeTime=3)
        
        # Pause for visual inspection
        input("[DEBUG] Press ENTER to continue to next test...")

        if ee_error > 0.03:
            print(f"[SKIP] Goal EE deviation too large ({ee_error:.4f} m).")
            goal_idx += 1
            continue

        # Plan motion
        joints, _ = get_current_joint_positions(panda.pandaId)
        plan = plan_motion_rrt(panda.pandaId, joints, goal_conf, obstacles=obstacles)

        if plan:
            print(f"[SUCCESS] Planned path from grid {start_idx} to shelf {goal_idx}")
            visualize_plan_step_by_step(panda.pandaId, joints, plan, sleep_time=visual_speed)
            time.sleep(1)
            start_idx += 1
            goal_idx += 1
        else:
            print(f"[FAIL] Planning failed from grid {start_idx} to shelf {goal_idx}")
            goal_idx += 1



if __name__ == '__main__':
    # Connect to PyBullet
    cid = p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())


    panda = PandaSim(p, [0, -0.6, 0])
    env = SimEnv(p, path="", gripperId=panda.pandaId)

    test_rrt_from_grid_to_shelf(p, panda, env)
    

    """# Load robot and environment
    panda = PandaSim(p, [0, -0.6, 0])
    env = SimEnv(p, path="", gripperId=panda.pandaId)
    obstacles = env.shelf_ids + [env.planeId]

    # Get movable joints
    joints, _ = get_current_joint_positions(panda.pandaId)

    for i, target_pos in enumerate(env.PLACE_POINTS):
        print(f"\n[INFO] Testing target {i}: {target_pos}")

        # Define goal pose for end-effector
        pos, orn = define_goal_pose(target_pos, euler_angles=[0, math.pi / 2, 0])
        goal_conf = compute_ik(panda.pandaId, 11, pos, orn)
        if goal_conf is None:
            print("[ERROR] No IK solution found.")
            continue

        # Apply IK pose temporarily to evaluate actual gripper placement
        set_joint_positions(panda.pandaId, joints, goal_conf)
        p.stepSimulation()
        time.sleep(0.05)

        # Measure deviation between target and actual wrist pose
        ee_link_index = 11
        ee_pos, _ = p.getLinkState(panda.pandaId, ee_link_index)[4:6]
        ee_error = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))

        # Visual markers
        sphere_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 1, 0, 1])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_id, basePosition=ee_pos)
        p.addUserDebugText("EE", ee_pos, textColorRGB=[1, 1, 0], lifeTime=5)
        p.addUserDebugText("Target", target_pos, textColorRGB=[0, 0, 1], lifeTime=5)

        if ee_error > 0.03:  # 3 cm threshold
            print(f"[SKIP] IK deviation too large ({ee_error:.4f} m) — discarding target {i}")
            continue

        # Plan motion to the goal joint config
        plan = plan_motion_rrt(panda.pandaId, joints, goal_conf, obstacles=obstacles)
        if plan:
            print(f"[SUCCESS] Target {i} reachable. Executing plan...")
            execute_motion_plan(panda.pandaId, joints, plan, sleep_time=1/10.)
            time.sleep(1)
        else:
            print(f"[FAIL] Could not find valid motion plan for target {i}")"""
    

    """
The Answer: It’s not off — it’s an illusion caused by link geometry
What you're observing is a visual mismatch between:

The origin of link 11 (used in compute_ik and getLinkState)

And the geometry of the robot mesh, which is not centered at that origin

Why?
Link 11’s visual mesh (CAD model) is offset forward from its frame origin

The frame is inside the wrist joint, but the mesh extends forward into the palm

So when link 11 is precisely positioned at the yellow dot...
→ the physical-looking gripper appears offset!

    """