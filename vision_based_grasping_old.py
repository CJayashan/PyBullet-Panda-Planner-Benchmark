"""
vision_based_grasping.py
------------------------

Full pipeline:

1) Load a scene with random objects + Franka Panda robot.
2) Render a depth image from an overhead camera.
3) Feed the depth map into a GG-CNN network: get grasp centre, angle, width.
4) Convert best pixel → world coordinates, compute grasp depth.
5) Drive the Panda arm to the grasp pose and close the gripper.
6) Check if the object is lifted above a threshold → success metric.

Author: (your name)
"""

# ── Standard library
from pathlib import Path
import sys
import time
import math

# ── Third-party libs
import numpy as np
import cv2
import torch
import pybullet as p

# ── Project helpers (added to sys.path just once)
ROOT = Path(__file__).resolve().parent        # .../pybullet_grasp
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from utils.simEnv           import SimEnv
import utils.tool           as tool
import utils.panda_sim_grasp_arm as PandaSim
from utils.camera           import Camera
from ggcnn.ggcnn            import GGCNNNet, drawGrasps, getGraspDepth


# ──────────────────────────────────────────────────────────────
# CONSTANTS
# ──────────────────────────────────────────────────────────────
FINGER_L1 = 0.015  # metres
FINGER_L2 = 0.005
SIM_SPEED = 0.2 # 1.0 = real-time, 0.5 = half speed, 2.0 = double speed

# Resolution for GG-CNN (teacher used 300×300)
GGCNN_INPUT_SIZE = 300

# Path to the trained GG-CNN checkpoint (relative is safer)
CKPT_PATH = ROOT / "ggcnn" / "ckpt" / "epoch_0213_acc_0.6374.pth"

# Database folder with object URDFs
DATABASE_PATH = ROOT / "models"

# ──────────────────────────────────────────────────────────────
def run(database_path: Path, start_idx: int, objs_num: int):
    """Main entry: returns (#successful grasps, #total attempts)."""

    # 1. Start physics
    cid   = p.connect(p.GUI)
    panda = PandaSim.PandaSimAuto(p, [-0.2, -0.6, 0])
    env   = SimEnv(p, str(database_path), panda.pandaId)
    camera = Camera()

    # 2. Load network on GPU if present
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] Loading GG-CNN on {device.upper()} …")
    ggcnn = GGCNNNet(str(CKPT_PATH), device=device)

    # 3. Load objects
    env.spawnCubes(start_idx, objs_num)

    success_grasp, sum_grasp, continue_fail = 0, 0, 0
    tt = 5   # render every 5 sim steps

    while True:
        # ------------------------------------------------------
        # a) Let the scene settle (5 seconds)
        for _ in range(240 * 10):
            p.stepSimulation()

        # b) Render + add noise
        depth_img = env.renderCameraDepthImage()
        depth_img = env.add_noise(depth_img)

        # c) Predict grasp with GG-CNN
        r, c, grasp_angle, grasp_width_px = ggcnn.predict(
            depth_img, input_size=GGCNN_INPUT_SIZE
        )

        # d) Convert to world coordinates
        grasp_width = camera.pixels_TO_length(grasp_width_px,
                                              depth_img[r, c])
        grasp_x, grasp_y, grasp_z_cam = camera.img2world([c, r],
                                                         depth_img[r, c])

        # e) Compute grasp depth
        L1_px = camera.length_TO_pixels(FINGER_L1, depth_img[r, c])
        L2_px = camera.length_TO_pixels(FINGER_L2, depth_img[r, c])
        grasp_depth = getGraspDepth(depth_img, r, c, grasp_angle,
                                    grasp_width_px, L1_px, L2_px)
        grasp_z = max(0.7 - grasp_depth, 0)

        # f) Console log
        print("\n" + "*" * 60)
        print(f"Grasp @ world: x={grasp_x:.3f}  y={grasp_y:.3f}  z={grasp_z:.3f}")
        print(f"Angle(rad)={grasp_angle:.2f}  Width(m)={grasp_width:.3f}")
        print("*" * 60)
        #cam = p.getDebugVisualizerCamera()
        #print("\n[DEBUG CAM INFO]")
        #print(f"Distance: {cam[10]}, Yaw: {cam[8]}, Pitch: {cam[9]}, Target: {cam[11]}")

        # g) Visualise grasp rectangle
        im_rgb   = tool.depth2Gray3(depth_img)
        im_grasp = drawGrasps(im_rgb,
                              [[r, c, grasp_angle, grasp_width_px]],
                              mode="line")
        cv2.imshow("Predicted grasp (RGB depth)", im_grasp)
        cv2.waitKey(30)

        # h) Execute grasp (arm go-to + close)  ----------------
        t = 0
        while True:
            p.stepSimulation()
            t += 1
            if t % tt == 0:
                time.sleep((1 / 240.) / SIM_SPEED)
            reached = panda.step([grasp_x, grasp_y, grasp_z],
                                 grasp_angle, grasp_width / 2)
            if reached:
                break

        # i) Success evaluation
        sum_grasp += 1
        lifted = env.evalGraspAndRemove(z_thresh=0.2)
        if lifted:
            success_grasp += 1
            continue_fail = 0
            if env.num_urdf == 0:   # table cleared!
                p.disconnect()
                return success_grasp, sum_grasp
        else:
            continue_fail += 1
            if continue_fail == 5:
                p.disconnect()
                return success_grasp, sum_grasp

        # j) Reset arm pose
        panda.setArmPos([0.5, -0.6, 0.2])

# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    START_IDX  = 0
    OBJS_NUM   = 6

    succ, total = run(DATABASE_PATH, START_IDX, OBJS_NUM)

    print("\n" + "=" * 40)
    print(f"Success rate : {succ}/{total} = {succ/total:.2%}")
    print(f"Cleared objs : {succ}/{OBJS_NUM} = {succ/OBJS_NUM:.2%}")
    
