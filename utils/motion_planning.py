import pybullet as p
import math
import time
import numpy as np
import heapq

from pybullet_planning import (
    plan_joint_motion,
    set_joint_positions,
    get_movable_joints,
    get_link_pose,
    get_sample_fn,
    get_collision_fn,
    get_extend_fn,
    get_distance_fn,
    BodySaver)

from pybullet_planning.motion_planners.rrt_connect import birrt as _rrt_connect

# Hyper-parameters to keep every planner honest
DEFAULT_MAX_ITER   = 5_000     # for RRT / RRT-Connect
DEFAULT_STEP_SIZE  = 0.05      # rad – for straight steps & RRT steer
DEFAULT_NUM_SAMPLES = 300      # for both PRM flavours
DEFAULT_K          = 8        # k-nearest neighbours



# Function to get the current joint configuration of the robot
def get_current_joint_positions(robot_id):
    joints = get_movable_joints(robot_id)
    current_conf = [p.getJointState(robot_id, j)[0] for j in joints]
    return joints, current_conf

# Define the target pose (position + orientation) for the end effector
def define_goal_pose(position, euler_angles=[3.14, 0, 0]):
    """
    Converts a target position and Euler angles into a quaternion pose.

    Args:
        position (list): [x, y, z] position in world coordinates
        euler_angles (list): [roll, pitch, yaw] in radians

    Returns:
        tuple: (position, orientation quaternion)
    """
    orientation = p.getQuaternionFromEuler(euler_angles)
    return position, orientation

# Compute inverse kinematics to get target joint configuration
def compute_ik(robot_id, end_effector_link, target_pos, target_orn):
    return p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=end_effector_link,
        targetPosition=target_pos,
        targetOrientation=target_orn,
        maxNumIterations=200,
        residualThreshold=1e-4
    )


# Plan a motion path using RRT from current to goal configuration

def plan_motion_rrt(robot_id, joints, start_conf, goal_conf,
                    obstacles=None,
                    max_iterations: int = DEFAULT_MAX_ITER,
                    step_size:     float = DEFAULT_STEP_SIZE):
    
    collision_fn = get_collision_fn(robot_id, joints, obstacles=obstacles,
                                    self_collisions=True,  
                                    distance_threshold=0.0)

    # force the robot to start pose so planner uses it as the current joint state
    for i, j in enumerate(joints):
        p.resetJointState(robot_id, j, start_conf[i])

    return plan_joint_motion(
        robot_id,
        joints,
        goal_conf,
        obstacles=obstacles,
        self_collisions=True,max_iterations=max_iterations, step_size=step_size
    )

# Execute a motion plan by stepping through the configurations
def execute_motion_plan(robot_id, joints, plan, sleep_time=1/240.):
    """
    Execute the planned motion by iterating through joint configurations.

    Args:
        robot_id (int): Robot body ID
        joints (list): Movable joint indices
        plan (list): List of joint configurations
        sleep_time (float): Time delay between steps (default: real-time)
    """
    for conf in plan:
        set_joint_positions(robot_id, joints, conf)
        p.stepSimulation()
        time.sleep(sleep_time)


def get_distance_fn(robot_id, joints):
    # Returns a function that computes Euclidean distance in joint space
    def distance_fn(q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))
    return distance_fn





def plan_motion_lazy_prm(robot_id, joints, start_conf, goal_conf, obstacles=[], 
                         num_samples=DEFAULT_NUM_SAMPLES, k=DEFAULT_K):
    # 1. Setup collision checker and sampling
    collision_fn = get_collision_fn(robot_id, joints, obstacles=obstacles, self_collisions=True,
                                    distance_threshold=0.0)
    sample_fn = get_sample_fn(robot_id, joints)
    distance_fn = get_distance_fn(robot_id, joints)

    # 2. Sample valid nodes (adapted from pyprm logic)
    roadmap = []
    while len(roadmap) < num_samples:
        q = sample_fn()
        if not collision_fn(q):
            roadmap.append(q)

    # 3. Add start and goal to roadmap (standard approach)
    
    if collision_fn(start_conf):
        print("[FAIL] start_conf is in collision")
        return None

    roadmap.append(start_conf)
    start_idx = len(roadmap) - 1

    roadmap.append(goal_conf)
    goal_idx = len(roadmap) - 1

    # 4. Build roadmap edges using K-NN (inspired by pyprm)
    def neighbors(i):
        dists = [(distance_fn(roadmap[i], roadmap[j]), j) for j in range(len(roadmap)) if j != i]
        return [j for _, j in sorted(dists)[:k]]

    graph = {i: neighbors(i) for i in range(len(roadmap))}
    checked_edges = {}

    # 5. Lazy collision-check only when edge is used (core Lazy PRM logic)
    def lazy_edge_valid(i, j):
        if (i, j) in checked_edges:
            return checked_edges[(i, j)]
        path_clear = not edge_in_collision(roadmap[i], roadmap[j])
        checked_edges[(i, j)] = path_clear
        checked_edges[(j, i)] = path_clear
        return path_clear

    # 6. Check collision along interpolated edge (adapted from pyprm edge checker)
    def edge_in_collision(q1, q2, steps=20):
        for alpha in np.linspace(0, 1, steps):
            q_interp = (1 - alpha) * np.array(q1) + alpha * np.array(q2)
            if collision_fn(q_interp.tolist()):
                return True
        return False

    # 7. Lazy Dijkstra Search (inspired by pyprm lazy evaluation strategy)
    def dijkstra_lazy(start_idx, goal_idx):
        queue = [(0, start_idx, [])]
        visited = set()

        while queue:
            cost, current, path = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            path = path + [current]

            if current == goal_idx:
                return [roadmap[i] for i in path]

            for neighbor in graph[current]:
                if neighbor not in visited:
                    if lazy_edge_valid(current, neighbor):
                        next_cost = cost + distance_fn(roadmap[current], roadmap[neighbor])
                        heapq.heappush(queue, (next_cost, neighbor, path))
        return None

    # 8. Run the search
    print("[INFO] Running Lazy PRM search...")
    result = dijkstra_lazy(start_idx, goal_idx)

    if result:
        print("[SUCCESS] Lazy PRM path found.")
    else:
        print("[FAIL] No path found with Lazy PRM.")

    return result


def plan_motion_prm(robot_id, joints, start_conf, goal_conf,
                    obstacles=None, num_samples=DEFAULT_NUM_SAMPLES, k=DEFAULT_K):
    """
    Straight (non-lazy) PRM:
    • samples 'num_samples' collision-free joint sets
    • builds k-nearest-neighbour edges *after* checking every edge
    • Dijkstra to find the shortest path
    """
    if obstacles is None:
        obstacles = []

    collision_fn = get_collision_fn(robot_id, joints,
                                    obstacles=obstacles, self_collisions=True)
    sample_fn    = get_sample_fn(robot_id, joints)
    distance_fn  = lambda q1, q2: np.linalg.norm(np.array(q1) - np.array(q2))

    # 1. sample nodes
    roadmap = []
    while len(roadmap) < num_samples:
        q = sample_fn()
        if not collision_fn(q):
            roadmap.append(q)

    # 2. add start & goal
    if collision_fn(start_conf) or collision_fn(goal_conf):
        print("[PRM] Start or goal in collision — aborting")
        return None
    roadmap += [start_conf, goal_conf]
    start_idx, goal_idx = len(roadmap) - 2, len(roadmap) - 1

    # 3. connect k-NN with *eager* edge checking
    def edge_free(q1, q2, steps=20):
        for a in np.linspace(0, 1, steps):
            q = (1 - a) * np.array(q1) + a * np.array(q2)
            if collision_fn(q.tolist()):
                return False
        return True

    graph = {i: [] for i in range(len(roadmap))}
    for i in range(len(roadmap)):
        # k nearest by joint-space distance
        dists = sorted(((distance_fn(roadmap[i], roadmap[j]), j)
                        for j in range(len(roadmap)) if j != i))[:k]
        for _, j in dists:
            if edge_free(roadmap[i], roadmap[j]):
                graph[i].append(j)
                graph[j].append(i)

    # 4. Dijkstra search
    queue = [(0, start_idx, [])]
    visited = set()
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        path = path + [node]
        if node == goal_idx:
            return [roadmap[i] for i in path]
        for nbr in graph[node]:
            if nbr not in visited:
                heapq.heappush(queue,
                               (cost + distance_fn(roadmap[node], roadmap[nbr]),
                                nbr, path))
    return None   # no connection found


def plan_motion_rrt_connect(robot_id, joints, start_conf, goal_conf,
                            obstacles=None, max_iterations: int = DEFAULT_MAX_ITER,
                            step_size:     float = DEFAULT_STEP_SIZE):
    """Wrap pybullet_planning's RRT-Connect (BiRRT)."""
    if obstacles is None:
        obstacles = []
    collision_fn = get_collision_fn(robot_id, joints,
                                    obstacles=obstacles, self_collisions=True)
    sample_fn    = get_sample_fn(robot_id, joints)
    distance_fn  = get_distance_fn(robot_id, joints)
    extend_fn    = get_extend_fn(robot_id,joints)      # default step function -- modified from None
    return _rrt_connect(start_conf, goal_conf, distance_fn,
                        sample_fn, extend_fn, collision_fn, max_iterations=max_iterations,step_size=step_size)



def plan_motion_straight(robot_id, joints, start_conf, goal_conf,
                         obstacles=None, step=0.05):
    """
    Straight-line joint-space planner.
    • step = max joint change per interpolation step (rad).
    • returns list [start, …, goal] or None if any pose collides.
    """
    if obstacles is None:
        obstacles = []
    collision_fn = get_collision_fn(robot_id, joints,
                                    obstacles=obstacles, self_collisions=True)

    # How many linear steps?
    diff = np.abs(np.array(goal_conf) - np.array(start_conf))
    n    = int(np.ceil(max(diff) / step)) + 1
    path = []
    for a in np.linspace(0, 1, n):
        q = (1 - a) * np.array(start_conf) + a * np.array(goal_conf)
        q = q.tolist()
        if collision_fn(q):
            return None              # blocked → give up
        path.append(q)
    return path