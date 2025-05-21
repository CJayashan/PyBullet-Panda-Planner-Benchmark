import time
import numpy as np
import math
from pathlib import Path 

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
# restposes for null space
jointPositions = (
    1.57,   # joint 1
   -0.5,   # joint 2
    0.0,   # joint 3
   -2.0,   # joint 4
    0.0,   # joint 5
    1.8,   # joint 6
    0.6,   # joint 7
    0.02,
    0.02
)
rp = jointPositions

class PandaSim(object):
    def __init__(self, bullet_client, offset):
        self.p = bullet_client
        self.p.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)
        
        flags = self.p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        orn=[0, 0, 0, 1]
        urdf_root = Path(__file__).resolve().parent.parent / "franka_panda"
        urdf_path = urdf_root / "panda_1.urdf"   # or "panda.urdf" if that’s what you have
        self.pandaId = self.p.loadURDF(
            str(urdf_path), np.array([0,0,0])+self.offset,
            orn, useFixedBase=True, flags=flags)
        index = 0
        self.state = 0
        self.control_dt = 1./240.
        self.finger_target = 0
        self.gripper_height = 0.2
        #create a constraint to keep the fingers centered
        c = self.p.createConstraint(self.pandaId,
                          9,
                          self.pandaId,
                          10,
                          jointType=self.p.JOINT_GEAR,
                          jointAxis=[1, 0, 0],
                          parentFramePosition=[0, 0, 0],
                          childFramePosition=[0, 0, 0])
        self.p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
    
        for j in range(self.p.getNumJoints(self.pandaId)):
            self.p.changeDynamics(self.pandaId, j, linearDamping=0, angularDamping=0)
            info = self.p.getJointInfo(self.pandaId, j)
            #print("info=",info)
            jointName = info[1]
            jointType = info[2]
            if (jointType == self.p.JOINT_PRISMATIC):
                self.p.resetJointState(self.pandaId, j, jointPositions[index]) 
                index=index+1

            if (jointType == self.p.JOINT_REVOLUTE):
                self.p.resetJointState(self.pandaId, j, jointPositions[index]) 
                index=index+1
        self.t = 0.

    def calcJointLocation(self, pos, orn):
        """
        Calculate joint positions of the robotic arm based on pos and orn (target position and orientation)
        Returns: list of joint angles for all joints
        """
        # Calculate inverse kinematics solution
        # self.pandaId: unique ID of the robot
        # pandaEndEffectorIndex: index of the end-effector link
        # pos: target position [x,y,z]
        # orn: target orientation as quaternion
        # ll, ul: lower/upper joint limits
        # jr: joint ranges
        # rp: rest poses (preferred joint angles)
        # maxNumIterations: maximum iterations for IK solver
        jointPoses = self.p.calculateInverseKinematics(self.pandaId, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=20)
        return jointPoses

    def setArmPos(self, pos):
        # Convert Euler angles [pi,0,pi/2] to quaternion for end-effector orientation
        # pi around x-axis: gripper points downward
        # 0 around y-axis: no rotation
        # pi/2 around z-axis: base orientation
        orn = self.p.getQuaternionFromEuler([math.pi,0.,math.pi / 2])
        
        # Calculate joint angles using inverse kinematics
        jointPoses = self.calcJointLocation(pos, orn)
        
        # Apply the calculated joint angles to the robot
        self.setArm(jointPoses)

    def setArm(self, jointPoses, maxVelocity=10):
        """
        Set robotic arm joint positions using position control
        Args:
            jointPoses: list of target joint angles
            maxVelocity: maximum joint velocity (default=10)
        """
        # Iterate through all 7 joints of the Panda arm
        for i in range(pandaNumDofs):   # pandaNumDofs = 7
            # Control each joint using position control
            # self.pandaId: robot ID
            # i: joint index
            # POSITION_CONTROL: control mode
            # jointPoses[i]: target joint angle
            # force: maximum force (5 * 240)
            # maxVelocity: maximum joint velocity
            self.p.setJointMotorControl2(self.pandaId, i, self.p.POSITION_CONTROL, 
                                    jointPoses[i], force=5 * 240., maxVelocity=maxVelocity)
        
    def setGripper(self, finger_target):
        """
        Set gripper finger positions
        Args:
            finger_target: target width between fingers
        """
        # Control both finger joints (joints 9 and 10)
        for i in [9,10]:
            # Position control for gripper fingers
            # force=20: maximum force for gripping
            self.p.setJointMotorControl2(self.pandaId, i, self.p.POSITION_CONTROL, 
                                    finger_target, force=20)

    def step(self, pos, angle, gripper_w):
        """
        Execute one step of the pick-and-place state machine
        Args:
            pos: [x, y, z] End-effector target position in world coordinates
            angle: gripper rotation angle in radians
            gripper_w: target gripper width
        Returns:
            bool: True if sequence completed, False otherwise
        """
        # Update internal state machine
        self.update_state()
        
        # Offset in z-direction to account for gripper geometry
        pos[2] += 0.048
        
        # State machine implementation
        if self.state == -1:
            # Invalid state, do nothing
            pass

        elif self.state == 0:
            # Initial position state
            pos[2] = 0.2  # Set safe height
            # Calculate orientation quaternion
            orn = self.p.getQuaternionFromEuler([math.pi,0.,angle + math.pi / 2])
            # Calculate and apply joint positions
            jointPoses = self.calcJointLocation(pos, orn)
            self.setArm(jointPoses)
            # Set initial gripper opening
            self.setGripper(gripper_w)
            return False

        elif self.state == 1:
            # Pre-grasp position above object
            pos[2] += 0.05  # Add height offset
            orn = self.p.getQuaternionFromEuler([math.pi,0.,angle + math.pi / 2])
            jointPoses = self.calcJointLocation(pos, orn)
            self.setArm(jointPoses)
            return False

        elif self.state == 2:
            # Grasp position - move to object
            orn = self.p.getQuaternionFromEuler([math.pi,0.,angle + math.pi / 2])
            jointPoses = self.calcJointLocation(pos, orn)
            # Slower movement for precise positioning
            self.setArm(jointPoses, maxVelocity=3)
            return False

        elif self.state == 3:
            # Close gripper to grasp object
            self.setGripper(0)  # Fully close gripper
            return False
        
        elif self.state == 4:
            # Lift object slightly (pre-lift position)
            pos[2] += 0.05  # Small lift
            orn = self.p.getQuaternionFromEuler([math.pi,0.,angle + math.pi / 2])
            jointPoses = self.calcJointLocation(pos, orn)
            # Slow movement while holding object
            self.setArm(jointPoses, maxVelocity=0.5)
            return False
        
        elif self.state == 5:
            # Move to higher position
            pos[2] = 0.3  # Set final height
            orn = self.p.getQuaternionFromEuler([math.pi,0.,angle + math.pi / 2])
            jointPoses = self.calcJointLocation(pos, orn)
            # Continue slow movement with object
            self.setArm(jointPoses, maxVelocity=0.5)
            return False

        elif self.state == 12:
            # Final state - reset everything
            self.reset()
            return True  # Sequence completed

    def reset(self):
        """
        Reset all state variables to initial values
        """
        self.state = 0      # Reset state machine
        self.state_t = 0    # Reset state timer
        self.cur_state = 0  # Reset current state tracker


class PandaSimAuto(PandaSim):
    def __init__(self, bullet_client, offset):
        PandaSim.__init__(self, bullet_client, offset)
        self.state_t = 0
        self.cur_state = 0
        self.states = [0, 1, 2, 3, 4, 5, 12]
        self.state_durations = [1.0, 0.5, 2.0, 0.5, 1.0, 1.0, 0.5]
    
    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]
