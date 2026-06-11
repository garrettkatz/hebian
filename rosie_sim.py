import pybullet as p
import pybullet_data
import numpy as np
import time
import os

class RosieEnvironment(object):

    def _load_urdf(self, use_self_collision=False):
        """
        load Rosie's URDF into the simulation and return its ID
        """
        # build path to Rosie URDF relative to this script's location so it works from any directory
        fpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "A-2240-06G.urdf")

        # load robot with base fixed at origin
        # enable self-collision detection if requested, otherwise load normally
        flags = p.URDF_USE_SELF_COLLISION if use_self_collision else 0
        robot_id = p.loadURDF(
            fpath,
            basePosition=[0,0,0],
            useFixedBase=True,
            flags=flags
            )
        return robot_id

    def __init__(self, use_self_collision=False, show=True):
        """
        set up the PyBullet simulation
        load the ground plane and Rosie
        query joint info
        """
        # connect to PyBullet with or without the GUI
        p.connect(p.GUI if show else p.DIRECT)
        # tell PyBullet where to find built-in assets like plane.urdf
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        # load flat ground plane
        p.loadURDF("plane.urdf")

        # load Rosie's URDF, passing self-collision flag if enabled
        self.robot_id = self._load_urdf(use_self_collision)
        self.num_joints = p.getNumJoints(self.robot_id)

        # enable collision detection between every pair of links on the same body
        if use_self_collision:
            for i in range(p.getNumJoints(self.robot_id)):
                for j in range(p.getNumJoints(self.robot_id)):
                    p.setCollisionFilterPair(self.robot_id, self.robot_id, i, j, 1)

        # containers for joint info
        self.joint_name = []
        self.joint_index = {}
        self.joint_fixed = []
        
        # query and store info for each joint
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode("UTF-8")
            self.joint_name.append(name)
            self.joint_index[name] = i
            # True if fixed joint, False if actuated
            self.joint_fixed.append(info[2] == p.JOINT_FIXED)

        # store indices of actuated joints only
        self.actuated_joints = [i for i, name in enumerate(self.joint_name) if name.startswith('J') and self.joint_fixed[i] == False]

        # get index of end effector
        self.end_effector_link = self.joint_index[self.joint_name[-1]]
        
        self.home_pose = np.array([0, -3.5, 1.0, 0, 0.5, 0])


    def get_position(self):
        """
        return current angles of all actuated joints as a numpy array
        """
        # get state tuples for all actuated joints
        joint_states = p.getJointStates(self.robot_id, self.actuated_joints)
        # extract just the position (index 0) from each state tuple
        joint_positions = np.array([joint_states[i][0] for i in range(len(joint_states))])
        return joint_positions
    
    def set_position(self, angles):
        """snap all actuated joints instantly to the given angle array (in radians)"""
        # teleport each actuated joint to its target angle
        for joint_idx, angle in zip(self.actuated_joints, angles):
            p.resetJointState(self.robot_id, joint_idx, angle)
    
    def goto_position(self, target, duration=5.):
        """
        move the arm in a smooth trajectory
        from its current pose to target (in radians)
        over given duration in seconds
        """
        current = self.get_position()
        # number of timesteps for given duration
        num_steps = int(duration * 240)
        # weights go from 0 to 1 across all steps
        weights = np.linspace(0, 1, num_steps).reshape(-1, 1)
        # interpolate between current and target to get full trajectory
        trajectory = weights * target + (1 - weights) * current

        # execute trajectory one step at a time
        for row in trajectory:
            self.set_position(row)
            p.stepSimulation()
            time.sleep(1/240)

    def get_ik(self, target_position):
        """takes an (x, y, z) and returns joint angles"""
        # build full 25-joint rest pose array with home pose values at actuated joint indices
        restPoses = [0] * 25
        for joint_idx, angle in zip(self.actuated_joints, self.home_pose):
            restPoses[joint_idx] = angle

        # IK outputs all 25 joint angles
        ik_joints = p.calculateInverseKinematics(self.robot_id, self.end_effector_link, target_position, restPoses)
        # IK returns all 6 actuated joint angles
        return [ik_joints[i] for i in range(len(self.actuated_joints))]

    def check_collision(self):
        """returns True if any collision is detected"""
        for c in p.getContactPoints(self.robot_id):
            # ignore the base resting on floor, this is normal and not a real collision
            if c[1] == 1 and c[2] == 0 and c[3] == 0 and c[4] == -1:
                continue
            return True
        return False

    def close(self):
        """disconnects from PyBullet to avoid memory leaks"""
        p.disconnect()

if __name__ == '__main__':
        # env1 uses default collision detection (floor only)
        env1 = RosieEnvironment()
        rng = np.random.default_rng()

        # iterates through 10 IK poses, detects floor collision
        for i in range(10):
            random_position = rng.random(3)
            angles = env1.get_ik(random_position)
            env1.set_position(angles)
            for _ in range(100):
                p.stepSimulation()
                time.sleep(1./240.)
            print(f"Target position: {random_position}")
            if env1.check_collision():
                print("Collision Detected")
            else:
                print("No collision")
            input("Press Enter to continue")
        env1.close()

        # env2 uses URDF_USE_SELF_COLLISION to detect arm self-collisions
        env2 = RosieEnvironment(use_self_collision=True)
        # hardcoded poses that trigger base and self-collisions
        hardcoded_poses = [
            # base collision
            np.array([-1.15856693, -2.30733897, -3.02441003, 2.19252047, 0.30061921, 0.53673647]),
            # self-collision
            np.array([-1.52050829,  2.42421686, -1.00946527, -1.7535704, 0.76910796, 0.89692566])
        ]

        # iterates through hardcoded poses, detects base/self-collisions
        for pose in hardcoded_poses:
            env2.set_position(pose)
            for _ in range(100):
                p.stepSimulation()
                time.sleep(1./240.)
            print(f"Joint angles: {pose}")
            if env2.check_collision():
                print("Collision Detected")
            else:
                print("No collision")
            input("Press Enter to continue")
        env2.close()
