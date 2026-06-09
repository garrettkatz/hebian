import pybullet as p
import pybullet_data
import numpy as np
import time
import os

class RosieEnvironment(object):

    def _load_urdf(self):
        """
        load Rosie's URDF into the simulation and return its ID
        """
        # build path to Rosie URDF relative to this script's location so it works from any directory
        fpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "A-2240-06G.urdf")

        # load robot with base fixed at origin
        robot_id = p.loadURDF(
            fpath,
            basePosition=[0,0,0],
            useFixedBase=True)
        return robot_id

    def __init__(self, show=True):
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

        # load Rosie and store its ID
        self.robot_id = self._load_urdf()
        self.num_joints = p.getNumJoints(self.robot_id)

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
        print(self.actuated_joints)


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

    def close(self):
        """disconnects from PyBullet to avoid memory leaks"""
        p.disconnect()

if __name__ == '__main__':
        env = RosieEnvironment()
        home_pose = np.array([0, -3.5, 1.0, 0, 0.5, 0])
        env.set_position(home_pose)
        env.close()
        

