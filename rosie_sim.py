import pybullet as p
import pybullet_data
import numpy as np
import hebi
import time
import os

class RosieEnvironment(object):

    def _load_urdf(self, use_self_collision=False):
        """
        load Rosie's URDF into the simulation and return its ID
        """
        # build path to Rosie URDF relative to this script's location so it works from any directory
        fpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "A-2240-06G.urdf")
        hrdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "A-2240-06G.hrdf")

        # enable self-collision detection if requested, otherwise load normally
        flags = p.URDF_USE_SELF_COLLISION if use_self_collision else 0
        robot_id = p.loadURDF(
            fpath,
            basePosition=[0, 0, 0],
            useFixedBase=True,
            flags=flags
        )

        self.arm_model = hebi.robot_model.import_from_hrdf(hrdf_path)

        return robot_id

    def __init__(self, use_self_collision=False, show=True, spawn_block=False):
        """
        set up the PyBullet simulation
        load the ground plane and Rosie
        query joint info
        """
        p.connect(p.GUI if show else p.DIRECT)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=50,
            cameraPitch=0,
            cameraTargetPosition=[0.2, 0, 0.1]
        )
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        self.robot_id = self._load_urdf(use_self_collision)
        self.num_joints = p.getNumJoints(self.robot_id)

        # disable gripper-internal collisions to avoid false positives
        GRIPPER_LINKS = list(range(12, 25))
        if use_self_collision:
            for i in GRIPPER_LINKS:
                for j in GRIPPER_LINKS:
                    p.setCollisionFilterPair(self.robot_id, self.robot_id, i, j, 0)

        # query and store joint info
        self.joint_name = []
        self.joint_index = {}
        self.joint_fixed = []

        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode("UTF-8")
            self.joint_name.append(name)
            self.joint_index[name] = i
            self.joint_fixed.append(info[2] == p.JOINT_FIXED)

        # store indices of actuated joints only (named 'J1' through 'J6')
        self.actuated_joints = [i for i, name in enumerate(self.joint_name) if name.startswith('J') and not self.joint_fixed[i]]

        # get index of end effector
        self.end_effector_link = self.joint_index[self.joint_name[-1]]

        # spawn block at known position
        if spawn_block:
            block_pos = [0.2717, -0.0342, 0.02]
            self.block_id = p.loadURDF("cube_small.urdf", basePosition=block_pos)
            p.changeDynamics(self.block_id, -1, lateralFriction=1000.0)
            p.changeDynamics(self.robot_id, 16, lateralFriction=1000.0)
            p.changeDynamics(self.robot_id, 20, lateralFriction=1000.0)

        self.home_pose = np.array([0.36322123, -4.00691022, -4.80736215, -5.51284089, -1.57079638, 1.9340176])

    def get_position(self):
        """return current angles of all actuated joints as a numpy array"""
        joint_states = p.getJointStates(self.robot_id, self.actuated_joints)
        return np.array([joint_states[i][0] for i in range(len(joint_states))])

    def set_position(self, angles):
        """snap all actuated joints instantly to the given angle array (in radians)"""
        for joint_idx, angle in zip(self.actuated_joints, angles):
            p.resetJointState(self.robot_id, joint_idx, angle)

    def reset_to_home(self, settle_steps=100):
        """snap arm back to home pose and let the sim settle, used at the start of each episode"""
        self.set_position(self.home_pose)
        for _ in range(settle_steps):
            p.stepSimulation()

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

        # using position control
        for row in trajectory:
            p.setJointMotorControlArray(
                self.robot_id,
                jointIndices=self.actuated_joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=row,
                targetVelocities=[0] * len(self.actuated_joints),
                positionGains=[0.25] * len(self.actuated_joints),
            )
            p.stepSimulation()
            time.sleep(1 / 240)

    def set_joint_targets(self, target_angles, position_gain=0.25):
        """apply joint position targets for ONE control update, no internal loop"""
        p.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.actuated_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_angles,
            targetVelocities=[0] * len(self.actuated_joints),
            positionGains=[position_gain] * len(self.actuated_joints)
        )

    def get_ik(self, target_position, target_orientation=None):
        """takes an (x, y, z) for the actual gripper/finger position and returns joint angles"""
        initial_positions = np.array(self.get_position(), dtype=float)

        if target_orientation is None:
            target_orientation = np.array([
                [ 9.99999932e-01, -5.23054545e-05, -3.66329958e-04],
                [-5.24679013e-05, -9.99999900e-01, -4.43448293e-04],
                [-3.66306726e-04,  4.43467484e-04, -9.99999835e-01]
            ], dtype=float)

        # HEBI's tool frame sits ~0.0835m below the actual fingertip position
        # in PyBullet's world frame. correct for it so callers can pass in the
        # position they actually want the fingers at.
        FINGER_Z_OFFSET = 0.08350
        hebi_target = np.array(target_position, dtype=float)
        hebi_target[2] -= FINGER_Z_OFFSET

        position_objective = hebi.robot_model.endeffector_position_objective(hebi_target)
        orientation_objective = hebi.robot_model.endeffector_so3_objective(target_orientation)
        return self.arm_model.solve_inverse_kinematics(initial_positions, position_objective, orientation_objective)

    def goto_cartesian(self, target_pos, duration=3.0):
        """move end effector smoothly in cartesian space"""
        # get current end effector position as starting point for interpolation
        current_pos = np.array(p.getLinkState(self.robot_id, self.end_effector_link)[0])
        target_pos = np.array(target_pos)

        # total number of simulation steps for the move
        num_steps = int(duration * 240)
        for i in range(num_steps):
            t = i / num_steps
            # linearly interpolate between current and target position
            interp_pos = (1 - t) * current_pos + t * target_pos
            # solve IK for interpolated position, from current joint state
            angles = self.get_ik(interp_pos)
            # apply joint angles using position control
            p.setJointMotorControlArray(
                self.robot_id,
                jointIndices=self.actuated_joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=angles,
                targetVelocities=[0] * len(self.actuated_joints),
                positionGains=[0.25] * len(self.actuated_joints), # low gain for smooth motion
            )
            # reapply gripper command each step to maintain grip force during motion
            self.set_gripper(0.55)
            p.stepSimulation()
            time.sleep(1. / 240.)

    def set_gripper(self, value):
        """set gripper position: -1.0 = open, 1.0 = closed"""
        for joint in [16, 20]:
            p.setJointMotorControl2(
                self.robot_id, joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=value,
                force=1000
            )

    def check_collision(self, block_id=None):
        """returns True if any collision is detected"""
        IGNORED_PAIRS = {(6, 8), (8, 10), (11, 13), (11, 17), (11, 21)}
        for c in p.getContactPoints(self.robot_id):
            link_a, link_b = c[3], c[4]
            other_body = c[2] # the body on the other side of the contact

            # ignore base resting on floor
            if c[2] == 0 and link_b == -1 and link_a == 0:
                continue
            # ignore permanent structural contacts
            if (min(link_a, link_b), max(link_a, link_b)) in IGNORED_PAIRS:
                continue
            # ignore gripper touching the block
            if block_id is not None and other_body == block_id:
                continue

            return True
        return False

    def close(self):
        """disconnects from PyBullet to avoid memory leaks"""
        p.disconnect()


if __name__ == '__main__':
    rng = np.random.default_rng()

    # # random IK poses with collision detection
    # env1 = RosieEnvironment(use_self_collision=True)
    # env1.set_position(env1.home_pose)

    # for i in range(10):
    #     random_position = np.array([
    #         rng.uniform(-0.5, 0.5),   # X: left and right
    #         rng.uniform(-0.5, 0.5),   # Y: forward and back
    #         rng.uniform(0.01, 0.6),   # Z: allow near-floor targets
    #     ])
    #     angles = env1.get_ik(random_position)
    #     env1.set_position(angles)
    #     for _ in range(1000):
    #         p.stepSimulation()
    #         time.sleep(1. / 240.)
    #     print(f"Target position: {random_position}")
    #     if env1.check_collision():
    #         print("Collision Detected")
    #     else:
    #         print("No collision")
    #     input("Press Enter to continue")

    # env1.close()

    # # hardcoded self-collision cases
    # env2 = RosieEnvironment(use_self_collision=True)
    # env2.set_position(env2.home_pose)

    # hardcoded_poses = [
    #     # base collision
    #     np.array([-1.15856693, -2.30733897, -3.02441003, 2.19252047, 0.30061921, 0.53673647]),
    #     # self-collision
    #     np.array([ 1.14554257,  2.61773381,  0.52162371, -1.72336358,  0.99948211,  2.05014016]),
    #     np.array([ 1.72623365,  2.92676335,  2.82157851, -2.15210368,  1.0023865,   1.88599717]),
    # ]

    # for pose in hardcoded_poses:
    #     env2.set_position(pose)
    #     for _ in range(1):
    #         p.stepSimulation()
    #         time.sleep(1. / 240.)
    #     print(f"Joint angles: {pose}")
    #     if env2.check_collision():
    #         print("Collision Detected")
    #     else:
    #         print("No collision")
    #     input("Press Enter to continue")

    # env2.close()

    # block grasping demo
    env3 = RosieEnvironment(use_self_collision=False, spawn_block=True)

    # start with 
    env3.set_position(env3.home_pose)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1. / 240.)

    hover_position = np.array([0.2717, -0.0342, 0.10])
    hover_angles = env3.get_ik(hover_position)
    env3.set_position(hover_angles)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)
    
    block_position = np.array([0.2717, -0.0342, 0.02])
    block_angles = env3.get_ik(block_position)
    env3.set_position(block_angles)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1. / 240.)
    left_aabb_min, left_aabb_max = p.getAABB(env3.robot_id, 16)
    right_aabb_min, right_aabb_max = p.getAABB(env3.robot_id, 20)
    print("left finger AABB:", left_aabb_min, left_aabb_max)
    print("right finger AABB:", right_aabb_min, right_aabb_max)
    # aabb_min, aabb_max = p.getAABB(env3.block_id)
    # print("block AABB min/max:", aabb_min, aabb_max)

    # left_finger_pos = np.array(p.getLinkState(env3.robot_id, 16)[0])
    # right_finger_pos = np.array(p.getLinkState(env3.robot_id, 20)[0])
    # print("left finger:", left_finger_pos)
    # print("right finger:", right_finger_pos)

    # block_pos, block_orn = p.getBasePositionAndOrientation(env3.block_id)
    # print("block position:", block_pos)
    input("Press Enter once you've visually confirmed the fingers look flush against the block's sides...")

    # grasp_position = np.array([0.2717, -0.0342, 0.02])
    # grasp_angles = env3.get_ik(grasp_position)
    # env3.set_position(grasp_angles)
    # for _ in range(100):
    #     p.stepSimulation()
    #     time.sleep(1./240.)

    input("Press Enter once you've visually confirmed the fingers look flush against the block's sides...")

    # # hover above block
    # hover_position = np.array([0.2717, -0.0342, 0.10])
    # hover_angles = env3.get_ik(hover_position)
    # print(hover_angles)
    # env3.set_position(hover_angles)
    # for _ in range(100):
    #     p.stepSimulation()
    #     time.sleep(1. / 240.)

    #     test_positions = [
    #     [0.22, -0.08, 0.02],
    #     [0.22,  0.02, 0.02],
    #     [0.32, -0.08, 0.02],
    #     [0.32,  0.02, 0.02],
    #     [0.27, -0.03, 0.02],
    # ]

    # FINGER_Z_OFFSET = 0.08350

    # for pos in test_positions:
    #     angles = env3.get_ik(np.array(pos))
    #     transform = np.eye(4)
    #     env3.arm_model.get_end_effector(angles, transform)
    #     achieved_hebi = transform[:3, 3]
    #     expected_hebi = np.array(pos) - np.array([0, 0, FINGER_Z_OFFSET])
    #     error = np.linalg.norm(achieved_hebi - expected_hebi)
    #     print(f"target={pos}  ik_error={error:.4f}")
    # info_left = p.getJointInfo(env3.robot_id, 16)
    # info_right = p.getJointInfo(env3.robot_id, 20)
    # print("left joint limits:", info_left[8], info_left[9])   # lower, upper
    # print("right joint limits:", info_right[8], info_right[9])
    
    #     positions = env3.get_position()
    # transform = np.eye(4)
    # env3.arm_model.get_end_effector(positions, transform)
    # hebi_pos = transform[:3, 3]

    # pybullet_ee_pos = np.array(p.getLinkState(env3.robot_id, env3.end_effector_link)[0])
    # left_finger_pos = np.array(p.getLinkState(env3.robot_id, 16)[0])
    # right_finger_pos = np.array(p.getLinkState(env3.robot_id, 20)[0])

    # print("HEBI FK pos:", hebi_pos)
    # print("PyBullet end_effector_link pos:", pybullet_ee_pos)
    # print("PyBullet left finger pos:", left_finger_pos)
    # print("PyBullet right finger pos:", right_finger_pos)

    # # descend to grasp position
    # block_position = np.array([0.2717, -0.0342, 0.001])
    # block_angles = env3.get_ik(block_position)
    # env3.set_position(block_angles)
    # for _ in range(100):
    #     p.stepSimulation()
    #     time.sleep(1. / 240.)

    # positions = np.array(env3.get_position(), dtype=float)  # the joint angles at your "good" pose
    # transform = np.eye(4)
    # env3.arm_model.get_end_effector(positions, transform)
    # good_orientation = transform[:3, :3]
    # print(good_orientation)

    # # close gripper
    # env3.set_gripper(0.3)
    # for _ in range(240):
    #     p.stepSimulation()
    #     time.sleep(1. / 240.)

    # # lift block
    # env3.goto_cartesian(hover_position, duration=2.0)
    # for _ in range(240):
    #     p.stepSimulation()
    #     time.sleep(1. / 240.)

    # env = RosieEnvironment(use_self_collision=False, spawn_block=True)
    # env.set_position(env.home_pose)
    # for _ in range(100):
    #     p.stepSimulation()

    # hover_position = np.array([0.2717, -0.0342, 0.10])
    # hover_angles = env.get_ik(hover_position)
    # env.set_position(hover_angles)
    # for _ in range(100):
    #     p.stepSimulation()


