from rosie_sim import RosieEnvironment
import gymnasium as gym
import numpy as np
import pybullet as p

class RosieGymEnv(gym.Env):
    
    def __init__(self, render=True):
        super().__init__()

        # create the underlying simulation
        self.robot = RosieEnvironment(use_self_collision=True, show=render, spawn_block=True)

        # how many physics ticks per step()
        self.ticks_per_step = 10

        # observation space: image + joint angles
        self.observation_space = gym.spaces.Dict({
            "image": gym.spaces.Box(low=0, high=255, shape=(84, 84, 3), dtype=np.uint8),
            "joints": gym.spaces.Box(low=-np.pi, high=np.pi, shape=(6,), dtype=np.float32)
        })

        # action space: delta joint angles, capped at ±0.05 radians per step
        self.action_space = gym.spaces.Box(
            low=-0.05, high=0.05, shape=(6,), dtype=np.float32
        )

        # camera resolution, matches image observation shape
        self.cam_width = 84
        self.cam_height = 84
        
        # defines field of view and clipping range
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov=120, aspect=self.cam_width / self.cam_height,
            nearVal=0.001, farVal=2.0
        )

        self.collision_penalty = 3.0

    def _get_camera_view_matrix(self):
        link_state = p.getLinkState(self.robot.robot_id, 11, computeForwardKinematics=True)
        raw_eye_pos = np.array(link_state[4])
        orientation = link_state[5]

        rot_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)
        forward_dir = rot_matrix[:, 2]
        up_dir = rot_matrix[:, 1]

        eye_pos = raw_eye_pos - up_dir * 0.06 + forward_dir * 0.07
        target_pos = eye_pos + forward_dir * 0.1

        return p.computeViewMatrix(
            cameraEyePosition=eye_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=up_dir
        )
        

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # teleports arm to home pose and let sim settle
        self.robot.reset_to_home()
        obs = self._get_obs()
        return obs, {}
    
    def _get_obs(self):

        self.view_matrix = self._get_camera_view_matrix()
        # capture RGB image from synthetic camera and discard width, height, depth, segmentation
        _, _, rgb, _, _ = p.getCameraImage(
            width=self.cam_width, height=self.cam_height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        # reshape flat array to (H, W, 4) and drop alpha channel
        image = np.array(rgb, dtype=np.uint8).reshape(self.cam_height, self.cam_width, 4)[:, :, :3]

        # get current joint angles
        joints = self.robot.get_position().astype(np.float32)

        return {"image": image, "joints": joints}
    
    def step(self, action):
        # compute new target angles by adding delta to current
        current = self.robot.get_position()
        target = current + action

        # queue motor commands then advance simulation
        self.robot.set_joint_targets(target)
        for _ in range(self.ticks_per_step):
            p.stepSimulation()

        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = False # set True when block reaches goal
        truncated = False # set True when episode hits step limit
        return obs, reward, terminated, truncated, {}
    
    def _compute_reward(self):
        # negative distance to goal, higher reward as block gets closer
        block_pos = np.array(p.getBasePositionAndOrientation(self.robot.block_id)[0])
        goal_pos = np.array([0.4, 0.1, 0.02]) # hardcoded goal position
        distance = np.linalg.norm(block_pos - goal_pos)
        reward = -distance
    
        if self.robot.check_collision(block_id=self.robot.block_id):
            reward -=self.collision_penalty

        return reward
    
    def close(self):
        self.robot.close()