from rosie_sim import RosieEnvironment
import gymnasium as gym
import numpy as np
import pybullet as p

class RosieGymEnv(gym.Env):
    
    def __init__(self, render=True):
        super().__init__()

        # create the underlying simulation
        self.robot = RosieEnvironment(use_self_collision=False, show=render, spawn_block=True)

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
        
        # fixed overhead camera looking down at the workspace
        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=[0.5, 0, 0.8],
            cameraTargetPosition=[0.3, 0, 0.2],
            cameraUpVector=[0, 0, 1]
        )

        # defines field of view and clipping range
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=self.cam_width / self.cam_height,
            nearVal=0.1, farVal=2.0
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # teleports arm to home pose and let sim settle
        self.robot.reset_to_home()
        obs = self._get_obs()
        return obs, {}
    
    def _get_obs(self):
        # capture RGB image from synthetic camera and discard width, height, depth, segmentation
        _, _, rgb, _, _ = p.getCameraImage(
            width=self.cam_width, height=self.cam_height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix
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
        return -distance
    
    def close(self):
        self.robot.close()