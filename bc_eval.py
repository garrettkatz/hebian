import numpy as np
import torch as tr
import torch.nn as nn
import pybullet as p
from rosie_gym_env import RosieGymEnv


# ---------------- network (must match training config) ----------------

def make_net(obs_dim=6, hid_dim=64, out_dim=6):
    return nn.Sequential(
        nn.Linear(obs_dim, hid_dim),
        nn.ReLU(),
        nn.Linear(hid_dim, out_dim),
    )


def load_trained_net(weights_path="bc_net_h64_lr0.01.pt", hid_dim=64):
    net = make_net(obs_dim=6, hid_dim=hid_dim, out_dim=6)
    net.load_state_dict(tr.load(weights_path))
    net.eval()
    return net


# ---------------- policy-driven episode ----------------

def run_policy_episode(env, net, block_pos, max_steps=400, settle_steps=150,
                        gripper_closed=0.9, gripper_settle_steps=240, verbose=False):
    """
    Run one episode letting the trained network choose actions instead of IK.
    Mirrors the structure of run_expert_episode in data_collection.py, but the
    network replaces move_cartesian_via_env/move_to_via_env entirely.
    """
    obs, _ = env.reset()
    env.robot.set_joint_targets(env.robot.home_pose)
    for _ in range(50):
        p.stepSimulation()

    # respawn block at the requested position (mirrors reset_episode_state)
    for joint in [16, 20]:
        p.resetJointState(env.robot.robot_id, joint, 0.0, targetVelocity=0.0)
    env.robot.set_gripper(0.0)
    for _ in range(settle_steps):
        p.stepSimulation()
    p.removeBody(env.robot.block_id)
    env.robot.block_id = p.loadURDF("cube_small.urdf", basePosition=block_pos)
    p.changeDynamics(env.robot.block_id, -1, lateralFriction=1000.0)
    p.changeDynamics(env.robot.robot_id, 16, lateralFriction=1000.0)
    p.changeDynamics(env.robot.robot_id, 20, lateralFriction=1000.0)
    for _ in range(settle_steps):
        p.stepSimulation()

    obs = env._get_obs()
    low = env.action_space.low
    high = env.action_space.high

    with tr.no_grad():
        for step in range(max_steps):
            joints_t = tr.from_numpy(obs["joints"].astype(np.float32)).unsqueeze(0)
            action = net(joints_t).squeeze(0).numpy()
            action = np.clip(action, low, high).astype(np.float32)
            obs, _, _, _, _ = env.step(action)

            if verbose and step % 50 == 0:
                block_z = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
                print(f"  step {step}: block_z={block_z:.4f}")

    # attempt grasp: close gripper and hold
    block_z_before = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
    env.robot.set_gripper(gripper_closed)
    for _ in range(gripper_settle_steps):
        p.stepSimulation()

    all_contacts = p.getContactPoints(bodyA=env.robot.robot_id, bodyB=env.robot.block_id)
    max_penetration = 0.0
    for c in all_contacts:
        if c[8] < max_penetration:
            max_penetration = c[8]

    # simple lift check: hold gripper, step physics, see if block rises
    for _ in range(150):
        env.robot.set_gripper(gripper_closed)
        p.stepSimulation()

    block_z_after = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
    height_ok = (block_z_after - block_z_before) > 0.03
    penetration_ok = max_penetration > -0.005
    success = height_ok and penetration_ok

    return success, block_z_before, block_z_after


def evaluate_policy(num_episodes=20, weights_path="bc_net_h64_lr0.01.pt", render=True, verbose=False):
    env = RosieGymEnv(render=render)
    net = load_trained_net(weights_path)

    rng = np.random.default_rng()
    successes = 0

    for ep in range(num_episodes):
        x = rng.uniform(0.22, 0.32)
        y = rng.uniform(-0.08, 0.02)
        block_pos = np.array([x, y, 0.02])

        success, z_before, z_after = run_policy_episode(env, net, block_pos, verbose=verbose)
        successes += int(success)
        print(f"episode {ep}: success={success}  block_z {z_before:.4f} -> {z_after:.4f}")

    env.close()
    print(f"\nnetwork policy success rate: {successes}/{num_episodes} "
          f"({100 * successes / num_episodes:.1f}%)")


if __name__ == "__main__":
    evaluate_policy(num_episodes=5, weights_path="bc_net_h64_lr0.01.pt", render=True, verbose=True)
