import numpy as np
import pybullet as p
from rosie_gym_env import RosieGymEnv


def reset_episode_state(env, block_pos, gripper_open=0.0, settle_steps=150, verbose=False):
    """
    Reset the gripper, respawn the block, and settle the simulation.
    """
    # reset gripper joints
    for joint in [16, 20]:
        p.resetJointState(env.robot.robot_id, joint, gripper_open, targetVelocity=0.0)

    env.robot.set_gripper(gripper_open)
    for _ in range(settle_steps):
        p.stepSimulation()

    p.removeBody(env.robot.block_id)
    env.robot.block_id = p.loadURDF("cube_small.urdf", basePosition=block_pos)
    pos, orn = p.getBasePositionAndOrientation(env.robot.block_id)
    if verbose:
        print(p.getEulerFromQuaternion(orn))
    p.changeDynamics(env.robot.block_id, -1, lateralFriction=1000.0)
    p.changeDynamics(env.robot.robot_id, 16, lateralFriction=1000.0)
    p.changeDynamics(env.robot.robot_id, 20, lateralFriction=1000.0)

    for _ in range(settle_steps):
        p.stepSimulation()


def move_to_via_env(env, target_angles, max_steps=200, tol=0.01, log=None, verbose=False):
    """
    Move to target joint angles while logging observations and actions.
    """
    obs = env._get_obs()
    low = env.action_space.low
    high = env.action_space.high

    for step in range(max_steps):
        current = env.robot.get_position()
        raw_delta = target_angles - current

        if np.all(np.abs(raw_delta) < tol):
            break

        action = np.clip(raw_delta, low, high).astype(np.float32)

        if log is not None:
            log["images"].append(obs["image"])
            log["joints"].append(obs["joints"])
            log["actions"].append(action)
            log["rewards"].append(env._compute_reward())

            obs, _, _, _, _ = env.step(action)
    else:
        if verbose:
            final_delta = np.abs(target_angles - env.robot.get_position()).max()
            print(f"  move_to_via_env: hit max_steps={max_steps} without converging, final_delta={final_delta:.4f}")

    return obs


def move_cartesian_via_env(env, target_pos, num_waypoints=15, max_steps_per_wp=100,
                            tol=0.01, log=None, min_z=0.015, cartesian_tol=0.01, verbose=False):
    """
    Move through Cartesian waypoints using IK while logging data.
    """
    start_pos = np.array(p.getLinkState(env.robot.robot_id, env.robot.end_effector_link)[0])
    target_pos = np.array(target_pos)
    if target_pos[2] < min_z:
        target_pos[2] = min_z

    obs = env._get_obs()
    for i in range(1, num_waypoints + 1):
        t = i / num_waypoints
        interp_pos = (1 - t) * start_pos + t * target_pos
        wp_angles = env.robot.get_ik(interp_pos)

        if verbose:
            check_transform = np.eye(4)
            env.robot.arm_model.get_end_effector(wp_angles, check_transform)
            print(f"waypoint {i}: requested_z={interp_pos[2]:.4f}  ik_solution_fk_z={check_transform[2,3]:.4f}")

        obs = move_to_via_env(env, wp_angles, max_steps=max_steps_per_wp, tol=tol, log=log, verbose=verbose)

        left_finger_pos = np.array(p.getLinkState(env.robot.robot_id, 16)[0])
        right_finger_pos = np.array(p.getLinkState(env.robot.robot_id, 20)[0])
        achieved_pos = (left_finger_pos + right_finger_pos) / 2.0
        cartesian_error = np.linalg.norm(achieved_pos - interp_pos)
        if cartesian_error > cartesian_tol:
            if verbose:
                print(f"  waypoint {i}: cartesian_error={cartesian_error:.4f}m exceeds tol={cartesian_tol}, "
                      f"re-solving IK from current position")
            wp_angles_retry = env.robot.get_ik(interp_pos)
            obs = move_to_via_env(env, wp_angles_retry, max_steps=max_steps_per_wp, tol=tol, log=log, verbose=verbose)

        if i == num_waypoints and verbose:
            final_left = np.array(p.getLinkState(env.robot.robot_id, 16)[0])
            final_right = np.array(p.getLinkState(env.robot.robot_id, 20)[0])
            final_finger_pos = (final_left + final_right) / 2.0
            print(f"final waypoint check: intended_z={interp_pos[2]:.4f}  achieved_finger_z={final_finger_pos[2]:.4f}  "
                  f"overshoot={final_finger_pos[2] - interp_pos[2]:.4f}")

    return obs


def run_expert_episode(env, block_pos, hover_height=0.15, grasp_height=0.08,
                        gripper_closed=0.9, gripper_settle_steps=240, verbose=False):
    """
    Run one expert grasp-and-lift episode and return logged data.
    """
    log = {"images": [], "joints": [], "actions": [], "rewards": []}

    obs, _ = env.reset()
    env.robot.set_joint_targets(env.robot.home_pose)
    for _ in range(50):
        p.stepSimulation()
    reset_episode_state(env, block_pos, verbose=verbose)

    hover_pos = np.array([block_pos[0], block_pos[1], hover_height])
    grasp_pos = np.array([block_pos[0], block_pos[1], grasp_height])

    move_cartesian_via_env(env, hover_pos, log=log, verbose=verbose)
    hover_angles = env.robot.get_ik(hover_pos)

    move_cartesian_via_env(env, grasp_pos, log=log, verbose=verbose)

    block_z_before = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
    gripper_pos = np.array(p.getLinkState(env.robot.robot_id, env.robot.end_effector_link)[0])
    left_finger_pos = np.array(p.getLinkState(env.robot.robot_id, 16)[0])
    right_finger_pos = np.array(p.getLinkState(env.robot.robot_id, 20)[0])
    if verbose:
        print(f"after descent: gripper_z={gripper_pos[2]:.4f} left_z={left_finger_pos[2]:.4f} "
              f"right_z={right_finger_pos[2]:.4f} target grasp_height={grasp_height}")

    env.robot.set_gripper(gripper_closed)
    for step in range(gripper_settle_steps):
        p.stepSimulation()
        if verbose and step % 60 == 0:
            block_z = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
            print(f"  closing step {step}: block_z={block_z:.4f}")

    contacts_left = p.getContactPoints(env.robot.robot_id, env.robot.block_id, 16)
    contacts_right = p.getContactPoints(env.robot.robot_id, env.robot.block_id, 20)
    if verbose:
        print(f"pre-lift contacts: left={len(contacts_left)} right={len(contacts_right)}")
        for c in contacts_left + contacts_right:
            print(f"    contact normal: {c[7]}")
    all_contacts = p.getContactPoints(bodyA=env.robot.robot_id, bodyB=env.robot.block_id)

    # group contacts by left and right finger
    contacts_left = [c for c in all_contacts if c[3] < 20]
    contacts_right = [c for c in all_contacts if c[3] >= 20]

    max_penetration = 0.0
    for c in all_contacts:
        if c[8] < max_penetration:
            max_penetration = c[8]

    if verbose:
        print(f"max_penetration={max_penetration:.4f}m")

    _, slip_detected, max_slip = _lift_with_gripper_held(env, hover_angles, gripper_closed, log, verbose=verbose)

    block_z_after = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
    if verbose:
        print(f"block_z_before={block_z_before:.4f}  block_z_after={block_z_after:.4f}  diff={block_z_after-block_z_before:.4f}")

    PENETRATION_LIMIT = -0.005  # 5mm
    height_ok = (block_z_after - block_z_before) > (hover_height * 0.3)
    penetration_ok = max_penetration > PENETRATION_LIMIT
    grip_success = height_ok and penetration_ok

    if verbose and height_ok and not penetration_ok:
        print(f"  -> height check passed but penetration too deep ({max_penetration:.4f}m), rejecting")

    return log, grip_success, slip_detected, max_slip


def _lift_with_gripper_held(env, target_angles, gripper_value, log,
                             max_steps=200, tol=0.01, slip_threshold=0.01, verbose=False):
    """
    Lift while holding the gripper closed and detect block slip.
    """
    obs = env._get_obs()
    low = env.action_space.low
    high = env.action_space.high

    def _relative_block_pos():
        block_pos = np.array(p.getBasePositionAndOrientation(env.robot.block_id)[0])
        gripper_pos = np.array(p.getLinkState(env.robot.robot_id, env.robot.end_effector_link)[0])
        return block_pos - gripper_pos

    prev_rel = _relative_block_pos()
    slip_detected = False
    max_slip = 0.0

    for _ in range(max_steps):
        current = env.robot.get_position()
        raw_delta = target_angles - current
        if np.all(np.abs(raw_delta) < tol):
            break

        action = np.clip(raw_delta, low, high).astype(np.float32)

        log["images"].append(obs["image"])
        log["joints"].append(obs["joints"])
        log["actions"].append(action)
        log["rewards"].append(env._compute_reward())

        target = current + action
        env.robot.set_joint_targets(target)
        for _ in range(env.ticks_per_step):
            env.robot.set_gripper(gripper_value)
            p.stepSimulation()

            rel = _relative_block_pos()
            tick_slip = np.linalg.norm(rel - prev_rel)
            max_slip = max(max_slip, tick_slip)
            if tick_slip > slip_threshold:
                slip_detected = True
                if verbose:
                    print(f"  slip spike: tick_slip={tick_slip:.4f} at rel_pos={rel}")
            prev_rel = rel

        obs = env._get_obs()

    return obs, slip_detected, max_slip

def split_train_test(data_path="expert_data.npz", train_path="expert_data_train.npz",
                      test_path="expert_data_test.npz", test_fraction=0.2, seed=42):
    """
    Split the dataset into train and test sets by episode.
    """
    data = np.load(data_path)

    images = data["images"]
    joints = data["joints"]
    actions = data["actions"]
    rewards = data["rewards"]
    episode_ids = data["episode_ids"]
    ep_block_x = data["ep_block_x"]
    ep_block_y = data["ep_block_y"]
    ep_success = data["ep_success"]
    ep_max_slip = data["ep_max_slip"]

    # split only successful episodes
    successful_ep_nums = np.unique(episode_ids)

    rng = np.random.default_rng(seed)
    shuffled = rng.permutation(successful_ep_nums)
    n_test = max(1, int(len(shuffled) * test_fraction))
    test_eps = set(shuffled[:n_test])
    train_eps = set(shuffled[n_test:])

    def _build_split(ep_set):
        mask = np.isin(episode_ids, list(ep_set))
        return {
            "images": images[mask],
            "joints": joints[mask],
            "actions": actions[mask],
            "rewards": rewards[mask],
            "episode_ids": episode_ids[mask],
        }

    train_data = _build_split(train_eps)
    test_data = _build_split(test_eps)

    np.savez_compressed(train_path, **train_data)
    np.savez_compressed(test_path, **test_data)

    print(f"split {len(successful_ep_nums)} successful episodes: "
          f"{len(train_eps)} train ({len(train_data['actions'])} transitions), "
          f"{len(test_eps)} test ({len(test_data['actions'])} transitions)")

def collect_dataset(num_episodes=10, save_path="expert_data.npz", render=False, verbose=False):
    env = RosieGymEnv(render=render)

    all_images, all_joints, all_actions, all_rewards, episode_ids = [], [], [], [], []
    ep_block_x, ep_block_y, ep_success, ep_max_slip = [], [], [], []
    rng = np.random.default_rng()

    def _save():
        np.savez_compressed(
            save_path,
            images=np.array(all_images, dtype=np.uint8),
            joints=np.array(all_joints, dtype=np.float32),
            actions=np.array(all_actions, dtype=np.float32),
            rewards=np.array(all_rewards, dtype=np.float32),
            episode_ids=np.array(episode_ids, dtype=np.int32),
            ep_block_x=np.array(ep_block_x, dtype=np.float32),
            ep_block_y=np.array(ep_block_y, dtype=np.float32),
            ep_success=np.array(ep_success, dtype=bool),
            ep_max_slip=np.array(ep_max_slip, dtype=np.float32),
        )
        print(f"saved {len(all_actions)} total transitions to {save_path} "
              f"({int(np.sum(ep_success))}/{len(ep_success)} episodes succeeded)")

    try:
        for ep in range(num_episodes):
            # sample a random block position
            x = rng.uniform(0.22, 0.32)
            y = rng.uniform(-0.08, 0.02)
            block_pos = np.array([x, y, 0.02])

            log, grip_success, slip_detected, max_slip = run_expert_episode(env, block_pos, verbose=verbose)

            n = len(log["actions"])
            # print episode summary
            print(f"episode {ep}: {n} steps logged, grip_success={grip_success}, "
                  f"slip_detected={slip_detected}, max_slip={max_slip:.4f}m")

            ep_block_x.append(block_pos[0])
            ep_block_y.append(block_pos[1])
            ep_success.append(bool(grip_success))
            ep_max_slip.append(float(max_slip))

            if not grip_success:
                print(f"  -> discarding episode {ep} (failed grasp)")
                continue

            all_images.extend(log["images"])
            all_joints.extend(log["joints"])
            all_actions.extend(log["actions"])
            all_rewards.extend(log["rewards"])
            episode_ids.extend([ep] * n)

            _save()

    except KeyboardInterrupt:
        print("\ninterrupted -- saving what was collected so far...")
    finally:
        env.close()
        _save()


if __name__ == "__main__":
    collect_dataset(num_episodes=150, save_path="expert_data.npz", render=True, verbose=False)
    split_train_test(data_path="expert_data.npz",
                      train_path="expert_data_train.npz",
                      test_path="expert_data_test.npz",
                      test_fraction=0.2)