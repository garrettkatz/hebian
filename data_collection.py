import numpy as np
import pybullet as p
from rosie_gym_env import RosieGymEnv


def reset_episode_state(env, block_pos, gripper_open=-1.0, settle_steps=150):
    """
    env.reset() only re-homes the arm joints, doesn't touch the block or
    gripper. just repositioning the block wasn't enough - pybullet's
    contact state kept carrying over episode to episode (slip crept up,
    grasps got less reliable the longer a run went).

    fix: fully respawn the block (remove + reload urdf) every episode.
    reapplies the same friction values from spawn_block in rosie_sim.py.
    """
    env.robot.set_gripper(gripper_open)
    for _ in range(settle_steps):
        p.stepSimulation()

    p.removeBody(env.robot.block_id)
    env.robot.block_id = p.loadURDF("cube_small.urdf", basePosition=block_pos)
    p.changeDynamics(env.robot.block_id, -1, lateralFriction=1000.0)
    p.changeDynamics(env.robot.robot_id, 16, lateralFriction=1000.0)
    p.changeDynamics(env.robot.robot_id, 20, lateralFriction=1000.0)

    for _ in range(settle_steps):
        p.stepSimulation()


# drives the arm toward a joint target using env.step() so every frame we
# record is a real (obs, action) pair, same format the policy will use later
def move_to_via_env(env, target_angles, max_steps=200, tol=0.01, log=None):
    """
    steps env toward target_angles bit by bit, logging (obs, action) pairs
    along the way. clips each delta to the action space bounds (+-0.05 rad)
    and stops early once every joint is within tol.
    """
    obs = env._get_obs()  # just reads current obs, no step needed here

    low = env.action_space.low
    high = env.action_space.high

    for _ in range(max_steps):
        current = env.robot.get_position()
        raw_delta = target_angles - current

        if np.all(np.abs(raw_delta) < tol):
            break

        action = np.clip(raw_delta, low, high).astype(np.float32)

        if log is not None:
            log["images"].append(obs["image"])
            log["joints"].append(obs["joints"])
            log["actions"].append(action)

        obs, _, _, _, _ = env.step(action)

    return obs


def move_cartesian_via_env(env, target_pos, num_waypoints=15, max_steps_per_wp=40,
                            tol=0.01, log=None):
    """
    goto_cartesian keeps the gripper path straight by solving ik at every
    small cartesian step. jumping straight to one joint-space target
    doesn't do that - the gripper can swing sideways and clip the block on
    the way down. this does the same straight-line approach but through
    env.step() so it still logs as real (obs, action) pairs.
    """
    start_pos = np.array(p.getLinkState(env.robot.robot_id, env.robot.end_effector_link)[0])
    target_pos = np.array(target_pos)

    obs = env._get_obs()
    for i in range(1, num_waypoints + 1):
        t = i / num_waypoints
        interp_pos = (1 - t) * start_pos + t * target_pos
        wp_angles = env.robot.get_ik(interp_pos)
        obs = move_to_via_env(env, wp_angles, max_steps=max_steps_per_wp, tol=tol, log=log)

    return obs


# full pick-and-lift episode. place-move can get bolted on later the same
# way, not doing that yet since it's out of scope for now
def run_expert_episode(env, block_pos, hover_height=0.10, grasp_height=0.006,
                        gripper_closed=0.65, gripper_settle_steps=240):
    """
    runs one grasp-and-lift demo, returns {"images", "joints", "actions"}
    logged at every step the arm moves. gripper open/close isn't part of
    the action space so those ticks aren't logged.
    """
    log = {"images": [], "joints": [], "actions": []}

    obs, _ = env.reset()
    # reset_to_home() teleports the joints but doesn't cancel the previous
    # episode's motor command, which keeps tugging the arm during later
    # steps. reissuing a real position command to home_pose fixes it.
    env.robot.set_joint_targets(env.robot.home_pose)
    for _ in range(50):
        p.stepSimulation()
    reset_episode_state(env, block_pos)

    hover_pos = np.array([block_pos[0], block_pos[1], hover_height])
    grasp_pos = np.array([block_pos[0], block_pos[1], grasp_height])

    # hover above the block, straight line in cartesian space
    move_cartesian_via_env(env, hover_pos, log=log)
    hover_angles = env.robot.get_ik(hover_pos)  # need this again later for the lift

    # descend to grasp height, also straight line
    # (stops it from swinging sideways into the block on the way down)
    move_cartesian_via_env(env, grasp_pos, log=log)

    # close the gripper - direct sim call, not something the env logs as an action
    block_z_before = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
    env.robot.set_gripper(gripper_closed)
    for _ in range(gripper_settle_steps):
        p.stepSimulation()

    # lift back to hover height. env.step() doesn't reapply gripper force
    # each tick like goto_cartesian does, so _lift_with_gripper_held
    # handles that manually.
    _, slip_detected, max_slip = _lift_with_gripper_held(env, hover_angles, gripper_closed, log)

    block_z_after = p.getBasePositionAndOrientation(env.robot.block_id)[0][2]
    # if the grip actually held, the block should be up near hover_height
    # by now. if it didn't, block_z_after will still be sitting near table height
    grip_success = (block_z_after - block_z_before) > (hover_height * 0.5)

    return log, grip_success, slip_detected, max_slip


def _lift_with_gripper_held(env, target_angles, gripper_value, log,
                             max_steps=200, tol=0.01, slip_threshold=0.01):
    """
    same as move_to_via_env, but keeps reapplying gripper force every tick
    during the lift (env.step() alone doesn't do that). also tracks the
    block's position relative to the gripper each tick - a solid grip
    keeps that offset steady, a slip shows up as a jump even if it gets
    "caught" again right after.
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

        target = current + action
        env.robot.set_joint_targets(target)
        for _ in range(env.ticks_per_step):
            env.robot.set_gripper(gripper_value)  # keep holding grip force during the move
            p.stepSimulation()

            rel = _relative_block_pos()
            tick_slip = np.linalg.norm(rel - prev_rel)
            max_slip = max(max_slip, tick_slip)
            if tick_slip > slip_threshold:
                slip_detected = True
            prev_rel = rel

        obs = env._get_obs()

    return obs, slip_detected, max_slip


# main collection loop
def collect_dataset(num_episodes=10, save_path="expert_data.npz", render=False):
    env = RosieGymEnv(render=render)

    all_images, all_joints, all_actions, episode_ids = [], [], [], []
    ep_block_x, ep_block_y, ep_success, ep_max_slip = [], [], [], []
    rng = np.random.default_rng()

    def _save():
        np.savez_compressed(
            save_path,
            images=np.array(all_images, dtype=np.uint8),
            joints=np.array(all_joints, dtype=np.float32),
            actions=np.array(all_actions, dtype=np.float32),
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
            # small +-1cm jitter instead of a truly fixed spot. sim is fully
            # deterministic given the same inputs, and using an exact fixed
            # position kept locking into the same repeated failure after a
            # handful of episodes. this breaks that up cheaply and is also
            # basically the first step toward real position randomization later
            jitter = rng.uniform(-0.01, 0.01, size=2)
            block_pos = np.array([0.2717 + jitter[0], -0.0342 + jitter[1], 0.02])

            log, grip_success, slip_detected, max_slip = run_expert_episode(env, block_pos)

            n = len(log["actions"])
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
            episode_ids.extend([ep] * n)

            # saving after every successful episode instead of waiting til
            # the end - lost a couple full runs already to ctrl+c / running
            # the wrong command, so now each success is safe on disk right away
            _save()

    except KeyboardInterrupt:
        print("\ninterrupted -- saving what was collected so far...")
    finally:
        env.close()
        _save()


if __name__ == "__main__":
    collect_dataset(num_episodes=25, save_path="expert_data.npz", render=False)