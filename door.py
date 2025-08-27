import matplotlib.pyplot as pt
import numpy as np
import torch as tr

def rotmat(angle, axis):
    # axis in (0,1,2) for xyz
    return tr.roll(tr.tensor([
        [tr.cos(angle), -tr.sin(angle), 0],
        [tr.sin(angle),  tr.cos(angle), 0],
        [0, 0, 1],
    ]), (axis+1, axis+1), dims=(0,1))

def render_frame(ax, frame):
    X, Y, Z, t = frame[:3,:].numpy().T
    ax.quiver(*t, *X, color="r", length=0.05)
    ax.quiver(*t, *Y, color="g", length=0.05)
    ax.quiver(*t, *Z, color="b", length=0.05)

def forward_kinematics(joint_info, angles):
    # joint_info: child translation wrt parent, child orientation wrt parent, child joint rotation axis
    frames = [tr.eye(4)]
    for (t, R, axis), angle in zip(joint_info, angles):
        child = tr.eye(4)
        child[:3, 3] = frames[-1][:3,3] + frames[-1][:3,:3] @ t
        child[:3,:3] = frames[-1][:3,:3] @ R @ rotmat(tr.tensor(angle), axis)
        frames.append(child)
    return frames

def render_chain(ax, joint_info, angles):
    frames = forward_kinematics(joint_info, angles)
    for i, frame in enumerate(frames):
        ax.plot(*frame[:3,3].numpy(), "ko")
        render_frame(ax, frame)
        if i > 0:
            xp,yp,zp = frames[i-1][:3,3]
            xc,yc,zc = frame[:3,3]
            ax.plot([xp,xc],[yp,yc],[zp,zc], "ko-")

def render_frames(ax, frames):
    for i, frame in enumerate(frames):
        ax.plot(*frame[:3,3].numpy(), "ko")
        render_frame(ax, frame)
        if i > 0:
            xp,yp,zp = frames[i-1][:3,3]
            xc,yc,zc = frame[:3,3]
            ax.plot([xp,xc],[yp,yc],[zp,zc], "ko-")
        
def sinusoid_interp(start, end, duration, num_pts):
    # start, end: waypoint joint tensors, flat
    # duration, num_pts: timing
    # returns pts, pos, vel: time points, joint position and velocity trajectory
    pts = tr.linspace(0, duration, num_pts).unsqueeze(1)
    pinterp = (1 + tr.cos(tr.pi * pts / duration)) / 2
    vinterp = - tr.sin(tr.pi * pts / duration) * tr.pi / duration / 2

    pos = start * pinterp + end * (1 - pinterp)
    vel = start * vinterp + end * (  - vinterp)

    return pts, pos, vel    


if __name__ == "__main__":

    for axis in (0,1,2):
        print("xyz"[axis])
        print(rotmat(tr.tensor(tr.pi/4), axis))

    # child translation wrt parent, child orientation wrt parent, child joint rotation axis
    affordance = tr.tensor([
        [ 0.,  0.,  1.],
        [-1.,  0.,  0.],
        [ 0., -1.,  0.],
    ])

    # door knob grip position: 37.5 inches above the ground
    # door is 36 inches wide, knob axle is 3 inches from door edge,
    # grip position is 2.5 more inches towards hinge, 2 inches in front of door
    # gripper spool: open is around -1.69, while -2 is almost closed
    inches_per_meter = 39.37
    knob_axle_z = 37.5 / inches_per_meter
    knob_axle_y = -33 / inches_per_meter
    knob_lever_dx = -2 / inches_per_meter
    lever_grip_dy = 2.5 / inches_per_meter

    joint_info = [
        (tr.tensor([.7, .35, 0]), tr.eye(3), 2), # hinge
        (tr.tensor([0, knob_axle_y, knob_axle_z]), tr.eye(3), 0), # knob axle
        (tr.tensor([knob_lever_dx, 0, 0]), tr.eye(3), 0), # knob lever
        (tr.tensor([0, lever_grip_dy, 0]), affordance, 0), # grasp point
    ]
    # joint_angles = [0]*len(joint_info)
    # joint_angles = [-tr.pi/6, 0, 0, 0]
    joint_angles = [-tr.pi/6, -tr.pi/4, 0, 0]

    ax = pt.figure().add_subplot(projection='3d')
    render_chain(ax, joint_info, joint_angles)
    pt.axis("equal")
    pt.show()

    # trajectory for rosie gripper
    num_pts = 30
    duration = 1.
    knob_turn = tr.zeros(num_pts, len(joint_info))
    knob_turn[:,1] = tr.linspace(0, -tr.pi/4, num_pts)
    door_open = tr.zeros(num_pts, len(joint_info))
    door_open[:,1] = -tr.pi/4
    door_open[:,0] = tr.linspace(0, -tr.pi/6, num_pts)

    knob_turn = sinusoid_interp(
        tr.tensor([0, 0, 0, 0]),
        tr.tensor([0, -tr.pi/4, 0, 0]),
        duration, num_pts)
    door_open = sinusoid_interp(
        tr.tensor([0, -tr.pi/4, 0, 0]),
        tr.tensor([-tr.pi/6, -tr.pi/4, 0, 0]),
        duration, num_pts)

    pts, pos, vel = knob_turn
    pt.plot(pts, pos, 'k-')
    pt.plot(pts, vel, 'r-')
    pt.plot((pts[1:] + pts[:-1])/2, (pos[1:] - pos[:-1]) / (pts[1:] - pts[:-1]), 'g-')
    pt.show()

    joint_traj = tr.concatenate((knob_turn[1], door_open[1]), dim=0)

    knob_turn_frames = []
    knob_turn_pts = knob_turn[0]
    for angles in knob_turn[1]:
        knob_turn_frames.append(forward_kinematics(joint_info, angles))
    door_open_frames = []
    door_open_pts = door_open[0]
    for angles in door_open[1]:
        door_open_frames.append(forward_kinematics(joint_info, angles))

    ax = pt.figure().add_subplot(projection='3d')
    for t, frames in enumerate(knob_turn_frames + door_open_frames):
        if t % 10 == 0:
            render_frames(ax, frames)
    pt.axis("equal")
    pt.show()    

    # sanity check: first and last eff target
    print('eff 0:')
    print(knob_turn_frames[0][-1])
    print(f'eff {num_pts}:')
    print(knob_turn_frames[-1][-1])
    print('eff -1:')
    print(door_open_frames[-1][-1])

    # use hebi for IK
    import hebi

    # do the IK along traj
    arm = hebi.robot_model.import_from_hrdf("A-2240-06G.hrdf")
    trajectories = {}
    for (key, pts, all_frames) in [("turn", knob_turn_pts, knob_turn_frames), ("open", door_open_pts, door_open_frames)]:
        trajectories[key] = (pts, [])
        most_recent = np.zeros(6)
        for frames in all_frames:
            targ = frames[-1]
            position_objective = hebi.robot_model.endeffector_position_objective(targ[:3,3].numpy())
            orientation_objective = hebi.robot_model.endeffector_so3_objective(targ[:3,:3].numpy())
            soln = arm.solve_inverse_kinematics(most_recent, position_objective, orientation_objective)
            trajectories[key][1].append(soln)
            most_recent = soln

    # display first joint angle command
    print("trajectory[0]:")
    print(trajectories["turn"][1][0])

    # save the trajectories
    import pickle as pk
    with open("door_trajs.pkl", "wb") as f: pk.dump(trajectories, f)
    

    # now do it in scope viz
    import time
    # eff_targets = eff_targets[:1] # sanity check

    # connect to group
    lookup = hebi.Lookup()
    group = lookup.get_group_from_names(['HEBI'], ['J1_base','J2_shoulder','J3_elbow','J4_wrist1','J5_wrist2','J6_wrist3'])
    num_joints = 6 if group is None else group.size # None when not connected

    # send the commands
    gc = hebi.GroupCommand(num_joints)
    gc.position = trajectory[0]
    if group is not None: group.send_command(gc)
    input("[Enter] to run traj...")

    for t,position in enumerate(trajectory[1:]):
        print(t, position.round(3))
        gc.position = position
        if group is not None:
            group.send_command(gc)
            time.sleep(0.1)



