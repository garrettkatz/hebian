# door knob grip position: 37.5 inches above the ground
# door is 36 inches wide, knob axle is 3 inches from door edge,
# grip position is 2.5 more inches towards hinge, 2 inches in front of door
# gripper spool: open is around -1.69, while -2 is almost closed

# initial positioning of the arm at the closed door knob:
# [-0.51974044,  1.15334185,  1.11663602, -3.0552607,  -1.05667999, -3.04651038]

import time
import hebi
import numpy as np

def goto_position(group, position):
    if group is not None and group.size != len(position):
        print("size mismatch")
        return

    # get current position
    if group is None:
        current = np.zeros(len(position))
    else:
        gf = hebi.GroupFeedback(group.size)
        gf = group.get_next_feedback(reuse_fbk=gf)
        current = gf.position

    # interpolate slow trajectory
    num_pts = 200
    interp = np.linspace(0, 1, num_pts)[:,np.newaxis]
    traj = interp * position + (1-interp) * current

    # stream the commands
    gc = hebi.GroupCommand(len(position))
    for t, pos in enumerate(traj):
        print(t, pos.round(3))
        gc.position = pos
        if group is not None:
            group.send_command(gc)
            time.sleep(2. / num_pts) # duration of 2 seconds

if __name__ == "__main__":

    lu = hebi.Lookup()
    group = lu.get_group_from_names(['Rosie'], ['J1_base','J2_shoulder','J3_elbow','J4_wrist1','J5_wrist2','J6_wrist3'])

    if group is not None:
        group.command_lifetime = 500
        gf = hebi.GroupFeedback(group.size)
        gf = group.get_next_feedback(reuse_fbk=gf)
        current = gf.position

        goto_position(group, np.array([-0.51974044,  1.15334185,  1.11663602, -3.0552607,  -1.05667999, -3.04651038]))
        goto_position(group, current)

