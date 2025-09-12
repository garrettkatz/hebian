"""
Run this file like `python -i hardware.py` on the Rosie hardware to enter an interactive Python prompt with an initialized RosieWrapper object.
"""
# door knob grip position: 37.5 inches above the ground
# door is 36 inches wide, knob axle is 3 inches from door edge,
# grip position is 2.5 more inches towards hinge, 2 inches in front of door
# gripper spool: open is around -1.69, while -2 is almost closed

# initial positioning of the arm at the closed door knob:
# [-0.51974044,  1.15334185,  1.11663602, -3.0552607,  -1.05667999, -3.04651038]

import time
import hebi
import numpy as np

class RosieWrapper:
    def __init__(self, command_lifetime=5000):
        """
        command_lifetime: in ms
        """

        lu = hebi.Lookup()
        self.arm_group = lu.get_group_from_names(['Rosie'], ['J1_base','J2_shoulder','J3_elbow','J4_wrist1','J5_wrist2','J6_wrist3'])
        self.grip_group = lu.get_group_from_names(['Rosie'], ['gripperSpool'])

        for group in (self.arm_group, self.grip_group):
            if group is not None:
                group.command_lifetime = command_lifetime

        if self.grip_group is not None:
            self.gripper = hebi.arm.Gripper(self.grip_group, -5, 1) # close effort -5, open effort 1

    def get_position(self):
        gf = hebi.GroupFeedback(self.arm_group.size)
        gf = self.arm_group.get_next_feedback(reuse_fbk=gf)
        current = gf.position
        return current
    
    def goto_position(self, position, duration = 2.):
        """
        position: numpy array of goal joint angles
        duration: in seconds
        """
        if self.arm_group is None:
            print("None group!")
            return

        if self.arm_group.size != len(position):
            print("size mismatch")
            return
    
        # get current position
        gf = hebi.GroupFeedback(self.arm_group.size)
        gf = self.arm_group.get_next_feedback(reuse_fbk=gf)
        current = gf.position
    
        # interpolate slow trajectory
        num_pts = 200
        interp = np.linspace(0, 1, num_pts)[:,np.newaxis]
        traj = interp * position + (1-interp) * current
    
        # stream the commands
        gc = hebi.GroupCommand(len(position))
        for t, pos in enumerate(traj):
            gc.position = pos
            self.arm_group.send_command(gc)
            time.sleep(duration / num_pts)

    def open(self):
        if self.grip_group is None:
            print("None group!")
            return
        self.gripper.open()
        self.gripper.send()

    def close(self):
        if self.grip_group is None:
            print("None group!")
            return
        self.gripper.close()
        self.gripper.send()

if __name__ == "__main__":

    rosie = RosieWrapper()

    # lu = hebi.Lookup()
    # group = lu.get_group_from_names(['Rosie'], ['J1_base','J2_shoulder','J3_elbow','J4_wrist1','J5_wrist2','J6_wrist3'])

    # if group is not None:
    #     group.command_lifetime = 5000 # 5s

            # gf = hebi.GroupFeedback(group.size)
        # gf = group.get_next_feedback(reuse_fbk=gf)
        # current = gf.position

        # goto_position(group, np.array([-0.51974044,  1.15334185,  1.11663602, -3.0552607,  -1.05667999, -3.04651038]))
        # goto_position(group, current)

