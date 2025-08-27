import time
import numpy as np
import hebi

lookup = hebi.Lookup()
group = lookup.get_group_from_names(['HEBI'], ['J1_base','J2_shoulder','J3_elbow','J4_wrist1','J5_wrist2','J6_wrist3'])
num_joints = 6 if group is None else group.size # None when not connected

# group_command = hebi.GroupCommand(group.size)
# group_command.position = np.random.randn(group.size)
# group.send_command(group_command) # watch the visualization change when you execute this

# FK said that at zero joints, gripper is at
# [[ 1.     0.    -0.     0.65 ]
#  [-0.     0.     1.     0.061]
#  [ 0.    -1.     0.    -0.028]
#  [ 0.     0.     0.     1.   ]]
# Colors: x is red, y is green, z is blue (rgb)
# First column is the X-axis of the gripper frame, expressed in base coordinates, etc.
# This means that gripper Z is the direction gripper is pointing, Y is the open/close direction, and X is their normal

# First target pose:
# gripper Y (open/close) should be vertical (around unturned knob) base z-axis
# gripper Z should be pointing towards the door (forward is base x-axis)
# gripper X should be parallel to knob (base Y axis)
# position should be about .5 meters above and in front of the base (base x and z axis)
# this orientation looks to be just rotating the gripper at zero angles 90 towards viewer
# alltogether you get:
# eff_target = np.array([
#  [ 0.,  0.,  1.,  0.5],
#  [-1.,  0.,  0.,  0.0],
#  [ 0., -1.,  0.,  0.5],
#  [ 0.,  0.,  0.,  1. ],
# ])
# now interpolating from here to final target:

num_pts = 10
knob_angles = np.linspace(np.pi, 5*np.pi/4, num_pts)
knob_anchor = np.array([[.5,0,.5]]).T
knob_grips = knob_anchor + 0.05*np.stack([np.cos(knob_angles), np.zeros(num_pts), np.sin(knob_angles)])

print(knob_grips.round(2))

rot_angles = np.linspace(0, -np.pi/4, num_pts)
rot_mats = np.zeros((3,3,num_pts))
rot_mats[0,2,:] = 1
rot_mats[1,0,:] = -np.cos(rot_angles)
rot_mats[2,0,:] = -np.sin(rot_angles)
rot_mats[1,1,:] = +np.sin(rot_angles)
rot_mats[2,1,:] = -np.cos(rot_angles)

print(rot_mats[:,:,0].round(3))
print(rot_mats[:,:,-1].round(3))

# now do the IK to solve all of these
arm = hebi.robot_model.import_from_hrdf("A-2240-06G.hrdf")
trajectory = []
most_recent = np.zeros(num_joints)
for t in range(num_pts):
    position_objective = hebi.robot_model.endeffector_position_objective(knob_grips[:,t])
    orientation_objective = hebi.robot_model.endeffector_so3_objective(rot_mats[:,:,t])
    soln = arm.solve_inverse_kinematics(most_recent, position_objective, orientation_objective)
    trajectory.append(soln)
    most_recent = soln

# double-check forward kinematics at end
transforms = arm.get_forward_kinematics('output', most_recent)
print(transforms[-1].round(3))

# send the commands
input("[Enter] to run traj...")
for t,position in enumerate(trajectory):
    print(t, position.round(3))
    gc = hebi.GroupCommand(num_joints)
    gc.position = position
    if group is not None:
        group.send_command(gc)
        time.sleep(0.1)


