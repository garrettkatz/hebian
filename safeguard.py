import numpy as np
import hebi
import os

# build path to HRDF relative to this script's location so it works from any directory
hrdf_path = os.path.join(os.path.dirname(__file__), "A-2240-06G.hrdf")
arm = hebi.robot_model.import_from_hrdf(hrdf_path)

def check_collision(joint_angle_array):
    # use forward kinematics to get transformation matrices for each joint
    transforms = arm.get_forward_kinematics('output', joint_angle_array)
    z_base = transforms[0][2,3] # z coordinate of J1_base, used as floor threshold
    z_gripper = transforms[-1][2, 3] # z coordinate of the gripper given input joint angles
    return z_gripper <= z_base

if __name__ == "__main__":
    # joint angles (radians) where gripper is below the base, should return True
    true_case = np.array([0., -0.18826023, 0., 0., 0., 0.])
    # joint angles (radians) where gripper is above the base, should return False
    false_case = np.array([0., 1.55314696, 0., 0., 0., 0.])
    assert check_collision(true_case) == True
    assert check_collision(false_case) == False