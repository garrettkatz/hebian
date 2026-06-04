import numpy as np
import hebi
import os

# build path to HRDF relative to this script's location so it works from any directory
hrdf_path = os.path.join(os.path.dirname(__file__), "A-2240-06G.hrdf")
arm = hebi.robot_model.import_from_hrdf(hrdf_path)

def check_collision(joint_angle_array, z_delta = 0.0, R=0.1):
    # use forward kinematics to get transformation matrices for each joint
    transforms = arm.get_forward_kinematics('output', joint_angle_array)
    z_base = transforms[0][2,3] # z coordinate of J1_base, used as floor threshold
    # (x, y, z) coordinates of the gripper given input joint angles
    x_gripper = transforms[-1][0,3]
    y_gripper = transforms[-1][1,3]
    z_gripper = transforms[-1][2,3]
    # returns True if gripper is below the floor threshold or within radius R of the base origin
    return z_gripper <= (z_base + z_delta) or np.sqrt(x_gripper ** 2 + y_gripper ** 2 + z_gripper ** 2) <= R

if __name__ == "__main__":
    # joint angles (radians) where gripper is below the floor, should return True
    true_case1 = np.array([0., -0.18826023, 0., 0., 0., 0.])
    # joint angles (radians) where gripper is within the the radius, should return True
    true_case2 = np.array([0., -0.18826023, 2.27088904, 0., 0., 0.])
    # joint angles (radians) where gripper is above the floor and not within the radius, should return False
    false_case = np.array([0., 1.55314696, 0., 0., 0., 0.])
    assert check_collision(true_case1) == True
    assert check_collision(true_case2) == True
    assert check_collision(false_case) == False