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
    
    # full self-collision check for non-adjacent joints
    for i in range(len(transforms)):
        x_joint1 = transforms[i][0,3]
        y_joint1 = transforms[i][1,3]
        z_joint1 = transforms[i][2,3]

        for j in range(i + 2, len(transforms)):
            x_joint2 = transforms[j][0,3]
            y_joint2 = transforms[j][1,3]
            z_joint2 = transforms[j][2,3]

            # collision if distance between two non-adjacent joints is within radius R
            if np.sqrt((x_joint2 - x_joint1) ** 2 + (y_joint2 - y_joint1) ** 2 + (z_joint2 - z_joint1) ** 2) <= R:
                return True
            
    # check floor penetration for each joint excluding the base
    for transform in transforms[1:]:
        z_joint = transform[2,3]

        if z_joint <= (z_base + z_delta):
            return True
        
    return False # no collision detected

if __name__ == "__main__":

    # joint angles (radians) where joint is below the floor, should return True
    true_case1 = np.array([0., -0.18826023, 0., 0., 0., 0.])

    # joint angles (radians) where gripper collides with base, should return True
    true_case2 = np.array([ 0., 0.80010599, -3.81226969, 0.37652045, -1.67080963, 0.02353253])

    # joint angles (radians) where two non-adjacent joints collide, should return True
    true_case3 = np.array([ 0., 0., 3.38868427, -6.28318548, -2.96509862, -0.04706506])

    # joint angles (radians) where joint is above the floor and not colliding with itself, should return False
    false_case = np.array([0., 1.55314696, 0., 0., 0., 0.])

    assert check_collision(true_case1) == True
    assert check_collision(true_case2) == True
    assert check_collision(true_case3) == True
    assert check_collision(false_case) == False
    