#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
sys.path.append('/home/mpc-ubuntu/EECS106A_Project/rtde-2.2.3/')
import test_ursim

import tf
import geometry_msgs.msg

import roslib
roslib.load_manifest("ur_kinematics")
from ur_kinematics.ur_kin_py import forward, inverse

# rospy.wait_for_service('compute_ik')
rospy.init_node('tf_listener')
listener = tf.TransformListener()

rate = rospy.Rate(10)
# while True:
#     if listener.frameExists("ar_marker_4") and listener.frameExists("ar_marker_1"):
#         # t = listener.getLatestCommonTime("camera_color_optical_frame", "ar_marker_0")
#         t = listener.getLatestCommonTime("ar_marker_4", "ar_marker_1")
#         # pos, quat = listener.lookupTransform("camera_color_optical_frame", "ar_marker_0", t)
#         pos, quat = listener.lookupTransform("ar_marker_4", "ar_marker_1", t)
#         print(pos, quat)
#     else:
#         print("Frames don't exist")
#         print(listener.getFrameStrings())
#     rate.sleep()

#     if rospy.is_shutdown():
#         sys.exit(1)

start_q = [0, -0.911, 1.23, 1.26, 1.57, 0]
zero_q = [0, 0, 0, 0, 0, 0]

bounds = 0.1 # 10 cm from start pos


def q_to_pos(q):
    pos_mat = forward(np.array(q))
    # print(pos_mat)
    xyz = [pos_mat[0, 3], pos_mat[1, 3], pos_mat[2, 3]]
    return xyz, pos_mat

_, start_mat = q_to_pos(start_q)

def fix_mat(mat):
    x = mat[0, 3]
    y = mat[1, 3]
    z = mat[2, 3]

    start_x = start_mat[0, 3]
    start_y = start_mat[1, 3]
    start_z = start_mat[2, 3]

    if abs(start_x - x) >= bounds:
        print("Hitting X bound")
        if start_x < x:
            mat[0, 3] = start_x + bounds
        else:
            mat[0, 3] = start_x - bounds

    if abs(start_y - y) >= bounds:
        print("Hitting Y bound")
        if start_y < y:
            mat[1, 3] = start_y + bounds
        else:
            mat[1, 3] = start_y - bounds

    if abs(start_z - z) >= bounds:
        print("Hitting Z bound")
        if start_z < z:
            mat[2, 3] = start_z + bounds
        else:
            mat[2, 3] = start_z - bounds

    mat[0:2, 0:2] = 0
    mat[0, 2] = -1
    mat[1, 1] = -1
    mat[2, 0] = -1

def pos_to_q(mat, guess):
    possible_qs = inverse(mat, 0)

    # print("Possible IK Solutions:")
    # print(possible_qs)

    best_sol_ind = np.argmin(np.sum(possible_qs - guess)**2)

    # print("Best solution: ", best_sol_ind)

    # print(np.shape(possible_qs))
    ret = possible_qs[best_sol_ind].tolist()

    ret[1] -= 2 * np.pi

    error = np.sum(np.array(ret) - guess)**2

    # print(error)
    if error > 1:
        return None

    return ret

def clamp(n, smallest, largest):
    ret = max(smallest, min(n, largest))
    if (ret != n):
        print("clamped")
    return ret

# Go to start before anything happens
print("Moving to home...")
while not rospy.is_shutdown():
    cur_q = test_ursim.getq()
    diff = [clamp(start_q[i] - cur_q[i], -0.01, 0.01) for i in range(6)]
    if sum(map(abs, diff)) < 0.01:
        break
    next_q = [cur_q[i] + diff[i] for i in range(6)]
    test_ursim.setq(next_q)
    rate.sleep()
print("Got home")

jiggles = 0

while not rospy.is_shutdown():
    try:
        # print(cur_pos)

        # pos_mat[2, 3] += (abs((time.time() % 4) - 2) - 1) / 5

        # if listener.frameExists("ar_marker_4") and listener.frameExists("ar_marker_1")and listener.frameExists("ar_marker_7"):
        if listener.frameExists("ar_marker_4") and listener.frameExists("ar_marker_1"):
            try:
                t = listener.getLatestCommonTime("ar_marker_4", "ar_marker_1")
                pos, quat = listener.lookupTransform("ar_marker_4", "ar_marker_1", t)

                # t2 = listener.getLatestCommonTime("ar_marker_1", "ar_marker_7")
                # diff_pos, _ = listener.lookupTransform("ar_marker_1", "ar_marker_7", t2)
            except:
                print("Lost a tracker")
                continue
            # print(pos, quat)

            cur_q, cur_force = test_ursim.getqforce()
            cur_pos, pos_mat = q_to_pos(cur_q)
            # print("cur_pos", cur_pos)

            # Modify Z based on angle in the X axis
            # pos_mat[2, 3] += clamp(quat[0] * 0.4, -0.04, 0.04)

            # Modify Y based on angle in the Z axis
            # pos_mat[1, 3] += clamp(quat[2] * 0.4, -0.04, 0.04)

            # Modify X based on translation between markers
            # diff_x = diff_pos[0]
            # if (diff_x > 0.6 and diff_x < 1):
            #     pos_mat[0, 3] += clamp((diff_x - 0.7) * 0.01, -0.01, 0.01)
            # print(diff_pos)

            # print(pos_mat)
            fix_mat(pos_mat) #mutates pos_mat
            # print(pos_mat)
            desired_q = pos_to_q(pos_mat, cur_q)
            # print(desired_q)
            if desired_q:
                test_ursim.setq(desired_q)
            else:
                # if jiggles < 10:
                #     print("IK within range not found, jiggling joints")
                #     new_q = [cur_q + ]
                # else:
                #     print("IK within range not found, stopping")
                #     break
                print("IK solution not found")
        else:
            print("Frames don't exist")
            # print(listener.getFrameStrings())
        rate.sleep()

        # print(inverse.__code__.co_varnames)

        # print(pos_mat)

        

    except KeyboardInterrupt:
        # test_ursim.current_q = getq()
        # test_ursim.setq(current_q)
        break

test_ursim.end()