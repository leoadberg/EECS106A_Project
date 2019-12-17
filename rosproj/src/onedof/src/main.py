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

from rawb_lib import *

# rospy.wait_for_service('compute_ik')
rospy.init_node('ar_controller')
listener = tf.TransformListener()

rate = rospy.Rate(500)

# Go to start before anything happens
move_home()

averaged_quat = [0, 0, 0]

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
            # print("cur_force", cur_force)

            print("pos", pos)
            print("cur_pos", cur_pos)

            for i in range(3):
                averaged_quat[i] = averaged_quat[i] * 0.5 + quat[i] * 0.5

            # Modify Z based on angle in the X axis
            pos_mat[2, 3] += clamp(averaged_quat[0] * 0.2, -0.02, 0.02)

            # Modify Y based on angle in the Z axis
            pos_mat[1, 3] += clamp(averaged_quat[2] * 0.2, -0.02, 0.02)

            # Modify X based on translation between markers
            # diff_x = diff_pos[0]
            # if (diff_x > 0.6 and diff_x < 1):
            #     pos_mat[0, 3] += clamp((diff_x - 0.7) * 0.01, -0.01, 0.01)
            # print(diff_pos)

            # print(pos_mat)
            fix_mat(pos_mat) #mutates pos_mat
            desired_q = pos_to_q(pos_mat, cur_q)
            if desired_q:
                test_ursim.setq(desired_q)
            else:
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
