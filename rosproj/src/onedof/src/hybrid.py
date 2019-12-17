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
rospy.init_node('hybrid_controller')
listener = tf.TransformListener()

rate = rospy.Rate(500)

d_scale1 = 0.6666
d_scale2 = 0.3333

# Move to start before anything happens
move_home()

averaged_quat = [0, 0, 0]
one_back_quat = [0, 0, 0]
averaged_force = [0, 0, 0, 0, 0, 0]
one_back_force = [0, 0, 0, 0, 0, 0]
dx = 0
dy = 0
dz = 0


while not rospy.is_shutdown():
    try:
        if listener.frameExists("ar_marker_4") and listener.frameExists("ar_marker_1"):
            try:
                t = listener.getLatestCommonTime("ar_marker_4", "ar_marker_1")
                pos, quat = listener.lookupTransform("ar_marker_4", "ar_marker_1", t)
            except:
                print("Lost a tracker")
                continue

            cur_q, cur_force = test_ursim.getqforce()
            cur_pos, pos_mat = q_to_pos(cur_q)

            # Update past values
            one_back_quat = averaged_quat
            one_back_force = averaged_force

            for i in range(3):
                averaged_quat[i] = averaged_quat[i] * 0.5 + quat[i] * 0.5
                averaged_force[i] = averaged_force[i] * 0.97 + cur_force[i] * 0.03

            # Compute derivative
            dz = (averaged_quat[0] - one_back_quat[0]) * d_scale1 + dz * d_scale2
            dy = (averaged_quat[2] - one_back_quat[2]) * d_scale1 + dy * d_scale2
            dx = (averaged_force[0] - one_back_force[0]) * d_scale1 + dx * d_scale2

            # Modify Z based on angle in the X axis
            pos_mat[2, 3] += clamp(averaged_quat[0] * 0.1 + dz * 0.01, -0.02, 0.02)

            # Modify Y based on angle in the Z axis
            pos_mat[1, 3] += clamp(averaged_quat[2] * 0.1 + dy * 0.01, -0.02, 0.02)

            # Modify X based on force
            pos_mat[0, 3] += clamp(-averaged_force[0] * 0.001 + dx * 0.00001, -0.01, 0.01)

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
