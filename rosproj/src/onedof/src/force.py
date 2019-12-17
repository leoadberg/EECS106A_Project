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
rospy.init_node('force_controller')
listener = tf.TransformListener()

rate = rospy.Rate(100)


# Go to start before anything happens
move_home()

averaged_force = [0, 0, 0, 0, 0, 0]

while not rospy.is_shutdown():
    try:

        cur_q, cur_force = test_ursim.getqforce()
        cur_pos, pos_mat = q_to_pos(cur_q)
        # print("cur_pos", cur_pos)
        # print("cur_force", cur_force)
        # print("cur_q", cur_q)

        # Calibrate for tray
        # cur_force[2] += 6.6
        # cur_force[2] *= 5

        # Modify XYZ based on force in those axes
        for i in range(len(averaged_force)):
            averaged_force[i] = averaged_force[i] * 0.97 + cur_force[i] * 0.03

        print(averaged_force)

        pos_mat[0, 3] += clamp(-averaged_force[0] * 0.002, -0.01, 0.01)
        pos_mat[1, 3] += clamp(-averaged_force[1] * 0.001, -0.01, 0.01)
        # pos_mat[1, 3] += clamp(averaged_force[5] * 0.005, -0.01, 0.01)
        pos_mat[2, 3] += clamp(averaged_force[2] * 0.01, -0.01, 0.01)
        # pos_mat[2, 3] += 0.01

        # print(pos_mat)
        fix_mat(pos_mat) #mutates pos_mat
        desired_q = pos_to_q(pos_mat, cur_q)
        if desired_q:
            test_ursim.setq(desired_q)
        else:
            print("IK solution not found")

        rate.sleep()



    except KeyboardInterrupt:
        # test_ursim.current_q = getq()
        # test_ursim.setq(current_q)
        break

test_ursim.end()
