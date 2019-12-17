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
rospy.init_node('cart_controller')
listener = tf.TransformListener()

rate = rospy.Rate(500)

d_scale1 = 0.6666
d_scale2 = 0.3333

# print(test_ursim.getq())
# test_ursim.end()
# exit(1)

# Go to start before anything happens
move_home()

averaged_quat = [0, 0, 0]
one_back_quat = [0, 0, 0]
averaged_force = [0, 0, 0, 0, 0, 0]
one_back_force = [0, 0, 0, 0, 0, 0]
dx = 0
dy = 0
dz = 0
cart_sum = 0
Kp = 1
Ki = 0
Kd = 0.1

last_delta = [0, 0, 0]
last_pos_cart = None

while not rospy.is_shutdown():
    try:
        if listener.frameExists("ar_marker_4") and listener.frameExists("ar_marker_1") and listener.frameExists("ar_marker_6"):
            try:
                t = listener.getLatestCommonTime("ar_marker_4", "ar_marker_1")
                pos, quat = listener.lookupTransform("ar_marker_4", "ar_marker_1", t)
                t_cart = listener.getLatestCommonTime("ar_marker_1", "ar_marker_6")
                pos_cart, quat_cart = listener.lookupTransform("ar_marker_1", "ar_marker_6", t)
            except:
                print("Lost a tracker")
                last_pos_cart = None

                cur_q = test_ursim.getq()
                _, pos_mat = q_to_pos(cur_q)

                for i in range(3): # Gets intended velocity
                    pos_mat[i, 3] += last_delta[i]
                    last_delta[i] *= 0.95
                fix_mat(pos_mat)

                desired_q = pos_to_q(pos_mat, cur_q)
                if desired_q:
                    test_ursim.setq(desired_q)
                else:
                    print("IK solution not found in stutter prevention")

                continue

            cur_q, cur_force = test_ursim.getqforce()
            cur_pos, pos_mat = q_to_pos(cur_q)

            print("pos_cart", pos_cart)


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

            cart_vel = [0, 0, 0]
            if last_pos_cart:
                for i in range(3):
                    cart_vel[i] = (pos_cart[i] - last_pos_cart[i]) * 0.05

            goal_angle = -pos_cart[1] * 0.05
            print("goal_angle", goal_angle)
            goal_angle = clamp(goal_angle, -0.005, 0.005)
            delta_angle = goal_angle + averaged_quat[0] * 0.1
            cart_sum += delta_angle
            if (Ki * cart_sum > 0.1):
                cart_sum = 0.1 / Ki
                print("rail high")
            elif (Ki * cart_sum < -0.1):
                cart_sum = -0.1 / Ki
                print("rail low")
            clamped_angle = clamp(Kp * delta_angle + Ki * cart_sum + Kd * cart_vel[1], -0.04, 0.04)
            pos_mat[2, 3] += clamped_angle


            # Modify Z based on angle in the X axis
            # pos_mat[2, 3] += clamp(averaged_quat[0] * 0.1 + dz * 0.01, -0.02, 0.02)
            # # Modify Y based on angle in the Z axis
            # pos_mat[1, 3] += clamp(averaged_quat[2] * 0.1 + dy * 0.01, -0.02, 0.02)

            # # Modify X based on force
            # pos_mat[0, 3] += clamp(-averaged_force[0] * 0.001 + dx * 0.00001, -0.01, 0.01)

            fix_mat(pos_mat) # Mutates pos_mat
            for i in range(3): # Gets intended velocity
                last_delta[i] = pos_mat[i, 3] - cur_pos[i]
            last_pos_cart = pos_cart

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
