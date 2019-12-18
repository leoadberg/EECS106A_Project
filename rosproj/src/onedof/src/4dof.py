#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
import subprocess
sys.path.append('/home/mpc-ubuntu/EECS106A_Project/rtde-2.2.3/')

import tf
import geometry_msgs.msg

import roslib
roslib.load_manifest("ur_kinematics")
from ur_kinematics.ur_kin_py import forward, inverse

rospy.init_node('fourdof_controller')
from rawb_lib import *

# rospy.wait_for_service('compute_ik')
listener = tf.TransformListener()

rate = rospy.Rate(500)

# Move to start before anything happens
move_home()

# Used to smooth sensor input
averaged_quat = [0, 0, 0, 0]
averaged_force = [0, 0, 0, 0, 0, 0]

tray_radius = 0.4 # Probably needs to be updated, this is in the scale of the AR tags which might not be reality

set_angle = 0
set_pos = [0, 0, 0]
translation = True

maxspeed = 0.02

# TODO: Figure this out
# lastzforce = 0
lastzforces = [0] * 10
lastknocktime = 0
def detect_knock(zforce):
    global lastzforces
    global lastknocktime
    # deltazforce = zforce - lastzforce
    deltazforce = zforce - max(lastzforces)
    lastzforces = lastzforces[1:] + [zforce]
    curtime = time.time()
    deltatime = curtime - lastknocktime
    ret = False
    if deltazforce < -5 and deltatime > 1: # Single knock detected
        print(deltazforce)
        lastknocktime = curtime

        if deltatime < 2:
            ret = True
            lastknocktime = 0
            print("Double knock")
        else:
            print("Single knock")
    return ret

while not rospy.is_shutdown():
    try:
        if listener.frameExists("ar_marker_4") and listener.frameExists("ar_marker_1"):
            try:
                t = listener.getLatestCommonTime("ar_marker_4", "ar_marker_1")
                ar_pos, quat = listener.lookupTransform("ar_marker_4", "ar_marker_1", t)
            except:
                print("Lost a tracker")
                continue

            cur_q, cur_force = test_ursim.getqforce()
            cur_pos, pos_mat = q_to_pos(cur_q)

            # print(pos_mat)
            # current_angle = -np.arcsin(pos_mat[0, 1])
            # print(current_angle)
            # continue

            for i in range(4):
                averaged_quat[i] = averaged_quat[i] * 0.5 + quat[i] * 0.5
            for i in range(3):
                averaged_force[i] = averaged_force[i] * 0.95 + cur_force[i] * 0.05

            if detect_knock(cur_force[2]):
                translation = not translation
                print("Switching to " + ("translational " if translation else "rotational") + " mode")
                subprocess.Popen(["paplay", "/usr/share/sounds/ubuntu/notifications/" + ("Positive" if translation else "Slick") + ".ogg"])

            cur_roll, cur_pitch, cur_yaw = tf.transformations.euler_from_quaternion(averaged_quat)

            if translation:
                # 3DOF Translational mode

                # Figure out the intended forward/backward adjustment
                xy_force = [averaged_force[0], averaged_force[1]]
                angle_unit_vector = angle_to_xy(set_angle)
                forward_back_adjust = scale(dot(xy_force, angle_unit_vector), 0.01)

                # Figure out the side-to-side forward/backward adjustment
                yaw_delta = cur_yaw - set_angle
                side_to_side_vector = angle_to_xy(set_angle + np.pi / 2)
                side_to_side_adjust = scale(side_to_side_vector, yaw_delta)

                # Sum the adjustment vectors (should be orthogonal)
                total_adjust_x = -1 * forward_back_adjust[0] + side_to_side_adjust[0]
                total_adjust_y = -1 * forward_back_adjust[1] + side_to_side_adjust[1]

                # Get pitch in the set_angle direction
                pitch = sum(dot(averaged_quat[0:2], angle_unit_vector))

                # Modify Z based on pitch
                pos_mat[2, 3] += clamp(pitch * 0.1, -maxspeed, maxspeed)

                # Modify Y based on adjustment
                pos_mat[1, 3] += clamp(total_adjust_y * 0.1, -maxspeed, maxspeed)

                # Modify X based on adjustment
                pos_mat[0, 3] += clamp(total_adjust_x * 0.1, -maxspeed, maxspeed)

                # Set set_pos to AR tag pos
                set_pos = [ar_pos[0], ar_pos[1], ar_pos[2]]

            else:
                # 1DOF Rotational mode
                # The main idea is to position the tray such that the AR tag is at set_pos without needing to move the hand (or only move the hand inward/outward)
                # We SHOULD be able to get away with only using AR (x,y) position here

                delta_x = ar_pos[0] - set_pos[0]
                delta_y = ar_pos[0] - set_pos[0]

                current_angle_vec = angle_to_xy(cur_yaw)

                # Figure out the gripper's position based on ar_pos and angle
                gripper_x = ar_pos[0] + tray_radius * current_angle_vec[0]
                gripper_y = ar_pos[1] + tray_radius * current_angle_vec[1]

                # Figure out the hand's position based on ar_pos and angle
                # Minus because vec points toward gripper not hand
                hand_x = ar_pos[0] - tray_radius * current_angle_vec[0]
                hand_y = ar_pos[1] - tray_radius * current_angle_vec[1]

                # Figure out angle and vector between hand and set_pos
                hand_center_angle = np.arctan2(hand_y - set_pos[1], hand_x - set_pos[0])
                hand_center_vec = angle_to_xy(hand_center_angle)

                # Figure out where to put gripper based on hand
                desired_gripper_x = set_pos[0] - tray_radius * hand_center_vec[0]
                desired_gripper_y = set_pos[1] - tray_radius * hand_center_vec[1]

                # How much to move the gripper to get it to the desired position
                gripper_delta_x = desired_gripper_x - gripper_x
                gripper_delta_y = desired_gripper_y - gripper_y

                # ***IMPORTANT*** The axes of the AR tag and the robot should be aligned or this might not work
                pos_mat[1, 3] -= clamp(gripper_delta_x * 0.1, -maxspeed, maxspeed)
                pos_mat[0, 3] += clamp(gripper_delta_y * 0.1, -maxspeed, maxspeed)

                # Set set_angle to opposite of hand_center_angle and bound it from -pi to pi
                set_angle = ((hand_center_angle + 2 * np.pi) % (2 * np.pi)) - np.pi

                print("gripper_delta_x", gripper_delta_x)
                print("gripper_delta_y", gripper_delta_y)
                print("set_angle", set_angle)


            fix_mat(pos_mat, set_angle) # Mutates pos_mat
            # fix_mat(pos_mat, 0) # Mutates pos_mat
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
