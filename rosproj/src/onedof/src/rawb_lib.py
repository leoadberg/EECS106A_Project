#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys

import tf
import geometry_msgs.msg

import roslib
roslib.load_manifest("ur_kinematics")
from ur_kinematics.ur_kin_py import forward, inverse

import test_ursim

# start_q = [0, -0.911, 1.23, 1.26, 1.57, 0]
start_q = [0.0005898475646972656, -1.290287808781006, 1.4657748381244105, 1.3952933984943847, 1.5710463523864746, -0.0010102430926721695]

zero_q = [0, 0, 0, 0, 0, 0]

bounds = [[-0.2, 0.2], [-0.3, 0.3], [-0.1, 0.3]]

base_rot = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]])

def gen_z_rot(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

def angle_to_xy(theta):
    return [np.cos(theta), np.sin(theta)]

def clamp(n, smallest, largest):
    ret = max(smallest, min(n, largest))
    # if (ret != n):
    #     print("clamped")
    return ret

def q_to_pos(q):
    pos_mat = forward(np.array(q))
    # print(pos_mat)
    xyz = [pos_mat[0, 3], pos_mat[1, 3], pos_mat[2, 3]]
    return xyz, pos_mat

_, start_mat = q_to_pos(start_q)

turning_rate = 0.2
def fix_mat(mat, theta=0):
    x = mat[0, 3]
    y = mat[1, 3]
    z = mat[2, 3]

    start_x = start_mat[0, 3]
    start_y = start_mat[1, 3]
    start_z = start_mat[2, 3]

    # Enforce that position is within bounds
    mat[0, 3] = clamp(mat[0, 3], start_x + bounds[0][0], start_x + bounds[0][1])
    mat[1, 3] = clamp(mat[1, 3], start_y + bounds[1][0], start_y + bounds[1][1])
    mat[2, 3] = clamp(mat[2, 3], start_z + bounds[2][0], start_z + bounds[2][1])

    # Enforce that end effector is vertical
    close_mat = np.isclose(base_rot, mat[0:3, 0:3], atol=0.01)
    if not (close_mat[0, 0] and close_mat[1, 0] and close_mat[2, 0] and close_mat[2, 1] and close_mat[2, 2]):
        print(base_rot)
        print(mat[0:3, 0:3])
        print(close_mat)
        print("Bad current orientation")
        sys.exit(1)

    # Interpolate between current angle and new angle
    current_angle = -np.arcsin(mat[0, 1])
    print(current_angle)
    new_angle = clamp(theta, current_angle - turning_rate, current_angle + turning_rate)
    mat[0:3, 0:3] = gen_z_rot(theta).dot(base_rot)
    # print(mat)

def pos_to_q(mat, guess):
    possible_qs = inverse(mat, 0)

    # print("Possible IK Solutions:")
    # print(possible_qs)

    best_sol_ind = np.argmin(np.sum(possible_qs - guess)**2)

    # print("Best solution: ", best_sol_ind)

    # print(np.shape(possible_qs))
    ret = possible_qs[best_sol_ind].tolist()

    # print("ret", ret)
    # print("guess", guess)

    if ret[0] > np.pi:
        # print("**** Wrapping base")
        ret[0] -= 2 * np.pi
    if ret[5] > np.pi:
        # print("**** Wrapping base")
        ret[5] -= 2 * np.pi
    ret[1] -= 2 * np.pi

    # print("ret2", ret)

    error = np.sum(np.array(ret) - guess)**2

    # print(error)
    if error > 1:
        return None

    return ret

homerate = rospy.Rate(200)
def move_home():
    print("Moving to home...")
    while not rospy.is_shutdown():
        cur_q = test_ursim.getq()
        diff = [clamp(start_q[i] - cur_q[i], -0.01, 0.01) for i in range(6)]
        if sum(map(abs, diff)) < 0.001:
            break
        next_q = [cur_q[i] + diff[i] for i in range(6)]
        test_ursim.setq(next_q)
        homerate.sleep()
    print("Got home")

def dot(a, b):
    if len(a) != len(b):
        print("Dotting different length vectors")
    return [a[i] * b[i] for i in range(len(a))]

def scale(a, b):
    return [x * b for x in a]

def mag(v):
    return np.sqrt(sum([x**2 for x in v]))
