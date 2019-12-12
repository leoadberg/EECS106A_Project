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


# start_q = [0, -0.911, 1.23, 1.26, 1.57, 0]
start_q = [0.0005898475646972656, -1.290287808781006, 1.4657748381244105, 1.3952933984943847, 1.5710463523864746, -0.0010102430926721695]

zero_q = [0, 0, 0, 0, 0, 0]

bounds = [[-0.2, 0.2], [-0.3, 0.3], [-0.1, 0.3]]

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

def fix_mat(mat):
    x = mat[0, 3]
    y = mat[1, 3]
    z = mat[2, 3]

    start_x = start_mat[0, 3]
    start_y = start_mat[1, 3]
    start_z = start_mat[2, 3]

    mat[0, 3] = clamp(mat[0, 3], start_x + bounds[0][0], start_x + bounds[0][1])
    mat[1, 3] = clamp(mat[1, 3], start_y + bounds[1][0], start_y + bounds[1][1])
    mat[2, 3] = clamp(mat[2, 3], start_z + bounds[2][0], start_z + bounds[2][1])

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
