#!/usr/bin/env python

import csv
import threading
import os
import socket
import sys
import logging
import time
import argparse
import datetime
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import numpy as np
import matplotlib.pyplot as plt


from numpy import sin,sign,eye, zeros
from numpy.linalg import norm,inv,det
from math import pi,sqrt,atan2,sin,cos, floor

previous_error=0
intergral=0
previous_tau=0
tau_intergral=0

K=5*pi/180
J=0
U=0.05


P=0.19
I=0
D=0.01
def interpol(vec,dt,t):
    i = int(t/dt)
    try:
        val = (vec[i+1]-vec[i])*(t%dt)/dt+vec[i]
    except:
        val = vec[-1]
        # print("Sequence Finished")
    return val
def current_theta(all,t):
    pass

def average(l1, l2, weight):
    l = []
    for i in range(len(l1)):
        l.append(l1[i] * weight + l2[i] * (weight - 1))
    return l

# Rotation Matrix Equations
#returns an array of the basic roation matrices(using right hand rule with codifiyes their alternating signs)
def Rx(roll):
    c,s = np.cos(roll),np.sin(roll)
    R = np.array([[1,0,0],[0,c,-s],[0,s,c]])
    return R

def Ry(pitch):
    c,s = np.cos(pitch),np.sin(pitch)
    R = np.array([[c,0,s],[0,1,0],[-s, 0, c]])
    return R

def Rz(yaw):
    c,s = np.cos(yaw),np.sin(yaw)
    R = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return R
# Frame transformation equations
#"changing the perspective of descibring the vector from one to another-alters the components not the vectors"
def transformMatrix(pose,theta):
    Ro = np.array([[-np.cos(theta+pi/2),np.sin(theta+pi/2),0],[0,0,-1],[-np.sin(theta+pi/2),-np.cos(theta+pi/2),0]])
    Po = np.array(pose[0:3]).reshape(3,1)
    Gst0 = np.concatenate((np.concatenate((Ro,Po),axis=1),np.array([[0,0,0,1]])),axis=0)
    # print(Gst0)
    return Gst0

def best_sol(sols, q_guess, weights):
    #from ur_kinetmatics
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6)*9999.
        for i in range(6):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2.*np.pi and
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if len(valid_sols) == 0:
        return None
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
    return valid_sols[best_sol_ind]

bto0 = np.array([[-1,0,0,0],
                 [0,-1,0,0],
                 [0,0,1,0],
                 [0,0,0,1]])
sixtoee = np.array([[0,-1,0,0],
                 [0,0,-1,0],
                 [1,0,0,0],
                 [0,0,0,1]])

#the defualt angles-for reference
joint_angles = [0.0, -0.61, -2.09, -2.53, 0, 0.52]
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joint_angles = [0.0, -0.6108652381980155, -2.094395102393195, -2.530727415391778, -2.7000623958883807e-13, 0.5235987755982989-pi]


#the default pose-for reference
pose = [0.09365703331396004, -0.23290000005331224, 0.5221708443498532, 1.2091995885951452, -1.2091995571574645, 1.209199556923491]



sim = True

if sim:
    HOST = "localhost" # The remote host
    freq = 3200
else:
    HOST = "192.168.1.2" # The remote host
    freq = 4500

#parameters- adds list of options to run (like the help lists)
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='192.168.1.2', help='name of host to connect to (localhost)')
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
parser.add_argument('--samples', type=int, default=freq, help='number of samples to record')
parser.add_argument('--frequency', type=int, default=100, help='the sampling frequency in Herz')
parser.add_argument('--config', default='bitupdate.xml', help='data configuration file to use (record_configuration.xml)')
parser.add_argument('--output', default='robot_data.csv', help='data output file to write to (robot_data.csv)')
parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
args = parser.parse_args()

blocking = 1/float(args.frequency)
lookahead = 0.05
gain = 800
maxPositionError =    0.0000000001
maxOrientationError = 0.0000000001
now = datetime.datetime.now()

if args.verbose:
    logging.basicConfig(level=logging.INFO)

conf = rtde_config.ConfigFile(args.config)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(HOST, args.port)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
if not con.send_output_setup(state_names, state_types, frequency = args.frequency):
    logging.error('Unable to configure output')
    sys.exit()
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

#start data synchronization
if not con.send_start():
    logging.error('Unable to start synchronization')
    sys.exit()


# Setpoints to move the robot to

setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]


setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0
setp.input_double_register_6 = 0
setp.input_double_register_7 = 0
setp.input_double_register_8 = 0
setp.input_double_register_9 = 0
setp.input_double_register_10 = 0
setp.input_double_register_11= 0
# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0
#these are only called when the code is ready to send the information to the robot
def setp_to_list(setp):
    list = []
    for i in range(0,12):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,12):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

if sim:
    poser = False
    T = 2
    waitingtime = 3
    pausetime = 3
    stopballtime = 2
    stopballtimewait = 2
else:
    poser = False
    T = 5
    waitingtime = 5
    pausetime = 5
    stopballtime = 3
    stopballtimewait = 2
avg = 120.0
body_frame = False
switch_state = 0

tinit = time.time()
tnow = time.time()
tstart = time.time()
i = 1
keep_running = True

start_q = [0, -0.911, 1.23, 1.26, 1.57, 0]
zero_p = [0, 0, 0, 0, 0, 0]
while keep_running:
    try:
        state = con.receive()
        if state is not None:
            tnow = time.time()-tnow
            avg +=1/tnow
            # print(avg/i,1/tnow)
            current_q = state.actual_q
            tnow = time.time()
            elapsed_time = time.time()-tinit
            time_manuever = time.time()-tstart

            if time_manuever < blocking:
                continue

            tstart = time.time()

            # print(current_q)

            partial_q = average(current_q, start_q, 0)
            print(partial_q)

            message_list = start_q + [1, blocking, lookahead, gain, maxPositionError, maxOrientationError]

            list_to_setp(setp, message_list)
            con.send(setp)
            #print(state.actual_TCP_force[3])
            state.speed_scaling = time.time()
            state.target_speed_fraction = sim
            state.joint_temperatures = message_list[0:6]
            state.safety_mode = switch_state
        else:
            sys.exit()

    except KeyboardInterrupt:
        keep_running = False
        setp1 = current_q + [1, blocking, lookahead, gain, maxPositionError, maxOrientationError]
        message_list = current_q
        list_to_setp(setp, message_list)
        con.send(setp)
    i += 1
sys.stdout.write("\rComplete!{:3d} samples.\n".format(i))

con.send_pause()
con.disconnect()
time.sleep(0.5)
