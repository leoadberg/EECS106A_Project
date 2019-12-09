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


def average(l1, l2, weight):
    l = []
    for i in range(len(l1)):
        l.append(l1[i] * weight + l2[i] * (weight - 1))
    return l


sim = True

if sim:
    # HOST = "localhost" # The remote host
    HOST = "157.131.107.222" # The remote host
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
body_frame = False
switch_state = 0

tinit = time.time()
tstart = time.time()
i = 1

start_q = [0, -0.911, 1.23, 1.26, 1.57, 0]
zero_q = [0, 0, 0, 0, 0, 0]


def getq():
    state = con.receive()
    if state is not None:
        current_q = state.actual_q
        return current_q
    else:
        sys.exit()

def setq(q):
    global tstart

    while time.time()-tstart < blocking:
        pass

    tstart = time.time()

    # print(current_q)

    message_list = q + [1, blocking, lookahead, gain, maxPositionError, maxOrientationError]

    list_to_setp(setp, message_list)
    con.send(setp)

def end():
    con.send_pause()
    con.disconnect()
    time.sleep(0.5)

keep_running = True
if __name__ == '__main__':
    while keep_running:
        try:
            cur_q = getq()
            print(cur_q)
            setq(start_q)

        except KeyboardInterrupt:
            keep_running = False
            current_q = getq()
            setp1 = current_q + [1, blocking, lookahead, gain, maxPositionError, maxOrientationError]
            message_list = current_q
            list_to_setp(setp, message_list)
            con.send(setp)
        i += 1
    sys.stdout.write("\rComplete!{:3d} samples.\n".format(i))

    end()
