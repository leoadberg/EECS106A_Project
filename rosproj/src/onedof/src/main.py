#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
import sys

import roslib
roslib.load_manifest("ur_kinematics")
from ur_kinematics.ur_kin_py import forward, inverse

#Wait for the IK service to become available
rospy.wait_for_service('compute_ik')
rospy.init_node('service_query')
arm = 'right'
#Create the function used to call the service
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
	
right_gripper = robot_gripper.Gripper('right')
right_gripper.calibrate()

while not rospy.is_shutdown():
    raw_input('Press [ Enter ]: ')
    
    #Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    #Alan does not have a gripper so replace link with 'right_wrist' instead
    link = arm + "_gripper"

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Pick Up
    #- Translation: [0.675, -0.499, -0.095]
    #- Rotation: in Quaternion [0.005, 1.000, -0.004, 0.007]

    # Place
    #- Translation: [0.973, -0.419, -0.090]
    #- Rotation: in Quaternion [0.022, 0.980, -0.012, 0.199]

    
    #- Translation: [0.781, -0.450, -0.081]
    #- Rotation: in Quaternion [-0.089, 0.906, 0.091, 0.405]
    
    #- Translation: [1.071, -0.247, -0.085]
    #- Rotation: in Quaternion [-0.080, 0.892, 0.045, 0.443]


    
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = 0.675
    request.ik_request.pose_stamped.pose.position.y = -0.499
    request.ik_request.pose_stamped.pose.position.z = -0.095        
    request.ik_request.pose_stamped.pose.orientation.x = 0.005
    request.ik_request.pose_stamped.pose.orientation.y = 1.000
    request.ik_request.pose_stamped.pose.orientation.z = -0.004
    request.ik_request.pose_stamped.pose.orientation.w = 0.007
    
    try:
        #Send the request to the service
        response = compute_ik(request)
        
        #Print the response HERE
        print(response)
        group = MoveGroupCommander(arm + "_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        ###group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK and execute
        group.go()
        right_gripper.close()
        rospy.sleep(1.0)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
    # Place
    request.ik_request.pose_stamped.pose.position.x = 0.973
    request.ik_request.pose_stamped.pose.position.y = -0.419
    request.ik_request.pose_stamped.pose.position.z = -0.090        
    request.ik_request.pose_stamped.pose.orientation.x = 0.022
    request.ik_request.pose_stamped.pose.orientation.y = 0.980
    request.ik_request.pose_stamped.pose.orientation.z = -0.012
    request.ik_request.pose_stamped.pose.orientation.w = 0.199
    
    try:
        #Send the request to the service
        response = compute_ik(request)
        
        #Print the response HERE
        print(response)
        group = MoveGroupCommander(arm + "_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        ###group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK and execute
        group.go()
        right_gripper.open()
        rospy.sleep(1.0)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e