#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Empty,String
from gps_handler.srv import *
from proxy_eura_smach.msg import ErrorMessage

global waitGPSData

def casesCallback(msg):
    global waitGPSData
    global lastGPS
    t1 = msg.data.split('\n')
    lastGPS = t1[:-1].split(";")

def waitForGPSData(AI,time):
    global waitGPSData,lastGPS
    start = rospy.get_time()
    s = rospy.Subscriber('gps_string',String,casesCallback)
    while waitGPSData and rospy.get_time()-start < time:
        if AI.preempt_requested():
                ROS_INFO("Go building GPS is being preempted")
                AI.service_preempt()
                return 'preempted'
        rospy.sleep(1.0/20.0)
    if not waitGPSData:
        return lastGPS
    else: 
        return 'Error'
