#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Empty,String,Int8
from gps_handler.srv import *
from ai_mapping_robot.msg import ErrorMessage
from ai_mapping_robot.msg import InitData
import tf.transformations as trans
from math import *
from pwm_serial_py.srv import Over_int



############### wait Init Data ##############################
def initDataCallback(msg):
    global waitInitDataMsg
    global initDataMsg
    initDataMsg = msg
    waitInitDataMsg = 0

def waitForInitData(time):
    global waitInitDataMsg,initDataMsg
    start = rospy.get_time()
    waitInitDataMsg = 1
    rospy.loginfo("wait InitData ...")
    s = rospy.Subscriber('init_data',InitData,initDataCallback)
    while waitInitDataMsg and rospy.get_time()-start < time:
        rospy.sleep(1.0/20.0)
    s.unregister()
    if not waitInitDataMsg:
        return initDataMsg
    else: 
        return 'Error'




#################Wait GPS List of waypoints ###################
def casesCallback(msg):
    global waitGPSData
    global lastGPS,t1
    t1 = msg.data.split('\n')
    lastGPS = t1[:-1].split(";")
    waitGPSData = 0

def waitForGPSData(AI,time):
    global waitGPSData,lastGPS,t1
    start = rospy.get_time()
    waitGPSData = 1
    s = rospy.Subscriber('gps_string',String,casesCallback)
    rospy.loginfo("wait GPSData ...")
    while waitGPSData and rospy.get_time()-start < time:
        if AI.preempt_requested():
                rospy.loginfo("Go building GPS is being preempted")
                AI.service_preempt()
                return 'preempted'
        rospy.sleep(1.0/20.0)
    s.unregister()
    if not waitGPSData:
        return (lastGPS,t1)
    else: 
        return 'Error'

################ Entry Init ######################################

def findHeading(listener,cmdPublisher,heading):
    (r,p,yaw)=(0,0,0)
    try:
       (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       (r,p,yaw) = trans.euler_from_quaternion(rot1)
    e = heading-yaw;
    #insert lidar data
    while abs(e)>0.1:
        try:
           (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
           rospy.loginfo("echec local_origin fcu")
        (r,p,yaw) = trans.euler_from_quaternion(rot1)
        err= heading-yaw
        u=20*(2/PI)*atan(tan(err/2));#atan for modulo 2*pi*/
        sendCommand(1500,1500+u);
    sendCommand(1500,1500);


def sendCommand(channelSpeed,channelYaw):
    try:
        send_pwm = rospy.ServiceProxy('/pwm_serial_send',Over_int)
        resp1 = send_pwm([channelSpeed,0,channelYaw,0,0,0,0,0])
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed : %s"%e

def dataCallback(msg):
    global waitDataMsg
    waitDataMsg = 0

def waitForRemote(time):
    global waitDataMsg
    start = rospy.get_time()
    waitDataMsg = 1
    rospy.loginfo("wait For Remote ...")
    s = rospy.Subscriber('/restart_msg',Int8,dataCallback)
    while waitDataMsg and rospy.get_time()-start < time:
        rospy.sleep(1.0/20.0)
    s.unregister()
    return not waitDataMsg


############ Restart #########################
def remoteGoCallback(msg):
    global waitRestartMsg,message
    waitRestartMsg = 0
    message = msg

def waitForRemoteGo(time):
    global waitRestartMsg,message
    message = Int8(0)
    start = rospy.get_time()
    waitRestartMsg = 1
    rospy.loginfo("wait RemoteGo ...")
    s = rospy.Subscriber('/restart_msg',Int8,remoteGoCallback)
    while waitRestartMsg and rospy.get_time()-start < time:
        rospy.sleep(1.0/20.0)
    s.unregister()
    return message
    
