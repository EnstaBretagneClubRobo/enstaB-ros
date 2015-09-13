#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from proxy_eura_smach import ErrorMessage

global state #0 init 1 gps 2 gpsA 3teleop 4entry 5 carto 6 proc stop 7 exit 8 exit Teleop  9 ret gps 10 ret GPSA 11 teleop
global dicNodeFail

#error type 0:node failure 1:stuck 2:order from remote  

dicNodeFail[0] = []
dicNodeFail[1] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]
dicNodeFail[2] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]
dicNodeFail[3] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]
dicNodeFail[4] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]
dicNodeFail[5] = ["mavros","pwm_serial","ccny_openni","hokuyo_node","visual_odometry","keyframe_mapper_node","hector_mapping"]
dicNodeFail[6] = ["mavros","pwm_serial","ccny_openni","hokuyo_node","visual_odometry","keyframe_mapper_node","hector_mapping"]
dicNodeFail[7] = ["mavros","pwm_serial","ccny_openni","hokuyo_node","hector_mapping"]
dicNodeFail[8] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]
dicNodeFail[9] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]
dicNodeFail[10] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]
dicNodeFail[11] = ["mavros","pwm_serial","ccny_openni","hokuyo_node"]


def ShutdownCallback():
    print 'shutdown' #send stop messages 



def setActualState(msg):
    global state
    state = msg.data
    

def failing_node_cb(msg):
    global dicNodeFail,state
    if dicNodeFail[state].count(msg.data):
       errorPub.publish(ErrorMessage(type1=0,string1=msg.data))

def remote_change_cb(msg):
    errorPub.publish(ErrorMessage(type1=2,int1=1))

def stuck_cb(msg):
    if [1,2,4,5,7,9,10]
    errorPub.publish(ErrorMessage(type1=1,int1=msg.data))

def drift_cb
    if [5,7].count[state]:
       errorPub.publish(ErrorMessage(type1=1))

rospy.init_node('gps_follow_car')
rospy.on_shutdown(ShutdownCallback)
errorPub = rospy.Publisher("/stop_command",ErrorMessage)
rospy.Subscriber("/set_state_proxy",Int8,setActualState)

rospy.Subscriber("/failing_node",String,failing_node_cb)
rospy.Subscriber("/remote_change_teleop",Empty,remote_change_cb)
rospy.Subscriber("/stuck_msg",Int8,stuck_cb)
rospy.Subscriber("/drift_msg",Empty,drift_cb)

