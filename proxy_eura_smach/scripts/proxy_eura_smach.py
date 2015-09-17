#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from proxy_eura_smach import ErrorMessage

global state #0 init 1 gps 2 gpsA 3teleop 4entry 5 carto 6 proc stop 7 exit 8 exit Teleop  9end exit 10 ret gps 11 ret GPSA 12 teleop
global dicNodeFail,setMonit,errorPub

#error type 0:node failure 1:stuck 2:order from remote  3:autonmous error
kinect =["rgbd_manager","camera_rgb_frame_tf",
         "camera_rgb_optical_frame","debayer",
         "openni_driver","rgbd_image_proc"]
dicNodeFail[0] = []
dicNodeFail[1] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[2] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[3] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[4] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[5] = ["mavros","pwm_serial","hokuyo_node",
                  "visual_odometry","keyframe_mapper_node","hector_mapping"]+kinect
dicNodeFail[6] = ["mavros","pwm_serial","hokuyo_node",
                  "visual_odometry","keyframe_mapper_node","hector_mapping"]+kinect
dicNodeFail[7] = ["mavros","pwm_serial","hokuyo_node","hector_mapping"]+kinect
dicNodeFail[8] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[9] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[10] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[11] = ["mavros","pwm_serial","hokuyo_node"]+kinect
dicNodeFail[12] = ["mavros","pwm_serial","hokuyo_node"]+kinect

neededNode = ["proxy_eura_smach","rc_received_node","gps_handler","pwm_serial"]

def ShutdownCallback():
    print 'shutdown' #send stop messages 



def setActualState(msg):
    global state,dicNodeFail,setMonit
    state = msg.data
    string = ""
    for i in dicNodeFail:
        string = i+'@'
    string = string[:-1]
    setMonit.publish(String(string))
        
    

def failing_node_cb(msg):
    global dicNodeFail,state
    if dicNodeFail[state].count(msg.data):
       errorPub.publish(ErrorMessage(type1=0,string1=msg.data))

def remote_change_cb(msg):
    global errorPub
    errorPub.publish(ErrorMessage(type1=2,int1=1))

def stuck_cb(msg):
    global errorPub,state
    if [1,2,4,5,7,9,10].count[state]
    errorPub.publish(ErrorMessage(type1=1,int1=msg.data))

def drift_cb(msg):
    global errorPub,state
    if [5,7].count[state]:
       errorPub.publish(ErrorMessage(type1=1))

def remote_change_cb(msg):
    global errorPub
    errorPub.publish(ErrorMessage(type1=2,int1=0))#equivalent to going teleop

def remote_stop_cb(msg):
    global errorPub
    errorPub.publish(ErrorMessage(type1=2,int1=1))

 #those message are expected from emergency stop pass the state or go back to the state
def remote_go_cb(msg):
    r = rospy.publisher("/restart_msg",Int8).publish(Int8(0))#ui can publish this message first
    r.unregister()

def remote_pass_cb(msg):
    r = rospy.publisher("/restart_msg",Int8).publish(Int8(1))#ui can publish this message first
    r.unregister()


rospy.init_node('proxy_eura_smach')
rospy.on_shutdown(ShutdownCallback)

errorPub = rospy.Publisher("/stop_command",ErrorMessage)
setMonit = rospy.Publisher("/set_monit_node",String)


rospy.Subscriber("/set_state_proxy",Int8,setActualState)
rospy.Subscriber("/failing_node",String,failing_node_cb)
rospy.Subscriber("/remote_change_teleop",Empty,remote_change_cb)
rospy.Subscriber("/remote_stop",Empty,remote_stop_cb)
rospy.Subscriber("/sig_rc_6",Empty,remote_stop_cb)
rospy.Subscriber("/sig_rc_5",Empty,remote_change_cb)
rospy.Subscriber("/sig_rc_4",Empty,remote_go_cb)
rospy.Subscriber("/sig_rc_3",Empty,remote_pass_cb)
rospy.Subscriber("/stuck_msg",Int8,stuck_cb)
rospy.Subscriber("/drift_msg",Empty,drift_cb)
rospy.Subscriber("/autonomous_error",Empty,auto_cb)
rospy.spin()
