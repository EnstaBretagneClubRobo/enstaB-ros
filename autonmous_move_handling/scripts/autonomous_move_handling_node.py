#!/usr/bin/env python

import rospy
import tf
from std_msgs import Empty,Int8


global posGPSBuilding,listener

posGPSBuilding = [[42.555,10.6]]

def ShutdownCallback():
    rospy.loginfo("Shutdown autom_handling") 

def stuck_cb(msg):
    print "tt"


def astar_arriv_cb(msg):
    global posGPSBuilding,listener
    #launch other tf form posGPSBuilding
    os.system("rosrun tf static_transforme_publisher %f %f %f 0 0 0 /map /gps_origin 100 &")%(posGPSBuilding[i][0],posGPSBuilding[i][1],0)
    print "tt"
     

rospy.init_node('autom_handling')
rospy.on_shutdown(ShutdownCallback)
listener = tf.TransformListener()
rospy.Subscriber("/stuck_msg",Empty,stuck_cb)
rospy.Subscriber("/astar_arrived",Empty,astar_arriv_cb)
rospy.spin()
