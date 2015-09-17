#!/usr/bin/env python

import rospy
import tf
from std_msgs import Empty,Int8
from nav_msgs import OccupancyGrid
from autonmous_move_handling.srv import *

global posGPSBuilding,listener,i
i = 0
posGPSBuilding = [[42.555,10.6]]

def ShutdownCallback():
    rospy.loginfo("Shutdown autom_handling") 

def stuck_cb(msg):
    print "tt"


def astar_arriv_cb(msg):
    global posGPSBuilding,listener,i
    #launch other tf form posGPSBuilding
    os.system("ruby /home/nuc1/ruby/TfFlaunchFilecreate.rb %s %f %f %f %f %f %f %s %s"%("map_gps_tf",posGPSBuilding[i][0],posGPSBuilding[i][1],0,0,0,0,"/map","/gps_origin"))
    os.system("roslaunch /home/nuc1/ruby/tf_static.launch &")
    i=i+1
     
def map_cb(map1):
    unknown=map1.data.count(-1)
    width = map1.info.width
    if float(unknown)/float(width**2) < 0.3:
       global finishMapping
       finishMapping = True; 

def handle_get_mapping_status(req):
    global finishMapping,map_sub,unregist
    if finishMapping and not unregist :
       map_sub.unregister()
       unregist = True
    return GetMappingStatusResponse(finishMapping)
  
global map_sub,finishMapping,unregist
finishMapping = False
unregist = False
rospy.init_node('autom_handling')
rospy.on_shutdown(ShutdownCallback)
listener = tf.TransformListener()
s = rospy.Service('/get_mapping_status', GetMappingStatus, handle_get_mapping_status)
rospy.Subscriber("/stuck_msg",Empty,stuck_cb)
rospy.Subscriber("/astar_arrived",Empty,astar_arriv_cb)
map_sub = rospy.Subscriber("/map",OccupancyGrid,map_cb)
rospy.spin()
