#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Empty,Int8
from nav_msgs.msg import OccupancyGrid
from autonmous_move_handling.srv import *
from autonmous_move_handling.msg import  AstarPoint
from start_node.msg import StartKillMsg

global posGPSBuilding,listener,i
i = 0
posGPSBuilding = [[42.555,10.6]]

def ShutdownCallback():
    rospy.loginfo("Shutdown autom_handling") 

def stuck_cb(msg):
    print "tt"


def astar_arriv_cb(msg):
    global posGPSBuilding,listener,i,exit,startKillPub,finishMapping
    if not exit:
      if i > len(posGPSBuilding):
         skm = StartKillMsg()
         skm.action  = 0
         skm.type = 0
         skm.nId = 6
         startKillPub.publish(skm)#stop astar path
         skm.action  = 0
         skm.type = 0
         skm.nId = 5
         startKillPub.publish(skm)#stop car controller
         finishMapping = True; 
      #launch other tf form posGPSBuilding
      os.system("ruby /home/nuc1/ruby/TfFlaunchFilecreate.rb %s %f %f %f %f %f %f %s %s"%("map_gps_tf",posGPSBuilding[i][0],posGPSBuilding[i][1],0,0,0,0,"/pointtofollow","/gps_origin"))
      os.system("roslaunch /home/nuc1/ruby/tf_static.launch &")
      i=i+1
      
    else:
      skm = StartKillMsg()
      skm.action  = 0
      skm.type = 0
      skm.nId = 6
      startKillPub.publish(skm)#stop astar path
      skm.action  = 0
      skm.type = 0
      skm.nId = 5
      startKillPub.publish(skm)#stop car controller
      
     

def astart_setPoint(msg):
    os.system("ruby /home/nuc1/ruby/TfFlaunchFilecreate.rb %s %f %f %f %f %f %f %s %s"%("map_gps_tf",msg.x,msg.y,0,0,0,0,"/pointtofollow","/gps_origin"))
    os.system("roslaunch /home/nuc1/ruby/tf_static.launch &")

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

def exit_cb(msg):
    global exit
    exit = True

global map_sub,finishMapping,unregist,exit,startKillPub
exit = False
finishMapping = False
unregist = False
rospy.init_node('autom_handling')
rospy.on_shutdown(ShutdownCallback)
listener = tf.TransformListener()
startKillPub = rospy.Publisher("/start_kill_node",StartKillMsg)
s = rospy.Service('/get_mapping_status', GetMappingStatus, handle_get_mapping_status)
rospy.Subscriber("/stuck_msg",Empty,stuck_cb)
rospy.Subscriber("/astar_arrived",Empty,astar_arriv_cb)
map_sub = rospy.Subscriber("/map",OccupancyGrid,map_cb)
rospy.Subscriber("/exit_build",Empty,exit_cb)
rospy.Subscriber("/astar_set_point", AstarPoint,astart_setPoint)
rospy.spin()
