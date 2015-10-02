#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import cPickle as pickle

def ShutdownCallback():
    print "shutdown sauve_map"

def map_cb(map1):
    global nMap,save
    if not save:
       return
    save=False
    with open("/home/nuc1/data/map-%d.bin"%nMap,"wb") as output:
         pickle.dump(map1,output,pickle.HIGHEST_PROTOCOL)
  
    rospy.sleep(40)
    save=True
    nMap+=1
   

global nMap,save
nMap = 0
save = True
rospy.init_node('save_map')
rospy.on_shutdown(ShutdownCallback)

rospy.Subscriber("/map",OccupancyGrid,map_cb,queue_size = None)
rospy.spin()
