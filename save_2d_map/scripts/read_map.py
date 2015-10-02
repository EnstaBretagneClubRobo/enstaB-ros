#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import cPickle as pickle

def ShutdownCallback():
    print "shutdown read_map"

    
nMap = 0
save = True
rospy.init_node('read_map')
rospy.on_shutdown(ShutdownCallback)
n = 30

r = rospy.Publisher("/map",OccupancyGrid,queue_size = 5)
while not rospy.is_shutdown():
  input = open("/home/nuc1/data/map-%d.bin"%n,"rb")
  map1 = pickle.load(input)
  map1.header.stamp = rospy.get_rostime()
  print map1.info.resolution
  print map1.info.width
 
  r.publish(map1)
