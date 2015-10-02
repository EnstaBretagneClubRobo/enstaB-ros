#!/usr/bin/env python


import rospy
import LatLongUTMconversion as LLtoUTM
from gps_handler.srv import *
#wgs-84 : 23

def ShutdownCallback():
    print "shutdown"

def is_near(req):
    if req.type1:
       (e,x1,y1) = LLtoUTM.LLtoUTM(23,req.yLat1,req.xLont1)
    else:
       (x1,y1) = (req.xLont1,req.yLat1)
    if req.type2:
       (e,x2,y2) = LLtoUTM.LLtoUTM(23,req.yLat2,req.xLont2)
    else:
       (x2,y2) = (req.xLont2,req.yLat2)
    
    return IsNearResponse((x1-x2)**2+(y1-y2)**2<=req.threshold**2)


rospy.init_node('gps_handler')
rospy.on_shutdown(ShutdownCallback)
serviceNear = rospy.Service('IsNear', IsNear, is_near)
rospy.spin()
