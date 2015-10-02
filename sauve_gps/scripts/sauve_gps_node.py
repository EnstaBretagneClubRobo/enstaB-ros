#!/usr/bin/env python

import rospy
import LatLongUTMconversion as LLtoUTM
import kml as k
import tf
import std_srvs.srv as sS
from std_msgs.msg import *

def ShutDownCallBack():
    k.closeGPS()
    k.close()
    print "shutdown"

def decdeg2dms(dd):
   is_positive = dd >= 0
   dd = abs(dd)
   minutes,seconds = divmod(dd*3600,60)
   degrees,minutes = divmod(minutes,60)
   degrees = degrees if is_positive else -degrees
   return (degrees,minutes,seconds)

global j
j=0
def serv_cb(msg):
      try:
          global j
          (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
          (Lat,Long) = LLtoUTM.UTMtoLL(23, trans1[1], trans1[0], '32T')
          lat = decdeg2dms(Lat)
          print trans1
          long1 = decdeg2dms(Long)
          print lat
          k.addPoint(j,Lat,Long)
          j+=1
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          print "try"
      return sS.EmptyResponse()

rospy.init_node('sauve_gps')
rospy.on_shutdown(ShutDownCallBack)
listener = tf.TransformListener()
rospy.Service("/save_opi",sS.Empty,serv_cb)
k.openFicGPS()
k.openFicOPI()
i=0
while not rospy.is_shutdown():
      try:
          (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
          (Lat,Long) = LLtoUTM.UTMtoLL(23, trans1[1], trans1[0], '32T')
          lat = decdeg2dms(Lat)
          print trans1
          long1 = decdeg2dms(Long)
          print lat
          k.addGPS(i,Long,Lat)
          i+=1
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          print "try"
      rospy.sleep(45)





