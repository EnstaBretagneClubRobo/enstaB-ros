#!/usr/bin/env python

import rospy
import tf
import math as m

def ShutdownCallback():
    print 'shutdown' #send stop messages 

def spin():
    
    try:
      global listener,br
      (x,ya) = listener.lookupTransform("local_origin", "false_fcu", rospy.Time(0))
      tmp =  (x[1],x[0],x[2])
      (r,p,y) = tf.transformations.euler_from_quaternion(ya)
      tmpRot = (r,p,m.pi+y)
      nY = tf.transformations.quaternion_from_euler(tmpRot[0],tmpRot[1],tmpRot[2])
      br.sendTransform(tmp,nY,rospy.Time(0),"fcu","local_origin")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print "error"
    
global listener,br
rospy.init_node('trans_prox')
rospy.on_shutdown(ShutdownCallback)
listener = tf.TransformListener()
br = tf.TransformBroadcaster()


while not rospy.is_shutdown():
      rospy.sleep(1.0/20.0)
      spin()
