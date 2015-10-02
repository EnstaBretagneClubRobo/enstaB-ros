#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Int8,Empty


def ShutdownCallback():
    print 'shutdown' #send stop messages 


rospy.init_node('testGPS')
rospy.on_shutdown(ShutdownCallback)
tPub = rospy.Publisher('gps_string',String,latch=True)

sPub = rospy.Publisher('/start_gps_follow', Int8,latch = True,queue_size = None)

coordAx =[42,57,15.6]
coordAy =[10,36,9.0]
coordBx =[42,57,16]
coordBy =[10,36,9.4]
coordCx =[42,57,15.7]
coordCy =[10,36,9.4]
coordDx =[42,57,15.8] 
coordDy =[10,36,8.8]

def retText(c1,c2):
    lat = c1[0]+(1/60.0)*c1[1]+(1/3600.0)*c1[2]
    longZ = c2[0]+(1/60.0)*c2[1]+(1/3600.0)*c2[2]
    return str(lat)+";"+str(longZ)


A = retText(coordAx,coordAy)
B = retText(coordBx,coordBy)
C = retText(coordCx,coordCy)
D = retText(coordDx,coordDy)

text = A+"\n"+B
print "sending"
tPub.publish(String(text))
m = Int8()
m.data = 1
sPub.publish(1)
print "end sending"
rospy.spin()


