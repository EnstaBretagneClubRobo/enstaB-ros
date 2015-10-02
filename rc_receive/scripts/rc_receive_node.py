#!/usr/bin/env python
import rospy
import mavros.msg as mav
from std_msgs.msg import Empty

states = [0,0,0,0,0,0,0,0]
channels = [0,0,0,0,0,0,0,0]
init  = 0


def ShutdownCallback():
    print 'shutdown' #send stop messages 

def sendInfo():
    global states,publishers
    for i in range(0,8):
        if states[i]:
           publishers[i].publish(Empty())
    

def check(channels,channel):
    global states
    for (x,y,i) in zip(channels,channel,range(0,len(channel))):
        if abs(x-y)>300:
           states[i] = 1;
    sendInfo()
    states = [0,0,0,0,0,0,0,0]

def look(channel):
    global init,channels
    if not init:
        channels = channel
        init = 1
    check(channels,channel)
    channels = channel
    	    

def callback(rcinmsg):
    look(rcinmsg.channels)
    
    

rospy.init_node('rc_receive')
rospy.on_shutdown(ShutdownCallback)

publishers =[]
for i in range(0,8):
  publ_name = "sig_rc_%d"%i 
  publishers.append(rospy.Publisher(publ_name, Empty, queue_size=2))


subscriber = rospy.Subscriber("mavros/rc/in", mav.RCIn, callback)

rospy.spin()
