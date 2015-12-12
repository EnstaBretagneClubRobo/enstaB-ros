#!/usr/bin/env python
import os, sys
import rospy as ros
from chatter_srv_msg.msg import MessageTuto 

ros_node_name = 'ros_chat'
login = 'anonymous'

def msg_callback(msg):
    if msg.login!=login:
       ros.loginfo("%s : %s"%(msg.login,msg.message))
  

if __name__ == '__main__':
   if len(sys.argv) < 2:
      print "Input Argument Error : login"
   if len(sys.argv) == 2:
      ros_node_name = '%s_node'%sys.argv[1]
   login = sys.argv[1]
   ros.init_node(ros_node_name)
   ros.Subscriber("chatter", MessageTuto, msg_callback)
   pub = ros.Publisher('chatter', MessageTuto, queue_size=10)
   
   while not ros.is_shutdown():
      mot=raw_input("%s : "%login);
      pub.publish(login,mot);


   ros.spin()
