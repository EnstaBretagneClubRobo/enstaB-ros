#!/usr/bin/env python
import os, sys
import rospy as ros
from chatter_srv_msg.msg import MessageTuto
from chatter_srv_msg.srv import *
ros_node_name = 'ros_chat'
login = 'anonymous'

def msg_callback(msg):
    if msg.login!=login:
       print "\n%s : %s"%(msg.login,msg.message)
  

if __name__ == '__main__':
   if len(sys.argv) < 2:
      print "Input Argument Error : login"
      exit(-1)
   if len(sys.argv) == 3:
      ros_node_name = '%s_node'%sys.argv[1]
   login = sys.argv[1]
   ros.init_node(ros_node_name,anonymous=True)
   print "waiting for service ..."
   ros.wait_for_service('login_check')
   print "Service Online Connexion Pending ..\n"
   connect_srv = ros.ServiceProxy('login_check',ServiceTuto)
   answer = connect_srv(login)
   print "connected!\n"
   if not answer.success:
       print "Name already taken"
       exit(0)
   ros.Subscriber("chatter", MessageTuto, msg_callback)
   pub = ros.Publisher('chatter', MessageTuto, queue_size=10)
   
   while not ros.is_shutdown():
      mot=raw_input("%s : "%login);
      if mot=='exit':
         print "deconnexion ..."
         ros.wait_for_service('login_check')
         deconnect_srv = ros.ServiceProxy('deconnect',ServiceTuto)
         answer = deconnect_srv(login)
         if answer.success:
           print "deconnected!"
           exit(0)
      pub.publish(login,mot);
      

   ros.spin()
