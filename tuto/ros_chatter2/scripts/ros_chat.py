#!/usr/bin/env python
import os, sys
import rospy as ros
from ros_chatter2.msg import chat_msg2 
from chatter_login.srv import *
ros_node_name = 'ros_chat'
login = 'anonymous'

def msg_callback(msg):
    if msg.username!=login:
       ros.loginfo("%s : %s"%(msg.username,msg.data))
  

if __name__ == '__main__':
   if len(sys.argv) < 2:
      print "Input Argument Error : login"
      exit(-1)
   if len(sys.argv) == 2:
      ros_node_name = '%s_node'%sys.argv[1]
   login = sys.argv[1]
   ros.init_node(ros_node_name)
   print "waiting for service ..."
   ros.wait_for_service('login_check')
   print "Service Online Connexion Pending ..\n"
   connect_srv = ros.ServiceProxy('login_check',Check_login)
   answer = connect_srv(login)
   print "connected!\n"
   if not answer.success:
       print "Name already taken"
       exit(0)
   ros.Subscriber("chatter", chat_msg2, msg_callback)
   pub = ros.Publisher('chatter', chat_msg2, queue_size=10)
   
   while not ros.is_shutdown():
      mot=raw_input("%s : "%login);
      if mot=='exit':
         print "deconnexion ..."
         ros.wait_for_service('login_check')
         deconnect_srv = ros.ServiceProxy('deconnect',Check_login)
         answer = deconnect_srv(login)
         if answer.success:
           print "deconnected!"
           exit(0)
      pub.publish(login,mot);
      

   ros.spin()
