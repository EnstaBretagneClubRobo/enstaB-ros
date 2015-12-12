#!/usr/bin/env python
import rospy
from chatter_srv_msg.srv import *

global name_list
name_list = []

def srv_callback(req):
    global name_list
    if req.login not in name_list:
        print "connexion %s\n"%req.login
        name_list.append(req.login)
        return ServiceTutoResponse(True)
    else:
        return ServiceTutoResponse(False)

def deconnexion(req):
    global name_list
    print "deconnexion %s\n"%req.login
    name_list.remove(req.login)
    return ServiceTutoResponse(True)

if __name__ == '__main__':
   rospy.init_node('logger')
   s = rospy.Service('login_check', ServiceTuto, srv_callback)
   s = rospy.Service('deconnect', ServiceTuto, deconnexion)

   rospy.spin()
