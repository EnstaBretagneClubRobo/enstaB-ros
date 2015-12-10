#!/usr/bin/env python
import rospy
from chatter_login.srv import *

global name_list
name_list = []

def srv_callback(req):
    global name_list
    if req.username not in name_list:
        print "connexion %s\n"%req.username
        name_list.append(req.username)
        return Check_loginResponse(True)
    else:
        return Check_loginResponse(False)

def deconnexion(req):
    global name_list
    print "deconnexion %s\n"%req.username
    name_list.remove(req.username)
    return Check_loginResponse(True)

if __name__ == '__main__':
   rospy.init_node('logger')
   s = rospy.Service('login_check', Check_login, srv_callback)
   s = rospy.Service('deconnect', Check_login, deconnexion)

   rospy.spin()
