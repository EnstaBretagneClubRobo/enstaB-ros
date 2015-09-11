#!/usr/bin/env python
import rospy
import rosnode
import os

class diagnostic_node(object):
    def __init__(self):
        rospy.init_node('pwm_serial')
        rospy.on_shutdown(self.ShutdownCallback)

    def ShutdownCallback(self):
        print 'shutdown' #send stop messages 

    def killNode(self,node_name):
        #print rosnode.kill_nodes(node_name): isn't effective
        command = "rosnode kill %s"%node_name
        p = os.popen(command,"r")
        while 1:
            line = p.readline()
            if not line: break
            print line
        

    def pingNode(self,node_name):
        res = rosnode.rosnode_ping(node_name,2,1)

    def checkVOChain(self):
        print ""

    def checkHSChain(self):
        print ""

    def checkOCVChain(self):
        print ""

    def checkControlChain(self):
        print ""

    def spin(self):
        rospy.spin()



node = diagnostic_node()
name = "car_calibration"
node.pingNode("car_calibration")
node.killNode("car_calibration")
node.pingNode("car_calibration")
#node.spin()

