#!/usr/bin/env python
import rospy
import rosnode
import os
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64

class observed_node(object):
     def __init__(self,name,msg_type,hertz=0.5):
        self.name = name
        self.hertz = hertz
        self.subscriber = None
        self.bool = 0
        self.publisher =  rospy.Publisher("publ_name", Float64, queue_size=2)
        self.msg_type = msg_type
     def checkPub(self):
        print "f"
        self.subscriber = rospy.Subscriber(self.name, self.msg_type, self.callback)
        rospy.sleep(rospy.sleep((1.0/self.hertz)*2))
        self.subscriber.unregister()
        if (self.bool<=0):
           d = Float64()
           d.data = 5
           self.publisher.publish(d)
        self.bool = 0

     def pingNode(self):
        res = rosnode.rosnode_ping(self.name,2,1)
        return res

     def callback(self,message):
        self.bool = self.bool+1       
         

class diagnostic_node(object):
    def __init__(self):
        rospy.init_node('diagnostic_node')
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
        

    def spin(self):
        n1 = observed_node("/map",OccupancyGrid,0.5)
        while not rospy.is_shutdown():
             n1.checkPub()
             n1.pingNode()



node = diagnostic_node()
node.spin()

