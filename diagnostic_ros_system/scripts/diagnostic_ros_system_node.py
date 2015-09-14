#!/usr/bin/env python
import rospy
import rosnode
import os
from std_msgs.msg import String
from std_srvs.srv import *

global pub

class observed_node(object):
     global pub
     def __init__(self,name,msg_type=String,hertz=0.5):
        self.name = name
        self.hertz = hertz
        self.subscriber = None
        self.bool = 0
        self.publisher = pub
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

     def publish(self):
        self.publisher.publish(String(i.name))
            
         

class diagnostic_node(object):
    def __init__(self):
        rospy.init_node('diagnostic_node')
        rospy.on_shutdown(self.ShutdownCallback)
        self.observed_nodes = node_list()
        self.needed_nodes = []
        self.resetting = 0

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
        global start
        while not rospy.is_shutdown() :
             if not self.resetting and start:
                  self.observed_nodes.pingNodes()
             rospy.sleep(1.0/20.0)

    def set_monit_node_cb(msg):
        self.resetting = 1 
        list1 = msg.data.split('@')
        list2=[]
        for i in list1:
           list2.append(observed_nodes(i))
        self.observed_nodes = node_list(list2)


class node_list(object):
      def __init__(self,nlist=[]):
          self.nList = nlist
      
      def pingNodes(self):
          global start
          for i in self.nList:
            if not i.pingNode() and start:
                i.publish()
                break

def start_diag_cb(srv):
    global start
    start = not start
    return EmptyResponse()
    

global start
start = 0               

pub = rospy.Publisher("/failing_node",String, queue_size=2)
node = diagnostic_node()
rospy.Subscriber("set_monit_node",String,node.set_monit_node_cb)
rospy.Service('start_diag',Empty, start_diag_cb)

node.spin()

