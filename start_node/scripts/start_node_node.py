#!/usr/bin/env python
import rospy
import rosnode
import os
import re


def killNode(node_name):
   #print rosnode.kill_nodes(node_name): isn't effective
   command = "rosnode kill %s"%node_name
   p = os.popen(command,"r")
   while 1:
       line = p.readline()
       if not line: break
   return line == 'killed'

def startLaunch(pkg,launchfile):
   command = "roslaunch %s %s &"%pkg%launchfile
   os.system(command)
    
def startRun(pkg,exe,arg=""):
    command = "roslaunch %s %s %s &"%pkg%exe%arg
    os.system(command)

def startRun(Args)
    startRun(Args[0],Args[1],Args[2])

def function():
    print "t"

class starter(object):
    def __init__(self):
       rospy.init_node('start_node')
       rospy.on_shutdown(self.ShutdownCallback)
       rospy.Subscriber(####, ####, self.SIGcallback)
       self.corressAlone = {}
       self.corressAlone[0] = ["hokuyo_node","hokuyo_node",""]
       self.corressEmsemble = {}
       self.corressEmsemble[0] = function
       self.reg = re.compile('.*\.launch')

    def ShutdownCallback(self):
       print 'shutdown' #send stop messages 


   
    def SIGcallback(msg)
       if msg.type == 1:
         if self.reg.match(self.corress[]):
            startLaunch(self.corress[])
         else:
            startRun(self.corress[])
       else:
          corressEmsemble[0]()

         
