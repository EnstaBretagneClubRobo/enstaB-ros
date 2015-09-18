#!/usr/bin/env python
import rospy
import rosnode
import os
import re
from start_node.msg import StartKillMsg
from std_srvs.srv import Empty

global kinect 
kinect =["camera_rgb_frame_tf",
         "camera_rgb_optical_frame","debayer",
         "openni_driver","rgbd_image_proc","rgbd_manager"]

def killNode(node_name):
   #print rosnode.kill_nodes(node_name): isn't effective
   command = "rosnode kill /%s"%node_name
   p = os.popen(command,"r")
   while 1:
       line = p.readline()
       if not line: break
   return line == 'killed'

def startLaunch(pkg,launchfile):
   command = "roslaunch %s %s &"%(pkg,launchfile)
   os.system(command)
    
def startRun(pkg,exe,arg=""):
    command = "rosrun %s %s %s &"%(pkg,exe,arg)
    os.system(command)

def startRun1(Args):
    startRun(Args[0],Args[1],Args[2])

def startLaunch1(args):
    startLaunch(args[0],args[1])

def killLaunch(props):
    for i in props[2]:
         killNode(i)

def killRun(props):
    reg = re.compile('.*\.py')
    if reg.match(props[1]):
       killNode(props[1].split('.py')[0])
    else:
       killNode(props[1])

def dummy(msg):
    rospy.loginfo("You should not asked this service")

class starter(object):
    def __init__(self):
       global kinect
       rospy.init_node('start_node')
       rospy.on_shutdown(self.ShutdownCallback)
       rospy.Subscriber("/start_kill_node", StartKillMsg, self.SIGcallback)
       self.corressAlone = {}
       self.corressAlone[0] = ["hokuyo_node","hokuyo_node","/dev/sensors/hokuyo_hhh"]
       self.corressAlone[1] = ["gps_follow_car","gps_follow_car.py",""]
       self.corressAlone[2] = ["save_node","save_node",""]
       self.corressAlone[3] = ["pwm_serial_py","pwm_serial_py_node.py",""]
       self.corressAlone[4] = ["autonmous_move_handling","autonomous_move_handling_node.py",""] 
       self.corressAlone[5] = ["drift_detection","drift_detection_node.py",""]
       self.corressEnsemble = {}
       self.corressEnsemble[0] = ["ccny_openni","openni.launch",kinect]
       self.corressEnsemble[1] = ["ccny_rgbd","vo+mapping.launch",["visual_odometry","keyframe_mapper_node"]]
       self.corressEnsemble[2] = ["","/home/nuc1/ruby/mapping_default.launch",["hector_mapping","tt_pub"]]
       self.corressEnsemble[3] = ["mavros","apm2_radio.launch",["mavros"]]
       self.corressEnsemble[4] = ["","~/launch/support.launch",[]]
       self.corressEnsemble[5] = ["car_controller","car_controller_launch.launch",["car_controller"]]
       self.reg = re.compile('.*\.launch')
       s = rospy.Service("/star_node_srv",Empty,dummy)

    def ShutdownCallback(self):
       print 'shutdown' #send stop messages 

    def SIGcallback(self,msg):
        if msg.action == 0:
           self.kill(msg)
        elif msg.action == 1:
           self.start(msg)
        else :
           self.kill(msg)
           self.start(msg)

    def kill(self,msg):
        if msg.type==0:
           killLaunch(self.corressEnsemble[msg.nId])
        else:
           killRun(self.corressAlone[msg.nId])
 
   
    def start(self,msg):
        if msg.type == 0:
           startLaunch1(self.corressEnsemble[msg.nId])
        else:
           startRun1(self.corressAlone[msg.nId])


node = starter()
rospy.spin()
            
