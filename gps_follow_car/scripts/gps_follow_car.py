#!/usr/bin/env python

import rospy
import tf
import tf.transformations as trans
import sensor_msgs
from math import *
from pwm_serial_py.srv import Over_int
import LatLongUTMconversion as LLtoUTM

global cases
global trans1,rot1,speed_,kUv_,ku_,listener

speed_ =1 
kUv_ =1 
ku_ =1

def checkPosSegment(a,b):
  vOrthoAB = [-(b[1]-a[1]),(b[0]-a[1])]
  vMB = [b[0]-trans1[0],b[1]-trans1[1]];
  # we do MB^BC BC is orthogonol to AB M the position of base_link
  return vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0]<0; #true if we didn't pass through B



def calcGPS():
  global trans1,rot1,cases,speed_,kUv_,ku_,listener
  if (!receivedData)
    return;  

  size = len(cases[0]);
  i=0;
  
  cases = waitCases;


  start =
  while ( !checkPosSegment(cases[0][i],cases[0][i],cases[0][i+1],cases[0][i+1]) && i < size-1) {
    i=i+1;
  }
  vABx = cases[0][i+1]-cases[0][i];
  vABy = cases[1][i+1]-cases[1][i];
  vAMx = trans1[0]-cases[0][i];
  vAMy = trans1[1]-cases[1][i];
  (roll,pitch,theta) = trans.euler_from_quaternion(rot1)
  #follow abstract ligne
  phi = atan2(vABy,vABx);
  eL = (vABx*vAMy-vABy*vAMx)/sqrt(vABx**2+vABy**2);#det([b-a,m-a])/norm(b-a);//distance a la ligne
  thetabar = phi-atan(eL);
  e = thetabar-theta;
  #insert lidar data
  u=ku_*(2/PI)*atan(tan(e/2));#atan for modulo 2*pi*/
  v = speed_ - kUv_*abs(u) ;#TODO find the right parameters
  sendCommand(1500+v,1500+u);


def sendCommand(channelSpeed,channelYaw):
    try:
        send_pwm = rospy.ServiceProxy('/pwm_serial_send',Over_int)
        resp1 = send_pwm([channelSpeed,0,channelYaw,0,0,0,0,0])
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed : %s"%e


def spin(): 
    global trans1,rot1
    try:
      (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
      countFail = 0 
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      countFail = countFail+1
    if countFail > 10
       print "error fatal"
       #send ERROR
    calcGPS()
    rospy.sleep(1.0/20)
    
def ShutdownCallback(self):
    print 'shutdown' #send stop messages 

def casesCallBack(msg):
    print "gps point" 
    #transform to UTM

rospy.init_node('drift_detection')
rospy.on_shutdown(ShutdownCallback)
listener = tf.TransformListener()
    
    

