#!/usr/bin/env python

import rospy
import tf
import tf.transformations as trans
import sensor_msgs
from math import *
from pwm_serial_py.srv import Over_int
import LatLongUTMconversion as LLtoUTM
from std_msgs.msg import String
from gps_follow
import numpy as n

global cases,start
global trans1,rot1,speed_,kUv_,ku_,listener

speed_ =1 
kUv_ =1 
ku_ =1

def checkPosSegment(a,b):
  vOrthoAB = [-(b[1]-a[1]),(b[0]-a[1])]
  vMB = [b[0]-trans1[0],b[1]-trans1[1]];
  # we do MB^BC BC is orthogonol to AB M the position of base_link
  return vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0]<0; #true if we didn't pass through B

global transOld
transOld =[999,999,0] #[630493.091685,4756982.12801,0]
def calcGPS():
  global trans1,rot1,cases,speed_,kUv_,ku_,listener,transOld
  if (!receivedData)
    return;  

  size = len(cases);
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
  w = calScan(theta)#insert lidar data
  attractLine(w,cases[0][i],vABx,vABy,trans1)
  #follow abstract ligne
  phi = atan2(vABy,vABx);
  eL = (vABx*vAMy-vABy*vAMx)/sqrt(vABx**2+vABy**2);#det([b-a,m-a])/norm(b-a);//distance a la ligne
  thetabar = phi-atan(eL);
  #e = thetabar-theta;
  w[0] += 200*cos(thetabar)#tranform direction to follow into uniforme potentiel field to integrate better with obstacle
  w[1] += 200*sin(thetabar)
  xp = 0
  if (transOld[0] == 999):
    xp = 0 
  else:
    xp = sqrt((trans1[0]-transOld[0])**2 +(trans1[0]-transOld[0])**2)
  vbar = sqrt(w[0]**2+w[1]**2);
  v = vbar - xp;
  nThetabar = atan2(w[1],w[0])
  u = 10*atan(tan((thetabar-theta)/2));
  #u=ku_*(2/pi)*atan(tan(e/2));#atan for modulo 2*pi*/
  #v = speed_ - kUv_*abs(u) ;#TODO find the right parameters
  sendCommand(1500+v,1500+u);


def attractLine(potentiel,linePoint,lineX,lineY,trans):
    orto = [-lineY/sqrt(lineY**2+lineX**2),lineX/sqrt(lineY**2+lineX**2))
    p = [trans[0]-linePoint[0],trans[1]-linePoint[1]]
    pot = [-orto[0]*(p[0]*orto[0]+p[1]*orto[1]),-orto[1]*(p[0]*orto[0]+p[1]*orto[1])]
    potentiel[0]+=pot[0]
    potentiel[1]+=pot[1]

def calScan(theta):
    global waitScan,scan,workOnScan
    scan = waitScan
    workOnScan = True
    obst = []
    w = [0,0]
    for (r,i) in zip(scan.ranges,range(0,len(san.ranges)):
        if not isnan(r) and n.isfinite(r):
           if r < 2:#repulsif obstacle 
             w[0] += 10*cos(theta+i*scan.angle_increment)/r#obst.append([r,scan.angle_min+i*scan.angle_increment])
             w[1] += 10*sin(theta+i*scan.angle_increment)/r
    workOnScan = False
    return w
  #/champ de vecteur
  #  nq=x(1:2)-qhat;
  #  w=vhat-2*(x(1:2)-phat)+nq/(norm(nq)^2);
  #  vbar=norm(w); thetabar = atan2(w(2),w(1));
  #  u = [vbar-x(3);10*atan(tan((thetabar-x(4))/2))];*/

def sendCommand(channelSpeed,channelYaw):
    try:
        send_pwm = rospy.ServiceProxy('/pwm_serial_send',Over_int)
        resp1 = send_pwm([channelSpeed,0,channelYaw,0,0,0,0,0])
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed : %s"%e


def spin(): 
    global trans1,rot1,cases,start
    try:
      (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
      countFail = 0 
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      countFail = countFail+1
    if countFail > 10
       rospy.loginfo( "error fatal TF")
       errorPub.publish(Empty())
       start = 0
    if not cases==[]:
       if len(cases) == 1:
          cases = [trans1[0],trans1[1]]+cases
       if start:
          calcGPS()
    rospy.sleep(1.0/20)
    
def ShutdownCallback():
    print 'shutdown' #send stop messages 

def casesCallBack(msg):
    print "gps point" 
    #transform to UTM
    t1 = msg.data.split('\n')
    cases = []
    for line in t1:
        t2 = line.split(";")
        Lat = t2[0]
        Long = t2[1]
        (e,x,y) = LLtoUTM.LLtoUTM(Lat,Long)
        cases.append([x,y])

def start_gps_follow_cb(msg):
    global start
    start = msg.data

def scan_cb(msg)
    global waitScan,scan,workOnScan
    if workOnScan:
       scan = msg
       waitScan = msg
    else:
       waitScan = msg

global scan
scan = 0
start = 0
rospy.init_node('gps_follow_car')
rospy.on_shutdown(ShutdownCallback)
listener = tf.TransformListener()
rospy.Subscriber('gps_string',String,casesCallback)
errorPub = rospy.Publisher("/autonomous_error",Empty)

rospy.Subscriber('/start_gps_follow', Int8, start_gps_follow_cb)
rospy.Subscriber('/scan', sensor_msgs.LaserScan, scan_cb)

while not rospy.is_shutdown():
   if start:
      spin()
    

