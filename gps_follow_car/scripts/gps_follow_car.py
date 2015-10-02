#!/usr/bin/env python

import rospy
import tf
import tf.transformations as trans
import sensor_msgs
from math import *
from pwm_serial_py.srv import Over_int
import LatLongUTMconversion as LLtoUTM
from std_msgs.msg import String,Int8,Empty
import numpy as n
from gps_handler.srv import *
from std_msgs.msg import Int8MultiArray

global cases,start
global trans1,rot1,speed_,kUv_,ku_,listener,receivedData
receivedData = False

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
  global trans1,rot1,cases,speed_,kUv_,ku_,listener,transOld,is_near_srv
  if not receivedData:
    return
  
  size = len(cases);
  i=0;
  
  cases;
  msgSRV = IsNearRequest()
  msgSRV.type1 = 1
  msgSRV.type2 = 1
  msgSRV.xLont1 = trans1[0]
  msgSRV.yLat1 = trans1[1]
  msgSRV.xLont2 = cases[size-1][0]
  msgSRV.yLat2 = cases[size-1][1]
  msgSRV.threshold = 1
  res = is_near_srv(msgSRV).response
  if res:
     sendCommand(1500,1500)
     start = False
     return
  
  while not checkPosSegment(cases[i],cases[i+1]) and i < size-2:
    i=i+1


  msgSRV.type1 = 1
  msgSRV.type2 = 1
  msgSRV.xLont1 = trans1[0]
  msgSRV.yLat1 = trans1[1]
  msgSRV.xLont2 = cases[i+1][0]
  msgSRV.yLat2 = cases[i+1][1]
  msgSRV.threshold = 1
  res = is_near_srv(msgSRV).response
  if res:
     i+=1
     return

  if i==size-2 and i is not 0:
     cases = [trans1,cases[i+1]]
     i = 0
  vABx = cases[i+1][0]-cases[i][0];
  vABy = cases[i+1][1]-cases[i][1];
  vAMx = trans1[0]-cases[i][0];
  vAMy = trans1[1]-cases[i][1];
  print vABx,vABy,vAMx,vAMy
  print cases
  (roll,pitch,theta) = trans.euler_from_quaternion(rot1)
  w = calScan(theta)#insert lidar data
  #attractLine(w,cases[i][0],vABx,vABy,trans1)
  #follow abstract ligne
  phi = atan2(vABy,vABx)
  eL = (vABx*vAMy-vABy*vAMx)/sqrt(vABx**2+vABy**2);#det([b-a,m-a])/norm(b-a);//distance a la ligne
  print phi,"angle"
  print w
  thetabar = phi-atan(eL/2);
  #e = thetabar-theta;
  w[0] += 20*cos(thetabar)#tranform direction to follow into uniforme potentiel field to integrate better with obstacle
  w[1] += 20*sin(thetabar)
  print w,thetabar,theta
  xp = 0
  if (transOld[0] == 999):
    xp = 0 
  else:
    xp = sqrt((trans1[0]-transOld[0])**2 +(trans1[0]-transOld[0])**2)
  vbar = sqrt(w[0]**2+w[1]**2)
  v = 200*(2/pi)*atan(vbar - xp);
  nThetabar = atan2(w[1],w[0])
  u = -150*(2/pi)*atan(tan((nThetabar-theta)/2));
  #u=ku_*(2/pi)*atan(tan(e/2));#atan for modulo 2*pi*/
  #v = speed_ - kUv_*abs(u) ;#TODO find the right parameters
  print vbar,nThetabar,v,u
  sendCommand(v,u)


#def attractLine(potentiel,linePoint,lineX,lineY,trans):
#    orto = [-lineY/sqrt(lineY**2+lineX**2),lineX/sqrt(lineY**2+lineX**2))
#    p = [trans[0]-linePoint[0],trans[1]-linePoint[1]]
#    pot = [-orto[0]*(p[0]*orto[0]+p[1]*orto[1]),-orto[1]*(p[0]*orto[0]+p[1]*orto[1])]
#    potentiel[0]+=pot[0]
#    potentiel[1]+=pot[1]

def calScan(theta):
    global waitScan,scan,workOnScan
    scan = waitScan
    workOnScan = True
    obst = []
    w = [0,0]
    for (r,i) in zip(scan.ranges,range(0,len(scan.ranges))):
        if not isnan(r) and n.isfinite(r) and i/3.0:
           if r < 3: #repulsif obstacle 
             w[0] += 1*cos(1*pi+(theta+scan.angle_min+i*scan.angle_increment))/r#obst.append([r,scan.angle_min+i*scan.angle_increment])
             w[1] += 1*sin(1*pi+(theta+scan.angle_min+i*scan.angle_increment))/r
    workOnScan = False
    return w
  #/champ de vecteur
  #  nq=x(1:2)-qhat;
  #  w=vhat-2*(x(1:2)-phat)+nq/(norm(nq)^2);
  #  vbar=norm(w); thetabar = atan2(w(2),w(1));
  #  u = [vbar-x(3);10*atan(tan((thetabar-x(4))/2))];*/

def sendCommand(channelSpeed,channelYaw):
    #pwmg = channelSpeed-channelYaw 
    #pwmd = channelSpeed+channelYaw
    delta  = -2*channelYaw
    mean = channelSpeed 
    #  try:
    #    
    #    #send_pwm = rospy.ServiceProxy('/pwm_serial_send',Over_int)
    pwm_pub.publish(data=[1500-pwmd,1500+mean-delta*0.35,1500-(+mean+delta*0.35),1500+pwmg,0,0,0,0])
    #    print "essay"
    #    return resp1.result
    #except rospy.ServiceException, e:
    #    print "Service call failed : %s"%e
global countFail,cases,trans1,rot1
trans1=[]
countFail = 0
cases = []
def spin(): 
    global trans1,rot1,cases,start,countFail
    try:
      (x,y) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
      countFail = 0 
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      return
      countFail = countFail+1
      rospy.loginfo("error tf")
    trans1 = x
    rot1=y
    print trans1
    if countFail > 10:
       rospy.loginfo( "error fatal TF")
       errorPub.publish(Empty())
       start = 0
    else:
       countFail = 0
    if not cases==[]:
       if len(cases) == 1:
          cases = [trans1[0],trans1[1]]+cases
       if start:
          calcGPS()
    rospy.sleep(1.0/20.0)
    
def ShutdownCallback():
    print 'shutdown' #send stop messages 

def casesCallBack(msg):
    print "gps point"
    global cases
    #transform to UTM
    t1 = msg.data.split('\n')
    cases = []
    for line in t1:
        t2 = line.split(";")
        Lat = float(t2[0])
        Long =float( t2[1])
        (e,x,y) = LLtoUTM.LLtoUTM(23,Lat,Long)
        cases.append([x,y])

def start_gps_follow_cb(msg):
    global start
    start = msg.data

def scan_cb(msg):
    global waitScan,scan,workOnScan,receivedData
    receivedData=True
    if workOnScan:
       waitScan = msg
    else:
       scan=msg
       waitScan = msg

global scan,is_near_srv,workOnScan
workOnScan = False
scan = 0
start = 0
rospy.init_node('gps_follow_car')
rospy.on_shutdown(ShutdownCallback)
listener = tf.TransformListener()
rospy.Subscriber('gps_string',String,casesCallBack)
errorPub = rospy.Publisher("/autonomous_error",Empty)
is_near_srv = rospy.ServiceProxy('IsNear',IsNear)
rospy.Subscriber('/start_gps_follow', Int8, start_gps_follow_cb)
rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, scan_cb)
pwm_pub = rospy.Publisher('pwm_serial',Int8MultiArray)

while not rospy.is_shutdown():
   if start:
      spin()
    

