#!/usr/bin/env python
import time

import roslib
import rospy
import message_filters
import smach
import smach_ros
import tf

from pwm_serial_py.srv import Over_int
from sensor_msgs.msg import Imu
from mavros.msg import *



def sendPWM(channelSpeed,channelYaw):
    try:
        send_pwm = rospy.ServiceProxy('/pwm_serial_send',Over_int)
        resp1 = send_pwm([channelSpeed,0,channelYaw,0,0,0,0,0])
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed : %s"%e

def syncroCB(rcout):
    global data_rec
    global imu
    global channelSpeed
    global channelYaw
    rospy.loginfo("rccb")
    if data_rec.closed:
        return
    ax = imu.linear_acceleration.x
    ay = imu.linear_acceleration.y
    az = imu.linear_acceleration.z
    quaternion = (imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w)
    theta=tf.transformations.euler_from_quaternion(quaternion)[2]
    buf = "%f,%f,%f,%f;%f,%d,%d,%d,%d\n" % (imu.header.stamp.to_sec(),ax,ay,az,theta,rcout.channels[0],rcout.channels[2],channelSpeed,channelYaw)
    data_rec.write(buf)

def imu_cb(nimu):
    global imu
    imu=nimu


class Init(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endInit'])

    def execute(self, userdata):
        global rcout_sub
        global imu_sub
        rospy.loginfo("init...")

        rospy.wait_for_service('/pwm_serial_send')
        imu_sub = rospy.Subscriber("/mavros/imu/data",Imu,imu_cb)
        rcout_sub = rospy.Subscriber("/mavros/rc/out",RCOut,syncroCB)
        return 'endInit'
 

class AngleCalib(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endAngle'])

    def execute(self, userdata):
        global data_rec
        global channelSpeed
        global channelYaw
        data_rec = open("/home/elessog/data/angle_calibration.txt","w") 
        for i in range(0,30):
            channelYaw = 1500+i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        for i in range(0,60):
            channelYaw = 1800-i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        channelYaw = 1500
        sendPWM(channelSpeed,channelYaw)
        data_rec.close()
        return 'endAngle'


class LinCalib(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endLin'])

    def execute(self, userdata):
        global data_rec
        global channelSpeed
        global channelYaw
        global rcout_sub
        global imu_sub
        data_rec = open("/home/elessog/data/lin_calibration.txt","w") 
        for i in range(0,30):
            channelSpeed = 1500+i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        for i in range(0,60):
            channelSpeed = 1800-i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        channelSpeed = 1500
        sendPWM(channelSpeed,channelYaw)
        data_rec.close()
        rcout_sub.unregister()
        imu_sub.unregister()
        return 'endLin'
 
#Init

rospy.init_node('car_calibration')
channelSpeed = 1500
channelYaw = 1500

sm_cal = smach.StateMachine(outcomes=['endCalib'])

with sm_cal:
    smach.StateMachine.add('Init',Init(),transitions={'endInit':'AngleCalib'})
    smach.StateMachine.add('AngleCalib',AngleCalib(),transitions={'endAngle':'LinCalib'})
    smach.StateMachine.add('LinCalib',LinCalib(),transitions={'endLin':'endCalib'})


sis = smach_ros.IntrospectionServer('server_name', sm_cal, '/SM_ROOT')
sis.start()
outcome = sm_cal.execute()
#rospy.spin()
sis.stop()




