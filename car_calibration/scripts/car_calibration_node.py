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
from std_msgs.msg import Empty


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
        smach.State.__init__(self, outcomes=['endAngle','preempted'])

    def execute(self, userdata):
        global data_rec
        global channelSpeed
        global channelYaw
        global rcout_sub
        global imu_sub
        data_rec = open("/home/nuc1/data/angle_calibration.txt","w") 
        for i in range(0,15):
            if self.preempt_requested():
                print "state angle is being preempted"
                self.service_preempt()
                return 'preempted'

            channelYaw = 1500+i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        for i in range(0,30):
            if self.preempt_requested():
                print "state angle is being preempted"
                self.service_preempt()
                return 'preempted'
            channelYaw = 1800-i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        channelYaw = 1500
        sendPWM(channelSpeed,channelYaw)
        data_rec.close()
        rcout_sub.unregister()
        imu_sub.unregister()
        return 'angle_succeeded'


class LinCalib(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['lin_done','preempted'])

    def execute(self, userdata):
        global data_rec
        global channelSpeed
        global channelYaw

        data_rec = open("/home/nuc1/data/lin_calibration.txt","w") 
        for i in range(0,15):
            if self.preempt_requested():
                print "state lin is being preempted"
                self.service_preempt()
                return 'preempted'
            channelSpeed = 1500+i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        for i in range(0,30):
            if self.preempt_requested():
                print "state lin is being preempted"
                self.service_preempt()
                return 'preempted'
            channelSpeed = 1800-i*10
            sendPWM(channelSpeed,channelYaw)
            time.sleep(0.5)
        channelSpeed = 1500
        sendPWM(channelSpeed,channelYaw)
        data_rec.close()
        return 'lin_succeeded'


def monitor_cb(ud, msg):
    return False

def child_term_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'


def lin_child_term_cb(outcome_map):
    if outcome_map['Lin_calc'] == 'lin_succeeded':
        return True
    elif outcome_map['Lin_STOP'] == 'invalid':
        return True
    else:
        return False


def lin_out_cb(outcome_map):
    if outcome_map['Lin_STOP'] == 'invalid':
        return 'lin_stop'
    elif outcome_map['Lin_calc'] == 'lin_succeeded':
        return 'lin_done'
    else:
        return 'lin_stop'

class stopMach(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endSTOP'])

    def execute(self, userdata):
        global channelSpeed
        global channelYaw
        channelSpeed = 1500
        channelYaw = 1500
        sendPWM(channelSpeed,channelYaw)
        time.sleep(3)
        return 'endSTOP'

#Init
def ShutdownCallback():
        print 'shutdown'
rospy.init_node('car_calibration')
channelSpeed = 1500
channelYaw = 1500
rospy.on_shutdown(ShutdownCallback)
sm_cal = smach.StateMachine(outcomes=['endCalib'])

angle_concurrence = smach.Concurrence(outcomes=['angle_done','angle_stop'],
                                      default_outcome='angle_done',
                                      child_termination_cb = child_term_cb,
                                      outcome_cb = out_cb)

lin_concurrence = smach.Concurrence(outcomes=['lin_done','lin_stop'],
                                      default_outcome='lin_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)


with angle_concurrence:
    smach.Concurrence.add('Angle_calc', AngleCalib())
    smach.Concurrence.add('Angle_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))


with lin_concurrence:
    smach.Concurrence.add('Lin_calc', LinCalib())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with sm_cal:
    smach.StateMachine.add('Init',Init(),transitions={'endInit':'LinCalib'})
    smach.StateMachine.add('AngleCalib',angle_concurrence,transitions={'angle_done':'endCalib','angle_stop':'STOP'})
    smach.StateMachine.add('LinCalib',lin_concurrence,transitions={'lin_done':'AngleCalib','lin_stop':'STOP'})
    smach.StateMachine.add('STOP',stopMach(),transitions={'endSTOP':'endCalib'})


sis = smach_ros.IntrospectionServer('server_name', sm_cal, '/SM_ROOT')
sis.start()
outcome = sm_cal.execute()
#rospy.spin()
sis.stop()




