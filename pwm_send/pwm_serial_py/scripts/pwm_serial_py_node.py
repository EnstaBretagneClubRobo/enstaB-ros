#!/usr/bin/env python
import rospy
import serial

from std_msgs.msg import Int16MultiArray
from pwm_serial_py.srv import *

class pwm_serial_node(object):
    def __init__(self):
        global last_msg
        rospy.init_node('pwm_serial')
        rospy.on_shutdown(self.ShutdownCallback)
        self.get_command = self.get_commandPolulu#ou getcommanPolulu
        last_msg = rospy.get_time()
        rospy.Service("pwm_serial_send",Over_int, self.pwmSendCallback)
        rospy.Subscriber("/pwm_serial",Int16MultiArray,self.pwm_msg_cb)

        port = '/dev/ttyACM2'

        try:
            self.ser = serial.Serial(port)
            self.ser.open()
            self.ser.write(chr(0xAA))
            self.ser.flush()
        except serial.serialutil.SerialException as e:
            rospy.logerr(rospy.get_name()+": Error opening or initialising port "+port+str(e))
            exit(1)

    def run(self):
        while not rospy.is_shutdown():
           global last_msg
           if rospy.get_time()-last_msg >7:
              self.pwm_sec_cb([1500]*8)
           rospy.sleep(1.0/20.0)

    def get_commandPolulu(self, channel, target):
        target = target * 4
        serialBytes = chr(0x84)+chr(channel)+chr(target & 0x7F)+chr((target >> 7) & 0x7F)
        return serialBytes

    def get_commandSSC32U(self,channel,target):
        serialBytes = "#%dP%d\r"% (channel, target)
 
        return serialBytes

    def ShutdownCallback(self):
        rospy.loginfo("Shutting down")
        if hasattr(self, 'ser'):
            for i in range(0,8):
                self.ser.write(self.get_command(i,0))
            self.ser.write(chr(0xA2))
            self.ser.close()

    def pwmSendCallback(self,data):
        global last_msg
        last_msg = rospy.get_time()
        print 1
        for i in range(0,8):
           self.ser.write(self.get_command(i,data.over_msg[i]))
        return 1

    def pwm_msg_cb(self,msg):
        global last_msg
        last_msg = rospy.get_time()
        print "start sending"
        for i in range(0,8):
           self.ser.write(self.get_command(i,msg.data[i]))
           rospy.sleep(0.00)
        print msg.data

    def pwm_sec_cb(self,data):
        for i in range(0,8):
           self.ser.write(self.get_command(i,data[i]))


#Init and run
pwm_serial_node().run()
