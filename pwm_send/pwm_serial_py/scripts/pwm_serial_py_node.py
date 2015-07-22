#!/usr/bin/env python
import rospy
import serial

from pwm_serial_py.srv import *

class pwm_serial_node(object):
    def __init__(self):
        rospy.init_node('pwm_serial')
        rospy.on_shutdown(self.ShutdownCallback)
        self.get_command = self.get_commandSSC32U#ou getcommanPolulu
        rospy.Service("pwm_serial_send",Over_int, self.pwmSendCallback)

        port = '/dev/ttyUSB0'

        try:
            self.ser = serial.Serial(port)
            self.ser.open()
            self.ser.write(chr(0xAA))
            self.ser.flush()
        except serial.serialutil.SerialException as e:
            rospy.logerr(rospy.get_name()+": Error opening or initialising port "+port)
            exit(1)

    def run(self):
        rospy.spin()

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
        rospy.loginfo("1")
        for i in range(0,8):
        	self.ser.write(self.get_command(i,data.over_msg[i]))
        rospy.loginfo("2")
        return 1


#Init and run
pwm_serial_node().run()
