#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 09:57:59 2020

@author: ant
"""
import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial

def callback(data):
    print('callback')
    rospy.loginfo('poop')
    print(data.linear.x)
    print(data.angular.z)
#    send(data)
#    x = data
    
def shutdown():
    if 'ser' in globals():
        ser.close()
    
def listener():
    rospy.init_node('CMD_TO_HERO')
    print('init')
    rospy.Subscriber("cmd_vel", Twist, callback)
    print('sub')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def send(data):
    if (('ser' not in locals()) or ('ser' not in globals())):
        global ser
        ser = serial.Serial('/dev/ttyUSB0', 115200)
    message = '[{},{}]'.format(data.linear.x, data.angular.z)
    print(message)
    ser.write(b'{}'.format(message))

listener()

rospy.on_shutdown(shutdown)
