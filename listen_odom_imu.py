#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 18 03:12:50 2020

@author: ant
"""

# import sys
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

import serial


def serial_listen():
    empty = True
    while empty:
        try:
            imu_in = ser.readline().decode('ASCII')
            imu_in = imu_in.split('-')
            quaternion = imu_in[0][2:-1].split(',')
            quaternion = [float(i) for i in quaternion]
            quaternion.append(quaternion.pop(0))
            gyro = imu_in[1][1:-1].split(',')
            gyro = [float(i) for i in gyro]
            accel = imu_in[2][1:-1].split(',')
            accel = [float(i) for i in accel]
            
            encoder_vel_left = float(imu_in[3])
            encoder_vel_right =float(imu_in[4][0:-2])
            empty = False
        except:
            pass
    
    return quaternion, gyro, accel, encoder_vel_left, encoder_vel_right

def talker():
    pub = rospy.Publisher('Imu', Imu, queue_size=1)
    rospy.init_node('LISTEN_ODOM_IMU')
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        quaternion, gyro, accel, encoder_vel_left, encoder_vel_right = serial_listen()
        Imu.orientation = quaternion
        Imu.angular_velocity = gyro
        Imu.linear_acceleration = accel
    
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
