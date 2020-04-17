#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 20 00:51:37 2020

@author: ant
"""

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
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point

import serial

global ser
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
def serial_listen():
	print("doing serial listen")
	empty = True
	while empty:
		try:
			imu_in = ser.readline().decode('ASCII')
			#imu_in = '[0.433,0.082,-0.271,0.856] [-0.061,-0.061,-0.061] [5591,-5879,12525] 0.000 0.000 \n'

			imu_in = imu_in.split(' ')
			quaternion = imu_in[0][1:-1].split(',')
			quaternion = [float(i) for i in quaternion]
			#quaternion.append(quaternion.pop(0))
			gyro = imu_in[1][1:-1].split(',')
			gyro = [float(i) for i in gyro]
			accel = imu_in[2][1:-1].split(',')
			accel = [float(i) for i in accel]

			#encoder_vel_left = (float(imu_in[3])*0.0001850586) #convert encoder clicks to m
			#encoder_vel_right = (float(imu_in[4])*0.0001850586) #convert encoder clicks to m
			encoder_pos = imu_in[3][1:-1].split(',')
			encoder_pos = [((float(i)/8192)*9.5*3.141)/39.37 for i in encoder_pos]

			encoder_vel = imu_in[4][1:-1].split(',')
			encoder_vel = [(float(i)*0.111) for i in encoder_vel]

			q = Quaternion()
			q.x = quaternion[1]
			q.y = quaternion[2]
			q.z = quaternion[0]
			#q.w = quaternion[3]
				

			g = Vector3()
			g.x = gyro[0]/57.2958
			g.y = gyro[1]/57.2958
			g.z = gyro[2]/57.2958
				

			a = Vector3()
			a.x = (accel[0]/16384)*9.80665
			a.y = (accel[1]/16384)*9.80665
			a.z = (accel[2]/16384)*9.80665


			twist_linear = Vector3()
	    		linear_velocity = (encoder_vel[0] + encoder_vel[1])/2
			twist_linear.y = linear_velocity

			twist_angular = Vector3()
	    		angular_velocity = (encoder_vel[1] - encoder_vel[0])/0.4191 #separation of wheels in m
			twist_angular.z = angular_velocity

			pose_pt = Point()
			pose_pt.y = (abs(encoder_pos[0]) + abs(encoder_pos[1]))/2

			empty = False
		except:
			print("failed to get message")
	ser.reset_input_buffer()
	return q, g ,a, twist_linear, twist_angular, pose_pt

def talker():
    imu_data = Imu()
    odom = Odometry()
    imu_pub = rospy.Publisher('Imu', Imu, queue_size=1)
    odom_pub = rospy.Publisher('Odom', Odometry, queue_size=1)
    rospy.init_node('LISTEN_ODOM_IMU')
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'IMU (BASE)'
        imu_data.header = h
        
        q, g, a, twist_linear, twist_angular, pose_pt = serial_listen()
        imu_data.orientation = q
        imu_data.angular_velocity = g
        imu_data.linear_acceleration = a
        
        covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imu_data.orientation_covariance = covariance
        imu_data.angular_velocity_covariance = covariance
        imu_data.linear_acceleration_covariance = covariance

        odom.twist.twist.linear = twist_linear
        odom.twist.twist.angular = twist_angular
        
	odom.pose.pose.position = pose_pt

        imu_pub.publish(imu_data)
	odom_pub.publish(odom)
        rospy.loginfo('PUBLISHED...')
        rate.sleep()


talker()

