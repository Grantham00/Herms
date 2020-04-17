#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *
import time
import numpy as np

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees

start = [0,0]
y_vel = 0
z_vel = 0
destination = [0,3]

regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
   global position_, previous_time, previous_enc, current_distance, time_stopped, yaw_
    
   y_vel = msg.twist.twist.linear.y
   z_vel = msg.twist.twist.angular.z

   enc = msg.pose.pose.position.y

   current_time = rospy.Time.now().to_sec()
   dt = current_time - previous_time
   delta_enc = abs(enc - previous_enc)
   if delta_enc < 0.01:
       time_stopped = time_stopped + dt
   else:
       time_stopped = 0
   if state == 0 or state==1 or state==6:
	position_[0] = position_[0] + delta_enc*math.cos(math.radians(yaw_+90))
	position_[1] = position_[1] + delta_enc*math.sin(math.radians(yaw_+90))
    # position
   #if state == 0:
    #   position_[1] = position_[1] + delta_enc
   #if state == 1 or state== 6:
    #   position_[0] = position_[0] - delta_enc
   previous_time = current_time
   previous_enc = enc   
   previous_position = position_

def clbk_imu(msg):
    global yaw_
    
    yaw_ = msg.orientation.z

def clbk_laser(msg):
    global regions_
    scans = msg.ranges
    #scans = np.delete(scans, np.where(scans == float('+inf')))
    section = []
    for i in range(8):
        section.append(int(round((len(scans)/8)*i)-(len(scans)/8)/2))

    regions_ = {
        #'back':   min(min(scans[843:57]), 10),
        'bright':   min(min(scans[58:170]), 10),
        'right':  min(min(scans[171:282]), 10),
        'fright': min(min(scans[283:394]), 10),
        'front':  min(min(scans[395:506]), 10),
        'fleft':  min(min(scans[507:618]), 10),
        'left':   min(min(scans[619:730]), 10),
        'bleft':   min(min(scans[731:842]), 10),
    }


def distance_to_desired():
	global current_distance 
	current_distance = math.sqrt((desired_position[0] - position_[0])**2 + (desired_position[1] - position_[1])**2)

def set_heading(deg):
	vel_msg.linear.y = 999
	vel_msg.angular.z = deg
	velocity_publisher.publish(vel_msg)
	time.sleep(5)
def hard_stop():
	vel_msg.linear.y = 555
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)
	time.sleep(1)

def Stop():
    global state, temp_position
    #prev_state = state
    #state = 6
    print('stop')
    while (y_vel >= 0.1 or z_vel >= 0.1):
	vel_msg.linear.y = 0
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)
	rate.sleep()
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    time.sleep(3)
    #state = prev_state


def turn_left_90():
    global state
    print('turn_left_90')
    initial_yaw = yaw_
    prev_state = state
    state = 4
    const = 1
    #vel_msg.linear.y = 0
    #vel_msg.angular.z = -0.5
    #velocity_publisher.publish(vel_msg)
    #rate.sleep()
    error = initial_yaw + 90
    while yaw_ < (initial_yaw + 90-1):
	error = (initial_yaw+90) - yaw_
        if(error/360 > 0.1):
		#print('while {} < {}'.format(yaw_, initial_yaw+90))
		print("time_stopped: {}".format(time_stopped))
        	vel_msg.linear.y = 0
        	vel_msg.angular.z = -(error/360)*const
        	velocity_publisher.publish(vel_msg)
        	rate.sleep()
        elif(error/360 > 0.1 and error/360 > 0):
		#print('while {} < {}'.format(yaw_, initial_yaw+90))
		print("time_stopped: {}".format(time_stopped))
        	vel_msg.linear.y = 0
        	vel_msg.angular.z = -0.1 * const
        	velocity_publisher.publish(vel_msg)
        	rate.sleep()
	if (time_stopped > 2 and (-(error/360)*const > -.4) and (-0.1*const > -.4)):
		const = const*1.025
	else:
		const = 1
    Stop()
    print('yaw: {}'.format(yaw_))
    state = prev_state


def forward():
    global state, const
    #prev_state = state
    #state = 5
    print('forward')
    print('distance to desired: {}'.format(current_distance))
    vel_msg.linear.y = -1*const
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    if (time_stopped > 2 and (-1*const > -3)):
	const = const*1.05
    else:
	const = 1
    #state = prev_state

def forward_a_little():
    global state
    const = 1
    print('forward_a_little')
    curr_pos = position_[0]
    small_distance = curr_pos - 1
    while position_[0] > small_distance:
	if regions_['front'] < 1:
	    state = 2
	    break
        vel_msg.linear.y = -1*const
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rate.sleep()
	print('position : {},{}'.format(position_[0],position_[1]))
	if (time_stopped > 2 and (-1*const > -3)):
	    const = const*1.05
	else:
	    const = 1
    Stop()

    
def turn_to_dest():
    global state
    print('turn_to_dest')
    #state = 4
    delta_y = position_[1]
    delta_x = position_[0]
    theta = math.atan2((desired_position[1] - delta_y),(desired_position[0] - delta_x))
    dest_theta = math.degrees(theta)-90
    dest_theta = dest_theta *-1
    print('THETA: {}'.format(dest_theta))
    set_heading(-dest_theta)
    Stop()


def main():
    global regions_, position_, previous_position, desired_position_, state, yaw_, yaw_error_allowed_, rate
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_
    global vel_msg, velocity_publisher
    global previous_enc, current_position, previous_distance, current_distance
    global previous_time, current_time, time_stopped
    global initial_position, desired_position, const
    initial_position = start
    position_ = start
    desired_position = destination
    const = 1
    current_position = [0,0]
    previous_enc = 0
    time_stopped = 0
    rospy.init_node('GO_HERMAN')
    previous_time = rospy.Time.now().to_sec()
    distance_to_desired()
    previous_distance = current_distance
    
    position_ = start
    previous_position = position_
    
    first_run = True
    state=0
    print("subscribing")
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_imu = rospy.Subscriber('/Imu', Imu, clbk_imu)
    sub_odom = rospy.Subscriber('/Odom', Odometry, clbk_odom)

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel_msg = Twist()

    # initialize going to the point
    #dist2line = distance_to_line()
    Stop()
    rate = rospy.Rate(20)
    turn_to_dest()
    print("starting loop")
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        distance_to_desired()
        if first_run:
	    print('first run')
            initial_yaw = yaw_
            state = 0
            first_run = False
	    print('state = {}'.format(state))
	    #print('position : {},{}'.format(position_[0],position_[1]))
	    #print('regions_front: {}'.format(regions_['front']))	

        if regions_ == None:
	    print('regions_ == None')
            continue
        #or regions_['fright'] < 1 or regions_['fleft'] < 1)
        if (state == 0 and ((regions_['front'] < 1) or (regions_['fleft'] < 1) or (regions_['fright'] < 1))): 

	    print('state=1')
	    print('regions_fright: {}'.format(regions_['fright']))		
	    print('regions_frontt: {}'.format(regions_['front']))	
	    print('regions_fleft: {}'.format(regions_['fleft']))
            previous_position = position_
            Stop()
	    state = 20
            set_heading(90)
	    Stop()
	    state=1

	    print('position : {},{}'.format(position_[0],position_[1]))
	    print('regions_front: {}'.format(regions_['front']))	
                    
        if state == 1 and regions_['front'] > 1:
            if ((regions_['fright'] < 1 or regions_['right'] < 1)):
                   forward()
                   
            if (state == 1 and ((regions_['front'] > 1) and (regions_['fright'] > 1 and regions_['right'] > 1))):
	        Stop()
	        forward_a_little()
		state = 20
	        turn_to_dest()
	        state = 0

        if state == 1 and regions_['front'] < 1 :
	    print('Cant move lmao')
	    state = 2
	    Stop()

        if (current_distance > 0.1) and state==0:
	    print('nothing happened...FORWARD!')
	    print('current_distance: {}'.format(current_distance))
            forward()
	if (abs(position_[1]- destination[1]) < 0.5):
	    Stop()
	    state = 2

	if state == 2:
	    print('Its over')
	    Stop()
	if state == 1000:
	    set_heading(-90)
	    state = 2

        #else:
	    #print('SUPER nothing')
            #Stop()
        
        rate.sleep()
	print("RAN! CURRENT STATE: {}".format(state))

	print('current distance: {}'.format(current_distance))
	print('regions_bright: {}'.format(regions_['bright']))	
	print('regions_right: {}'.format(regions_['right']))
	print('regions_fright: {}'.format(regions_['fright']))		
	print('regions_frontt: {}'.format(regions_['front']))	
	print('regions_fleft: {}'.format(regions_['fleft']))
	print('regions_left: {}'.format(regions_['left']))	
	print('regions_bleft: {}'.format(regions_['bleft']))
	print('TIME STOPPED: {}'.format(time_stopped))
	print('position : {},{}'.format(position_[0],position_[1]))
if __name__ == "__main__":
    main()
