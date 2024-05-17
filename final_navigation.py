#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
import time
from random import randrange
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import smbus
import RPi.GPIO as GPIO 
from time import sleep 

LINEAR_VEL = 0.2
ANGULAR_VEL = 2.5
REL_LIN_VEL = LINEAR_VEL/100
REL_ANG_VEL = ANGULAR_VEL/100
STOP_DISTANCE = 0.3
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)                       
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

# Set pins for RBG and LEDs
GPIO.setwarnings(False) # Ignore warning
GPIO.setmode(GPIO.BCM) # Use physical pin numbering
GPIO.setup(24, GPIO.OUT, initial=GPIO.LOW) # Blink blue for victim 
GPIO.setup(25, GPIO.OUT, initial=GPIO.LOW) # Light green for on 


class Obstacle():
	def __init__(self):
		self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.obstacle()

	# stop with q  key
	def is_key_available(self):
		return select.select([sys.stdin],[],[],0) == ([sys.stdin],[],[])
	def get_key(self):
		return sys.stdin.read(1)
        
	def get_scan(self):
		scan = rospy.wait_for_message('scan', LaserScan)
		scan_filter = []
       
		samples = len(scan.ranges)                                            
		samples_view = 360				# Read 360 degrees            
        
		if samples_view > samples:
			samples_view = samples
		if samples_view is 1:
			scan_filter.append(scan.ranges[0])
		else:
			left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
			right_lidar_samples_ranges = samples_view//2
            
			left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
			right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
			scan_filter.extend(left_lidar_samples + right_lidar_samples)

		for i in range(samples_view):
			if scan_filter[i] == float('Inf'):
				scan_filter[i] = 3.5
			elif scan_filter[i] < 0.001:
				scan_filter[i] = 1
			elif math.isnan(scan_filter[i]):
				scan_filter[i] = 0
		return scan_filter

	# Method for RBG readings 
    def getAndUpdateColour(self):  
  		block = bus.read_i2c_block_data(0x44, 0x09, 6)
    	green = int(block[1])*256 + int(block[0])
     	red = int(block[3])*256 + int(block[2])
    	blue = int(block[5])*256 + int(block[4])
		blue = blue*2

		if blue < red and green < red:
			GPIO.output(24, GPIO.HIGH) # Turn on
			print("Victim found")
			
		else:
			GPIO.output(24, GPIO.LOW) # Turn off
			
	def obstacle(self):
		twist = Twist()

		Runtime = time.time() + 120			# Set runtime to 120 s
		Avg_Lin_Speed = 0
		Speed_Updates = 0
		Speed_Accumulation = 0
		Collision_Counter = 0
		Collision_Delay = time.time()
  		victim_Counter = 0
		victim_Delay = time.time()

	# Run loop
		while not rospy.is_shutdown() and time.time() < Runtime:
			self.getAndUpdateColour()			# Call RBG method
   			GPIO.output(25, GPIO.HIGH)			# Turn on green light
		
  			# Stop with q key
			if self.is_key_available():
				char = self.get_key()
				if char.lower() == 'q':
					twist.linear.x = 0.0
					twist.angular.z = 0.0
					self._cmd_pub.publish(twist)
					return

			lidar_distances = self.get_scan()
			
		# Navigational logic
			if min(lidar_distances[160:200]) < SAFE_STOP_DISTANCE / 1.7:		# 0,21 m, way too close, U turn 
				rospy.loginfo("Safe dist reached - U turn left")
				twist.linear.x = 0
				twist.angular.z = ANGULAR_VEL

			elif min(lidar_distances[160:200]) > SAFE_STOP_DISTANCE:			# No obstacles, traight 
				rospy.loginfo("Full speed forward")
				twist.linear.x = LINEAR_VEL
				twist.angular.z = 0
    
    
			elif min(lidar_distances[180:220]) < SAFE_STOP_DISTANCE:
				if min(lidar_distances[180:220]) < SAFE_STOP_DISTANCE / 2:      # Obstacle very close, 0,175 m
					rospy.loginfo("Hard right turn")
					twist.linear.x = 0
					twist.angular.z = -REL_ANG_VEL * 80
				else:            												# Obstacle left
					rospy.loginfo("Regular right turn")
					twist.linear.x = REL_LIN_VEL * 70
					twist.angular.z = -REL_ANG_VEL * 30
     
			elif min(lidar_distances[140:180]) < SAFE_STOP_DISTANCE:      
				if min(lidar_distances[140:180]) < SAFE_STOP_DISTANCE / 2: 		# Obstacle very close, 0,175 m  
					rospy.loginfo("Hard left turn")
					twist.linear.x = 0
					twist.angular.z = REL_ANG_VEL * 80
				else: 															# Obstacle left
					rospy.loginfo("Regular left turn")
					twist.linear.x = REL_LIN_VEL * 70
					twist.angular.z = REL_ANG_VEL * 30

			else:																# If all else, U turn 
				rospy.loginfo("U turn left")
				twist.linear.x = 0
				twist.angular.z = ANGULAR_VEL
    
			rospy.loginfo("Min distance in front: %f", min(lidar_distances[135:225]))
   
		# Collision counter
			if min(lidar_distances[135:225]) <= LIDAR_ERROR + 0.07 and Collision_Delay <= time.time(): 	
				Collision_Counter += 1						# Collision counter if distance is smaller than 0.05 + 0.07 = 12 cm		
				Collision_Delay = time.time() + 2			# Delay so it only counts one collision
				rospy.loginfo("Collision detetcted! Total collisions: %f", Collision_Counter)

		# Victim counter
			# Color logic
			block = bus.read_i2c_block_data(0x44, 0x09, 6)
			green = int(block[1])*256 + int(block[0])
			red = int(block[3])*256 + int(block[2])
			blue = int(block[5])*256 + int(block[4])
			blue = blue*2
   
			if blue < red and green < red and red - blue > 1000 and red - green > 1000 and victim_Delay <= time.time():
				victim_Counter += 1						
				victim_Delay = time.time() + 2			

			self._cmd_pub.publish(twist)
			Speed_Updates += 1
			Speed_Accumulation += twist.linear.x
			Avg_Lin_Speed = Speed_Accumulation / Speed_Updates			# Calculate avarage linear speed 
			rospy.loginfo("Average linear speed: %f", Avg_Lin_Speed)

		rospy.loginfo("Times up, Stopping robot")
		twist.linear.x = 0
		twist.angular.z = 0
		self._cmd_pub.publish(twist)
		rospy.loginfo("Final average linear speed: %f", Avg_Lin_Speed)
		rospy.loginfo("Final collision count: %f", Collision_Counter)
  		rospy.loginfo("Final victim count: %f", victim_Counter)
			
def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()