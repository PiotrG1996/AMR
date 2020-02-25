#!/usr/bin/env python

#Author: Piotr Gapski
#Last Modification: Jan 29th 2020
#Task: Final project AMR

"""
Libraries & Functions
"""
import rospy
from geometry_msgs.msg import Twist

# Publish the speed to the robot
class vel_publisher:

    def __init__(self):
        """
        Set default values and create objects used to publish the velocity
        """
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)  	#create new publisher	
	
        self.linear_speed = 0.5  
        self.angular_speed = 0.3
        self.speed = Twist()    #Object to be published
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
    """
    Move linearly
    """
    def move_linear(self,linear_speed):
        self.speed.linear.x = linear_speed
        self.pub.publish(self.speed)

    """
    Move angular
    """
    def move_angular(self,angular_speed):
        self.speed.angular.z = angular_speed
        self.pub.publish(self.speed)
    """
    Stop the robot
    """
    def stop(self):
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
