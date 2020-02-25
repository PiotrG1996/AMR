#! /usr/bin/env python

#Author: Piotr Gapski
#Last Modification: Jan 29th 2020
#Task: Final project AMR

"""
Libraries & Functions
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

# Enable laser scanner to calculate and detect obstacles on its way
class follow_obstacle:

    """
    Create a constructor
    """
    def __init__(self):
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    # Publish speed
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback_laser) # Receive data from the sensors
        self.active_ = False
        self.pub_ = None
        self.regions_ = {                  # Initialize regions range
                'right': 0,
                'front_right': 0,
                'front': 0,
                'front_left': 0,
                'left': 0,
        }
        self.state_ = 0
        self.state_dict_ = {            #Three posible states while following the object or wall
            0: 'find the obstacle',
            1: 'turn left',
            2: 'follow the obstacle',
        }
        self.linear_x = 0
        self.angular_z = 0
        self.msg = Twist()
    """
    Set maximum range to 4 and update distance between region and obstacle
    """
    def callback_laser(self, msg):
        self.regions_ = {
            'left':   min(min(msg.ranges[54:90]), 4),
            'front_left':  min(min(msg.ranges[18:54]), 4),
            'front':  min(min(msg.ranges[342:360]),min(msg.ranges[0:18]), 4),
            'front_right': min(min(msg.ranges[306:342]), 4),
            'right':  min(min(msg.ranges[270:305]), 4),
        }
       
        self.take_action()    # Depending on the values, decide which state should be chosen

    def change_state(self,state):
        if state is not self.state_:
            self.state_ = state
    """
    Depending on values from callback_laser choose correct state
    """
    def take_action(self):
        regions = self.regions_
        self.linear_x = 0
        self.angular_z = 0

        state_description = ''

        d = 1

        if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif regions['front'] > d and regions['front_left'] > d and regions['front_right'] < d:
            state_description = 'case 3 - front_right'
            self.change_state(2)
        elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] > d:
            state_description = 'case 4 - front_left'
            self.change_state(0)
        elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] < d:
            state_description = 'case 5 - front and front_right'
            self.change_state(1)
        elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] > d:
            state_description = 'case 6 - front and front_left'
            self.change_state(1)
        elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] < d:
            state_description = 'case 7 - front and front_left and front_right'
            self.change_state(1)
        elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] < d:
            state_description = 'case 8 - front_left and front_right'
            self.change_state(0)
        elif regions['front'] < 0.4 or regions['front_left'] < 0.4 or regions['front_right'] < 0.4 or regions['left'] < 0.4 or regions['right'] > 0.4:
            state_description = 'case 9 - Too close'
            self.change_state(1)
        else:
            state_description = 'unknown case'
            
    """
    Move in circles to find obstacle
    """
    def find_obstacle(self):
        msg = Twist()
        msg.linear.x = 0.4
        msg.angular.z = -0.4
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.4
        msg.linear.x = 0
        return msg
    """
    Follow the obstacle
    """
    def follow_the_obstacle(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0
        return msg
    """
    Run the program
    """
    def run(self):
        rate = rospy.Rate(20)
        pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        msg = Twist()
        if self.state_ == 0:
            msg = self.find_obstacle()
        elif self.state_ == 1:
            msg = self.turn_left()
        elif self.state_ == 2:
            msg = self.follow_the_obstacle()

        pub_.publish(msg)

        rate.sleep()
        return
    """
    Based on laser scanner check regions and return status
    """
    def check_sensors(self):
        regions = self.regions_
        if regions['front'] < 0.8 or regions['front_left'] < 0.4 or regions['front_right'] < 0.4:
            return 'object_infront'
        else:
            return 'free'
