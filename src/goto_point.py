#!/usr/bin/env python

# Author: Piotr Gapski
# Last Modification: Jan 29th 2020
# Task: Final project AMR

# Source: http://openbookproject.net/thinkcs/python/english3e/classes_and_objects_I.html


"""
Libraries & Functions
"""
import rospy
from tf.transformations import euler_from_quaternion
from math import atan2
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion,  Twist
from std_msgs.msg import Float32MultiArray

# Functions provide the shortest way to the individual goal


class goto_point:

    """
    Create a constructor
    """

    def __init__(self):
        # get position of robot and objects
        sub = rospy.Subscriber("/gazebo/model_states",
                               ModelStates, self.callback)
        self.x = 0              # Contains x coordinate
        self.y = 0              # Contains y coordinate
        self.theta = 0          # Define rotation's angle
        self.goal = Point()     # Instantiate an object of type Point
        self.speed = [0, 0]      # Initialize speed
        self.angle_to_goal = 0
        self.return_values = [0, 0, 0, 0, 0, 0]  # X, Y  - Angle to goal

    """
    Allocate exact position of robot in variables x,y,rot_q
    """

    def callback(self, msg):

        self.x = msg.pose[1].position.x
        self.y = msg.pose[1].position.y
        rot_q = msg.pose[1].orientation
        (roll, pitch, self.theta) = euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    """
    Define position of actual goal
    """

    def set_goal(self, goto_x, goto_y):
        self.goal.x = goto_x
        self.goal.y = goto_y
        print(self.goal.x, self.goal.y)
    """
    Imaginary line leads to the next goal
    """

    def return_allvalues(self):
        self.return_values[0] = self.x
        self.return_values[1] = self.y
        self.return_values[2] = self.angle_to_goal
        return self.return_values

    """
    Filtr the array and choose the closest point
    """

    def close_goal(self):
    """
    Move linearly the goal or turn till the angle is the right one
    """

    def move(self):

        inc_x = self.goal.x - self.x  # Distance between robot and goal
        inc_y = self.goal.y - self.y
        # print(inc_y, inc_x)
        self.angle_to_goal = atan2(inc_y, inc_x)

        # my_speed = [self.speed[0], self.speed[1]]
        if abs(self.angle_to_goal - self.theta) > 0.2:
            self.speed[0] = 0.0  # Turn if angle is too large
            self.speed[1] = 0.45
        else:
            self.speed[0] = 0.55  # If not, move forward
            self.speed[1] = 0.0

        # The goal is reached, stop.
        if((abs(inc_x) <= 0.5) or (abs(inc_y) <= 0.5)):
            self.speed[0] = 0.0
            self.speed[1] = 0.0
        return(self.speed)
