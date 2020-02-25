#!/usr/bin/python

#Author: Piotr Gapski
#Last Modification: Jan 29th 2020
#Task: Final project AMR

"""
Libraries & Functions
"""
import rospy
from goal_publisher.msg import PointArray #Message type

# Subscribe goals from the Publisher and send them to the main class
class goal_subscriber:
    """
    Create a constructor
    """
    def __init__(self):
        rospy.Subscriber('/goals', PointArray, self.callback)
        self.goals = []
	self.status = 0

    def callback(self, msg):     # The goals are being sent
        self.goals = msg.goals
        # print(self.goals)

    def receive_goals(self):
        return self.goals       # Recive the goals

    def started(self):          # Check if array with goals is sent
        if self.goals == []:
           self.status = 0      # If true, send status 0
        # print(self.status)
	   return  self.status

        else:
	   self.status = 1          # If false, send status 1
	   return  self.status
