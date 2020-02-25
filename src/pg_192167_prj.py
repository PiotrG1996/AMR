#!/usr/bin/python

# Author: Piotr Gapski
# Last Modification: Jan 29th 2020
# Task: Final project AMR


"""
Libraries & Functions
"""
from follow_obstacle import follow_obstacle
from goto_point import goto_point
from goal_subscriber import goal_subscriber
from vel_publisher import vel_publisher
import math
import time
import rospy
from math import tan

"""
Classes
"""

# Main class contains all necessary functions provided by other files and allows to run the robot


class Final_Project:
    """
    Create object for every class
    """

    def __init__(self):
        self.vel_pub = vel_publisher()
        self.goal_subs = goal_subscriber()
        self.goto_point = goto_point()
        self.follow_obstacle = follow_obstacle()
        self.goals_points = []    # Make list of 0 length - store goals inside
    """
    Run the program
    """

    def run(self):
        state_description = ''
        while(self.goal_subs.started() == 0):             # While goals are available
            if (self.goal_subs.started() == 1):           # Break if goals are not sent
                break
        self.goals_points = self.goal_subs.receive_goals()  # Store recived points
        i = 0
       # max_goals = len(goals_points)
        while(i < 30):
            # Get one closest individual point
            self.goto_point.set_goal(
                self.goals_points[i].x, self.goals_points[i].y)
            # Store information about imaginary line
            counter = 0
            while(True):

                # Contains the robots actual position and angle to the goal
                values = []
                # Move towards the goal with defined speed
                my_speed = self.goto_point.move()
                # Publish linear and angular speeds
                self.vel_pub.move_linear(my_speed[0])
                # print(my_speed)
                self.vel_pub.move_angular(my_speed[1])
                # print(my_speed)
                status = self.follow_obstacle.check_sensors()   # Check status of laser scanner
                if(status == 'object_infront'):
                    # Save the position where the object was found
                    values = self.goto_point.return_allvalues()
                    x_enc = values[0]
                    y_enc = values[1]
                    angle_togoal = values[2]
                    # Restart if robot doesn't find any object after defined end time
                    start = time.time()
                    """
                        Follow the object until the imaginary line is found
                        """
                    while(True):

                        # Store robots actual position while following the obstacle
                        values = self.goto_point.return_allvalues()
                        x_diff = abs(values[0] - x_enc)
                        y_diff = abs(values[1] - y_enc)
                        self.follow_obstacle.run()  # Follow the obstacle

                        # Is the robot moving to the closest point?
                        if (((abs(x_diff - abs((y_diff / tan(angle_togoal))))) < 0.1) and y_diff > 0.2):
                            status = 'free'
                            counter = counter + 1
                            print(counter, "No obstacle, let's go!")
                            # The robot might be in a loop, therefore get out of there and restart the count
                            if(counter > 8):
                                start_follow = time.time()
                                counter = 0
                                print("Stucked in a loop")
                                # Get out of the loop following the nearest object or turning in circles
                                while((time.time() - start_follow) < 70):
                                    self.follow_obstacle.run()  # For 70 seconds
                                    """
                                    Continue moving towards the goal
                                    """
                            break

                        end = time.time()
                        # Count to 5 mins if not found imaginary line to the closest point
                        if((end - start) > 100):
                            break  # Stop following it and try to reach the goal once again
                if (my_speed == [0, 0]):  # If the both speeds are zero, then a goal was reached
                    break
            i = i + 1
            print"Goal Reached: ", i
            if(i == 30):
                print"All goals reached!"
                return


def main():
    rospy.init_node("final_project")
    app = Final_Project()
    r = rospy.Rate(4)
    app.run()
    while not rospy.is_shutdown():
        r.sleep()


main()
