#!/usr/bin/env python

""" nav_test.py - Version 1.1 2013-12-20

    Command a robot to move autonomously among a number of goal
    locations defined in the map frame. On each round, select a
    new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total
    distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from random import sample
from math import pow, sqrt


class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()
        route = list()

        # locations['loc_1'] = Pose(
        #     Point(-0.917, 5.665, 0.000),
        #     Quaternion(0.000, 0.000, 0.478, 0.879))
        # locations['loc_2'] = Pose(
        #     Point(0.730, 7.884, 0.000),
        #     Quaternion(0.000, 0.000, 0.948, 0.317))
        # locations['loc_3'] = Pose(
        #     Point(-3.719, 4.401, 0.000),
        #     Quaternion(0.000, 0.000, 0.733, 0.680))
        # locations['loc_4'] = Pose(
        #     Point(0.720, 2.229, 0.000),
        #     Quaternion(0.000, 0.000, 0.786, 0.618))
        # locations['loc_5'] = Pose(
        #     Point(1.471, 1.007, 0.000),
        #     Quaternion(0.000, 0.000, 0.480, 0.877))
        # locations['loc_6'] = Pose(
        #     Point(-0.861, -0.019, 0.000),
        #     Quaternion(0.000, 0.000, 0.892, -0.451))

        # locations['loc_1'] = Pose(
        #     Point(-13.079, 10.644, 0.000),
        #     Quaternion(0.000, 0.000, 0.952, 0.305))
        # locations['loc_2'] = Pose(
        #     Point(-8.872, 14.910, 0.000),
        #     Quaternion(0.000, 0.000, -0.836, 0.549))
        # locations['loc_3'] = Pose(
        #     Point(-4.766, 12.545, 0.000),
        #     Quaternion(0.000, 0.000, 0.950, 0.314))
        # locations['loc_4'] = Pose(
        #     Point(-1.727, 10.284, 0.000),
        #     Quaternion(0.000, 0.000, 0.986, 0.169))
        # locations['loc_5'] = Pose(
        #     Point(0.947, 7.799, 0.000),
        #     Quaternion(0.000, 0.000, 0.940, 0.340))
        # locations['loc_6'] = Pose(
        #     Point(-1.116, 5.282, 0.000),
        #     Quaternion(0.000, 0.000, 0.450, 0.893))
        locations['loc_1'] = Pose(
            Point(1.312, 7.452, 0.00),
            Quaternion(0.000, 0.000, 0.961, 0.277))
        locations['loc_2'] = Pose(
            Point(-0.993, 9.096, 0.000),
            Quaternion(0.000, 0.000, 0.950, 0.312))
        locations['loc_3'] = Pose(
            Point(-3.958, 11.547, 0.000),
            Quaternion(0.000, 0.000, 0.950, 0.313))
        locations['loc_4'] = Pose(
            Point(-4.766, 12.545, 0.000),
            Quaternion(0.000, 0.000, 0.950, 0.314))
        locations['loc_5'] = Pose(
            Point(-8.897, 14.415, 0.000),
            Quaternion(0.000, 0.000, 0.930, -0.367))
        locations['loc_6'] = Pose(
            Point(0.730, 7.884, 0.000),
            Quaternion(0.000, 0.000, 0.948, 0.317))



 
        route = ['loc_1', 'loc_2', 'loc_3', 'loc_4', 'loc_5', 'loc_6']
        # route = ['loc_1', 'loc_2']

        # Publisher to manually control the robot
        # e.g. to stop it
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Pub the current location
        self.currLoc = rospy.Publisher('curr_loc', String, queue_size=5)

        # whether the robot is idle, pub for turn to visitor
        self.robotState = rospy.Publisher('robot_state', String, queue_size=1)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()

        self.currLoc.publish('loc_0')
        rospy.logdebug("self.currLoc.publish('loc_0')")

        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(route)
        n_goals = 0
        n_successes = 0
        i = 0
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        endSign = False

        # Get the initial pose from the user
        rospy.loginfo(
            "*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped,
                         self.update_initial_pose)

        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("Starting navigation test")

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,
            # start with a new random sequence
            if i < n_locations:
                self.robotState.publish('working')
                # Get the next location in the current route
                location = route[i]

                # Keep track of the distance traveled.
                # Use updated initial pose if available.
                if initial_pose.header.stamp == "":
                    distance = sqrt(pow(locations[location].position.x -
                                        locations[last_location].position.x, 2) +
                                    pow(locations[location].position.y -
                                        locations[last_location].position.y, 2))
                else:
                    rospy.loginfo("Updating current pose.")
                    distance = sqrt(pow(locations[location].position.x -
                                        initial_pose.pose.pose.position.x, 2) +
                                    pow(locations[location].position.y -
                                        initial_pose.pose.pose.position.y, 2))
                    initial_pose.header.stamp = ""

                # Store the last location for distance calculations
                last_location = location

                # Increment the counters
                i += 1
                n_goals += 1

                # Set up the next goal location
                self.goal = MoveBaseGoal()
                self.goal.target_pose.pose = locations[location]
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()

                # Let the user know where the robot is going next
                rospy.loginfo("Going to: " + str(location))

                # Start the robot toward the next location
                self.move_base.send_goal(self.goal)

                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(
                    rospy.Duration(300))

                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    self.currLoc.publish('timeout')  # pub info for AIUI
                    rospy.loginfo("Timed out achieving goal")
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                        n_successes += 1
                        distance_traveled += distance
                        self.currLoc.publish(location)  # pub info for AIUI
                        rospy.loginfo("State:" + str(state))
                    else:
                        rospy.loginfo("Goal failed with error code: " +
                                      str(goal_states[state]))

                # How long have we been running?
                running_time = rospy.Time.now() - start_time
                running_time = running_time.secs / 60.0

                # Print a summary success/failure, distance traveled and time
                # elapsed
                rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                              str(n_goals) + " = " +
                              str(100 * n_successes / n_goals) + "%")
                rospy.loginfo("Running time: " +
                              str(trunc(running_time, 1)) +
                              " min Distance: " +
                              str(trunc(distance_traveled, 1)) + " m")
                self.robotState.publish('idle')
                rospy.sleep(self.rest_time)
            elif not endSign:
                endSign = True
                self.robotState.publish('end')
                self.currLoc.publish('end')
                rospy.loginfo('Tour end')
            else:
                pass

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")


