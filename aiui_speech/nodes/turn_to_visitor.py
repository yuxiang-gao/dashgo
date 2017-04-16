#!/usr/bin/env python

""" turn_to_visitor.py - Version 1.1 2017-4-16
    Enable the robot to react to wakeuo calls from
    AIUI board and turn itself towards the sound source.
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, pi
from std_msgs.msg import Int16
import PyKDL


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


class TurnToVisitor():
    def __init__(self):
        # Give the node a name
        rospy.init_node('turn_to_visitor', anonymous=False)

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        # A flag to determine whether or not turn_to_visitor is enabled
        self.enabled = False

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Publisher to reset the beam number
        self.resetBeam = rospy.Publisher('reset_beam', Int16, queue_size=1)

        # How fast will we update the robot's movement?
        rate = 20

        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)

        # Set the rotation speed in radians per second
        angular_speed = 0.5

        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)

        # Subscribe to the beam_angle topic to receive wake up angles.
        rospy.Subscriber('beam_angle', Int16, self.angle_callback)

        # Subscribe to the robot_idle topic to receive wake up angles.
        rospy.Subscriber('robot_idle', Int16, self.idle_callback)

        # Set the rotation angle to Pi radians (180 degrees)
        goal_angle = (self.beamAngle - 180) / 180.0 * pi

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(
                    self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo(
                    "Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        # Get the starting position values
        (position, rotation) = self.get_odom()

        # Stop the robot before the rotation
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

        # Set the movement command to a rotation
        move_cmd.angular.z = angular_speed

        # Track the last angle measured
        last_angle = rotation

        # Track how far we have turned
        turn_angle = 0

        # A flag to determine if it is ok to reset beam angle
        looped = False

        while (self.enabled and
               abs(turn_angle + angular_tolerance) < abs(goal_angle) and
               not rospy.is_shutdown()):
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            r.sleep()

            # Get the current rotation
            (position, rotation) = self.get_odom()

            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)

            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation
            looped = True

        # Reset beam number
        if looped:
            self.resetBeam.publish(2)
            looped = False

        # Stop the robot before the next loop
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def angle_callback(self, msg):
        self.beamAngle = msg.data

    def idle_callback(self, msg):
        if msg.data:
            self.enabled = True
        else:
            self.enabled = False

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        TurnToVisitor()
        rospy.spin()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
