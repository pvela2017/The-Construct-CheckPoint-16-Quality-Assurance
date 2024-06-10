#! /usr/bin/env python3
import rospy
import time
import actionlib

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math

class WaypointActionClass(object):
    # create messages that are used to publish feedback/result
    _feedback = WaypointActionFeedback()
    _result = WaypointActionResult()
    
    # topics
    _pub_cmd_vel = None
    _sub_odom = None

    # go to point vars
    # robot state variables
    _position = Point()
    _yaw = 0
    # machine state
    _state = 'idle'
    # goal
    _des_pos = Point()
    # parameters
    _yaw_precision = math.pi / 90 # +/- 2 degree allowed
    _dist_precision = 0.05

    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer("tortoisebot_as", WaypointActionAction, self.goal_callback, False)
        self._as.start()

        # define a loop rate
        self._rate = rospy.Rate(25)

        # Flags
        self.step1_flag = True
        self.step2_flag = True
        self.final_flag = True

        # topics
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)
        rospy.loginfo("Action server started")

    def _clbk_odom(self, msg):
        # position
        self._position = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]

    def goal_callback(self, goal):
        rospy.loginfo("goal %s received" % str(goal))
        # helper variables
        success = True
        # define desired position and errors
        self._des_pos = goal.position
        desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
        err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
        err_yaw = desired_yaw - self._yaw

        # Reset flags
        self.step1_flag = True
        self.step2_flag = True
        self.final_flag = True

        # perform task
        while self.final_flag and success:
            # update vars
            desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
            err_yaw = desired_yaw - self._yaw
            final_err_yaw = self._des_pos.z - self._yaw
            err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
            #rospy.loginfo("Current Yaw: %s" % str(self._yaw))
            #rospy.loginfo("Desired Yaw: %s" % str(desired_yaw))
            #rospy.loginfo("Error Yaw: %s" % str(err_yaw))
            # logic goes here
            if self._as.is_preempt_requested():
                # cancelled
                rospy.loginfo("The goal has been cancelled/preempted")
                self._as.set_preempted()
                success = False

            if math.fabs(err_yaw) > self._yaw_precision and self.step1_flag:
                # fix yaw
                #rospy.loginfo("fix yaw")
                self._state = 'fix yaw'
                twist_msg = Twist()
                twist_msg.angular.z = 0.65 if err_yaw > 0 else -0.65
                self._pub_cmd_vel.publish(twist_msg)
            elif math.fabs(err_yaw) <= self._yaw_precision and self.step1_flag:
                rospy.loginfo("1")
                self.step1_flag = False

            if err_pos > self._dist_precision and not self.step1_flag and self.step2_flag:
                # go to point
                #rospy.loginfo("go to point")
                self._state = 'go to point'
                twist_msg = Twist()
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0
                twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
                self._pub_cmd_vel.publish(twist_msg)
            elif err_pos <= self._dist_precision and not self.step1_flag and self.step2_flag:
                rospy.loginfo("2")
                self.step2_flag = False

            if math.fabs(final_err_yaw) > 0.05 and not self.step1_flag and not self.step2_flag:
                # rotate in place
                #rospy.loginfo("rotate in place")
                self._state = 'rotate in place'
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.1 if final_err_yaw > 0 else -0.1
                self._pub_cmd_vel.publish(twist_msg)
            elif math.fabs(final_err_yaw) <= 0.05 and not self.step1_flag and not self.step2_flag:
                rospy.loginfo("3")
                self.final_flag = False

            # send feedback
            self._feedback.position = self._position
            self._feedback.state = self._state
            self._as.publish_feedback(self._feedback)

            # loop rate
            self._rate.sleep()

        twist_msg = Twist() # stop
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self._pub_cmd_vel.publish(twist_msg)    

        # return success
        if success:
            self._result.success = True
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('tortoisebot_as')
    WaypointActionClass()
    rospy.spin()