#! /usr/bin/env python3

from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import actionlib
import rospy
import rosunit
import unittest
import rostest
import time
PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_waypoints_integration_test'

class TestRobotControl(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.init_orientation = Quaternion()
        self.current_orientation = Quaternion()
        self.current_position = Point()
        self.init_yaw = 0
        self.final_yaw = 0
        self.position_1 = Point(0.5, 0.5, 0.0) # angle must be in rad, angle is w/r initial position
        self.service_call_reset()
        self.get_init_position()
        self.service_call()

    def get_init_position(self):
        rospy.wait_for_message("/odom", Odometry, timeout=10)
        self.init_orientation = self.current_orientation
        self.init_yaw = self.quaternion_to_euler(self.init_orientation)

    def odom_callback(self, msg):
        self.current_orientation = msg.pose.pose.orientation
        self.current_position = msg.pose.pose.position

    def quaternion_to_euler(self, msg):
        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def service_call(self):
        action_client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        action_client.wait_for_server()
        tests = [self.position_1]

        for point in tests:
            print("Requesting %s+%s+%s" % (point.x, point.y, point.z))
            goal = WaypointActionGoal(point)
            action_client.send_goal(goal)
            action_client.wait_for_result(rospy.Duration.from_sec(20.0))

    def service_call_reset(self):
        rospy.wait_for_service('/gazebo/reset_world')
        s = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resp = s.call(EmptyRequest())

    def test_correct_rotation(self):
        self.final_yaw = self.quaternion_to_euler(self.current_orientation)
        yaw_diff = abs(self.init_yaw - self.final_yaw)
        self.assertTrue((yaw_diff <= 2.0), "Integration error. Rotation was not between the expected values.")

    def test_correct_position(self):
        x_diff = abs(self.current_position.x - self.position_1.x)
        y_diff = abs(self.current_position.y - self.position_1.y)
        self.assertTrue((x_diff <= 0.05 and y_diff <= 0.05), "Integration error. Position was not between the expected values.")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRobotControl)