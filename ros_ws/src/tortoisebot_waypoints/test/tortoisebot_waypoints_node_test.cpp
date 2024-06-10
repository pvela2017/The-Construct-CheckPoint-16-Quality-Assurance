#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executors.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <memory>
#include <chrono>
#include <thread>

using GoToPointAction = tortoisebot_waypoints::action::WaypointAction;
using GoalHandleGoToPoint = rclcpp_action::ServerGoalHandle<GoToPointAction>;

class RclCppFixture 
{
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;


class TestRobotControl : public ::testing::Test, public rclcpp::Node 
{
private:
  // ROS variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<GoToPointAction>::SharedPtr actionClient_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

public:
  geometry_msgs::msg::Quaternion initOrientation_;
  geometry_msgs::msg::Quaternion currentOrientation_;
  geometry_msgs::msg::Point currentPosition_;
  double initYaw_;
  double finalYaw_;
  geometry_msgs::msg::Point position_1_;

  explicit TestRobotControl(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("test_node", options)
  {
    // Clients and subscriber declaration
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TestRobotControl::odomCallback, this, std::placeholders::_1), options1);
    actionClient_ = rclcpp_action::create_client<GoToPointAction>(this, "tortoisebot_as");
    gazebo_client_ = this->create_client<std_srvs::srv::Empty>("/reset_world");

    // Get Initial position
    initOrientation_ = geometry_msgs::msg::Quaternion();
    currentOrientation_ = geometry_msgs::msg::Quaternion();
    currentPosition_ = geometry_msgs::msg::Point();
    initYaw_ = 0;
    finalYaw_ = 0;

    // Goal point 
    position_1_ = geometry_msgs::msg::Point();
    position_1_.x = 0.5;
    position_1_.y = 0.5;
    position_1_.z = 0.0;

    // Reset Gazebo world
    serviceCallReset();
    initYaw_ = quaternionToEuler(initOrientation_);
    // Call the action Goal
    serviceCall();
  }

  void serviceCallReset() 
  {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    // Check if the service is available
    if (!gazebo_client_->wait_for_service(std::chrono::seconds(1))) 
    {
        RCLCPP_INFO(this->get_logger(), "Gazebo world was not started");
        return;
    }
    // Call the service
    // https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
    auto result = gazebo_client_->async_send_request(request);
    // Wait for the answer
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Gazebo world restarted");
    }
    else 
    {
        return;
    }
  }

  void serviceCall() 
  {
    // Check if the action is available
    if (!actionClient_->wait_for_action_server(std::chrono::seconds(1))) 
    {
        RCLCPP_INFO(this->get_logger(), "Action was not started");
        return;
    }
    // Create the goal
    auto goalMsg = GoToPointAction::Goal();
    goalMsg.position = position_1_;
    // Send goal
    auto futureGoal = actionClient_->async_send_goal(goalMsg);

    // Wait 30 seconds for the robot to reach the position
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(25)) 
    {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to avoid high CPU usage
    }

  }

  double quaternionToEuler(const geometry_msgs::msg::Quaternion& msg) 
  {
    // Transform quaternion to euler
    tf2::Quaternion tfQuat(msg.x, msg.y, msg.z, msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    // Odom callback
    currentOrientation_ = msg->pose.pose.orientation;
    currentPosition_ = msg->pose.pose.position;
  }
};

TEST_F(TestRobotControl, CorrectRotationTest) 
{
  finalYaw_ = quaternionToEuler(currentOrientation_);
  double yawDiff = std::abs(initYaw_ - finalYaw_);
  //ASSERT_TRUE(true) << "Integration error. Position was not between the expected values.";
  ASSERT_TRUE(yawDiff <= 2.0) << "Integration error. Rotation was not between the expected values.";
}

TEST_F(TestRobotControl, CorrectPositionTest) 
{
  double xDiff = std::abs(currentPosition_.x - position_1_.x);
  double yDiff = std::abs(currentPosition_.y - position_1_.y);
  //ASSERT_TRUE(true) << "Integration error. Position was not between the expected values.";
  ASSERT_TRUE(xDiff <= 0.07 && yDiff <= 0.07) << "Integration error. Position was not between the expected values.";
}