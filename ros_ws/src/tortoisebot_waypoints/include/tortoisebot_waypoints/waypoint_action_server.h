#ifndef WAYPOINT_ACTION_SERVER_H
#define WAYPOINT_ACTION_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>
#include <memory>


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using GoToPointAction = tortoisebot_waypoints::action::WaypointAction;
using GoalHandleGoToPoint = rclcpp_action::ServerGoalHandle<GoToPointAction>;

class WaypointActionServer : public rclcpp::Node 
{
public:
    explicit WaypointActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()); 
    ~WaypointActionServer();


private:
    rclcpp_action::Server<GoToPointAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    geometry_msgs::msg::Point position_;
    double yaw_;
    std::string state_;
    geometry_msgs::msg::Point des_pos_;
    double yaw_precision_;
    double dist_precision_;
    bool step1_flag_;
    bool step2_flag_;
    bool final_flag_;

    void _clbk_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
 
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GoToPointAction::Goal>);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPoint>);
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPoint>);
    void executeCB(const std::shared_ptr<GoalHandleGoToPoint>);
};

#endif // WAYPOINT_ACTION_SERVER_H