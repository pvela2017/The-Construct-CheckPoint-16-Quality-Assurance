#include "tortoisebot_waypoints/waypoint_action_server.h"

using namespace std::placeholders;

WaypointActionServer::WaypointActionServer(const rclcpp::NodeOptions &options) : Node("waypoint_action", options), step1_flag_(true), step2_flag_(true), final_flag_(true) 
{
    //Parameters
    this->declare_parameter<double>("yaw_precision", M_PI / 90);
    this->declare_parameter<double>("dist_precision", 0.05);
    this->get_parameter("yaw_precision", yaw_precision_);
    this->get_parameter("dist_precision", dist_precision_);

    // Callbacks
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_;

    action_server_ = rclcpp_action::create_server<GoToPointAction>(
                     this,
                     "/tortoisebot_as", 
                     std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
                     std::bind(&WaypointActionServer::handle_cancel, this, _1),
                     std::bind(&WaypointActionServer::handle_accepted, this, _1));

    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&WaypointActionServer::_clbk_odom, this, _1), options1);  
}

WaypointActionServer::~WaypointActionServer(){}

rclcpp_action::GoalResponse WaypointActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPointAction::Goal> goal)
{
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointActionServer::handle_cancel(const std::shared_ptr<GoalHandleGoToPoint> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointActionServer::handle_accepted(const std::shared_ptr<GoalHandleGoToPoint> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&WaypointActionServer::executeCB, this, _1), goal_handle}.detach();
    
}

void WaypointActionServer::executeCB(const std::shared_ptr<GoalHandleGoToPoint> goal_handle) 
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPointAction::Feedback>();
    auto result = std::make_shared<GoToPointAction::Result>();

    bool success = true;
    des_pos_ = goal->position;
    double desired_yaw = atan2(des_pos_.y - position_.y, des_pos_.x - position_.x);
    double err_pos = sqrt(pow(des_pos_.y - position_.y, 2) + pow(des_pos_.x - position_.x, 2));
    double err_yaw = desired_yaw - yaw_;
    step1_flag_ = true;
    step2_flag_ = true;
    final_flag_ = true;

    rclcpp::Rate rate(10);
    while (final_flag_ && success) 
    {
        desired_yaw = atan2(des_pos_.y - position_.y, des_pos_.x - position_.x);
        err_yaw = desired_yaw - yaw_;
        double final_err_yaw = des_pos_.z - yaw_;
        err_pos = sqrt(pow(des_pos_.y - position_.y, 2) + pow(des_pos_.x - position_.x, 2));

        if (fabs(err_yaw) > yaw_precision_ && step1_flag_) {
            state_ = "fix yaw";
            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            twist_msg->angular.z = (err_yaw > 0) ? 0.65 : -0.65;
            pub_cmd_vel_->publish(std::move(twist_msg));
        }
        else if (fabs(err_yaw) <= yaw_precision_ && step1_flag_) {
            RCLCPP_INFO(this->get_logger(), "1");
            step1_flag_ = false;
        }

        if (err_pos > dist_precision_ && !step1_flag_ && step2_flag_) {
            state_ = "go to point";
            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            twist_msg->linear.x = 0.2;
            twist_msg->angular.z = (err_yaw > 0) ? 0.1 : -0.1;
            pub_cmd_vel_->publish(std::move(twist_msg));
        }
        else if (err_pos <= dist_precision_ && !step1_flag_ && step2_flag_) {
            RCLCPP_INFO(this->get_logger(), "2");
            step2_flag_ = false;
        }

        if (fabs(final_err_yaw) > 0.05 && !step1_flag_ && !step2_flag_) {
            state_ = "rotate in place";
            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            twist_msg->linear.x = 0.0;
            twist_msg->angular.z = (final_err_yaw > 0) ? 0.5 : -0.5;
            pub_cmd_vel_->publish(std::move(twist_msg));
        }
        else if (fabs(final_err_yaw) <= 0.05 && !step1_flag_ && !step2_flag_) {
            RCLCPP_INFO(this->get_logger(), "3");
            final_flag_ = false;
        }
        rate.sleep();
    }

    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = 0;
    twist_msg->angular.z = 0;
    pub_cmd_vel_->publish(std::move(twist_msg));
    result->success = success;
    goal_handle->succeed(result);
}

void WaypointActionServer::_clbk_odom(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    position_ = msg->pose.pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
}
