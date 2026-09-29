#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PathFollowerNode : public rclcpp::Node
{
public:
    PathFollowerNode();

private:
    struct SpeedPoint {
        double x;
        double y;
        double distance;
        double speed;
        double time;
    };

    struct Projection {
        double distance;
        double error;
    };

    void onPath(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void onTimer();
    void stop();
    Projection project(double x, double y) const;
    SpeedPoint sample(double distance) const;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr speed_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<SpeedPoint> path_;
    std::string map_frame_;
    std::string base_frame_;
    double progress_ = 0.0;
    double command_speed_ = 0.0;
    double command_acceleration_ = 0.0;
    double command_yaw_rate_ = 0.0;
    std::chrono::steady_clock::time_point last_control_;

    double control_rate_;
    double max_speed_;
    double max_acceleration_;
    double max_deceleration_;
    double max_jerk_;
    double max_lateral_acceleration_;
    double max_angular_speed_;
    double max_angular_acceleration_;
    double min_lookahead_;
    double max_lookahead_;
    double lookahead_gain_;
    double goal_tolerance_;
    double max_path_error_;
    double max_pose_age_;
};
