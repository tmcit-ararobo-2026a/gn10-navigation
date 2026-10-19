#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
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
    /** @brief Clear translation without interrupting independent bucket tracking. */
    void clear_translation();
    void clear_path(bool failed);
    /** @brief Validate path positions, frames and bounded preprocessing size. */
    bool valid_path(const nav_msgs::msg::Path& path) const;
    /** @brief Resolve bearing from fresh bucket position; never use its orientation. */
    bool bucket_heading(double x, double y, double yaw, double& error);
    /** @brief Publish standard velocity and an LED-only availability indication. */
    void publish_velocity(const geometry_msgs::msg::Twist& velocity, bool can_move);
    Projection project(double x, double y) const;
    SpeedPoint sample(double distance) const;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr speed_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_move_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<SpeedPoint> path_;
    std::string map_frame_;
    std::string base_frame_;
    std::string bucket_frame_;
    double progress_         = 0.0;
    double command_vx_world_ = 0.0;
    double command_vy_world_ = 0.0;
    double command_ax_world_ = 0.0;
    double command_ay_world_ = 0.0;
    double command_yaw_rate_ = 0.0;
    bool path_failed_        = false;
    int64_t bucket_stamp_ns_ = 0;
    std::chrono::steady_clock::time_point bucket_updated_;
    std::chrono::steady_clock::time_point last_control_;

    double control_rate_;
    double max_speed_;
    double max_acceleration_;
    double max_deceleration_;
    double max_jerk_;
    double max_lateral_acceleration_;
    double curvature_window_;
    double max_angular_speed_;
    double max_angular_acceleration_;
    double heading_gain_;
    double wheel_radius_;
    double wheel_center_distance_;
    double max_wheel_angular_speed_;
    std::vector<double> wheel_mount_angles_deg_;
    double min_lookahead_;
    double max_lookahead_;
    double lookahead_gain_;
    double goal_tolerance_;
    double goal_approach_speed_;
    double max_path_error_;
    double max_pose_age_;
    double max_bucket_age_;
};
