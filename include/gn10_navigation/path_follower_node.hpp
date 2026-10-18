#pragma once

#include <chrono>
#include <memory>
#include <set>
#include <rclcpp_action/rclcpp_action.hpp>
#include "gn10_navigation/action/follow_path.hpp"
#include "gn10_navigation/msg/control_lease.hpp"
#include "gn10_navigation/msg/path_velocity.hpp"
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
    using FollowPath = gn10_navigation::action::FollowPath;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;
    /** @brief Finish once, zero output and erase every path/control history. */
    void finish_action(uint8_t error_code, const std::string & reason);
    /** @brief Validate full path/frame/quaternion before accepting an Action. */
    bool valid_path(const nav_msgs::msg::Path & path) const;
    /** @brief Admit current, strictly advancing ControlLease snapshots. */
    void accept_lease(const gn10_navigation::msg::ControlLease & lease);
    /** @brief Verify matching lease, monotonic expiry and clock progress. */
    bool lease_fresh();
    /** @brief Publish generation-tagged base-frame velocity. */
    void publish_velocity(const geometry_msgs::msg::Twist & velocity, bool valid);
    /** @brief Publish progress for the currently active goal only. */
    void publish_feedback(double goal_yaw_error);
    rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> active_goal_;
    rclcpp::Publisher<gn10_navigation::msg::PathVelocity>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<gn10_navigation::msg::ControlLease>::SharedPtr lease_subscription_;
    gn10_navigation::msg::ControlLease lease_{};
    std::chrono::steady_clock::time_point lease_deadline_{};
    int64_t last_lease_ros_now_ns_{};
    bool have_lease_{};
    bool cancellation_requested_{};
    bool legacy_input_enabled_{};
    uint64_t execution_session_{};
    uint64_t execution_generation_{};
    uint64_t velocity_sequence_{};
    std::set<uint64_t> retired_sessions_;
    double lease_timeout_s_{};
    double goal_heading_tolerance_rad_{};
    double goal_yaw_{};
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
    double command_vx_world_ = 0.0;
    double command_vy_world_ = 0.0;
    double command_ax_world_ = 0.0;
    double command_ay_world_ = 0.0;
    double command_yaw_rate_ = 0.0;
    double heading_target_ = 0.0;
    bool heading_locked_ = false;
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
    std::string heading_mode_;
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
};
