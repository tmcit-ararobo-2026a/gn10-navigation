#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gn10_navigation/navigation_types.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace gn10_navigation {

class PathFollowerNode : public rclcpp::Node
{
public:
    explicit PathFollowerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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

    void declareAndGetParameters();
    void onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onPlannedPath(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void controlLoop();

    void publishStatus();
    void publishCmdVel(double vx_local, double vy_local, double wz);
    void publishLookaheadMarker(double x, double y);
    void stop();
    Projection project(double x, double y) const;
    SpeedPoint sample(double distance) const;

    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters
    );

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_id_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // パラメータ
    double control_rate_;
    double max_speed_;
    double max_acceleration_;
    double max_deceleration_;
    double max_jerk_;
    double max_lateral_acceleration_;
    double curvature_window_;
    double max_angular_velocity_;
    double max_angular_acceleration_;
    double min_lookahead_;
    double max_lookahead_;
    double lookahead_gain_;
    double goal_tolerance_;
    double goal_approach_speed_;
    double goal_position_gain_;
    double goal_angle_adjust_distance_;
    double goal_yaw_tolerance_;
    double max_path_error_;
    double max_initial_path_error_;
    double max_pose_age_;
    double heading_gain_;

    std::string map_frame_;
    std::string base_frame_;
    std::string bucket_frame_;
    double max_bucket_age_;

    // 内部状態
    NavigationStatus status_{NavigationStatus::Stanby};
    NavigationCommand last_cmd_{NavigationCommand::Stanby};
    TargetId current_target_id_{TargetId::StartZone};

    std::vector<SpeedPoint> path_;
    std::vector<geometry_msgs::msg::PoseStamped> source_path_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool has_path_{false};
    bool goal_latched_{false};
    double progress_{0.0};

    // 前回速度 (加速度制限用)
    double command_vx_world_{0.0};
    double command_vy_world_{0.0};
    double command_ax_world_{0.0};
    double command_ay_world_{0.0};
    double command_yaw_rate_{0.0};
    std::chrono::steady_clock::time_point last_control_;
};

}  // namespace gn10_navigation
