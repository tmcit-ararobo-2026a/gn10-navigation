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
    void declareAndGetParameters();
    void onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onPlannedPath(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void controlLoop();

    void publishStatus();
    void publishCmdVel(double vx_local, double vy_local, double wz);
    void publishLookaheadMarker(double x, double y);

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
    double control_rate_;      // 50.0 Hz
    double max_speed_;         // 1.5 m/s
    double max_acceleration_;  // 1.0 m/s^2
    double min_lookahead_;     // 0.4 m
    double goal_tolerance_;    // 0.03 m
    double heading_gain_;      // 2.0
    double creep_gain_;        // 終端偏差Pゲイン
    double min_creep_speed_;   // 0.05 m/s

    std::string map_frame_;
    std::string base_frame_;
    std::string bucket_frame_;
    double max_bucket_age_;

    // 内部状態
    NavigationStatus status_{NavigationStatus::Stanby};
    NavigationCommand last_cmd_{NavigationCommand::Stanby};
    TargetId current_target_id_{TargetId::StartZone};

    std::vector<geometry_msgs::msg::PoseStamped> current_path_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool has_path_{false};

    // 前回速度 (加速度制限用)
    double current_vx_local_{0.0};
    double current_vy_local_{0.0};
    double current_wz_{0.0};
};

}  // namespace gn10_navigation
