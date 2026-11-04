#ifndef GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_
#define GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gn10_navigation/navigation_types.hpp"
#include "lidar_perception_system/msg/bucket_bottom.hpp"

namespace gn10_navigation {

class MovingBucketLauncherNode : public rclcpp::Node {
public:
    explicit MovingBucketLauncherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void loadParameters();
    void onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onBatteryVoltage(const std_msgs::msg::Float32::ConstSharedPtr msg);
    void onBucketBottom(const lidar_perception_system::msg::BucketBottom::ConstSharedPtr msg);
    void calculateAndPublishSpeed(const geometry_msgs::msg::Point& bucket_bottom_pos, const std::string& frame_id);

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_id_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<lidar_perception_system::msg::BucketBottom>::SharedPtr bucket_bottom_sub_;

    // Publisher (Shared topic with target_manager_node; published ONLY when target_id == MovingBucket)
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr belt_launcher_speed_pub_;

    // State
    uint8_t current_target_id_{255};
    NavigationCommand nav_cmd_{NavigationCommand::Stanby};
    float current_battery_voltage_{20.0f};
    rclcpp::Time last_voltage_time_{0, 0, RCL_ROS_TIME};

    // Parameters
    std::string base_frame_{"base_link"};
    double launcher_offset_x_{0.2};   // Launcher exit offset relative to base_link [m]
    double launcher_offset_z_{0.35};  // Launcher exit height relative to base_link [m]
    double launch_angle_deg_{45.0};   // Launch elevation angle [deg]
    double min_velocity_{3.0};        // Min allowable launcher velocity [m/s]
    double max_velocity_{7.75};       // Max allowable launcher velocity [m/s]
    double velocity_scale_{1.05};     // Drag & slip scale coefficient
    double velocity_offset_{0.1};     // Constant offset [m/s]
    double voltage_ref_{20.0};        // Reference battery voltage [V]
    double voltage_comp_gain_{0.02};  // Voltage compensation coefficient per Volt
    double max_tracking_age_sec_{0.5};// Max allowable age for bottom measurement [s]
};

}  // namespace gn10_navigation

#endif  // GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_

