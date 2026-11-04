#include "gn10_navigation/moving_bucket_launcher_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>

namespace gn10_navigation {

MovingBucketLauncherNode::MovingBucketLauncherNode(const rclcpp::NodeOptions& options)
    : Node("moving_bucket_launcher_node", options)
{
    loadParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    target_id_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/target_id",
        10,
        std::bind(&MovingBucketLauncherNode::onTargetId, this, std::placeholders::_1)
    );

    nav_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/navigation_command",
        10,
        std::bind(&MovingBucketLauncherNode::onNavigationCommand, this, std::placeholders::_1)
    );

    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/robot/feedback/drive_battery_voltage",
        10,
        std::bind(&MovingBucketLauncherNode::onBatteryVoltage, this, std::placeholders::_1)
    );

    bucket_bottom_sub_ = this->create_subscription<lidar_perception_system::msg::BucketBottom>(
        "/bucket_opening_tracker/bottom",
        10,
        std::bind(&MovingBucketLauncherNode::onBucketBottom, this, std::placeholders::_1)
    );

    // Shared publisher with target_manager_node.
    // Important: publish ONLY when target_id == MovingBucket to avoid collision.
    belt_launcher_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/operation/belt_launcher_speed", 10
    );

    RCLCPP_INFO(this->get_logger(), "MovingBucketLauncherNode initialized.");
}

void MovingBucketLauncherNode::loadParameters()
{
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("launcher_offset_x", 0.2);
    this->declare_parameter<double>("launcher_offset_z", 0.35);
    this->declare_parameter<double>("launch_angle_deg", 45.0);
    this->declare_parameter<double>("min_velocity", 3.0);
    this->declare_parameter<double>("max_velocity", 7.75);
    this->declare_parameter<double>("velocity_scale", 1.05);
    this->declare_parameter<double>("velocity_offset", 0.1);
    this->declare_parameter<double>("voltage_ref", 20.0);
    this->declare_parameter<double>("voltage_comp_gain", 0.02);
    this->declare_parameter<double>("max_tracking_age_sec", 0.6);

    base_frame_           = this->get_parameter("base_frame").as_string();
    launcher_offset_x_    = this->get_parameter("launcher_offset_x").as_double();
    launcher_offset_z_    = this->get_parameter("launcher_offset_z").as_double();
    launch_angle_deg_     = this->get_parameter("launch_angle_deg").as_double();
    min_velocity_         = this->get_parameter("min_velocity").as_double();
    max_velocity_         = this->get_parameter("max_velocity").as_double();
    velocity_scale_       = this->get_parameter("velocity_scale").as_double();
    velocity_offset_      = this->get_parameter("velocity_offset").as_double();
    voltage_ref_          = this->get_parameter("voltage_ref").as_double();
    voltage_comp_gain_    = this->get_parameter("voltage_comp_gain").as_double();
    max_tracking_age_sec_ = this->get_parameter("max_tracking_age_sec").as_double();
}

void MovingBucketLauncherNode::onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    current_target_id_ = msg->data;
}

void MovingBucketLauncherNode::onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    nav_cmd_ = static_cast<NavigationCommand>(msg->data);
}

void MovingBucketLauncherNode::onBatteryVoltage(const std_msgs::msg::Float32::ConstSharedPtr msg)
{
    current_battery_voltage_ = msg->data;
    last_voltage_time_ = this->now();
}

void MovingBucketLauncherNode::onBucketBottom(
    const lidar_perception_system::msg::BucketBottom::ConstSharedPtr msg
)
{
    // CRITICAL: Avoid collision with target_manager_node.
    // Only publish when current target is MovingBucket (ID 0).
    if (current_target_id_ != static_cast<uint8_t>(TargetId::MovingBucket)) {
        return;
    }

    if (!msg->valid) {
        RCLCPP_DEBUG(this->get_logger(), "BucketBottom measurement is invalid. Skipping speed calculation.");
        return;
    }

    // Check freshness
    rclcpp::Time obs_time(msg->last_observation_stamp);
    if ((this->now() - obs_time).seconds() > max_tracking_age_sec_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Bucket tracking stale (age=%.2f s). Skipping speed calculation.",
            (this->now() - obs_time).seconds()
        );
        return;
    }

    calculateAndPublishSpeed(msg->center, msg->header.frame_id);
}

void MovingBucketLauncherNode::calculateAndPublishSpeed(
    const geometry_msgs::msg::Point& bucket_bottom_pos,
    const std::string& frame_id
)
{
    // Transform bucket bottom point to base_frame
    geometry_msgs::msg::PointStamped point_in, point_robot;
    point_in.header.frame_id = frame_id;
    point_in.header.stamp = rclcpp::Time(0);  // Latest available TF
    point_in.point = bucket_bottom_pos;

    try {
        point_robot = tf_buffer_->transform(point_in, base_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Could not transform bucket point from '%s' to '%s': %s",
            frame_id.c_str(),
            base_frame_.c_str(),
            ex.what()
        );
        return;
    }

    // Calculate relative horizontal distance L and vertical delta Z relative to launcher muzzle
    const double rel_x = point_robot.point.x - launcher_offset_x_;
    const double rel_y = point_robot.point.y;
    const double rel_z = point_robot.point.z - launcher_offset_z_;

    const double horizontal_dist = std::hypot(rel_x, rel_y);
    if (horizontal_dist < 0.5) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Bucket too close (L=%.2f m)", horizontal_dist);
        return;
    }

    // Ballistic trajectory model:
    // delta_z = L * tan(theta) - (g * L^2) / (2 * v0^2 * cos^2(theta))
    // => v0 = sqrt( (g * L^2) / (2 * cos^2(theta) * (L * tan(theta) - delta_z)) )
    constexpr double g = 9.80665;
    const double theta_rad = launch_angle_deg_ * M_PI / 180.0;
    const double cos_theta = std::cos(theta_rad);
    const double tan_theta = std::tan(theta_rad);

    const double denom = 2.0 * cos_theta * cos_theta * (horizontal_dist * tan_theta - rel_z);
    if (denom <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Target bucket unreachable at L=%.2f m, delta_z=%.2f m with angle=%.1f deg",
            horizontal_dist,
            rel_z,
            launch_angle_deg_
        );
        return;
    }

    const double v_ballistic = std::sqrt((g * horizontal_dist * horizontal_dist) / denom);

    // Apply scaling and empirical offset
    double v_target = v_ballistic * velocity_scale_ + velocity_offset_;

    // Battery voltage compensation
    if (current_battery_voltage_ >= 15.0f && (this->now() - last_voltage_time_).seconds() <= 2.0) {
        const double v_err = voltage_ref_ - static_cast<double>(current_battery_voltage_);
        v_target *= (1.0 + voltage_comp_gain_ * v_err);
    }

    // Clamp within hardware limit
    const double v_clamped = std::clamp(v_target, min_velocity_, max_velocity_);

    // Publish
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(v_clamped);
    belt_launcher_speed_pub_->publish(speed_msg);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        250,
        "MovingBucket target: L=%.2f m, dZ=%.2f m -> belt_speed=%.2f m/s (raw=%.2f m/s)",
        horizontal_dist,
        rel_z,
        v_clamped,
        v_target
    );
}

}  // namespace gn10_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::MovingBucketLauncherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

