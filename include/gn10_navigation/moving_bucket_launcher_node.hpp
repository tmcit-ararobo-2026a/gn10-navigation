#ifndef GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_
#define GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>

#include "gn10_navigation/navigation_types.hpp"
#include "lidar_perception_system/msg/bucket_bottom.hpp"
#include "lidar_perception_system/msg/moving_bucket_track.hpp"

namespace gn10_navigation {

class MovingBucketLauncherNode : public rclcpp::Node {
public:
    explicit MovingBucketLauncherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void loadParameters();
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters
    );

    void onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onBucketOpening(const lidar_perception_system::msg::MovingBucketTrack::ConstSharedPtr msg);
    void onBucketBottom(const lidar_perception_system::msg::BucketBottom::ConstSharedPtr msg);

    bool isPublishAllowed() const;
    void calculateAndPublishSpeed(
        const geometry_msgs::msg::Point& opening_pos_map,
        const std::string& frame_id
    );

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_id_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<lidar_perception_system::msg::MovingBucketTrack>::SharedPtr bucket_opening_sub_;
    rclcpp::Subscription<lidar_perception_system::msg::BucketBottom>::SharedPtr bucket_bottom_sub_;

    // Publisher (Shared topic with target_manager_node; published ONLY when target_id == MovingBucket and nav_cmd == Move)
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr belt_launcher_speed_pub_;

    // Dynamic parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // State
    uint8_t current_target_id_{255};
    NavigationCommand nav_cmd_{NavigationCommand::Stanby};

    // Parameters (managed via config/moving_bucket_launcher_params.yaml & dynamic reconfigure)
    std::string base_frame_{"base_link"};
    std::string opening_topic_{"/bucket_opening_tracker/target"};
    std::string bottom_topic_{"/bucket_opening_tracker/bottom"};
    bool use_bottom_topic_{false};      // false: use upper opening directly; true: derive opening from bottom + bucket_height
    double bucket_height_{0.255};       // Bucket height from bottom to upper opening [m]
    double aim_z_offset_{0.0};          // Vertical offset relative to upper opening center [m]
    double launcher_offset_x_{0.348};   // Effective release X offset in base_link [m]
    double launcher_offset_z_{1.034};   // Effective release Z height in base_link [m]
    double launch_angle_deg_{45.88};    // Effective release elevation angle [deg]
    double velocity_quad_{0.0};         // Quadratic term: v_belt = quad * v0^2 + scale * v0 + offset
    double velocity_scale_{0.806};      // Linear scale from ballistic v0 to belt release speed
    double velocity_offset_{-0.243};    // Constant offset for belt release speed [m/s]
    double min_velocity_{2.5};          // Min allowable launcher velocity [m/s]
    double max_velocity_{7.75};         // Max allowable launcher velocity [m/s]
    double min_horizontal_dist_{0.6};   // Min allowable horizontal distance [m]
    double max_horizontal_dist_{5.5};   // Max allowable horizontal distance [m]
    double max_tracking_age_sec_{0.5};  // Max allowable observation age [s]
};

}  // namespace gn10_navigation

#endif  // GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_
