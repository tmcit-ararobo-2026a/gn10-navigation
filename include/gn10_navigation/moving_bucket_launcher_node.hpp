#ifndef GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_
#define GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_

/**
 * @file moving_bucket_launcher_node.hpp
 * @brief 移動バケツ追従時のリアルタイム弾道射出速度計算ノード
 *
 * 【射出速度逆算アルゴリズムの概要】
 * 1. 排他配信制御:
 *    - target_id == MovingBucket (7) かつ navigation_command == Move (2) の時のみ計算・配信。
 *    - それ以外のモードでは target_manager_node とのトピック干渉を防ぐため配信を完全停止し、
 *      内部のEMAフィルタ状態をリセット。
 *
 * 2. 開口部相対幾何計算:
 *    - use_tf=true (既定) により calculation_rate (20Hz) で TF (base_link -> bucket_bottom)
 * をルックアップ。
 *    - is_bottom_frame=true の場合は Z 座標に bucket_height (0.255m)
 * を加算して上部開口部高さを算出。
 *    - 水平距離 L = hypot(base_x, base_y)
 *    - 実効水平飛行距離: Delta x = L - launcher_offset_x
 *    - 実効鉛直高低差: Delta z = (z_open + aim_z_offset) - launcher_offset_z
 *
 * 3. 3次元放物弾道方程式による初速逆算:
 *    - 必要弾道初速 v0:
 *      v0 = sqrt( (g * Delta x^2) / (2 * cos^2(theta) * (Delta x * tan(theta) - Delta z)) )
 *      (theta = launch_angle_deg: 48.09 deg)
 *
 * 4. ベルトリリース速度変換 & 指数平滑化 (EMA) LPF:
 *    - v_target = velocity_quad * v0^2 + velocity_scale * v0 + velocity_offset
 *    - min_velocity 〜 max_velocity へクランプ。
 *    - v_filtered = (1 - alpha) * v_filtered + alpha * v_clamped (alpha = speed_filter_alpha: 0.25)
 *    - /robot/operation/belt_launcher_speed へ発行。
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>
#include <vector>

#include "gn10_navigation/navigation_types.hpp"
#include "lidar_perception_system/msg/bucket_bottom.hpp"
#include "lidar_perception_system/msg/moving_bucket_track.hpp"

namespace gn10_navigation {

class MovingBucketLauncherNode : public rclcpp::Node
{
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
    void onTimer();

    bool isPublishAllowed() const;
    void calculateAndPublishSpeed(
        const geometry_msgs::msg::Point& opening_pos_map, const std::string& frame_id
    );
    void calculateAndPublishSpeedFromBase(double base_x, double base_y, double base_z);

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Periodic timer for speed calculation
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_id_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<lidar_perception_system::msg::MovingBucketTrack>::SharedPtr
        bucket_opening_sub_;
    rclcpp::Subscription<lidar_perception_system::msg::BucketBottom>::SharedPtr bucket_bottom_sub_;

    // Publisher (Shared topic with target_manager_node; published ONLY when target_id ==
    // MovingBucket and nav_cmd == Move)
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr belt_launcher_speed_pub_;

    // Dynamic parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // State
    uint8_t current_target_id_{255};
    NavigationCommand nav_cmd_{NavigationCommand::Stanby};
    double filtered_speed_{0.0};
    bool filtered_speed_valid_{false};

    // Parameters (config/moving_bucket_launcher_params.yaml と整合)
    std::string base_frame_{"base_link"};
    std::string bucket_frame_{"bucket_bottom"};
    bool use_tf_{true};  // true: lookup bucket pose directly via TF (same source as path_follower)
    bool is_bottom_frame_{
        true};  // true: bucket_frame is bottom, add bucket_height to get opening center
    double calculation_rate_{20.0};    // Periodic speed update rate [Hz]
    double speed_filter_alpha_{0.25};  // Low-pass filter coefficient on launcher speed
    std::string opening_topic_{"/bucket_opening_tracker/target"};
    std::string bottom_topic_{"/bucket_opening_tracker/bottom_track"};
    bool use_bottom_topic_{false};  // false: use upper opening directly; true: derive opening from
                                    // bottom + bucket_height
    double bucket_height_{0.255};   // Bucket height from bottom to upper opening [m]
    double aim_z_offset_{0.0};      // Vertical offset relative to upper opening center [m]
    double launcher_offset_x_{0.179};  // Effective release X offset in base_link [m]
    double launcher_offset_z_{0.837};  // Effective release Z height in base_link [m]
    double launch_angle_deg_{48.09};   // Effective release elevation angle [deg]
    double velocity_quad_{0.005};      // Quadratic term: v_belt = quad * v0^2 + scale * v0 + offset
    double velocity_scale_{1.0};       // Linear scale from ballistic v0 to belt release speed
    double velocity_offset_{-1.5000};  // Constant offset for belt release speed [m/s]
    double min_velocity_{2.50};        // Min allowable launcher velocity [m/s]
    double max_velocity_{7.75};        // Max allowable launcher velocity [m/s]
    double min_horizontal_dist_{0.6};  // Min allowable horizontal distance [m]
    double max_horizontal_dist_{5.5};  // Max allowable horizontal distance [m]
    double max_tracking_age_sec_{0.5};  // Max allowable observation age [s]
};

}  // namespace gn10_navigation

#endif  // GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NODE_HPP_
