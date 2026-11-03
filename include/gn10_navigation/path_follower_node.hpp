#pragma once

#include <chrono>
#include <cmath>
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

class AnglePidController
{
public:
    AnglePidController(
        double kp = 2.2,
        double ki = 0.05,
        double kd = 0.35,
        double i_limit = 0.5,
        double d_filter_alpha = 0.4
    )
        : kp_(kp), ki_(ki), kd_(kd), i_limit_(i_limit), d_filter_alpha_(d_filter_alpha)
    {
    }

    void setGains(double kp, double ki, double kd, double i_limit)
    {
        kp_      = kp;
        ki_      = ki;
        kd_      = kd;
        i_limit_ = i_limit;
    }

    void setFilterAlpha(double alpha)
    {
        d_filter_alpha_ = std::clamp(alpha, 0.0, 1.0);
    }

    void reset()
    {
        integral_              = 0.0;
        prev_error_            = 0.0;
        has_prev_              = false;
        d_filtered_derivative_ = 0.0;
    }

    double update(double error, double dt, bool enable_integral = true)
    {
        if (dt <= 1e-4) {
            return 0.0;
        }

        const double p_term = kp_ * error;

        if (enable_integral) {
            integral_ += error * dt;
            if (i_limit_ > 0.0) {
                integral_ = std::clamp(integral_, -i_limit_, i_limit_);
            }
        } else {
            integral_ = 0.0;
        }
        const double i_term = ki_ * integral_;

        double d_term = 0.0;
        if (has_prev_) {
            double diff = error - prev_error_;
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            const double raw_derivative = diff / dt;
            // 1次ローパスフィルタで微分ノイズを低減し振動を抑制
            d_filtered_derivative_ = (1.0 - d_filter_alpha_) * d_filtered_derivative_ + d_filter_alpha_ * raw_derivative;
            d_term = kd_ * d_filtered_derivative_;
        }
        prev_error_ = error;
        has_prev_   = true;

        return p_term + i_term + d_term;
    }

private:
    double kp_{2.2};
    double ki_{0.05};
    double kd_{0.35};
    double i_limit_{0.5};
    double d_filter_alpha_{0.4};
    double integral_{0.0};
    double prev_error_{0.0};
    double d_filtered_derivative_{0.0};
    bool has_prev_{false};
};

class PathFollowerNode : public rclcpp::Node
{
public:
    explicit PathFollowerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    struct SpeedPoint {
        double x;
        double y;
        double yaw;
        double distance;
        double speed;
        double time;
        double curvature{0.0};   // 外積符号付き曲率 [1/m] (+: 左旋回, -: 右旋回)
        double tangent_x{1.0};   // 進行方向単位接線ベクトル X
        double tangent_y{0.0};   // 進行方向単位接線ベクトル Y
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
    double control_rate_{50.0};
    double max_speed_{2.0};
    double max_acceleration_{3.0};
    double max_deceleration_{3.0};
    double max_jerk_{50.0};
    double max_lateral_acceleration_{2.0};
    double curvature_window_{0.10};
    double max_angular_velocity_{2.5};
    double max_angular_acceleration_{5.0};
    double path_interpolation_step_{0.02};
    double decel_effective_ratio_{0.85};

    double position_kp_{4.0};
    double position_ki_{0.5};
    double max_fb_speed_{0.60};
    double min_docking_speed_{0.02};
    double settle_zone_distance_{0.10};

    double heading_kp_{3.0};
    double heading_ki_{0.2};
    double heading_kd_{0.1};
    double heading_integral_max_{0.3};
    double heading_d_filter_alpha_{0.4};
    double min_docking_yaw_rate_{0.03};

    double goal_tolerance_{0.005};
    double goal_yaw_tolerance_{0.0175};
    int goal_settle_count_{5};
    double max_path_error_{0.30};
    double max_initial_path_error_{0.45};
    double max_pose_age_{0.2};
    double control_comm_delay_{0.025};
    double max_delay_compensation_{0.15};

    AnglePidController heading_pid_;
    AnglePidController tracking_pid_;

    std::string map_frame_;
    std::string base_frame_;
    std::string bucket_frame_;
    double max_bucket_age_{0.5};

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
    int settle_count_{0};

    // 位置制御積分器・前回指令速度 (1次加速度制限用)
    double pos_integral_x_{0.0};
    double pos_integral_y_{0.0};
    double command_vx_world_{0.0};
    double command_vy_world_{0.0};
    double command_yaw_rate_{0.0};
    std::chrono::steady_clock::time_point last_control_{std::chrono::steady_clock::now()};
};

}  // namespace gn10_navigation
