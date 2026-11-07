#pragma once

/**
 * @file path_follower_node.hpp
 * @brief NHKロボコン2026向け高精度・高速経路追従制御ノード
 *
 * 【自動移動制御アルゴリズムの概要】
 * 1. 経路前処理 (onPlannedPath):
 *    - 計画経路 (/planned_path) を微小一定間隔 (path_interpolation_step: 0.02m) でリサンプリング。
 *    - 各点での単位接線ベクトル (tangent_x, tangent_y) および幾何曲率 kappa を算出。
 *    - 幾何横加速度制限: v_lat = sqrt(max_lateral_acceleration / |kappa|)
 *    - 旋回角速度要求制限: 運動連鎖則 omega = (dtheta/ds) * v より、旋回が先行して遅れないよう
 *      v_turn = (max_angular_velocity * turn_yaw_rate_ratio) / |dtheta/ds|
 *    - 始点ロケットスタートランプ: 発進時のデッドロック防止と機敏な加速
 *    - S字速度プロファイル (躍度 jerk 制限付き加減速)
 *    - 前方走査 (最大加速度制限) および後方走査 (実効減速度比率 decel_effective_ratio
 * による安全減速)
 *
 * 2. 周期制御ループ (controlLoop: 50Hz):
 *    - TF (map -> base_link) による自己位置取得と鮮度監視 (max_pose_age: 0.3s)
 *    - MovingBucket 追従モード (Tracking):
 *      - TF (map -> bucket_bottom) の取得と一時ロスト猶予 (tracking_timeout: 1.0s)
 *      - 目標位置の指数平滑化 (EMA) LPF (tracking_pos_filter_alpha: 0.25)
 *      - 目標Yaw角計算と不感帯処理 (tracking_deadband: 0.015 rad) によるハンチング防止
 *      - 追従専用PID制御 (tracking_kp=3.5, kd=0.0) + 角速度・角加速度リミッタ
 *    - 経路追従モード (Moving):
 *      - 現在位置を経路へ直交射影 (project) し、横偏差が max_path_error を超えたら非常停止 (FAIL)
 *      - ゴール判定: ゴール近傍 (settle_zone_distance: 0.5m) かつ位置誤差 <= goal_tolerance
 * (0.02m), 角度誤差 <= goal_yaw_tolerance (0.0175 rad) を goal_settle_count (5周期) 連続維持で Goal
 * 確定
 *      - 遅延補償参照点 (ref_pt) の接線FF速度: v_ff = v_ref * tangent
 *      - 直交位置FB補正: 巡航中は直交偏差PD補正 (<= max_cruise_fb_speed)、
 *        ゴール近傍では目標点へのPID収束 (<= max_fb_speed) + 最小起動速度リフト (min_docking_speed)
 *      - 1次最大加速度リミッタ (max_acceleration, max_deceleration) による滑らかな指令値推移
 *      - 姿勢制御: 目標YawへのPID制御 (heading_kp, heading_kd) + 1次角加速度リミッタ
 */

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
        double kp             = 2.2,
        double ki             = 0.05,
        double kd             = 0.35,
        double i_limit        = 0.5,
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
            d_filtered_derivative_ =
                (1.0 - d_filter_alpha_) * d_filtered_derivative_ + d_filter_alpha_ * raw_derivative;
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
        double curvature{0.0};  // 外積符号付き曲率 [1/m] (+: 左旋回, -: 右旋回)
        double tangent_x{1.0};  // 進行方向単位接線ベクトル X
        double tangent_y{0.0};  // 進行方向単位接線ベクトル Y
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

    // パラメータ (config/path_follower_params.yaml と整合)
    double control_rate_{50.0};
    double max_speed_{3.0};
    double max_acceleration_{5.4};
    double max_deceleration_{8.0};
    double max_jerk_{60.0};
    double max_lateral_acceleration_{3.0};
    double curvature_window_{0.50};
    double max_angular_velocity_{3.0};
    double max_angular_acceleration_{24.0};
    double turn_yaw_rate_ratio_{0.75};
    double min_turn_speed_{0.3};
    double path_interpolation_step_{0.02};
    double decel_effective_ratio_{0.60};

    double position_kp_{8.0};
    double position_ki_{0.0};
    double position_kd_{0.0};
    double position_d_filter_alpha_{0.4};
    double max_fb_speed_{0.70};
    double max_cruise_fb_speed_{3.00};
    double min_docking_speed_{0.20};
    double settle_zone_distance_{0.50};

    double heading_kp_{5.0};
    double heading_ki_{0.0};
    double heading_kd_{1.0};
    double heading_integral_max_{0.3};
    double heading_d_filter_alpha_{0.6};
    double min_docking_yaw_rate_{0.03};

    double goal_tolerance_{0.02};
    double goal_yaw_tolerance_{0.0175};
    int goal_settle_count_{5};
    double max_path_error_{0.30};
    double max_initial_path_error_{0.45};
    double max_pose_age_{0.3};
    double control_comm_delay_{0.02};
    double max_delay_compensation_{0.15};

    AnglePidController heading_pid_;
    AnglePidController tracking_pid_;

    std::string map_frame_{"map"};
    std::string base_frame_{"base_link"};
    std::string bucket_frame_{"bucket_bottom"};
    double max_bucket_age_{0.5};

    // 移動バケツ追従制御パラメータ
    double tracking_kp_{3.5};
    double tracking_ki_{0.0};
    double tracking_kd_{0.0};
    double tracking_integral_max_{0.2};
    double tracking_d_filter_alpha_{0.2};
    double tracking_deadband_{0.015};         // Yaw誤差不感帯 [rad] (~0.86 deg)
    double tracking_pos_filter_alpha_{0.25};  // 目標バケツ位置低周波通過フィルタ係数
    double max_tracking_angular_velocity_{2.0};
    double max_tracking_angular_acceleration_{12.0};
    double tracking_timeout_{1.0};  // バケツTFロスト判定猶予時間 [s]

    // 移動バケツ追従内部状態
    double tracked_bucket_x_{0.0};
    double tracked_bucket_y_{0.0};
    bool has_tracked_bucket_{false};
    rclcpp::Time last_bucket_seen_time_{0, 0, RCL_ROS_TIME};

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

    // 位置制御積分器・微分器・前回指令速度 (1次加速度制限用)
    double pos_integral_x_{0.0};
    double pos_integral_y_{0.0};
    double prev_pos_error_x_{0.0};
    double prev_pos_error_y_{0.0};
    double pos_d_filtered_x_{0.0};
    double pos_d_filtered_y_{0.0};
    bool has_prev_pos_error_{false};
    bool was_in_settle_zone_{false};
    double command_vx_world_{0.0};
    double command_vy_world_{0.0};
    double command_yaw_rate_{0.0};
    std::chrono::steady_clock::time_point last_control_{std::chrono::steady_clock::now()};
};

}  // namespace gn10_navigation
