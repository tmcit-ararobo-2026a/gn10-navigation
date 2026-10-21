#include "gn10_navigation/path_follower_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gn10_navigation {

static double normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

static double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions& options)
    : Node("path_follower_node", options)
{
    declareAndGetParameters();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    nav_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/navigation_command",
        10,
        std::bind(&PathFollowerNode::onNavigationCommand, this, std::placeholders::_1)
    );

    target_id_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/target_id",
        10,
        std::bind(&PathFollowerNode::onTargetId, this, std::placeholders::_1)
    );

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path",
        10,
        std::bind(&PathFollowerNode::onPlannedPath, this, std::placeholders::_1)
    );

    // Publishers
    status_pub_ =
        this->create_publisher<std_msgs::msg::UInt8>("/robot/operation/navigation_status", 10);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/operation/cmd_vel", 10);

    lookahead_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("/debug/lookahead_point", 10);

    // Dynamic parameters callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PathFollowerNode::onSetParameters, this, std::placeholders::_1)
    );

    // Control timer
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_      = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PathFollowerNode::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "PathFollowerNode initialized at %.1f Hz.", control_rate_);
}

void PathFollowerNode::declareAndGetParameters()
{
    this->declare_parameter("control_rate", 50.0);
    this->declare_parameter("max_speed", 1.5);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("max_acceleration", 1.0);
    this->declare_parameter("min_lookahead", 0.4);
    this->declare_parameter("goal_tolerance", 0.03);
    this->declare_parameter("heading_gain", 2.0);
    this->declare_parameter("creep_gain", 2.0);
    this->declare_parameter("min_creep_speed", 0.05);

    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("bucket_frame", "bucket_bottom");
    this->declare_parameter("max_bucket_age", 0.5);

    control_rate_         = this->get_parameter("control_rate").as_double();
    max_speed_            = this->get_parameter("max_speed").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    max_acceleration_     = this->get_parameter("max_acceleration").as_double();
    min_lookahead_        = this->get_parameter("min_lookahead").as_double();
    goal_tolerance_       = this->get_parameter("goal_tolerance").as_double();
    heading_gain_         = this->get_parameter("heading_gain").as_double();
    creep_gain_           = this->get_parameter("creep_gain").as_double();
    min_creep_speed_      = this->get_parameter("min_creep_speed").as_double();

    map_frame_      = this->get_parameter("map_frame").as_string();
    base_frame_     = this->get_parameter("base_frame").as_string();
    bucket_frame_   = this->get_parameter("bucket_frame").as_string();
    max_bucket_age_ = this->get_parameter("max_bucket_age").as_double();
}

rcl_interfaces::msg::SetParametersResult PathFollowerNode::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        const auto& name = param.get_name();
        if (name == "max_speed") {
            max_speed_ = param.as_double();
        } else if (name == "max_angular_velocity") {
            max_angular_velocity_ = param.as_double();
        } else if (name == "max_acceleration") {
            max_acceleration_ = param.as_double();
        } else if (name == "min_lookahead") {
            min_lookahead_ = param.as_double();
        } else if (name == "goal_tolerance") {
            goal_tolerance_ = param.as_double();
        } else if (name == "heading_gain") {
            heading_gain_ = param.as_double();
        } else if (name == "creep_gain") {
            creep_gain_ = param.as_double();
        } else if (name == "min_creep_speed") {
            min_creep_speed_ = param.as_double();
        }
    }
    return result;
}

void PathFollowerNode::onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    last_cmd_ = static_cast<NavigationCommand>(msg->data);

    if (last_cmd_ == NavigationCommand::Move) {
        if (current_target_id_ == TargetId::MovingBucket) {
            status_ = NavigationStatus::Tracking;
            RCLCPP_INFO(this->get_logger(), "Transition to TRACKING for MovingBucket.");
        } else {
            status_ = NavigationStatus::Moving;
            RCLCPP_INFO(this->get_logger(), "Transition to MOVING.");
        }
    } else if (last_cmd_ == NavigationCommand::Stanby || last_cmd_ == NavigationCommand::Sleep) {
        if (status_ == NavigationStatus::Moving || status_ == NavigationStatus::Tracking ||
            status_ == NavigationStatus::Goal || status_ == NavigationStatus::Fail) {
            status_ = NavigationStatus::Ready;
            RCLCPP_INFO(
                this->get_logger(), "Command Stanby/Sleep received -> Transition to READY."
            );
        }
    }
}

void PathFollowerNode::onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    current_target_id_ = static_cast<TargetId>(msg->data);
    if (status_ == NavigationStatus::Moving && current_target_id_ == TargetId::MovingBucket) {
        status_ = NavigationStatus::Tracking;
        RCLCPP_INFO(
            this->get_logger(), "Target changed to MovingBucket -> Transition to TRACKING."
        );
    }
}

void PathFollowerNode::onPlannedPath(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty planned path.");
        if (status_ == NavigationStatus::Moving) {
            status_ = NavigationStatus::Fail;
            RCLCPP_ERROR(this->get_logger(), "Path is empty while Moving -> Transition to FAIL.");
        }
        has_path_ = false;
        current_path_.clear();
        return;
    }

    current_path_ = msg->poses;
    goal_pose_    = current_path_.back();
    has_path_     = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Received new path with %zu poses. Goal=(%.3f, %.3f)",
        current_path_.size(),
        goal_pose_.pose.position.x,
        goal_pose_.pose.position.y
    );
}

void PathFollowerNode::controlLoop()
{
    // 1. ロボット自己位置の取得 (map -> base_link)
    geometry_msgs::msg::TransformStamped robot_tf;
    bool tf_ok = false;
    try {
        robot_tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        tf_ok    = true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Cannot get robot transform (%s -> %s): %s",
            map_frame_.c_str(),
            base_frame_.c_str(),
            ex.what()
        );
    }

    // 状態遷移: 自己位置推定成功で Standby -> Ready
    if (status_ == NavigationStatus::Stanby && tf_ok) {
        status_ = NavigationStatus::Ready;
        RCLCPP_INFO(this->get_logger(), "TF acquired -> Transition from STANDBY to READY.");
    }

    // 自己位置エラー時のフェイルセーフ
    if (!tf_ok) {
        if (status_ == NavigationStatus::Moving || status_ == NavigationStatus::Tracking) {
            status_ = NavigationStatus::Fail;
            RCLCPP_ERROR(
                this->get_logger(), "Lost TF while moving/tracking -> Transition to FAIL."
            );
        }
        publishCmdVel(0.0, 0.0, 0.0);
        publishStatus();
        return;
    }

    double robot_x   = robot_tf.transform.translation.x;
    double robot_y   = robot_tf.transform.translation.y;
    double robot_yaw = getYawFromQuaternion(robot_tf.transform.rotation);

    // 2. 状態ごとの制御分岐
    switch (status_) {
        case NavigationStatus::Stanby:
        case NavigationStatus::Ready:
        case NavigationStatus::Fail: {
            publishCmdVel(0.0, 0.0, 0.0);
            break;
        }

        case NavigationStatus::Goal: {
            publishCmdVel(0.0, 0.0, 0.0);
            if (has_path_) {
                double dx     = goal_pose_.pose.position.x - robot_x;
                double dy     = goal_pose_.pose.position.y - robot_y;
                double d_goal = std::hypot(dx, dy);
                // 目標地点の許容誤差範囲外へ移動した場合は Ready へ戻る
                if (d_goal > goal_tolerance_ * 2.0) {
                    status_ = NavigationStatus::Ready;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Robot moved away from goal (d=%.3f) -> Transition to READY.",
                        d_goal
                    );
                }
            }
            break;
        }

        case NavigationStatus::Tracking: {
            // Trackingモード: 並進移動を行わず (vx=0, vy=0)、TF (map -> bucket_bottom)
            // の方向へ向ける旋回P制御 (wz)
            geometry_msgs::msg::TransformStamped bucket_tf;
            bool bucket_ok = false;
            try {
                bucket_tf =
                    tf_buffer_->lookupTransform(map_frame_, bucket_frame_, tf2::TimePointZero);
                rclcpp::Time stamp(bucket_tf.header.stamp);
                if (max_bucket_age_ <= 0.0 || (this->now() - stamp).seconds() < max_bucket_age_) {
                    bucket_ok = true;
                } else {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "Bucket TF is stale (age=%.3f s).",
                        (this->now() - stamp).seconds()
                    );
                }
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Cannot get bucket TF (%s -> %s): %s",
                    map_frame_.c_str(),
                    bucket_frame_.c_str(),
                    ex.what()
                );
            }

            if (!bucket_ok) {
                status_ = NavigationStatus::Fail;
                RCLCPP_ERROR(
                    this->get_logger(), "Bucket TF missing or stale -> Transition to FAIL."
                );
                publishCmdVel(0.0, 0.0, 0.0);
                break;
            }

            double bucket_x   = bucket_tf.transform.translation.x;
            double bucket_y   = bucket_tf.transform.translation.y;
            double target_yaw = std::atan2(bucket_y - robot_y, bucket_x - robot_x);
            double diff_yaw   = normalizeAngle(target_yaw - robot_yaw);
            double wz         = heading_gain_ * diff_yaw;
            wz                = std::clamp(wz, -max_angular_velocity_, max_angular_velocity_);

            publishCmdVel(0.0, 0.0, wz);
            break;
        }

        case NavigationStatus::Moving: {
            if (!has_path_ || current_path_.empty()) {
                publishCmdVel(0.0, 0.0, 0.0);
                break;
            }

            double goal_x   = goal_pose_.pose.position.x;
            double goal_y   = goal_pose_.pose.position.y;
            double goal_yaw = getYawFromQuaternion(goal_pose_.pose.orientation);

            double d_goal = std::hypot(goal_x - robot_x, goal_y - robot_y);

            // ① 完全停止区間 (d_goal <= 0.03 m)
            if (d_goal <= goal_tolerance_) {
                status_ = NavigationStatus::Goal;
                publishCmdVel(0.0, 0.0, 0.0);
                RCLCPP_INFO(
                    this->get_logger(),
                    "Goal reached within tolerance (d=%.3f m) -> Transition to GOAL.",
                    d_goal
                );
                break;
            }

            double vx_world = 0.0;
            double vy_world = 0.0;
            double wz       = 0.0;

            // ② 終端位置偏差FB区間 (0.03 m < d_goal < 0.15 m)
            if (d_goal < 0.15) {
                // v_creep = max(0.05, d_goal * K_p)
                double v_creep = std::max(min_creep_speed_, d_goal * creep_gain_);
                v_creep        = std::min(v_creep, max_speed_);

                double dir_x = (goal_x - robot_x) / d_goal;
                double dir_y = (goal_y - robot_y) / d_goal;

                vx_world = v_creep * dir_x;
                vy_world = v_creep * dir_y;

                double diff_yaw = normalizeAngle(goal_yaw - robot_yaw);
                wz              = heading_gain_ * diff_yaw;
                wz              = std::clamp(wz, -max_angular_velocity_, max_angular_velocity_);

                publishLookaheadMarker(goal_x, goal_y);
            }
            // ③ 通常追従区間 (d_goal >= 0.15 m): Pure Pursuit
            else {
                // 1. パス上でロボットに最も近い点のインデックスを探す
                size_t closest_idx = 0;
                double min_dist_sq = std::numeric_limits<double>::infinity();
                for (size_t i = 0; i < current_path_.size(); ++i) {
                    double px   = current_path_[i].pose.position.x;
                    double py   = current_path_[i].pose.position.y;
                    double d_sq = (px - robot_x) * (px - robot_x) + (py - robot_y) * (py - robot_y);
                    if (d_sq < min_dist_sq) {
                        min_dist_sq = d_sq;
                        closest_idx = i;
                    }
                }

                // 2. 最寄り点から前方に向かって min_lookahead_ 先の注視点を探索
                size_t lookahead_idx = closest_idx;
                for (size_t i = closest_idx; i < current_path_.size(); ++i) {
                    double px     = current_path_[i].pose.position.x;
                    double py     = current_path_[i].pose.position.y;
                    double dist   = std::hypot(px - robot_x, py - robot_y);
                    lookahead_idx = i;
                    if (dist >= min_lookahead_) {
                        break;
                    }
                }

                double target_x = current_path_[lookahead_idx].pose.position.x;
                double target_y = current_path_[lookahead_idx].pose.position.y;
                publishLookaheadMarker(target_x, target_y);

                double dx = target_x - robot_x;
                double dy = target_y - robot_y;
                double L  = std::hypot(dx, dy);

                if (L > 1e-4) {
                    vx_world = max_speed_ * (dx / L);
                    vy_world = max_speed_ * (dy / L);
                }

                // 向き: パス接線方向へのP制御
                double path_yaw =
                    getYawFromQuaternion(current_path_[lookahead_idx].pose.orientation);
                double diff_yaw = normalizeAngle(path_yaw - robot_yaw);
                wz              = heading_gain_ * diff_yaw;
                wz              = std::clamp(wz, -2.0, 2.0);
            }

            // ワールド座標系 (map) 速度指示値を機体ローカル座標系 (base_link) に変換
            // vx_local =  vx_world * cos(theta) + vy_world * sin(theta)
            // vy_local = -vx_world * sin(theta) + vy_world * cos(theta)
            double cos_yaw         = std::cos(robot_yaw);
            double sin_yaw         = std::sin(robot_yaw);
            double target_vx_local = vx_world * cos_yaw + vy_world * sin_yaw;
            double target_vy_local = -vx_world * sin_yaw + vy_world * cos_yaw;

            // 加速度制限の適用
            double dt     = 1.0 / control_rate_;
            double max_dv = max_acceleration_ * dt;

            double diff_vx = target_vx_local - current_vx_local_;
            double diff_vy = target_vy_local - current_vy_local_;
            double diff_v  = std::hypot(diff_vx, diff_vy);

            if (diff_v > max_dv && diff_v > 1e-6) {
                target_vx_local = current_vx_local_ + (diff_vx / diff_v) * max_dv;
                target_vy_local = current_vy_local_ + (diff_vy / diff_v) * max_dv;
            }

            current_vx_local_ = target_vx_local;
            current_vy_local_ = target_vy_local;
            current_wz_       = wz;  // パスに沿って向きは変更しないので未使用

            publishCmdVel(current_vx_local_, current_vy_local_, 0.0);
            break;
        }
    }

    // 毎周期、ナビゲーション状態をパブリッシュ
    publishStatus();
}

void PathFollowerNode::publishCmdVel(double vx_local, double vy_local, double wz)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = vx_local;
    twist.linear.y  = vy_local;
    twist.linear.z  = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = wz;
    cmd_vel_pub_->publish(twist);
}

void PathFollowerNode::publishStatus()
{
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(status_);
    status_pub_->publish(msg);
}

void PathFollowerNode::publishLookaheadMarker(double x, double y)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id    = map_frame_;
    marker.header.stamp       = this->now();
    marker.ns                 = "lookahead";
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::SPHERE;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x    = x;
    marker.pose.position.y    = y;
    marker.pose.position.z    = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.15;
    marker.scale.y            = 0.15;
    marker.scale.z            = 0.15;
    marker.color.r            = 1.0f;
    marker.color.g            = 0.0f;
    marker.color.b            = 0.0f;
    marker.color.a            = 0.8f;
    lookahead_marker_pub_->publish(marker);
}

}  // namespace gn10_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
