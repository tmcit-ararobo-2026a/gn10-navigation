#include "gn10_navigation/path_follower_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace gn10_navigation {

namespace {

constexpr double kEpsilon = 1e-6;

void limitVector(double& x, double& y, double limit)
{
    const double magnitude = std::hypot(x, y);
    if (magnitude > limit && magnitude > kEpsilon) {
        x *= limit / magnitude;
        y *= limit / magnitude;
    }
}

bool finitePose(const geometry_msgs::msg::PoseStamped& pose)
{
    return std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y);
}

struct MotionSample {
    double distance;
    double speed;
};

std::vector<MotionSample> makeSCurve(
    double length,
    double max_speed,
    double max_acceleration,
    double max_deceleration,
    double max_jerk
)
{
    const auto ramp_time = [max_jerk](double speed, double acceleration) {
        const double jerk_time = std::min(acceleration / max_jerk, std::sqrt(speed / max_jerk));
        const double flat_time = std::max(0.0, speed / acceleration - jerk_time);
        return std::pair<double, double>{jerk_time, flat_time};
    };
    const auto ramp_distance = [&](double speed) {
        const auto [up_jerk, up_flat]     = ramp_time(speed, max_acceleration);
        const auto [down_jerk, down_flat] = ramp_time(speed, max_deceleration);
        return speed * (up_jerk + down_jerk + 0.5 * (up_flat + down_flat));
    };

    double low  = 0.0;
    double high = max_speed;
    for (int iteration = 0; iteration < 60; ++iteration) {
        const double middle = 0.5 * (low + high);
        if (ramp_distance(middle) <= length) {
            low = middle;
        } else {
            high = middle;
        }
    }

    const double peak_speed = low;
    if (peak_speed <= kEpsilon) {
        return {
            {   0.0, 0.0},
            {length, 0.0}
        };
    }
    const auto [up_jerk, up_flat]     = ramp_time(peak_speed, max_acceleration);
    const auto [down_jerk, down_flat] = ramp_time(peak_speed, max_deceleration);
    const double cruise_time = std::max(0.0, (length - ramp_distance(peak_speed)) / peak_speed);
    const std::vector<std::pair<double, double>> phases = {
        {    up_jerk,  max_jerk},
        {    up_flat,       0.0},
        {    up_jerk, -max_jerk},
        {cruise_time,       0.0},
        {  down_jerk, -max_jerk},
        {  down_flat,       0.0},
        {  down_jerk,  max_jerk}
    };

    std::vector<MotionSample> samples{
        {0.0, 0.0}
    };
    double distance     = 0.0;
    double speed        = 0.0;
    double acceleration = 0.0;
    for (const auto& [duration, jerk] : phases) {
        double elapsed = 0.0;
        while (elapsed + kEpsilon < duration) {
            const double step_s = std::min(0.01, duration - elapsed);
            distance += speed * step_s + 0.5 * acceleration * step_s * step_s +
                        jerk * step_s * step_s * step_s / 6.0;
            speed += acceleration * step_s + 0.5 * jerk * step_s * step_s;
            acceleration += jerk * step_s;
            elapsed += step_s;
            samples.push_back({distance, std::max(0.0, speed)});
        }
    }
    samples.back() = {length, 0.0};
    return samples;
}

double sCurveSpeed(const std::vector<MotionSample>& samples, double distance)
{
    const auto iterator = std::lower_bound(
        samples.begin(), samples.end(), distance, [](const MotionSample& point, double value) {
            return point.distance < value;
        }
    );
    if (iterator == samples.begin()) {
        return iterator->speed;
    }
    if (iterator == samples.end()) {
        return 0.0;
    }
    const auto& previous = *(iterator - 1);
    const double fraction =
        (distance - previous.distance) / std::max(iterator->distance - previous.distance, kEpsilon);
    return previous.speed + fraction * (iterator->speed - previous.speed);
}

double normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm_sq <= kEpsilon) {
        return 0.0;
    }
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

}  // namespace

PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions& options)
    : Node("path_follower_node", options)
{
    declareAndGetParameters();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

    status_pub_ =
        this->create_publisher<std_msgs::msg::UInt8>("/robot/operation/navigation_status", 10);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/operation/cmd_vel", 10);

    lookahead_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("/debug/reference_point", 10);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PathFollowerNode::onSetParameters, this, std::placeholders::_1)
    );

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
    this->declare_parameter("max_speed", 2.0);
    this->declare_parameter("max_acceleration", 3.0);
    this->declare_parameter("max_deceleration", 3.0);
    this->declare_parameter("max_jerk", 50.0);
    this->declare_parameter("max_lateral_acceleration", 2.0);
    this->declare_parameter("curvature_window", 0.10);
    this->declare_parameter("max_angular_velocity", 2.5);
    this->declare_parameter("max_angular_acceleration", 5.0);
    this->declare_parameter("turn_yaw_rate_ratio", 0.75);
    this->declare_parameter("min_turn_speed", 0.3);
    this->declare_parameter("path_interpolation_step", 0.02);
    this->declare_parameter("decel_effective_ratio", 0.85);

    this->declare_parameter("position_kp", 4.0);
    this->declare_parameter("position_ki", 0.5);
    this->declare_parameter("max_fb_speed", 0.60);
    this->declare_parameter("min_docking_speed", 0.02);
    this->declare_parameter("settle_zone_distance", 0.10);

    this->declare_parameter("heading_kp", 3.0);
    this->declare_parameter("heading_ki", 0.2);
    this->declare_parameter("heading_kd", 0.1);
    this->declare_parameter("heading_integral_max", 0.3);
    this->declare_parameter("heading_d_filter_alpha", 0.4);
    this->declare_parameter("min_docking_yaw_rate", 0.03);

    this->declare_parameter("goal_tolerance", 0.005);
    this->declare_parameter("goal_yaw_tolerance", 0.0175);
    this->declare_parameter("goal_settle_count", 5);
    this->declare_parameter("max_path_error", 0.30);
    this->declare_parameter("max_initial_path_error", 0.45);
    this->declare_parameter("max_pose_age", 0.2);
    this->declare_parameter("control_comm_delay", 0.025);
    this->declare_parameter("max_delay_compensation", 0.15);

    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("bucket_frame", "bucket_bottom");
    this->declare_parameter("max_bucket_age", 0.5);

    control_rate_             = this->get_parameter("control_rate").as_double();
    max_speed_                = this->get_parameter("max_speed").as_double();
    max_acceleration_         = this->get_parameter("max_acceleration").as_double();
    max_deceleration_         = this->get_parameter("max_deceleration").as_double();
    max_jerk_                 = this->get_parameter("max_jerk").as_double();
    max_lateral_acceleration_ = this->get_parameter("max_lateral_acceleration").as_double();
    curvature_window_         = this->get_parameter("curvature_window").as_double();
    max_angular_velocity_     = this->get_parameter("max_angular_velocity").as_double();
    max_angular_acceleration_ = this->get_parameter("max_angular_acceleration").as_double();
    turn_yaw_rate_ratio_      = this->get_parameter("turn_yaw_rate_ratio").as_double();
    min_turn_speed_           = this->get_parameter("min_turn_speed").as_double();
    path_interpolation_step_  = this->get_parameter("path_interpolation_step").as_double();
    decel_effective_ratio_    = this->get_parameter("decel_effective_ratio").as_double();

    position_kp_          = this->get_parameter("position_kp").as_double();
    position_ki_          = this->get_parameter("position_ki").as_double();
    max_fb_speed_         = this->get_parameter("max_fb_speed").as_double();
    min_docking_speed_    = this->get_parameter("min_docking_speed").as_double();
    settle_zone_distance_ = this->get_parameter("settle_zone_distance").as_double();

    heading_kp_             = this->get_parameter("heading_kp").as_double();
    heading_ki_             = this->get_parameter("heading_ki").as_double();
    heading_kd_             = this->get_parameter("heading_kd").as_double();
    heading_integral_max_   = this->get_parameter("heading_integral_max").as_double();
    heading_d_filter_alpha_ = this->get_parameter("heading_d_filter_alpha").as_double();
    min_docking_yaw_rate_   = this->get_parameter("min_docking_yaw_rate").as_double();

    goal_tolerance_         = this->get_parameter("goal_tolerance").as_double();
    goal_yaw_tolerance_     = this->get_parameter("goal_yaw_tolerance").as_double();
    goal_settle_count_      = static_cast<int>(this->get_parameter("goal_settle_count").as_int());
    max_path_error_         = this->get_parameter("max_path_error").as_double();
    max_initial_path_error_ = this->get_parameter("max_initial_path_error").as_double();
    max_pose_age_           = this->get_parameter("max_pose_age").as_double();
    control_comm_delay_     = this->get_parameter("control_comm_delay").as_double();
    max_delay_compensation_ = this->get_parameter("max_delay_compensation").as_double();

    heading_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
    heading_pid_.setFilterAlpha(heading_d_filter_alpha_);
    tracking_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
    tracking_pid_.setFilterAlpha(heading_d_filter_alpha_);

    map_frame_      = this->get_parameter("map_frame").as_string();
    base_frame_     = this->get_parameter("base_frame").as_string();
    bucket_frame_   = this->get_parameter("bucket_frame").as_string();
    max_bucket_age_ = this->get_parameter("max_bucket_age").as_double();

    if (!std::isfinite(control_rate_) || control_rate_ <= 0.0 || !std::isfinite(max_speed_) ||
        max_speed_ <= 0.0 || !std::isfinite(max_acceleration_) || max_acceleration_ <= 0.0 ||
        !std::isfinite(max_deceleration_) || max_deceleration_ <= 0.0 ||
        !std::isfinite(max_jerk_) || max_jerk_ <= 0.0 ||
        !std::isfinite(max_lateral_acceleration_) || max_lateral_acceleration_ <= 0.0 ||
        !std::isfinite(curvature_window_) || curvature_window_ <= 0.0 ||
        !std::isfinite(max_angular_velocity_) || max_angular_velocity_ <= 0.0 ||
        !std::isfinite(max_angular_acceleration_) || max_angular_acceleration_ <= 0.0 ||
        !std::isfinite(turn_yaw_rate_ratio_) || turn_yaw_rate_ratio_ <= 0.0 ||
        turn_yaw_rate_ratio_ > 1.0 || !std::isfinite(min_turn_speed_) || min_turn_speed_ <= 0.0 ||
        !std::isfinite(path_interpolation_step_) || path_interpolation_step_ <= 0.0 ||
        !std::isfinite(position_kp_) || position_kp_ < 0.0 ||
        !std::isfinite(position_ki_) || position_ki_ < 0.0 ||
        !std::isfinite(max_fb_speed_) || max_fb_speed_ <= 0.0 ||
        !std::isfinite(min_docking_speed_) || min_docking_speed_ < 0.0 ||
        !std::isfinite(settle_zone_distance_) || settle_zone_distance_ < 0.0 ||
        !std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0 ||
        !std::isfinite(goal_yaw_tolerance_) || goal_yaw_tolerance_ <= 0.0 ||
        goal_settle_count_ < 1 || !std::isfinite(max_path_error_) || max_path_error_ <= 0.0 ||
        !std::isfinite(max_initial_path_error_) || max_initial_path_error_ < max_path_error_ ||
        !std::isfinite(decel_effective_ratio_) || decel_effective_ratio_ <= 0.0 ||
        decel_effective_ratio_ > 1.0 || !std::isfinite(control_comm_delay_) ||
        control_comm_delay_ < 0.0 || !std::isfinite(max_delay_compensation_) ||
        max_delay_compensation_ < 0.0 || !std::isfinite(heading_d_filter_alpha_) ||
        heading_d_filter_alpha_ < 0.0 || heading_d_filter_alpha_ > 1.0 || map_frame_.empty() ||
        base_frame_.empty()) {
        throw std::invalid_argument("Invalid path follower configuration");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Parameters loaded: max_speed=%.2f, max_accel=%.2f, max_decel=%.2f, "
        "pos_kp=%.2f, pos_ki=%.2f, max_fb=%.2f, goal_tol=%.4f, goal_yaw_tol=%.4f, "
        "settle_zone=%.2f, settle_count=%d, heading_pid=[%.2f, %.2f, %.2f]",
        max_speed_,
        max_acceleration_,
        max_deceleration_,
        position_kp_,
        position_ki_,
        max_fb_speed_,
        goal_tolerance_,
        goal_yaw_tolerance_,
        settle_zone_distance_,
        goal_settle_count_,
        heading_kp_,
        heading_ki_,
        heading_kd_
    );
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
        } else if (name == "max_acceleration") {
            max_acceleration_ = param.as_double();
        } else if (name == "max_deceleration") {
            max_deceleration_ = param.as_double();
        } else if (name == "max_jerk") {
            max_jerk_ = param.as_double();
        } else if (name == "max_lateral_acceleration") {
            max_lateral_acceleration_ = param.as_double();
        } else if (name == "curvature_window") {
            curvature_window_ = std::max(0.01, param.as_double());
        } else if (name == "max_angular_velocity") {
            max_angular_velocity_ = param.as_double();
        } else if (name == "max_angular_acceleration") {
            max_angular_acceleration_ = param.as_double();
        } else if (name == "turn_yaw_rate_ratio") {
            turn_yaw_rate_ratio_ = std::clamp(param.as_double(), 0.05, 1.0);
        } else if (name == "min_turn_speed") {
            min_turn_speed_ = std::max(0.01, param.as_double());
        } else if (name == "path_interpolation_step") {
            path_interpolation_step_ = std::max(0.005, param.as_double());
        } else if (name == "position_kp") {
            position_kp_ = std::max(0.0, param.as_double());
        } else if (name == "position_ki") {
            position_ki_ = std::max(0.0, param.as_double());
        } else if (name == "max_fb_speed") {
            max_fb_speed_ = std::max(0.01, param.as_double());
        } else if (name == "min_docking_speed") {
            min_docking_speed_ = std::max(0.0, param.as_double());
        } else if (name == "settle_zone_distance") {
            settle_zone_distance_ = std::max(0.0, param.as_double());
        } else if (name == "goal_tolerance") {
            goal_tolerance_ = std::max(1e-4, param.as_double());
        } else if (name == "goal_yaw_tolerance") {
            goal_yaw_tolerance_ = std::max(1e-4, param.as_double());
        } else if (name == "goal_settle_count") {
            goal_settle_count_ = std::max(1, static_cast<int>(param.as_int()));
        } else if (name == "max_path_error") {
            max_path_error_ = std::max(0.01, param.as_double());
        } else if (name == "max_initial_path_error") {
            max_initial_path_error_ = std::max(max_path_error_, param.as_double());
        } else if (name == "max_pose_age") {
            max_pose_age_ = std::max(0.01, param.as_double());
        } else if (name == "heading_kp") {
            heading_kp_ = param.as_double();
            heading_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
            tracking_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
        } else if (name == "heading_ki") {
            heading_ki_ = param.as_double();
            heading_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
            tracking_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
        } else if (name == "heading_kd") {
            heading_kd_ = param.as_double();
            heading_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
            tracking_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
        } else if (name == "heading_integral_max") {
            heading_integral_max_ = std::max(0.0, param.as_double());
            heading_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
            tracking_pid_.setGains(heading_kp_, heading_ki_, heading_kd_, heading_integral_max_);
        } else if (name == "heading_d_filter_alpha") {
            heading_d_filter_alpha_ = std::clamp(param.as_double(), 0.0, 1.0);
            heading_pid_.setFilterAlpha(heading_d_filter_alpha_);
            tracking_pid_.setFilterAlpha(heading_d_filter_alpha_);
        } else if (name == "min_docking_yaw_rate") {
            min_docking_yaw_rate_ = std::max(0.0, param.as_double());
        } else if (name == "control_comm_delay") {
            control_comm_delay_ = std::max(0.0, param.as_double());
        } else if (name == "max_delay_compensation") {
            max_delay_compensation_ = std::max(0.0, param.as_double());
        } else if (name == "decel_effective_ratio") {
            decel_effective_ratio_ = std::clamp(param.as_double(), 0.1, 1.0);
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Parameter '%s' updated to: %s",
            name.c_str(),
            param.value_to_string().c_str()
        );
    }
    return result;
}

void PathFollowerNode::onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    last_cmd_ = static_cast<NavigationCommand>(msg->data);

    if (last_cmd_ == NavigationCommand::Move) {
        if (goal_latched_) {
            return;
        }
        if (current_target_id_ == TargetId::MovingBucket) {
            if (status_ != NavigationStatus::Tracking) {
                status_ = NavigationStatus::Tracking;
                RCLCPP_INFO(this->get_logger(), "Transition to TRACKING for MovingBucket.");
            }
        } else {
            if (status_ != NavigationStatus::Moving) {
                status_ = NavigationStatus::Moving;
                RCLCPP_INFO(this->get_logger(), "Transition to MOVING.");
            }
        }
    } else if (last_cmd_ == NavigationCommand::Stanby || last_cmd_ == NavigationCommand::Sleep) {
        goal_latched_ = false;
        settle_count_ = 0;
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
    const auto target_id = static_cast<TargetId>(msg->data);
    if (target_id != current_target_id_) {
        goal_latched_ = false;
        settle_count_ = 0;
        if (status_ == NavigationStatus::Goal) {
            status_ = NavigationStatus::Ready;
        }
    }
    current_target_id_ = target_id;
    if (status_ == NavigationStatus::Moving && current_target_id_ == TargetId::MovingBucket) {
        status_ = NavigationStatus::Tracking;
        RCLCPP_INFO(
            this->get_logger(), "Target changed to MovingBucket -> Transition to TRACKING."
        );
    }
}

void PathFollowerNode::onPlannedPath(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    if ((status_ == NavigationStatus::Moving || status_ == NavigationStatus::Goal) && has_path_ &&
        !msg->poses.empty()) {
        const auto& incoming_goal    = msg->poses.back().pose.position;
        const double goal_difference = std::hypot(
            incoming_goal.x - goal_pose_.pose.position.x,
            incoming_goal.y - goal_pose_.pose.position.y
        );
        const double yaw_difference = std::abs(normalizeAngle(
            getYawFromQuaternion(msg->poses.back().pose.orientation) -
            getYawFromQuaternion(goal_pose_.pose.orientation)
        ));
        if (msg->header.frame_id == map_frame_ && goal_difference <= kEpsilon &&
            yaw_difference <= kEpsilon) {
            return;
        }
    }

    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty planned path.");
        if (status_ == NavigationStatus::Moving) {
            status_ = NavigationStatus::Fail;
            RCLCPP_ERROR(this->get_logger(), "Path is empty while Moving -> Transition to FAIL.");
        }
        stop();
        has_path_ = false;
        path_.clear();
        source_path_.clear();
        progress_ = 0.0;
        return;
    }

    if (msg->header.frame_id != map_frame_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Rejected path in frame '%s'; expected '%s'",
            msg->header.frame_id.c_str(),
            map_frame_.c_str()
        );
        has_path_ = false;
        source_path_.clear();
        return;
    }

    if (status_ != NavigationStatus::Moving) {
        stop();
    } else {
        pos_integral_x_ = 0.0;
        pos_integral_y_ = 0.0;
        heading_pid_.reset();
    }
    path_.clear();
    source_path_  = msg->poses;
    progress_     = 0.0;
    goal_latched_ = false;
    settle_count_ = 0;

    for (const auto& pose : msg->poses) {
        if (!finitePose(pose) ||
            (!pose.header.frame_id.empty() && pose.header.frame_id != map_frame_)) {
            path_.clear();
            source_path_.clear();
            has_path_ = false;
            RCLCPP_WARN(this->get_logger(), "Rejected path with invalid point or frame.");
            return;
        }
        const double pose_yaw = getYawFromQuaternion(pose.pose.orientation);
        if (path_.empty()) {
            path_.push_back(
                {pose.pose.position.x,
                 pose.pose.position.y,
                 pose_yaw,
                 0.0,
                 max_speed_,
                 0.0,
                 0.0,
                 1.0,
                 0.0}
            );
            continue;
        }
        const auto start = path_.back();
        const double step_distance =
            std::hypot(pose.pose.position.x - start.x, pose.pose.position.y - start.y);
        if (step_distance <= kEpsilon) {
            path_.back().yaw = pose_yaw;
            continue;
        }
        const int steps = std::max(
            1, static_cast<int>(std::ceil(step_distance / path_interpolation_step_ - kEpsilon))
        );
        const double delta_yaw = normalizeAngle(pose_yaw - start.yaw);
        for (int step = 1; step <= steps; ++step) {
            const double fraction = static_cast<double>(step) / steps;
            path_.push_back(
                {start.x + fraction * (pose.pose.position.x - start.x),
                 start.y + fraction * (pose.pose.position.y - start.y),
                 normalizeAngle(start.yaw + fraction * delta_yaw),
                 start.distance + fraction * step_distance,
                 max_speed_,
                 0.0,
                 0.0,
                 1.0,
                 0.0}
            );
        }
    }

    if (path_.size() < 2) {
        path_.clear();
        source_path_.clear();
        has_path_ = false;
        RCLCPP_WARN(this->get_logger(), "Path needs at least two distinct points.");
        return;
    }

    for (std::size_t index = 1; index + 1 < path_.size(); ++index) {
        const auto previous      = sample(path_[index].distance - curvature_window_);
        const auto& current      = path_[index];
        const auto next          = sample(path_[index].distance + curvature_window_);
        const double ab          = std::hypot(current.x - previous.x, current.y - previous.y);
        const double bc          = std::hypot(next.x - current.x, next.y - current.y);
        const double ac          = std::hypot(next.x - previous.x, next.y - previous.y);
        const double cross       = (current.x - previous.x) * (next.y - current.y) -
                                   (current.y - previous.y) * (next.x - current.x);
        const double denominator = ab * bc * ac;

        // 局所3点での曲率も評価し、急カーブ頂点での曲率過小評価を防止する
        const auto& prev_local = path_[index - 1];
        const auto& next_local = path_[index + 1];
        const double ab_loc    = std::hypot(current.x - prev_local.x, current.y - prev_local.y);
        const double bc_loc    = std::hypot(next_local.x - current.x, next_local.y - current.y);
        const double ac_loc    = std::hypot(next_local.x - prev_local.x, next_local.y - prev_local.y);
        const double cross_loc = (current.x - prev_local.x) * (next_local.y - current.y) -
                                 (current.y - prev_local.y) * (next_local.x - current.x);
        const double denom_loc = ab_loc * bc_loc * ac_loc;
        const double local_curv =
            (denom_loc > kEpsilon) ? (2.0 * cross_loc / denom_loc) : 0.0;

        if (denominator > kEpsilon) {
            const double signed_curvature = 2.0 * cross / denominator;
            path_[index].curvature        = signed_curvature;
            const double curvature_mag =
                std::max(std::abs(signed_curvature), std::abs(local_curv));
            path_[index].speed = std::min(
                max_speed_, std::sqrt(max_lateral_acceleration_ / std::max(curvature_mag, kEpsilon))
            );
        }

        double tx     = next.x - previous.x;
        double ty     = next.y - previous.y;
        double t_norm = std::hypot(tx, ty);
        if (t_norm > kEpsilon) {
            path_[index].tangent_x = tx / t_norm;
            path_[index].tangent_y = ty / t_norm;
        } else {
            path_[index].tangent_x = std::cos(current.yaw);
            path_[index].tangent_y = std::sin(current.yaw);
        }

        // 旋回角速度要求（|dθ/ds|）に応じた並進速度制限:
        // 運動連鎖則: ω = dθ/dt = (dθ/ds) * (ds/dt) = yaw_rate_spatial * v
        // 最大許容角速度 max_angular_velocity_ に対し、PIDフィードバック補正用のヘッドルームを残すため、
        // 計画時の許容角速度上限を (max_angular_velocity_ * turn_yaw_rate_ratio_) とする。
        // これにより、急旋回区間で高速直進して旋回が追いつかなくなる現象を防止する。
        const double delta_dist = std::max(path_[index].distance - path_[index - 1].distance, kEpsilon);
        const double delta_yaw  = std::abs(normalizeAngle(path_[index].yaw - path_[index - 1].yaw));
        if (delta_yaw > 1e-4) {
            const double yaw_rate_spatial      = delta_yaw / delta_dist;
            const double allowed_plan_yaw_rate = max_angular_velocity_ * turn_yaw_rate_ratio_;
            const double turn_speed_limit      = allowed_plan_yaw_rate / yaw_rate_spatial;
            path_[index].speed = std::min(path_[index].speed, std::max(min_turn_speed_, turn_speed_limit));
        }
    }

    // 始点・終点の接線ベクトル設定（2点のみの場合も正しく単位ベクトルを計算）
    if (path_.size() == 2) {
        const double dx     = path_[1].x - path_[0].x;
        const double dy     = path_[1].y - path_[0].y;
        const double t_norm = std::hypot(dx, dy);
        if (t_norm > kEpsilon) {
            path_[0].tangent_x = path_[1].tangent_x = dx / t_norm;
            path_[0].tangent_y = path_[1].tangent_y = dy / t_norm;
        }
    } else {
        path_.front().tangent_x = path_[1].tangent_x;
        path_.front().tangent_y = path_[1].tangent_y;
        path_.back().tangent_x  = path_[path_.size() - 2].tangent_x;
        path_.back().tangent_y  = path_[path_.size() - 2].tangent_y;
    }

    // 発進時（distance == 0.0）の特異点デッドロック防止:
    // 停止からの加速は制御ループの1次加速度リミッタ（max_acceleration_ * dt）が保証するため、
    // 空間速度プロファイルの始点には1ステップ分の最低発進速度（または現在速度）を与える。
    const double current_speed = std::hypot(command_vx_world_, command_vy_world_);
    const double launch_speed  = std::min(
        max_speed_,
        std::max(current_speed, std::sqrt(2.0 * max_acceleration_ * path_interpolation_step_))
    );

    const double total_distance = path_.back().distance;
    const auto speed_profile    = makeSCurve(
        total_distance, max_speed_, max_acceleration_, max_deceleration_, max_jerk_
    );
    for (auto& point : path_) {
        double s_speed = sCurveSpeed(speed_profile, point.distance);
        if (point.distance < 0.5 * total_distance) {
            s_speed = std::max(s_speed, launch_speed);
        }
        // ゴール近傍 (settle_zone_distance_ 以内) では位置P制御 (position_kp_ * remaining) と
        // 連続的に接続するよう目標速度上限を抑え、後方走査で手前から滑らかに減速させる
        const double remaining = total_distance - point.distance;
        if (remaining <= settle_zone_distance_) {
            s_speed = std::min(s_speed, std::min(max_fb_speed_, position_kp_ * remaining));
        }
        point.speed = std::min(point.speed, s_speed);
    }
    path_.front().speed = std::min(path_.front().speed, launch_speed);
    path_.back().speed  = 0.0;

    // 前方走査（加速制限）
    for (std::size_t index = 1; index < path_.size(); ++index) {
        const double distance = path_[index].distance - path_[index - 1].distance;
        path_[index].speed    = std::min(
            path_[index].speed,
            std::sqrt(
                path_[index - 1].speed * path_[index - 1].speed + 2.0 * max_acceleration_ * distance
            )
        );
    }

    // 後方走査（コーナー進入およびゴール停止に向けた減速制限）
    const double effective_deceleration = max_deceleration_ * decel_effective_ratio_;
    for (std::size_t index = path_.size() - 1; index > 0; --index) {
        const double distance  = path_[index].distance - path_[index - 1].distance;
        path_[index - 1].speed = std::min(
            path_[index - 1].speed,
            std::sqrt(
                path_[index].speed * path_[index].speed + 2.0 * effective_deceleration * distance
            )
        );
    }

    goal_pose_ = msg->poses.back();
    has_path_  = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Received new path with %zu interpolated points (total=%.3f m). Goal=(%.3f, %.3f, yaw=%.3f)",
        path_.size(),
        total_distance,
        goal_pose_.pose.position.x,
        goal_pose_.pose.position.y,
        getYawFromQuaternion(goal_pose_.pose.orientation)
    );
}

void PathFollowerNode::stop()
{
    command_vx_world_ = 0.0;
    command_vy_world_ = 0.0;
    command_yaw_rate_ = 0.0;
    pos_integral_x_   = 0.0;
    pos_integral_y_   = 0.0;
    heading_pid_.reset();
    tracking_pid_.reset();
    publishCmdVel(0.0, 0.0, 0.0);
}

PathFollowerNode::Projection PathFollowerNode::project(double x, double y) const
{
    Projection best{progress_, std::numeric_limits<double>::infinity()};
    for (std::size_t index = 1; index < path_.size(); ++index) {
        const auto& previous = path_[index - 1];
        const auto& current  = path_[index];
        if (current.distance + 0.1 < progress_) {
            continue;
        }
        const double dx          = current.x - previous.x;
        const double dy          = current.y - previous.y;
        const double denominator = dx * dx + dy * dy;
        if (denominator <= kEpsilon) {
            continue;
        }
        const double fraction =
            std::clamp(((x - previous.x) * dx + (y - previous.y) * dy) / denominator, 0.0, 1.0);
        const double error =
            std::hypot(x - previous.x - fraction * dx, y - previous.y - fraction * dy);
        if (error < best.error) {
            best = {previous.distance + fraction * (current.distance - previous.distance), error};
        }
    }
    return best;
}

PathFollowerNode::SpeedPoint PathFollowerNode::sample(double distance) const
{
    distance            = std::clamp(distance, 0.0, path_.back().distance);
    const auto iterator = std::lower_bound(
        path_.begin(), path_.end(), distance, [](const SpeedPoint& point, double value) {
            return point.distance < value;
        }
    );
    if (iterator == path_.begin()) {
        return *iterator;
    }
    if (iterator == path_.end()) {
        return path_.back();
    }
    const auto& previous = *(iterator - 1);
    const double fraction =
        (distance - previous.distance) / std::max(iterator->distance - previous.distance, kEpsilon);
    const double delta_yaw = normalizeAngle(iterator->yaw - previous.yaw);
    double cur_tx = previous.tangent_x + fraction * (iterator->tangent_x - previous.tangent_x);
    double cur_ty = previous.tangent_y + fraction * (iterator->tangent_y - previous.tangent_y);
    const double t_norm = std::hypot(cur_tx, cur_ty);
    if (t_norm > kEpsilon) {
        cur_tx /= t_norm;
        cur_ty /= t_norm;
    }
    return {
        previous.x + fraction * (iterator->x - previous.x),
        previous.y + fraction * (iterator->y - previous.y),
        normalizeAngle(previous.yaw + fraction * delta_yaw),
        distance,
        previous.speed + fraction * (iterator->speed - previous.speed),
        0.0,
        previous.curvature + fraction * (iterator->curvature - previous.curvature),
        cur_tx,
        cur_ty
    };
}

void PathFollowerNode::controlLoop()
{
    const auto current_time = std::chrono::steady_clock::now();
    const double delta_time =
        std::clamp(std::chrono::duration<double>(current_time - last_control_).count(), 0.001, 0.1);
    last_control_ = current_time;

    geometry_msgs::msg::TransformStamped robot_tf;
    try {
        robot_tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& exception) {
        if (status_ == NavigationStatus::Moving || status_ == NavigationStatus::Tracking) {
            status_ = NavigationStatus::Fail;
        }
        stop();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Cannot get robot transform (%s -> %s): %s",
            map_frame_.c_str(),
            base_frame_.c_str(),
            exception.what()
        );
        publishStatus();
        return;
    }

    const double pose_age  = (this->now() - rclcpp::Time(robot_tf.header.stamp)).seconds();
    const double robot_x   = robot_tf.transform.translation.x;
    const double robot_y   = robot_tf.transform.translation.y;
    const double robot_yaw = getYawFromQuaternion(robot_tf.transform.rotation);

    if (!std::isfinite(pose_age) || pose_age > max_pose_age_ || pose_age < -0.1 ||
        !std::isfinite(robot_x) || !std::isfinite(robot_y) || !std::isfinite(robot_yaw)) {
        if (status_ == NavigationStatus::Moving || status_ == NavigationStatus::Tracking) {
            status_ = NavigationStatus::Fail;
        }
        stop();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Robot pose is stale or invalid (age=%.3f s, max=%.3f s).",
            pose_age,
            max_pose_age_
        );
        publishStatus();
        return;
    }

    if (status_ == NavigationStatus::Stanby) {
        status_ = NavigationStatus::Ready;
    }

    switch (status_) {
        case NavigationStatus::Stanby:
        case NavigationStatus::Ready:
        case NavigationStatus::Fail:
        case NavigationStatus::Goal:
            stop();
            break;

        case NavigationStatus::Tracking: {
            geometry_msgs::msg::TransformStamped bucket_tf;
            try {
                bucket_tf =
                    tf_buffer_->lookupTransform(map_frame_, bucket_frame_, tf2::TimePointZero);
                const double age = (this->now() - rclcpp::Time(bucket_tf.header.stamp)).seconds();
                if (max_bucket_age_ > 0.0 && (age > max_bucket_age_ || age < -0.1)) {
                    throw tf2::TransformException("bucket transform is stale");
                }
            } catch (const tf2::TransformException& exception) {
                status_ = NavigationStatus::Fail;
                stop();
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Cannot get valid bucket transform: %s",
                    exception.what()
                );
                break;
            }
            const double target_yaw = std::atan2(
                bucket_tf.transform.translation.y - robot_y,
                bucket_tf.transform.translation.x - robot_x
            );
            const double yaw_error = normalizeAngle(target_yaw - robot_yaw);
            const double pid_rate  = tracking_pid_.update(yaw_error, delta_time, true);
            const double target_rate =
                std::clamp(pid_rate, -max_angular_velocity_, max_angular_velocity_);
            command_yaw_rate_ += std::clamp(
                target_rate - command_yaw_rate_,
                -max_angular_acceleration_ * delta_time,
                max_angular_acceleration_ * delta_time
            );
            publishCmdVel(0.0, 0.0, command_yaw_rate_);
            break;
        }

        case NavigationStatus::Moving: {
            if (!has_path_ || path_.size() < 2) {
                stop();
                break;
            }

            // 1. 経路への直交射影と逸脱チェック
            const auto nearest = project(robot_x, robot_y);
            const double path_error_limit =
                (progress_ <= kEpsilon) ? max_initial_path_error_ : max_path_error_;
            if (nearest.error > path_error_limit) {
                status_ = NavigationStatus::Fail;
                stop();
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Path deviation %.3f m exceeds limit %.3f m -> FAIL.",
                    nearest.error,
                    path_error_limit
                );
                break;
            }
            progress_ = std::max(progress_, nearest.distance);

            // 2. ゴール誤差の評価と到達判定 (Settle Counter)
            const double dx_goal        = goal_pose_.pose.position.x - robot_x;
            const double dy_goal        = goal_pose_.pose.position.y - robot_y;
            const double dist_goal      = std::hypot(dx_goal, dy_goal);
            const double goal_yaw       = getYawFromQuaternion(goal_pose_.pose.orientation);
            const double goal_yaw_error = normalizeAngle(goal_yaw - robot_yaw);

            const double remaining_path = path_.back().distance - progress_;
            const bool in_settle_zone   = (remaining_path <= settle_zone_distance_);

            if (in_settle_zone && dist_goal <= goal_tolerance_ &&
                std::abs(goal_yaw_error) <= goal_yaw_tolerance_) {
                settle_count_++;
                if (settle_count_ >= goal_settle_count_) {
                    goal_latched_ = true;
                    status_       = NavigationStatus::Goal;
                    path_.clear();
                    has_path_     = false;
                    progress_     = 0.0;
                    settle_count_ = 0;
                    stop();
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Goal reached and settled (pos_err=%.4f m, yaw_err=%.4f rad).",
                        dist_goal,
                        goal_yaw_error
                    );
                    break;
                }
            } else {
                settle_count_ = 0;
            }

            // 3. 遅延補償を加味した参照点 (ref_pt: FF速度・姿勢用) と直交射影点 (proj_pt: 位置FB用)
            const double total_delay =
                std::clamp(pose_age + control_comm_delay_, 0.0, max_delay_compensation_);
            const double current_speed = std::hypot(command_vx_world_, command_vy_world_);
            const double ref_dist =
                std::min(path_.back().distance, progress_ + current_speed * total_delay);
            const auto ref_pt  = sample(ref_dist);
            const auto proj_pt = sample(progress_);

            // 4. 接線フィードフォワード速度 + 位置フィードバック速度の合成
            // 巡航中 (remaining_path > settle_zone_distance_) は:
            //   v_ff = ref_pt.speed * tangent (遅延補償付き接線速度)
            //   v_fb = position_kp_ * (proj_pt - robot_pos) (純粋な直交横ズレ補正、先読み二重加算なし)
            // ゴール近傍 (in_settle_zone) では:
            //   前処理により境界 remaining == settle_zone_distance_ で ref_pt.speed == position_kp_ * remaining
            //   となるよう連続接続されているため、ゴール座標 (goal_pose_) への直接PI制御へ滑らかに一致する。
            double vx_desired = 0.0;
            double vy_desired = 0.0;

            if (in_settle_zone) {
                pos_integral_x_ = std::clamp(pos_integral_x_ + dx_goal * delta_time, -0.05, 0.05);
                pos_integral_y_ = std::clamp(pos_integral_y_ + dy_goal * delta_time, -0.05, 0.05);

                if (dist_goal > goal_tolerance_) {
                    double vx_fb = position_kp_ * dx_goal + position_ki_ * pos_integral_x_;
                    double vy_fb = position_kp_ * dy_goal + position_ki_ * pos_integral_y_;
                    limitVector(vx_fb, vy_fb, max_fb_speed_);
                    vx_desired = vx_fb;
                    vy_desired = vy_fb;

                    // ゴール近傍での静止摩擦補償（最小起動速度リフト）
                    const double speed_desired = std::hypot(vx_desired, vy_desired);
                    if (speed_desired < min_docking_speed_) {
                        if (speed_desired > kEpsilon) {
                            vx_desired *= min_docking_speed_ / speed_desired;
                            vy_desired *= min_docking_speed_ / speed_desired;
                        } else {
                            vx_desired = min_docking_speed_ * (dx_goal / dist_goal);
                            vy_desired = min_docking_speed_ * (dy_goal / dist_goal);
                        }
                    }
                }
            } else {
                pos_integral_x_ = 0.0;
                pos_integral_y_ = 0.0;

                const double vx_ff = ref_pt.speed * ref_pt.tangent_x;
                const double vy_ff = ref_pt.speed * ref_pt.tangent_y;

                const double ex = proj_pt.x - robot_x;
                const double ey = proj_pt.y - robot_y;
                double vx_fb    = position_kp_ * ex;
                double vy_fb    = position_kp_ * ey;
                limitVector(vx_fb, vy_fb, max_fb_speed_);

                vx_desired = vx_ff + vx_fb;
                vy_desired = vy_ff + vy_fb;
            }
            limitVector(vx_desired, vy_desired, max_speed_);

            // 5. 1次の最大加速度リミッタ（機体保護・スリップ防止）
            double dv_x              = vx_desired - command_vx_world_;
            double dv_y              = vy_desired - command_vy_world_;
            const double accel_limit = (std::hypot(vx_desired, vy_desired) < current_speed)
                                           ? max_deceleration_
                                           : max_acceleration_;
            limitVector(dv_x, dv_y, accel_limit * delta_time);
            command_vx_world_ += dv_x;
            command_vy_world_ += dv_y;

            // 6. 姿勢（Yaw角）PID制御 + 最小角速度リフト + 1次角加速度リミッタ
            const double target_yaw = in_settle_zone ? goal_yaw : ref_pt.yaw;
            const double yaw_error  = normalizeAngle(target_yaw - robot_yaw);
            double target_yaw_rate  = heading_pid_.update(yaw_error, delta_time, in_settle_zone);

            if (in_settle_zone && std::abs(yaw_error) > goal_yaw_tolerance_) {
                if (std::abs(target_yaw_rate) < min_docking_yaw_rate_) {
                    target_yaw_rate = std::copysign(min_docking_yaw_rate_, yaw_error);
                }
            }
            target_yaw_rate =
                std::clamp(target_yaw_rate, -max_angular_velocity_, max_angular_velocity_);
            command_yaw_rate_ += std::clamp(
                target_yaw_rate - command_yaw_rate_,
                -max_angular_acceleration_ * delta_time,
                max_angular_acceleration_ * delta_time
            );

            // 7. 旋回ドリフト遅延補償と車体ローカル座標系への変換
            const double predicted_yaw =
                normalizeAngle(robot_yaw + command_yaw_rate_ * total_delay);
            const double cos_yaw = std::cos(predicted_yaw);
            const double sin_yaw = std::sin(predicted_yaw);

            publishCmdVel(
                cos_yaw * command_vx_world_ + sin_yaw * command_vy_world_,
                -sin_yaw * command_vx_world_ + cos_yaw * command_vy_world_,
                command_yaw_rate_
            );
            publishLookaheadMarker(ref_pt.x, ref_pt.y);
            break;
        }
    }
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
    marker.ns                 = "reference_point";
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::SPHERE;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x    = x;
    marker.pose.position.y    = y;
    marker.pose.position.z    = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.10;
    marker.scale.y            = 0.10;
    marker.scale.z            = 0.10;
    marker.color.r            = 0.0f;
    marker.color.g            = 1.0f;
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
