#include "gn10_navigation/path_follower_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gn10_navigation {

namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kPathStep = 0.05;

void limitVector(double& x, double& y, double limit)
{
    const double magnitude = std::hypot(x, y);
    if (magnitude > limit) {
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
    double length, double max_speed, double max_acceleration,
    double max_deceleration, double max_jerk)
{
    const auto ramp_time = [max_jerk](double speed, double acceleration) {
        const double jerk_time = std::min(acceleration / max_jerk,
                                          std::sqrt(speed / max_jerk));
        const double flat_time = std::max(0.0, speed / acceleration - jerk_time);
        return std::pair<double, double>{jerk_time, flat_time};
    };
    const auto ramp_distance = [&](double speed) {
        const auto [up_jerk, up_flat] = ramp_time(speed, max_acceleration);
        const auto [down_jerk, down_flat] = ramp_time(speed, max_deceleration);
        return speed * (up_jerk + down_jerk + 0.5 * (up_flat + down_flat));
    };

    double low = 0.0;
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
        return {{0.0, 0.0}, {length, 0.0}};
    }
    const auto [up_jerk, up_flat] = ramp_time(peak_speed, max_acceleration);
    const auto [down_jerk, down_flat] = ramp_time(peak_speed, max_deceleration);
    const double cruise_time =
        std::max(0.0, (length - ramp_distance(peak_speed)) / peak_speed);
    const std::vector<std::pair<double, double>> phases = {
        {up_jerk, max_jerk}, {up_flat, 0.0}, {up_jerk, -max_jerk},
        {cruise_time, 0.0},
        {down_jerk, -max_jerk}, {down_flat, 0.0}, {down_jerk, max_jerk}};

    std::vector<MotionSample> samples{{0.0, 0.0}};
    double distance = 0.0;
    double speed = 0.0;
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
        samples.begin(), samples.end(), distance,
        [](const MotionSample& point, double value) { return point.distance < value; });
    if (iterator == samples.begin()) {
        return iterator->speed;
    }
    if (iterator == samples.end()) {
        return 0.0;
    }
    const auto& previous = *(iterator - 1);
    const double fraction = (distance - previous.distance) /
                            std::max(iterator->distance - previous.distance, kEpsilon);
    return previous.speed + fraction * (iterator->speed - previous.speed);
}

}  // namespace

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
    this->declare_parameter("max_acceleration", 1.0);
    this->declare_parameter("max_deceleration", 2.5);
    this->declare_parameter("max_jerk", 12.0);
    this->declare_parameter("max_lateral_acceleration", 0.6);
    this->declare_parameter("curvature_window", 0.10);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("max_angular_acceleration", 2.0);
    this->declare_parameter("min_lookahead", 0.15);
    this->declare_parameter("max_lookahead", 0.40);
    this->declare_parameter("lookahead_gain", 0.10);
    this->declare_parameter("goal_tolerance", 0.06);
    this->declare_parameter("goal_approach_speed", 0.30);
    this->declare_parameter("max_path_error", 0.30);
    this->declare_parameter("max_initial_path_error", 0.45);
    this->declare_parameter("max_pose_age", 0.5);
    this->declare_parameter("heading_gain", 2.0);

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
    curvature_window_        = this->get_parameter("curvature_window").as_double();
    max_angular_velocity_     = this->get_parameter("max_angular_velocity").as_double();
    max_angular_acceleration_ = this->get_parameter("max_angular_acceleration").as_double();
    min_lookahead_            = this->get_parameter("min_lookahead").as_double();
    max_lookahead_            = this->get_parameter("max_lookahead").as_double();
    lookahead_gain_           = this->get_parameter("lookahead_gain").as_double();
    goal_tolerance_           = this->get_parameter("goal_tolerance").as_double();
    goal_approach_speed_      = this->get_parameter("goal_approach_speed").as_double();
    max_path_error_           = this->get_parameter("max_path_error").as_double();
    max_initial_path_error_   = this->get_parameter("max_initial_path_error").as_double();
    max_pose_age_             = this->get_parameter("max_pose_age").as_double();
    heading_gain_             = this->get_parameter("heading_gain").as_double();

    map_frame_      = this->get_parameter("map_frame").as_string();
    base_frame_     = this->get_parameter("base_frame").as_string();
    bucket_frame_   = this->get_parameter("bucket_frame").as_string();
    max_bucket_age_ = this->get_parameter("max_bucket_age").as_double();

    if (!std::isfinite(control_rate_) || control_rate_ <= 0.0 ||
        !std::isfinite(max_speed_) || max_speed_ <= 0.0 ||
        !std::isfinite(max_acceleration_) || max_acceleration_ <= 0.0 ||
        !std::isfinite(max_deceleration_) || max_deceleration_ <= 0.0 ||
        !std::isfinite(max_jerk_) || max_jerk_ <= 0.0 ||
        !std::isfinite(min_lookahead_) || min_lookahead_ <= 0.0 ||
        !std::isfinite(max_lookahead_) || max_lookahead_ < min_lookahead_ ||
        !std::isfinite(curvature_window_) || curvature_window_ <= 0.0 ||
        curvature_window_ > min_lookahead_ ||
        !std::isfinite(max_path_error_) || max_path_error_ <= 0.0 ||
        !std::isfinite(max_initial_path_error_) ||
        max_initial_path_error_ < max_path_error_ ||
        map_frame_.empty() || base_frame_.empty()) {
        throw std::invalid_argument("Invalid path follower configuration");
    }
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
        } else if (name == "max_angular_velocity") {
            max_angular_velocity_ = param.as_double();
        } else if (name == "max_angular_acceleration") {
            max_angular_acceleration_ = param.as_double();
        } else if (name == "min_lookahead") {
            min_lookahead_ = param.as_double();
        } else if (name == "max_lookahead") {
            max_lookahead_ = param.as_double();
        } else if (name == "lookahead_gain") {
            lookahead_gain_ = param.as_double();
        } else if (name == "goal_tolerance") {
            goal_tolerance_ = param.as_double();
        } else if (name == "goal_approach_speed") {
            goal_approach_speed_ = param.as_double();
        } else if (name == "max_path_error") {
            max_path_error_ = param.as_double();
        } else if (name == "max_initial_path_error") {
            max_initial_path_error_ = param.as_double();
        } else if (name == "max_pose_age") {
            max_pose_age_ = param.as_double();
        } else if (name == "heading_gain") {
            heading_gain_ = param.as_double();
        }
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
            status_ = NavigationStatus::Tracking;
            RCLCPP_INFO(this->get_logger(), "Transition to TRACKING for MovingBucket.");
        } else {
            status_ = NavigationStatus::Moving;
            RCLCPP_INFO(this->get_logger(), "Transition to MOVING.");
        }
    } else if (last_cmd_ == NavigationCommand::Stanby || last_cmd_ == NavigationCommand::Sleep) {
        goal_latched_ = false;
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
    if ((status_ == NavigationStatus::Moving || status_ == NavigationStatus::Goal) &&
        has_path_ && !msg->poses.empty()) {
        const auto& incoming_goal = msg->poses.back().pose.position;
        const double goal_difference = std::hypot(
            incoming_goal.x - goal_pose_.pose.position.x,
            incoming_goal.y - goal_pose_.pose.position.y);
        if (msg->header.frame_id == map_frame_ && goal_difference <= kEpsilon) {
            return;
        }
    }

    bool same_path = has_path_ || goal_latched_;
    if (same_path && msg->header.frame_id != map_frame_) {
        same_path = false;
    }
    if (same_path && msg->poses.size() != source_path_.size()) {
        same_path = false;
    }
    if (same_path) {
        for (std::size_t index = 0; index < msg->poses.size(); ++index) {
            const auto& incoming_pose = msg->poses[index].pose.position;
            const auto& stored_pose = source_path_[index].pose.position;
            if (std::hypot(
                    incoming_pose.x - stored_pose.x,
                    incoming_pose.y - stored_pose.y) > kEpsilon) {
                same_path = false;
                break;
            }
        }
    }
    if (same_path) {
        return;
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

    stop();
    path_.clear();
    source_path_ = msg->poses;
    progress_ = 0.0;
    goal_latched_ = false;
    if (msg->header.frame_id != map_frame_) {
        RCLCPP_WARN(
            this->get_logger(), "Rejected path in frame '%s'; expected '%s'",
            msg->header.frame_id.c_str(), map_frame_.c_str());
        has_path_ = false;
        source_path_.clear();
        return;
    }

    for (const auto& pose : msg->poses) {
        if (!finitePose(pose) ||
            (!pose.header.frame_id.empty() && pose.header.frame_id != map_frame_)) {
            path_.clear();
            source_path_.clear();
            has_path_ = false;
            RCLCPP_WARN(this->get_logger(), "Rejected path with invalid point or frame.");
            return;
        }
        if (path_.empty()) {
            path_.push_back({pose.pose.position.x, pose.pose.position.y, 0.0, max_speed_, 0.0});
            continue;
        }
        const auto start = path_.back();
        const double step_distance =
            std::hypot(pose.pose.position.x - start.x, pose.pose.position.y - start.y);
        if (step_distance <= kEpsilon) {
            continue;
        }
        const int steps = std::max(
            1, static_cast<int>(std::ceil(step_distance / kPathStep - kEpsilon)));
        for (int step = 1; step <= steps; ++step) {
            const double fraction = static_cast<double>(step) / steps;
            path_.push_back({
                start.x + fraction * (pose.pose.position.x - start.x),
                start.y + fraction * (pose.pose.position.y - start.y),
                start.distance + fraction * step_distance,
                max_speed_,
                0.0});
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
        const auto previous = sample(path_[index].distance - curvature_window_);
        const auto& current = path_[index];
        const auto next = sample(path_[index].distance + curvature_window_);
        const double ab = std::hypot(current.x - previous.x, current.y - previous.y);
        const double bc = std::hypot(next.x - current.x, next.y - current.y);
        const double ac = std::hypot(next.x - previous.x, next.y - previous.y);
        const double cross = (current.x - previous.x) * (next.y - current.y) -
                             (current.y - previous.y) * (next.x - current.x);
        const double denominator = ab * bc * ac;
        if (denominator > kEpsilon) {
            const double curvature = 2.0 * std::abs(cross) / denominator;
            path_[index].speed = std::min(
                max_speed_, std::sqrt(max_lateral_acceleration_ / std::max(curvature, kEpsilon)));
        }
    }

    path_.front().speed = 0.0;
    path_.back().speed = 0.0;
    const auto speed_profile = makeSCurve(
        path_.back().distance, max_speed_, max_acceleration_, max_deceleration_, max_jerk_);
    for (auto& point : path_) {
        point.speed = std::min(point.speed, sCurveSpeed(speed_profile, point.distance));
    }
    for (std::size_t index = 1; index < path_.size(); ++index) {
        const double distance = path_[index].distance - path_[index - 1].distance;
        path_[index].speed = std::min(
            path_[index].speed,
            std::sqrt(path_[index - 1].speed * path_[index - 1].speed +
                      2.0 * max_acceleration_ * distance));
    }
    for (std::size_t index = path_.size() - 1; index > 0; --index) {
        const double distance = path_[index].distance - path_[index - 1].distance;
        path_[index - 1].speed = std::min(
            path_[index - 1].speed,
            std::sqrt(path_[index].speed * path_[index].speed +
                      2.0 * max_deceleration_ * distance));
    }

    goal_pose_ = msg->poses.back();
    has_path_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Received new path with %zu poses. Goal=(%.3f, %.3f)",
        path_.size(),
        goal_pose_.pose.position.x,
        goal_pose_.pose.position.y
    );
}

void PathFollowerNode::stop()
{
    command_vx_world_ = 0.0;
    command_vy_world_ = 0.0;
    command_ax_world_ = 0.0;
    command_ay_world_ = 0.0;
    command_yaw_rate_ = 0.0;
    publishCmdVel(0.0, 0.0, 0.0);
}

PathFollowerNode::Projection PathFollowerNode::project(double x, double y) const
{
    Projection best{progress_, std::numeric_limits<double>::infinity()};
    for (std::size_t index = 1; index < path_.size(); ++index) {
        const auto& previous = path_[index - 1];
        const auto& current = path_[index];
        if (current.distance + 0.1 < progress_) {
            continue;
        }
        const double dx = current.x - previous.x;
        const double dy = current.y - previous.y;
        const double denominator = dx * dx + dy * dy;
        if (denominator <= kEpsilon) {
            continue;
        }
        const double fraction = std::clamp(
            ((x - previous.x) * dx + (y - previous.y) * dy) / denominator, 0.0, 1.0);
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
    distance = std::clamp(distance, 0.0, path_.back().distance);
    const auto iterator = std::lower_bound(
        path_.begin(), path_.end(), distance,
        [](const SpeedPoint& point, double value) { return point.distance < value; });
    if (iterator == path_.begin()) {
        return *iterator;
    }
    if (iterator == path_.end()) {
        return path_.back();
    }
    const auto& previous = *(iterator - 1);
    const double fraction =
        (distance - previous.distance) / std::max(iterator->distance - previous.distance, kEpsilon);
    return {
        previous.x + fraction * (iterator->x - previous.x),
        previous.y + fraction * (iterator->y - previous.y),
        distance,
        previous.speed + fraction * (iterator->speed - previous.speed),
        previous.time + fraction * (iterator->time - previous.time)};
}

void PathFollowerNode::controlLoop()
{
    const auto current_time = std::chrono::steady_clock::now();
    const double delta_time = std::clamp(
        std::chrono::duration<double>(current_time - last_control_).count(), 0.001, 0.1);
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
            this->get_logger(), *this->get_clock(), 2000,
            "Cannot get robot transform (%s -> %s): %s",
            map_frame_.c_str(), base_frame_.c_str(), exception.what());
        publishStatus();
        return;
    }

    const double pose_age = (this->now() - rclcpp::Time(robot_tf.header.stamp)).seconds();
    const double robot_x = robot_tf.transform.translation.x;
    const double robot_y = robot_tf.transform.translation.y;
    const double robot_yaw = getYawFromQuaternion(robot_tf.transform.rotation);
    if (!std::isfinite(pose_age) || pose_age > max_pose_age_ || pose_age < -0.1 ||
        !std::isfinite(robot_x) || !std::isfinite(robot_y) || !std::isfinite(robot_yaw)) {
        if (status_ == NavigationStatus::Moving || status_ == NavigationStatus::Tracking) {
            status_ = NavigationStatus::Fail;
        }
        stop();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Robot pose is stale or invalid (age=%.3f).", pose_age);
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
            stop();
            break;
        case NavigationStatus::Goal:
            stop();
            break;
        case NavigationStatus::Tracking: {
            geometry_msgs::msg::TransformStamped bucket_tf;
            try {
                bucket_tf = tf_buffer_->lookupTransform(
                    map_frame_, bucket_frame_, tf2::TimePointZero);
                const double age =
                    (this->now() - rclcpp::Time(bucket_tf.header.stamp)).seconds();
                if (max_bucket_age_ > 0.0 && (age > max_bucket_age_ || age < -0.1)) {
                    throw tf2::TransformException("bucket transform is stale");
                }
            } catch (const tf2::TransformException& exception) {
                status_ = NavigationStatus::Fail;
                stop();
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Cannot get valid bucket transform: %s", exception.what());
                break;
            }
            const double target_yaw = std::atan2(
                bucket_tf.transform.translation.y - robot_y,
                bucket_tf.transform.translation.x - robot_x);
            const double target_rate = std::clamp(
                heading_gain_ * normalizeAngle(target_yaw - robot_yaw),
                -max_angular_velocity_, max_angular_velocity_);
            command_yaw_rate_ += std::clamp(
                target_rate - command_yaw_rate_,
                -max_angular_acceleration_ * delta_time,
                max_angular_acceleration_ * delta_time);
            publishCmdVel(0.0, 0.0, command_yaw_rate_);
            break;
        }
        case NavigationStatus::Moving: {
            if (!has_path_ || path_.size() < 2) {
                stop();
                break;
            }
            const auto nearest = project(robot_x, robot_y);
            double path_error_limit = max_path_error_;
            if (progress_ <= kEpsilon) {
                path_error_limit = max_initial_path_error_;
            }
            if (nearest.error > path_error_limit) {
                status_ = NavigationStatus::Fail;
                stop();
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Path deviation %.3f m exceeds limit %.3f m.",
                    nearest.error, path_error_limit);
                break;
            }
            progress_ = std::max(progress_, nearest.distance);
            const double goal_distance = std::hypot(
                robot_x - goal_pose_.pose.position.x, robot_y - goal_pose_.pose.position.y);
            const double remaining = path_.back().distance - progress_;
            if (progress_ >= path_.back().distance - goal_tolerance_ &&
                goal_distance <= goal_tolerance_) {
                goal_latched_ = true;
                status_ = NavigationStatus::Goal;
                path_.clear();
                has_path_ = false;
                progress_ = 0.0;
                stop();
                RCLCPP_INFO(this->get_logger(), "Path complete.");
                break;
            }

            const double command_speed =
                std::hypot(command_vx_world_, command_vy_world_);
            const double lookahead = std::clamp(
                min_lookahead_ + lookahead_gain_ * command_speed,
                min_lookahead_, max_lookahead_);
            const auto target = sample(progress_ + lookahead);
            const double target_distance = std::hypot(target.x - robot_x, target.y - robot_y);
            double desired_speed = sample(
                progress_ + std::min(0.5 * lookahead, 0.5 * remaining)).speed;
            desired_speed = std::min(
                desired_speed, std::sqrt(std::max(0.0, 2.0 * max_deceleration_ * remaining)));
            if (remaining <= goal_tolerance_ && goal_distance > goal_tolerance_) {
                desired_speed = std::min(
                    {goal_approach_speed_, max_speed_, 1.5 * goal_distance});
            }
            const double desired_vx = target_distance > kEpsilon ?
                desired_speed * (target.x - robot_x) / target_distance : 0.0;
            const double desired_vy = target_distance > kEpsilon ?
                desired_speed * (target.y - robot_y) / target_distance : 0.0;
            double desired_ax = (desired_vx - command_vx_world_) / delta_time;
            double desired_ay = (desired_vy - command_vy_world_) / delta_time;
            double acceleration_limit = max_acceleration_;
            if (desired_speed < command_speed) {
                acceleration_limit = max_deceleration_;
            }
            limitVector(desired_ax, desired_ay, acceleration_limit);
            double delta_ax = desired_ax - command_ax_world_;
            double delta_ay = desired_ay - command_ay_world_;
            limitVector(delta_ax, delta_ay, max_jerk_ * delta_time);
            command_ax_world_ += delta_ax;
            command_ay_world_ += delta_ay;
            command_vx_world_ += command_ax_world_ * delta_time;
            command_vy_world_ += command_ay_world_ * delta_time;
            limitVector(command_vx_world_, command_vy_world_, max_speed_);

            const double cosine = std::cos(robot_yaw);
            const double sine = std::sin(robot_yaw);
            publishCmdVel(
                cosine * command_vx_world_ + sine * command_vy_world_,
                -sine * command_vx_world_ + cosine * command_vy_world_,
                0.0);
            publishLookaheadMarker(target.x, target.y);
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