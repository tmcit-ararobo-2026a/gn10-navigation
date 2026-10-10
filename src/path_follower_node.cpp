#include "gn10_navigation/path_follower_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kPathStep = 0.05;
constexpr double kRadiansPerDegree = 0.017453292519943295;

void limitVector(double& x, double& y, double limit)
{
    const double magnitude = std::hypot(x, y);
    if (magnitude > limit) {
        x *= limit / magnitude;
        y *= limit / magnitude;
    }
}

bool finitePoint(const geometry_msgs::msg::PoseStamped& pose)
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
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (low + high);
        if (ramp_distance(mid) <= length) {
            low = mid;
        } else {
            high = mid;
        }
    }
    const double peak = low;
    const auto [up_jerk, up_flat] = ramp_time(peak, max_acceleration);
    const auto [down_jerk, down_flat] = ramp_time(peak, max_deceleration);
    const double cruise_time = std::max(0.0, (length - ramp_distance(peak)) / peak);
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
            const double dt = std::min(0.01, duration - elapsed);
            distance += speed * dt + 0.5 * acceleration * dt * dt +
                        jerk * dt * dt * dt / 6.0;
            speed += acceleration * dt + 0.5 * jerk * dt * dt;
            acceleration += jerk * dt;
            elapsed += dt;
            samples.push_back({distance, std::max(0.0, speed)});
        }
    }
    samples.back() = {length, 0.0};
    return samples;
}

double sCurveSpeed(const std::vector<MotionSample>& samples, double distance)
{
    const auto it = std::lower_bound(samples.begin(), samples.end(), distance,
        [](const MotionSample& point, double value) { return point.distance < value; });
    if (it == samples.begin()) {
        return it->speed;
    }
    if (it == samples.end()) {
        return 0.0;
    }
    const auto& previous = *(it - 1);
    const double fraction = (distance - previous.distance) /
        std::max(it->distance - previous.distance, kEpsilon);
    return previous.speed + fraction * (it->speed - previous.speed);
}

}  // namespace

PathFollowerNode::PathFollowerNode() : Node("path_follower_node")
{
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    max_speed_ = declare_parameter<double>("max_speed", 1.5);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 1.0);
    max_deceleration_ = declare_parameter<double>("max_deceleration", 1.0);
    max_jerk_ = declare_parameter<double>("max_jerk", 2.0);
    max_lateral_acceleration_ = declare_parameter<double>("max_lateral_acceleration", 0.7);
    curvature_window_ = declare_parameter<double>("curvature_window", 0.15);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.0);
    max_angular_acceleration_ = declare_parameter<double>("max_angular_acceleration", 4.0);
    heading_mode_ = declare_parameter<std::string>("heading_mode", "fixed");
    heading_gain_ = declare_parameter<double>("heading_gain", 2.0);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0);
    wheel_center_distance_ = declare_parameter<double>("wheel_center_distance", 0.0);
    max_wheel_angular_speed_ = declare_parameter<double>("max_wheel_angular_speed", 0.0);
    wheel_mount_angles_deg_ = declare_parameter<std::vector<double>>(
        "wheel_mount_angles_deg", {0.0, 120.0, 240.0});
    min_lookahead_ = declare_parameter<double>("min_lookahead", 0.3);
    max_lookahead_ = declare_parameter<double>("max_lookahead", 0.9);
    lookahead_gain_ = declare_parameter<double>("lookahead_gain", 0.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.08);
    goal_approach_speed_ = declare_parameter<double>("goal_approach_speed", 0.3);
    max_path_error_ = declare_parameter<double>("max_path_error", 0.5);
    max_pose_age_ = declare_parameter<double>("max_pose_age", 0.5);

    for (double value : {control_rate_, max_speed_, max_acceleration_, max_deceleration_,
                         max_jerk_, max_lateral_acceleration_, curvature_window_, max_angular_speed_,
                         max_angular_acceleration_, heading_gain_, min_lookahead_, max_lookahead_,
                         goal_tolerance_, goal_approach_speed_, max_path_error_, max_pose_age_}) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("Path follower limits must be finite and positive");
        }
    }
    if (!std::isfinite(lookahead_gain_) || lookahead_gain_ < 0.0 ||
        min_lookahead_ > max_lookahead_ || map_frame_.empty() || base_frame_.empty() ||
        (heading_mode_ != "fixed" && heading_mode_ != "hold" && heading_mode_ != "tangent")) {
        throw std::invalid_argument("Invalid path follower configuration");
    }
    if (curvature_window_ > min_lookahead_ + kEpsilon) {
        throw std::invalid_argument("curvature_window must not exceed min_lookahead");
    }
    if (!std::isfinite(wheel_radius_) || !std::isfinite(wheel_center_distance_) ||
        !std::isfinite(max_wheel_angular_speed_) || wheel_radius_ < 0.0 ||
        wheel_center_distance_ < 0.0 || max_wheel_angular_speed_ < 0.0 ||
        wheel_mount_angles_deg_.size() != 3 ||
        std::any_of(wheel_mount_angles_deg_.begin(), wheel_mount_angles_deg_.end(),
                    [](double angle) { return !std::isfinite(angle); })) {
        throw std::invalid_argument("Invalid omni wheel configuration");
    }
    if (max_wheel_angular_speed_ > 0.0) {
        if (wheel_radius_ <= 0.0 || wheel_center_distance_ <= 0.0) {
            throw std::invalid_argument("Wheel speed limit requires wheel radius and center distance");
        }
        const double wheel_speed_limit = wheel_radius_ * max_wheel_angular_speed_;
        if (max_speed_ > wheel_speed_limit) {
            RCLCPP_WARN(get_logger(), "max_speed %.2f m/s limited to %.2f m/s by wheel speed",
                        max_speed_, wheel_speed_limit);
            max_speed_ = wheel_speed_limit;
        }
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/planned_path", rclcpp::QoS(1),
        std::bind(&PathFollowerNode::onPath, this, std::placeholders::_1));
    speed_path_pub_ = create_publisher<nav_msgs::msg::Path>("/speed_path", rclcpp::QoS(1).transient_local());
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/robot/operation/cmd_vel", 10);
    last_control_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&PathFollowerNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "Path follower ready at %.1f Hz, heading_mode=%s, max_speed=%.2f m/s",
                control_rate_, heading_mode_.c_str(), max_speed_);
}

void PathFollowerNode::stop()
{
    command_vx_world_ = 0.0;
    command_vy_world_ = 0.0;
    command_ax_world_ = 0.0;
    command_ay_world_ = 0.0;
    command_yaw_rate_ = 0.0;
    cmd_pub_->publish(geometry_msgs::msg::Twist());
}

void PathFollowerNode::onPath(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    stop();
    path_.clear();
    progress_ = 0.0;
    heading_locked_ = false;
    nav_msgs::msg::Path cleared_path;
    cleared_path.header.frame_id = map_frame_;
    cleared_path.header.stamp = now();
    speed_path_pub_->publish(cleared_path);

    if (msg->header.frame_id != map_frame_) {
        RCLCPP_WARN(get_logger(), "Rejected path in frame '%s'; expected '%s'",
                    msg->header.frame_id.c_str(), map_frame_.c_str());
        return;
    }

    for (const auto& pose : msg->poses) {
        if (!finitePoint(pose) ||
            (!pose.header.frame_id.empty() && pose.header.frame_id != map_frame_)) {
            path_.clear();
            RCLCPP_WARN(get_logger(), "Rejected path with invalid point or frame");
            return;
        }
        const double x = pose.pose.position.x;
        const double y = pose.pose.position.y;
        if (path_.empty()) {
            path_.push_back({x, y, 0.0, max_speed_, 0.0});
            continue;
        }
        const auto start = path_.back();
        const double ds = std::hypot(x - start.x, y - start.y);
        if (!std::isfinite(ds) || !std::isfinite(start.distance + ds)) {
            path_.clear();
            RCLCPP_WARN(get_logger(), "Rejected path with invalid length");
            return;
        }
        if (ds > 1e-4) {
            const int steps = std::max(1, static_cast<int>(std::ceil(ds / kPathStep - 1e-9)));
            for (int step = 1; step <= steps; ++step) {
                const double fraction = static_cast<double>(step) / steps;
                path_.push_back({start.x + fraction * (x - start.x),
                                 start.y + fraction * (y - start.y),
                                 start.distance + fraction * ds, max_speed_, 0.0});
            }
        }
    }
    if (path_.size() < 2) {
        path_.clear();
        RCLCPP_WARN(get_logger(), "Path needs at least two distinct points");
        return;
    }

    // 制御の追従距離以下の区間で曲率を評価し、A*格子の細かなジグザグを平均化する。
    // Curvature limits lateral acceleration; yaw is independent for an omni base.
    for (std::size_t i = 1; i + 1 < path_.size(); ++i) {
        const auto a = sample(path_[i].distance - curvature_window_);
        const auto& b = path_[i];
        const auto c = sample(path_[i].distance + curvature_window_);
        const double ab = std::hypot(b.x - a.x, b.y - a.y);
        const double bc = std::hypot(c.x - b.x, c.y - b.y);
        const double ac = std::hypot(c.x - a.x, c.y - a.y);
        const double cross = (b.x - a.x) * (c.y - b.y) -
                             (b.y - a.y) * (c.x - b.x);
        const double denominator = ab * bc * ac;
        if (denominator <= kEpsilon) {
            path_[i].speed = 0.0;
            continue;
        }
        const double curvature = 2.0 * std::abs(cross) / denominator;
        if (curvature > kEpsilon) {
            path_[i].speed = std::min(max_speed_,
                std::sqrt(max_lateral_acceleration_ / curvature));
        }
    }
    path_.front().speed = 0.0;
    path_.back().speed = 0.0;

    const auto s_curve = makeSCurve(path_.back().distance, max_speed_, max_acceleration_,
                                     max_deceleration_, max_jerk_);
    for (auto& point : path_) {
        point.speed = std::min(point.speed, sCurveSpeed(s_curve, point.distance));
    }

    // Forward and backward passes enforce acceleration and stopping distance.
    for (std::size_t i = 1; i < path_.size(); ++i) {
        const double ds = path_[i].distance - path_[i - 1].distance;
        path_[i].speed = std::min(path_[i].speed,
            std::sqrt(path_[i - 1].speed * path_[i - 1].speed + 2.0 * max_acceleration_ * ds));
    }
    for (std::size_t i = path_.size() - 1; i > 0; --i) {
        const double ds = path_[i].distance - path_[i - 1].distance;
        path_[i - 1].speed = std::min(path_[i - 1].speed,
            std::sqrt(path_[i].speed * path_[i].speed + 2.0 * max_deceleration_ * ds));
    }

    nav_msgs::msg::Path speed_path;
    speed_path.header.frame_id = map_frame_;
    speed_path.header.stamp = now();
    for (std::size_t i = 0; i < path_.size(); ++i) {
        if (i > 0) {
            const double ds = path_[i].distance - path_[i - 1].distance;
            path_[i].time = path_[i - 1].time +
                2.0 * ds / std::max(path_[i - 1].speed + path_[i].speed, kEpsilon);
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_;
        pose.header.stamp = rclcpp::Time(speed_path.header.stamp) +
                            rclcpp::Duration::from_seconds(path_[i].time);
        pose.pose.position.x = path_[i].x;
        pose.pose.position.y = path_[i].y;
        pose.pose.orientation.w = 1.0;
        speed_path.poses.push_back(pose);
    }
    speed_path_pub_->publish(speed_path);
    RCLCPP_INFO(get_logger(), "Speed path: %zu points, %.2f m, nominal %.2f s",
                path_.size(), path_.back().distance, path_.back().time);
}

PathFollowerNode::Projection PathFollowerNode::project(double x, double y) const
{
    Projection best{progress_, std::numeric_limits<double>::infinity()};
    for (std::size_t i = 1; i < path_.size(); ++i) {
        const auto& a = path_[i - 1];
        const auto& b = path_[i];
        if (b.distance + 0.1 < progress_) {
            continue;
        }
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double fraction = std::clamp(((x - a.x) * dx + (y - a.y) * dy) /
            (dx * dx + dy * dy), 0.0, 1.0);
        const double error = std::hypot(x - a.x - fraction * dx, y - a.y - fraction * dy);
        if (error < best.error) {
            best = {a.distance + fraction * (b.distance - a.distance), error};
        }
    }
    return best;
}

PathFollowerNode::SpeedPoint PathFollowerNode::sample(double distance) const
{
    distance = std::clamp(distance, 0.0, path_.back().distance);
    const auto it = std::lower_bound(path_.begin(), path_.end(), distance,
        [](const SpeedPoint& point, double value) { return point.distance < value; });
    if (it == path_.begin()) {
        return *it;
    }
    if (it == path_.end()) {
        return path_.back();
    }
    const auto& a = *(it - 1);
    const double fraction = (distance - a.distance) / (it->distance - a.distance);
    return {a.x + fraction * (it->x - a.x), a.y + fraction * (it->y - a.y), distance,
            a.speed + fraction * (it->speed - a.speed),
            a.time + fraction * (it->time - a.time)};
}

void PathFollowerNode::onTimer()
{
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::clamp(std::chrono::duration<double>(steady_now - last_control_).count(),
                                 0.001, 0.1);
    last_control_ = steady_now;
    if (path_.empty()) {
        stop();
        return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& error) {
        stop();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Pose TF unavailable: %s", error.what());
        return;
    }
    const double age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
    if (!std::isfinite(age) || age > max_pose_age_ || age < -0.1) {
        stop();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Pose TF stale: %.2f s", age);
        return;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;
    const double yaw = tf2::getYaw(transform.transform.rotation);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        stop();
        return;
    }
    const auto nearest = project(x, y);
    if (nearest.error > max_path_error_) {
        stop();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Path deviation %.2f m exceeds limit", nearest.error);
        return;
    }
    progress_ = std::max(progress_, nearest.distance);
    const double goal_distance = std::hypot(x - path_.back().x, y - path_.back().y);
    if (progress_ >= path_.back().distance - goal_tolerance_ &&
        goal_distance <= goal_tolerance_) {
        stop();
        path_.clear();
        nav_msgs::msg::Path cleared_path;
        cleared_path.header.frame_id = map_frame_;
        cleared_path.header.stamp = now();
        speed_path_pub_->publish(cleared_path);
        RCLCPP_INFO(get_logger(), "Path complete");
        return;
    }

    if (heading_mode_ != "fixed" && !heading_locked_) {
        heading_target_ = yaw;
        heading_locked_ = true;
    }

    const double command_speed = std::hypot(command_vx_world_, command_vy_world_);
    const double lookahead = std::clamp(min_lookahead_ + lookahead_gain_ * command_speed,
                                         min_lookahead_, max_lookahead_);
    const auto target = sample(progress_ + lookahead);
    const double dx = target.x - x;
    const double dy = target.y - y;
    const double target_distance = std::hypot(dx, dy);
    const double remaining = path_.back().distance - progress_;

    // Holonomic pure pursuit points the translational velocity at the lookahead point.
    double desired_speed = sample(progress_ + std::min(0.5 * lookahead, 0.5 * remaining)).speed;
    desired_speed = std::min(desired_speed, std::sqrt(2.0 * max_deceleration_ * remaining));
    if (remaining <= goal_tolerance_ && goal_distance > goal_tolerance_) {
        // 経路終端への射影が先に到達しても、横方向の残差を低速で解消する。
        desired_speed = std::min({max_speed_, goal_approach_speed_, 1.5 * goal_distance});
    }
    const double desired_vx = target_distance > kEpsilon ? desired_speed * dx / target_distance : 0.0;
    const double desired_vy = target_distance > kEpsilon ? desired_speed * dy / target_distance : 0.0;
    double desired_ax = (desired_vx - command_vx_world_) / dt;
    double desired_ay = (desired_vy - command_vy_world_) / dt;
    const double acceleration_limit = desired_speed < command_speed ?
        max_deceleration_ : max_acceleration_;
    limitVector(desired_ax, desired_ay, acceleration_limit);
    double delta_ax = desired_ax - command_ax_world_;
    double delta_ay = desired_ay - command_ay_world_;
    limitVector(delta_ax, delta_ay, max_jerk_ * dt);
    command_ax_world_ += delta_ax;
    command_ay_world_ += delta_ay;
    command_vx_world_ += command_ax_world_ * dt;
    command_vy_world_ += command_ay_world_ * dt;
    limitVector(command_vx_world_, command_vy_world_, max_speed_);

    if (heading_mode_ == "fixed") {
        // 方位推定の変動や経路の曲がりによって回転指令を発生させない。
        command_yaw_rate_ = 0.0;
    } else {
        if (heading_mode_ == "tangent") {
            const auto before = sample(progress_ - 0.1);
            const auto after = sample(progress_ + 0.1);
            heading_target_ = std::atan2(after.y - before.y, after.x - before.x);
        }
        const double heading_error = std::atan2(std::sin(heading_target_ - yaw),
                                                 std::cos(heading_target_ - yaw));
        const double desired_yaw_rate = std::clamp(heading_gain_ * heading_error,
                                                   -max_angular_speed_, max_angular_speed_);
        command_yaw_rate_ = std::clamp(desired_yaw_rate,
            command_yaw_rate_ - max_angular_acceleration_ * dt,
            command_yaw_rate_ + max_angular_acceleration_ * dt);
        command_yaw_rate_ = std::clamp(command_yaw_rate_, -max_angular_speed_, max_angular_speed_);
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = std::cos(yaw) * command_vx_world_ +
                       std::sin(yaw) * command_vy_world_;
    command.linear.y = -std::sin(yaw) * command_vx_world_ +
                       std::cos(yaw) * command_vy_world_;
    command.angular.z = command_yaw_rate_;
    if (max_wheel_angular_speed_ > 0.0) {
        double peak_wheel_speed = 0.0;
        for (double angle_degrees : wheel_mount_angles_deg_) {
            const double angle = angle_degrees * kRadiansPerDegree;
            const double wheel_speed =
                (-std::sin(angle) * command.linear.x +
                  std::cos(angle) * command.linear.y +
                  wheel_center_distance_ * command.angular.z) / wheel_radius_;
            peak_wheel_speed = std::max(peak_wheel_speed, std::abs(wheel_speed));
        }
        if (peak_wheel_speed > max_wheel_angular_speed_) {
            const double scale = max_wheel_angular_speed_ / peak_wheel_speed;
            command.linear.x *= scale;
            command.linear.y *= scale;
            command.angular.z *= scale;
        }
    }
    cmd_pub_->publish(command);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
