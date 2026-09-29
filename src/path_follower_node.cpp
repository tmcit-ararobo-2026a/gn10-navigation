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
    max_speed_ = declare_parameter<double>("max_speed", 1.0);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 1.0);
    max_deceleration_ = declare_parameter<double>("max_deceleration", 1.0);
    max_jerk_ = declare_parameter<double>("max_jerk", 2.0);
    max_lateral_acceleration_ = declare_parameter<double>("max_lateral_acceleration", 0.7);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.0);
    max_angular_acceleration_ = declare_parameter<double>("max_angular_acceleration", 4.0);
    min_lookahead_ = declare_parameter<double>("min_lookahead", 0.3);
    max_lookahead_ = declare_parameter<double>("max_lookahead", 0.9);
    lookahead_gain_ = declare_parameter<double>("lookahead_gain", 0.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.08);
    max_path_error_ = declare_parameter<double>("max_path_error", 0.5);
    max_pose_age_ = declare_parameter<double>("max_pose_age", 0.5);

    for (double value : {control_rate_, max_speed_, max_acceleration_, max_deceleration_,
                         max_jerk_, max_lateral_acceleration_, max_angular_speed_,
                         max_angular_acceleration_, min_lookahead_, max_lookahead_,
                         goal_tolerance_, max_path_error_, max_pose_age_}) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("Path follower limits must be finite and positive");
        }
    }
    if (!std::isfinite(lookahead_gain_) || lookahead_gain_ < 0.0 ||
        min_lookahead_ > max_lookahead_ || map_frame_.empty() || base_frame_.empty()) {
        throw std::invalid_argument("Invalid path follower configuration");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/planned_path", rclcpp::QoS(1),
        std::bind(&PathFollowerNode::onPath, this, std::placeholders::_1));
    speed_path_pub_ = create_publisher<nav_msgs::msg::Path>("/speed_path", rclcpp::QoS(1).transient_local());
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/robot/command/cmd_vel", 10);
    last_control_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&PathFollowerNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "Path follower ready at %.1f Hz", control_rate_);
}

void PathFollowerNode::stop()
{
    command_speed_ = 0.0;
    command_acceleration_ = 0.0;
    command_yaw_rate_ = 0.0;
    cmd_pub_->publish(geometry_msgs::msg::Twist());
}

void PathFollowerNode::onPath(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    stop();
    path_.clear();
    progress_ = 0.0;
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

    // Curvature and yaw-rate limits form a local speed envelope.
    for (std::size_t i = 1; i + 1 < path_.size(); ++i) {
        const auto& a = path_[i - 1];
        const auto& b = path_[i];
        const auto& c = path_[i + 1];
        const double ab = std::hypot(b.x - a.x, b.y - a.y);
        const double bc = std::hypot(c.x - b.x, c.y - b.y);
        const double ac = std::hypot(c.x - a.x, c.y - a.y);
        const double cross = (b.x - a.x) * (c.y - b.y) -
                             (b.y - a.y) * (c.x - b.x);
        const double curvature = 2.0 * std::abs(cross) / (ab * bc * ac);
        if (curvature > kEpsilon) {
            path_[i].speed = std::min({max_speed_,
                std::sqrt(max_lateral_acceleration_ / curvature),
                max_angular_speed_ / curvature});
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
    if (progress_ >= path_.back().distance - goal_tolerance_ &&
        std::hypot(x - path_.back().x, y - path_.back().y) <= goal_tolerance_) {
        stop();
        path_.clear();
        nav_msgs::msg::Path cleared_path;
        cleared_path.header.frame_id = map_frame_;
        cleared_path.header.stamp = now();
        speed_path_pub_->publish(cleared_path);
        RCLCPP_INFO(get_logger(), "Path complete");
        return;
    }

    const double lookahead = std::clamp(min_lookahead_ + lookahead_gain_ * command_speed_,
                                         min_lookahead_, max_lookahead_);
    const auto target = sample(progress_ + lookahead);
    const double dx = target.x - x;
    const double dy = target.y - y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double curvature = 2.0 * local_y / std::max(dx * dx + dy * dy, kEpsilon);
    const double remaining = path_.back().distance - progress_;

    // The spatial profile is the speed target. The command integrator limits jerk.
    double desired_speed = sample(progress_ + std::min(0.5 * lookahead, 0.5 * remaining)).speed;
    desired_speed = std::min(desired_speed, std::sqrt(2.0 * max_deceleration_ * remaining));
    if (std::abs(curvature) > kEpsilon) {
        desired_speed = std::min(desired_speed, max_angular_speed_ / std::abs(curvature));
    }
    if (local_x < 0.0) {
        desired_speed = 0.0;
    }
    const double desired_acceleration = std::clamp(
        (desired_speed - command_speed_) / dt, -max_deceleration_, max_acceleration_);
    command_acceleration_ = std::clamp(desired_acceleration,
        command_acceleration_ - max_jerk_ * dt,
        command_acceleration_ + max_jerk_ * dt);
    command_speed_ = std::clamp(command_speed_ + command_acceleration_ * dt, 0.0, max_speed_);

    double desired_yaw_rate = command_speed_ * curvature;
    if (local_x < 0.0) {
        desired_yaw_rate = std::clamp(2.0 * std::atan2(local_y, local_x),
                                      -max_angular_speed_, max_angular_speed_);
    }
    command_yaw_rate_ = std::clamp(desired_yaw_rate,
        command_yaw_rate_ - max_angular_acceleration_ * dt,
        command_yaw_rate_ + max_angular_acceleration_ * dt);
    command_yaw_rate_ = std::clamp(command_yaw_rate_, -max_angular_speed_, max_angular_speed_);

    geometry_msgs::msg::Twist command;
    command.linear.x = command_speed_;
    command.angular.z = command_yaw_rate_;
    cmd_pub_->publish(command);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
