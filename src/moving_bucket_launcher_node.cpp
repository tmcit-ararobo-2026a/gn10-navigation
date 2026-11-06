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
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

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

    bucket_opening_sub_ = this->create_subscription<lidar_perception_system::msg::MovingBucketTrack>(
        opening_topic_,
        10,
        std::bind(&MovingBucketLauncherNode::onBucketOpening, this, std::placeholders::_1)
    );

    bucket_bottom_sub_ = this->create_subscription<lidar_perception_system::msg::BucketBottom>(
        bottom_topic_,
        10,
        std::bind(&MovingBucketLauncherNode::onBucketBottom, this, std::placeholders::_1)
    );

    // Shared publisher with target_manager_node.
    // Published ONLY when target_id == MovingBucket (7) and nav_cmd == Move (1).
    belt_launcher_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/operation/belt_launcher_speed", 10
    );

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MovingBucketLauncherNode::onSetParameters, this, std::placeholders::_1)
    );

    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, calculation_rate_));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&MovingBucketLauncherNode::onTimer, this)
    );

    RCLCPP_INFO(
        this->get_logger(),
        "MovingBucketLauncherNode initialized (use_tf=%s, bucket_frame='%s', rate=%.1f Hz, opening_topic='%s', use_bottom_topic=%s, bucket_height=%.3f m).",
        use_tf_ ? "true" : "false",
        bucket_frame_.c_str(),
        calculation_rate_,
        opening_topic_.c_str(),
        use_bottom_topic_ ? "true" : "false",
        bucket_height_
    );
}

void MovingBucketLauncherNode::loadParameters()
{
    this->declare_parameter<std::string>("base_frame", base_frame_);
    this->declare_parameter<std::string>("bucket_frame", bucket_frame_);
    this->declare_parameter<bool>("use_tf", use_tf_);
    this->declare_parameter<bool>("is_bottom_frame", is_bottom_frame_);
    this->declare_parameter<double>("calculation_rate", calculation_rate_);
    this->declare_parameter<double>("speed_filter_alpha", speed_filter_alpha_);
    this->declare_parameter<std::string>("opening_topic", opening_topic_);
    this->declare_parameter<std::string>("bottom_topic", bottom_topic_);
    this->declare_parameter<bool>("use_bottom_topic", use_bottom_topic_);
    this->declare_parameter<double>("bucket_height", bucket_height_);
    this->declare_parameter<double>("aim_z_offset", aim_z_offset_);
    this->declare_parameter<double>("launcher_offset_x", launcher_offset_x_);
    this->declare_parameter<double>("launcher_offset_z", launcher_offset_z_);
    this->declare_parameter<double>("launch_angle_deg", launch_angle_deg_);
    this->declare_parameter<double>("velocity_quad", velocity_quad_);
    this->declare_parameter<double>("velocity_scale", velocity_scale_);
    this->declare_parameter<double>("velocity_offset", velocity_offset_);
    this->declare_parameter<double>("min_velocity", min_velocity_);
    this->declare_parameter<double>("max_velocity", max_velocity_);
    this->declare_parameter<double>("min_horizontal_dist", min_horizontal_dist_);
    this->declare_parameter<double>("max_horizontal_dist", max_horizontal_dist_);
    this->declare_parameter<double>("max_tracking_age_sec", max_tracking_age_sec_);

    base_frame_           = this->get_parameter("base_frame").as_string();
    bucket_frame_         = this->get_parameter("bucket_frame").as_string();
    use_tf_               = this->get_parameter("use_tf").as_bool();
    is_bottom_frame_      = this->get_parameter("is_bottom_frame").as_bool();
    calculation_rate_     = this->get_parameter("calculation_rate").as_double();
    speed_filter_alpha_   = this->get_parameter("speed_filter_alpha").as_double();
    opening_topic_        = this->get_parameter("opening_topic").as_string();
    bottom_topic_         = this->get_parameter("bottom_topic").as_string();
    use_bottom_topic_     = this->get_parameter("use_bottom_topic").as_bool();
    bucket_height_        = this->get_parameter("bucket_height").as_double();
    aim_z_offset_         = this->get_parameter("aim_z_offset").as_double();
    launcher_offset_x_    = this->get_parameter("launcher_offset_x").as_double();
    launcher_offset_z_    = this->get_parameter("launcher_offset_z").as_double();
    launch_angle_deg_     = this->get_parameter("launch_angle_deg").as_double();
    velocity_quad_        = this->get_parameter("velocity_quad").as_double();
    velocity_scale_       = this->get_parameter("velocity_scale").as_double();
    velocity_offset_      = this->get_parameter("velocity_offset").as_double();
    min_velocity_         = this->get_parameter("min_velocity").as_double();
    max_velocity_         = this->get_parameter("max_velocity").as_double();
    min_horizontal_dist_  = this->get_parameter("min_horizontal_dist").as_double();
    max_horizontal_dist_  = this->get_parameter("max_horizontal_dist").as_double();
    max_tracking_age_sec_ = this->get_parameter("max_tracking_age_sec").as_double();
}

rcl_interfaces::msg::SetParametersResult MovingBucketLauncherNode::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        const auto& name = param.get_name();
        if (name == "base_frame") {
            base_frame_ = param.as_string();
        } else if (name == "bucket_frame") {
            bucket_frame_ = param.as_string();
        } else if (name == "use_tf") {
            use_tf_ = param.as_bool();
        } else if (name == "is_bottom_frame") {
            is_bottom_frame_ = param.as_bool();
        } else if (name == "calculation_rate") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val <= 0.0) {
                result.successful = false;
                result.reason = "calculation_rate must be positive";
                return result;
            }
            calculation_rate_ = val;
            const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, calculation_rate_));
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&MovingBucketLauncherNode::onTimer, this)
            );
        } else if (name == "speed_filter_alpha") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val <= 0.0 || val > 1.0) {
                result.successful = false;
                result.reason = "speed_filter_alpha must be in (0.0, 1.0]";
                return result;
            }
            speed_filter_alpha_ = val;
        } else if (name == "use_bottom_topic") {
            use_bottom_topic_ = param.as_bool();
        } else if (name == "bucket_height") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val < 0.0) {
                result.successful = false;
                result.reason = "bucket_height must be finite and non-negative";
                return result;
            }
            bucket_height_ = val;
        } else if (name == "aim_z_offset") {
            const double val = param.as_double();
            if (!std::isfinite(val)) {
                result.successful = false;
                result.reason = "aim_z_offset must be finite";
                return result;
            }
            aim_z_offset_ = val;
        } else if (name == "launcher_offset_x") {
            const double val = param.as_double();
            if (!std::isfinite(val)) {
                result.successful = false;
                result.reason = "launcher_offset_x must be finite";
                return result;
            }
            launcher_offset_x_ = val;
        } else if (name == "launcher_offset_z") {
            const double val = param.as_double();
            if (!std::isfinite(val)) {
                result.successful = false;
                result.reason = "launcher_offset_z must be finite";
                return result;
            }
            launcher_offset_z_ = val;
        } else if (name == "launch_angle_deg") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val <= 0.0 || val >= 89.0) {
                result.successful = false;
                result.reason = "launch_angle_deg must be in (0.0, 89.0)";
                return result;
            }
            launch_angle_deg_ = val;
        } else if (name == "velocity_quad") {
            const double val = param.as_double();
            if (!std::isfinite(val)) {
                result.successful = false;
                result.reason = "velocity_quad must be finite";
                return result;
            }
            velocity_quad_ = val;
        } else if (name == "velocity_scale") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val <= 0.0) {
                result.successful = false;
                result.reason = "velocity_scale must be finite and positive";
                return result;
            }
            velocity_scale_ = val;
        } else if (name == "velocity_offset") {
            const double val = param.as_double();
            if (!std::isfinite(val)) {
                result.successful = false;
                result.reason = "velocity_offset must be finite";
                return result;
            }
            velocity_offset_ = val;
        } else if (name == "min_velocity") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val < 0.0 || val > max_velocity_) {
                result.successful = false;
                result.reason = "min_velocity must be in [0.0, max_velocity]";
                return result;
            }
            min_velocity_ = val;
        } else if (name == "max_velocity") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val < min_velocity_) {
                result.successful = false;
                result.reason = "max_velocity must be >= min_velocity";
                return result;
            }
            max_velocity_ = val;
        } else if (name == "min_horizontal_dist") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val < 0.0 || val >= max_horizontal_dist_) {
                result.successful = false;
                result.reason = "min_horizontal_dist must be in [0.0, max_horizontal_dist)";
                return result;
            }
            min_horizontal_dist_ = val;
        } else if (name == "max_horizontal_dist") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val <= min_horizontal_dist_) {
                result.successful = false;
                result.reason = "max_horizontal_dist must be > min_horizontal_dist";
                return result;
            }
            max_horizontal_dist_ = val;
        } else if (name == "max_tracking_age_sec") {
            const double val = param.as_double();
            if (!std::isfinite(val) || val <= 0.0) {
                result.successful = false;
                result.reason = "max_tracking_age_sec must be positive";
                return result;
            }
            max_tracking_age_sec_ = val;
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

bool MovingBucketLauncherNode::isPublishAllowed() const
{
    return current_target_id_ == static_cast<uint8_t>(TargetId::MovingBucket) &&
           nav_cmd_ == NavigationCommand::Move;
}

void MovingBucketLauncherNode::onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    if (current_target_id_ != msg->data) {
        current_target_id_ = msg->data;
        if (!isPublishAllowed()) {
            filtered_speed_valid_ = false;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Target ID updated to: %u (%s)",
            current_target_id_,
            toString(static_cast<TargetId>(current_target_id_))
        );
    }
}

void MovingBucketLauncherNode::onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    const auto received_cmd = static_cast<NavigationCommand>(msg->data);
    if (nav_cmd_ != received_cmd) {
        nav_cmd_ = received_cmd;
        if (!isPublishAllowed()) {
            filtered_speed_valid_ = false;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Navigation command updated to: %u (%s)",
            msg->data,
            toString(received_cmd)
        );
    }
}

void MovingBucketLauncherNode::onTimer()
{
    if (!isPublishAllowed()) {
        filtered_speed_valid_ = false;
        return;
    }

    if (!use_tf_) {
        return;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            base_frame_, bucket_frame_, tf2::TimePointZero
        );
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Could not lookup transform '%s' -> '%s': %s",
            base_frame_.c_str(),
            bucket_frame_.c_str(),
            ex.what()
        );
        return;
    }

    const auto stamp_time = rclcpp::Time(tf_stamped.header.stamp);
    if (stamp_time.nanoseconds() > 0 && max_tracking_age_sec_ > 0.0) {
        const double age_sec = (this->now() - stamp_time).seconds();
        if (age_sec > max_tracking_age_sec_ || age_sec < -0.1) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Bucket transform '%s' -> '%s' is stale (age=%.2f s > %.2f s). Skipping speed calculation.",
                base_frame_.c_str(),
                bucket_frame_.c_str(),
                age_sec,
                max_tracking_age_sec_
            );
            return;
        }
    }

    const double base_x = tf_stamped.transform.translation.x;
    const double base_y = tf_stamped.transform.translation.y;
    const double base_z = tf_stamped.transform.translation.z + (is_bottom_frame_ ? bucket_height_ : 0.0);

    calculateAndPublishSpeedFromBase(base_x, base_y, base_z);
}

void MovingBucketLauncherNode::onBucketOpening(
    const lidar_perception_system::msg::MovingBucketTrack::ConstSharedPtr msg
)
{
    if (use_bottom_topic_) {
        return;
    }

    // Avoid collision with target_manager_node and do not publish when not in Move state.
    if (!isPublishAllowed()) {
        filtered_speed_valid_ = false;
        return;
    }

    if (!msg->valid) {
        RCLCPP_DEBUG(this->get_logger(), "MovingBucketTrack measurement is invalid. Skipping speed calculation.");
        return;
    }

    rclcpp::Time obs_time(msg->last_observation_stamp);
    if (obs_time.nanoseconds() == 0) {
        obs_time = msg->header.stamp;
    }
    const double age_sec = (this->now() - obs_time).seconds();
    if (age_sec > max_tracking_age_sec_ || age_sec < -0.1) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Bucket opening track stale (age=%.2f s > %.2f s). Skipping speed calculation.",
            age_sec,
            max_tracking_age_sec_
        );
        return;
    }

    calculateAndPublishSpeed(msg->position, msg->header.frame_id);
}

void MovingBucketLauncherNode::onBucketBottom(
    const lidar_perception_system::msg::BucketBottom::ConstSharedPtr msg
)
{
    if (!use_bottom_topic_) {
        return;
    }

    // Avoid collision with target_manager_node and do not publish when not in Move state.
    if (!isPublishAllowed()) {
        filtered_speed_valid_ = false;
        return;
    }

    if (!msg->valid) {
        RCLCPP_DEBUG(this->get_logger(), "BucketBottom measurement is invalid. Skipping speed calculation.");
        return;
    }

    rclcpp::Time obs_time(msg->last_observation_stamp);
    if (obs_time.nanoseconds() == 0) {
        obs_time = msg->header.stamp;
    }
    const double age_sec = (this->now() - obs_time).seconds();
    if (age_sec > max_tracking_age_sec_ || age_sec < -0.1) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Bucket bottom track stale (age=%.2f s > %.2f s). Skipping speed calculation.",
            age_sec,
            max_tracking_age_sec_
        );
        return;
    }

    geometry_msgs::msg::Point opening_pos = msg->center;
    opening_pos.z += bucket_height_;
    calculateAndPublishSpeed(opening_pos, msg->header.frame_id);
}

void MovingBucketLauncherNode::calculateAndPublishSpeed(
    const geometry_msgs::msg::Point& opening_pos_map,
    const std::string& frame_id
)
{
    // Transform upper bucket opening position to base_frame (base_link)
    geometry_msgs::msg::PointStamped point_in, point_robot;
    point_in.header.frame_id = frame_id;
    point_in.header.stamp = rclcpp::Time(0);  // Latest available TF
    point_in.point = opening_pos_map;

    try {
        point_robot = tf_buffer_->transform(point_in, base_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Could not transform bucket opening from '%s' to '%s': %s",
            frame_id.c_str(),
            base_frame_.c_str(),
            ex.what()
        );
        return;
    }

    calculateAndPublishSpeedFromBase(point_robot.point.x, point_robot.point.y, point_robot.point.z);
}

void MovingBucketLauncherNode::calculateAndPublishSpeedFromBase(
    double base_x,
    double base_y,
    double base_z
)
{
    // Horizontal distance in base_link frame and relative distance from effective release point
    const double base_dist = std::hypot(base_x, base_y);
    if (base_dist < min_horizontal_dist_ || base_dist > max_horizontal_dist_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Bucket horizontal distance out of range (L=%.2f m, valid=[%.2f, %.2f] m)",
            base_dist,
            min_horizontal_dist_,
            max_horizontal_dist_
        );
        return;
    }

    const double delta_x = base_dist - launcher_offset_x_;
    if (delta_x <= 0.05) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Bucket too close to launcher release offset (delta_x=%.2f m)",
            delta_x
        );
        return;
    }

    const double target_z = base_z + aim_z_offset_;
    const double delta_z = target_z - launcher_offset_z_;

    // Ballistic trajectory model in base_link frame:
    // z(L) = z0 + delta_x * tan(theta) - (g * delta_x^2) / (2 * v0^2 * cos^2(theta))
    // => v0 = sqrt( (g * delta_x^2) / (2 * cos^2(theta) * (delta_x * tan(theta) - delta_z)) )
    constexpr double g = 9.80665;
    const double theta_rad = launch_angle_deg_ * M_PI / 180.0;
    const double cos_theta = std::cos(theta_rad);
    const double tan_theta = std::tan(theta_rad);

    const double denom = 2.0 * cos_theta * cos_theta * (delta_x * tan_theta - delta_z);
    if (denom <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Target opening unreachable at L=%.2f m (dx=%.2f m), dZ=%.2f m with angle=%.2f deg",
            base_dist,
            delta_x,
            delta_z,
            launch_angle_deg_
        );
        return;
    }

    const double v_ballistic = std::sqrt((g * delta_x * delta_x) / denom);

    // Convert ballistic speed v0 to belt release velocity command:
    // v_belt = velocity_quad * v0^2 + velocity_scale * v0 + velocity_offset
    const double v_target =
        velocity_quad_ * v_ballistic * v_ballistic +
        velocity_scale_ * v_ballistic +
        velocity_offset_;

    // Clamp within hardware limits
    const double v_clamped = std::clamp(v_target, min_velocity_, max_velocity_);

    // Low-pass filter to smooth distance measurement noise and prevent launcher motor jitter
    if (!filtered_speed_valid_) {
        filtered_speed_ = v_clamped;
        filtered_speed_valid_ = true;
    } else {
        filtered_speed_ = (1.0 - speed_filter_alpha_) * filtered_speed_ +
                          speed_filter_alpha_ * v_clamped;
    }

    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(filtered_speed_);
    belt_launcher_speed_pub_->publish(speed_msg);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        250,
        "MovingBucket opening: L=%.2f m (dx=%.2f m), z_open=%.2f m (dZ=%.2f m) -> v0=%.2f m/s, belt_speed=%.2f m/s (raw=%.2f m/s)",
        base_dist,
        delta_x,
        target_z,
        delta_z,
        v_ballistic,
        filtered_speed_,
        v_target
    );
}

}  // namespace gn10_navigation

#ifndef GN10_NAVIGATION_MOVING_BUCKET_LAUNCHER_NO_MAIN
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::MovingBucketLauncherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif
