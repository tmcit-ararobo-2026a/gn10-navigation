#include "gn10_navigation/target_manager_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace gn10_navigation {

TargetManagerNode::TargetManagerNode(const rclcpp::NodeOptions& options)
    : Node("target_manager_node", options)
{
    loadParameters();

    repository_.setFilePath(yaml_file_path_);
    if (!repository_.load()) {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not load shooting points from '%s'. Starting with empty points.",
            repository_.getResolvedFilePath().c_str()
        );
    } else {
        RCLCPP_INFO(
            this->get_logger(),
            "Successfully loaded %zu shooting points from '%s'.",
            repository_.size(),
            repository_.getResolvedFilePath().c_str()
        );
    }

    // Subscriptions
    target_id_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/target_id",
        10,
        std::bind(&TargetManagerNode::onTargetId, this, std::placeholders::_1)
    );

    nav_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/robot/navigation_command",
        10,
        std::bind(&TargetManagerNode::onNavigationCommand, this, std::placeholders::_1)
    );

    // Publishers
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    belt_launcher_speed_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("/robot/operation/belt_launcher_speed", 10);

    // Services
    update_point_srv_ = this->create_service<gn10_navigation::srv::UpdatePoint>(
        "/target_manager/update_point",
        std::bind(
            &TargetManagerNode::handleUpdatePoint,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    save_points_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/target_manager/save_points",
        std::bind(
            &TargetManagerNode::handleSavePoints, this, std::placeholders::_1, std::placeholders::_2
        )
    );

    RCLCPP_INFO(this->get_logger(), "TargetManagerNode initialized.");
}

void TargetManagerNode::loadParameters()
{
    this->declare_parameter<std::string>("yaml_file_path", "config/shooting_points.yaml");
    this->declare_parameter<std::string>("map_frame", "map");

    yaml_file_path_ = this->get_parameter("yaml_file_path").as_string();
    map_frame_      = this->get_parameter("map_frame").as_string();
}

void TargetManagerNode::onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    const uint8_t new_target_id = msg->data;
    if (current_target_id_ != new_target_id) {
        RCLCPP_INFO(
            this->get_logger(),
            "Target ID updated to: %u (%s)",
            new_target_id,
            toString(static_cast<TargetId>(new_target_id))
        );
        current_target_id_ = new_target_id;
        // すでに Move 状態のまま TargetId が切り替わった場合も即座に新目標へのゴールを発行
        if (nav_cmd_ == NavigationCommand::Move) {
            publishGoalPose(current_target_id_);
        }
    }
}

void TargetManagerNode::onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    NavigationCommand received_nav_cmd = static_cast<NavigationCommand>(msg->data);
    if (received_nav_cmd != nav_cmd_) {
        RCLCPP_INFO(
            this->get_logger(),
            "Navigation command received: %u (%s)",
            msg->data,
            toString(received_nav_cmd)
        );
    }

    if (nav_cmd_ != NavigationCommand::Move && received_nav_cmd == NavigationCommand::Move) {
        last_published_target_id_ = 255;  // 新規移動開始時は確実に発行
        publishGoalPose(current_target_id_);
    }

    nav_cmd_ = received_nav_cmd;
}

void TargetManagerNode::publishGoalPose(uint8_t target_id)
{
    const auto now_time = this->now();
    if (last_published_target_id_ == target_id &&
        (now_time - last_published_time_).seconds() < 0.2) {
        return;
    }

    auto pt_opt = repository_.get(target_id);
    if (!pt_opt.has_value()) {
        RCLCPP_WARN(this->get_logger(), "No shooting point found for TargetId %u", target_id);
        return;
    }

    last_published_target_id_ = target_id;
    last_published_time_      = now_time;

    const auto& pt = pt_opt.value();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = this->now();
    pose_msg.header.frame_id = map_frame_;

    pose_msg.pose.position.x = pt.x;
    pose_msg.pose.position.y = pt.y;
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pt.yaw);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    goal_pose_pub_->publish(pose_msg);
    RCLCPP_INFO(
        this->get_logger(),
        "Published goal_pose for target %u (%s): x=%.3f, y=%.3f, yaw=%.3f rad",
        target_id,
        pt.name.c_str(),
        pt.x,
        pt.y,
        pt.yaw
    );

    publishBeltLauncherSpeed(target_id);
}

void TargetManagerNode::publishBeltLauncherSpeed(uint8_t target_id)
{
    if (target_id == static_cast<uint8_t>(TargetId::MovingBucket)) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "Skipping belt_launcher_speed publication for MovingBucket (target_id=%u)",
            target_id
        );
        return;
    }

    auto pt_opt = repository_.get(target_id);
    if (!pt_opt.has_value()) {
        return;
    }

    const auto& pt = pt_opt.value();
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(pt.target_velocity);
    belt_launcher_speed_pub_->publish(speed_msg);
    RCLCPP_INFO(
        this->get_logger(),
        "Published belt_launcher_speed for target %u (%s): %.3f m/s",
        target_id,
        pt.name.c_str(),
        speed_msg.data
    );
}

void TargetManagerNode::handleUpdatePoint(
    const std::shared_ptr<gn10_navigation::srv::UpdatePoint::Request> request,
    std::shared_ptr<gn10_navigation::srv::UpdatePoint::Response> response
)
{
    uint8_t id = request->target_id;
    TargetPoint pt;
    auto existing = repository_.get(id);
    if (existing.has_value()) {
        pt = existing.value();
    }
    pt.x               = request->x;
    pt.y               = request->y;
    pt.yaw             = request->yaw;
    pt.target_velocity = request->target_velocity;
    pt.memo            = request->memo;
    if (pt.name.empty()) {
        pt.name = toString(static_cast<TargetId>(id));
    }

    repository_.update(id, pt);

    RCLCPP_INFO(
        this->get_logger(),
        "Updated in-memory point for target %u: x=%.3f, y=%.3f, yaw=%.3f, target_velocity=%.3f, memo='%s'",
        id,
        pt.x,
        pt.y,
        pt.yaw,
        pt.target_velocity,
        pt.memo.c_str()
    );

    response->success = true;
    response->message = "Point updated successfully in memory.";
}

void TargetManagerNode::handleSavePoints(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
{
    const std::string& path = repository_.getResolvedFilePath();
    if (repository_.save()) {
        response->success = true;
        response->message = "Successfully saved shooting points to " + path;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
        response->success = false;
        response->message = "Failed to save shooting points to " + path;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

}  // namespace gn10_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::TargetManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
