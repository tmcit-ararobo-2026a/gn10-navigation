#include "gn10_navigation/target_manager_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace gn10_navigation {

TargetManagerNode::TargetManagerNode(const rclcpp::NodeOptions& options)
    : Node("target_manager_node", options)
{
    loadParameters();

    resolved_file_path_ = resolveFilePath(yaml_file_path_);
    if (!loadFromFile(resolved_file_path_)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not load shooting points from '%s'. Starting with empty points.",
            resolved_file_path_.c_str()
        );
    } else {
        RCLCPP_INFO(
            this->get_logger(),
            "Successfully loaded %zu shooting points from '%s'.",
            shooting_points_.size(),
            resolved_file_path_.c_str()
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

    // Publisher
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

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

std::string TargetManagerNode::resolveFilePath(const std::string& path)
{
    if (std::filesystem::exists(path)) {
        return path;
    }

    try {
        std::string share_dir = ament_index_cpp::get_package_share_directory("gn10_navigation");
        std::string pkg_path  = share_dir + "/" + path;
        if (std::filesystem::exists(pkg_path)) {
            return pkg_path;
        }
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(this->get_logger(), "Failed to get share directory: %s", e.what());
    }

    return path;
}

bool TargetManagerNode::loadFromFile(const std::string& file_path)
{
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        if (!config["shooting_points"]) {
            RCLCPP_ERROR(this->get_logger(), "YAML does not contain 'shooting_points' key.");
            return false;
        }

        shooting_points_.clear();
        auto sp = config["shooting_points"];

        if (sp.IsMap()) {
            for (auto it = sp.begin(); it != sp.end(); ++it) {
                uint8_t id = static_cast<uint8_t>(it->first.as<int>());
                TargetPoint pt;
                if (it->second["name"]) {
                    pt.name = it->second["name"].as<std::string>();
                }
                if (it->second["x"]) {
                    pt.x = it->second["x"].as<double>();
                }
                if (it->second["y"]) {
                    pt.y = it->second["y"].as<double>();
                }
                if (it->second["yaw"]) {
                    pt.yaw = it->second["yaw"].as<double>();
                }
                if (it->second["memo"]) {
                    pt.memo = it->second["memo"].as<std::string>();
                }
                shooting_points_[id] = pt;
            }
        } else if (sp.IsSequence()) {
            for (size_t i = 0; i < sp.size(); ++i) {
                uint8_t id = static_cast<uint8_t>(i);
                auto node  = sp[i];
                if (node["target_id"]) {
                    id = static_cast<uint8_t>(node["target_id"].as<int>());
                } else if (node["id"]) {
                    id = static_cast<uint8_t>(node["id"].as<int>());
                }
                TargetPoint pt;
                if (node["name"]) {
                    pt.name = node["name"].as<std::string>();
                }
                if (node["x"]) {
                    pt.x = node["x"].as<double>();
                }
                if (node["y"]) {
                    pt.y = node["y"].as<double>();
                }
                if (node["yaw"]) {
                    pt.yaw = node["yaw"].as<double>();
                }
                if (node["memo"]) {
                    pt.memo = node["memo"].as<std::string>();
                }
                shooting_points_[id] = pt;
            }
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            this->get_logger(), "Error reading YAML '%s': %s", file_path.c_str(), e.what()
        );
        return false;
    }
}

bool TargetManagerNode::saveToFile(const std::string& file_path)
{
    try {
        YAML::Node root;
        for (const auto& [id, pt] : shooting_points_) {
            YAML::Node node;
            node["target_id"] = static_cast<int>(id);
            node["name"]      = pt.name;
            node["x"]         = pt.x;
            node["y"]         = pt.y;
            node["yaw"]       = pt.yaw;
            node["memo"]      = pt.memo;
            root["shooting_points"].push_back(node);
        }

        std::filesystem::path p(file_path);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }

        std::ofstream fout(file_path);
        if (!fout.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file for writing: %s", file_path.c_str());
            return false;
        }
        fout << root;
        fout.close();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            this->get_logger(), "Error writing YAML '%s': %s", file_path.c_str(), e.what()
        );
        return false;
    }
}

void TargetManagerNode::onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    current_target_id_ = msg->data;
    RCLCPP_INFO(
        this->get_logger(),
        "Target ID updated to: %u (%s)",
        current_target_id_,
        toString(static_cast<TargetId>(current_target_id_))
    );
}

void TargetManagerNode::onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
    current_nav_cmd_ = static_cast<NavigationCommand>(msg->data);
    RCLCPP_INFO(
        this->get_logger(),
        "Navigation command received: %u (%s)",
        msg->data,
        toString(current_nav_cmd_)
    );

    if (current_nav_cmd_ == NavigationCommand::Move) {
        publishGoalPose(current_target_id_);
    }
}

void TargetManagerNode::publishGoalPose(uint8_t target_id)
{
    auto it = shooting_points_.find(target_id);
    if (it == shooting_points_.end()) {
        RCLCPP_WARN(this->get_logger(), "No shooting point found for TargetId %u", target_id);
        return;
    }

    const auto& pt = it->second;

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
}

void TargetManagerNode::handleUpdatePoint(
    const std::shared_ptr<gn10_navigation::srv::UpdatePoint::Request> request,
    std::shared_ptr<gn10_navigation::srv::UpdatePoint::Response> response
)
{
    uint8_t id = request->target_id;
    auto& pt   = shooting_points_[id];
    pt.x       = request->x;
    pt.y       = request->y;
    pt.yaw     = request->yaw;
    pt.memo    = request->memo;
    if (pt.name.empty()) {
        pt.name = toString(static_cast<TargetId>(id));
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Updated in-memory point for target %u: x=%.3f, y=%.3f, yaw=%.3f, memo='%s'",
        id,
        pt.x,
        pt.y,
        pt.yaw,
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
    if (saveToFile(resolved_file_path_)) {
        response->success = true;
        response->message = "Successfully saved shooting points to " + resolved_file_path_;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
        response->success = false;
        response->message = "Failed to save shooting points to " + resolved_file_path_;
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
