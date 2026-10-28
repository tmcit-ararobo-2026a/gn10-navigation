#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gn10_navigation/navigation_types.hpp"
#include "gn10_navigation/srv/update_point.hpp"
#include "gn10_navigation/target_repository.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace gn10_navigation {

class TargetManagerNode : public rclcpp::Node
{
public:
    explicit TargetManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void loadParameters();

    void onTargetId(const std_msgs::msg::UInt8::ConstSharedPtr msg);
    void onNavigationCommand(const std_msgs::msg::UInt8::ConstSharedPtr msg);

    void publishGoalPose(uint8_t target_id);

    void handleUpdatePoint(
        const std::shared_ptr<gn10_navigation::srv::UpdatePoint::Request> request,
        std::shared_ptr<gn10_navigation::srv::UpdatePoint::Response> response
    );

    void handleSavePoints(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_id_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;

    rclcpp::Service<gn10_navigation::srv::UpdatePoint>::SharedPtr update_point_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_points_srv_;

    std::string yaml_file_path_;
    std::string map_frame_;

    uint8_t current_target_id_{0};
    NavigationCommand nav_cmd_{NavigationCommand::Stanby};

    TargetRepository repository_;
};

}  // namespace gn10_navigation
