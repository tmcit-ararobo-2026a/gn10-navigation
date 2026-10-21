#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace gn10_navigation {

class PathPlannerNode : public rclcpp::Node
{
public:
    explicit PathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void declareAndGetParameters();
    void onCostmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void onGoalPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    // 座標変換ヘルパー
    std::pair<int, int> worldToGrid(double wx, double wy) const;
    std::pair<double, double> gridToWorld(int gx, int gy) const;
    bool isInsideGrid(int gx, int gy) const;
    bool isPassable(int gx, int gy) const;
    std::optional<std::pair<int, int>> findNearestPassable(int gx, int gy, int max_radius_cells)
        const;

    // 8方向A*探索
    std::vector<std::pair<int, int>> aStarSearch(int start_x, int start_y, int goal_x, int goal_y);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // パラメータ
    std::string map_frame_;
    std::string base_frame_;
    double planning_cost_factor_;  // a_star.planning_cost (3.0)

    // コストマップ情報
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool has_costmap_{false};
};

}  // namespace gn10_navigation
