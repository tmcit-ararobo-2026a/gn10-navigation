#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    void getCostmapMsg(nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg);

    // A*が使用するCostmap　1次元配列で保持
    std::vector<uint8_t> planning_grid_;

    // mapのdata確認用
    std::string map_frame_ = "map";

    // Map information
    double resolution_   = 0.0;
    unsigned int width_  = 0;
    unsigned int height_ = 0;
    double origin_x_     = 0.0;
    double origin_y_     = 0.0;
};
