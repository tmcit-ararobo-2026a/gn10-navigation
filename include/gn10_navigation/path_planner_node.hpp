#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    // コストマップを受信するコールバック
    void getCostmapMsg(
        const nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg
    );

    // コストマップのSubscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        costmap_sub_;

    // 2次元コストマップ grid_[y][x]
    std::vector<std::vector<int>> grid_;

    // マップの情報
    unsigned int width_ = 0;
    unsigned int height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
};