#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "gn10_navigation/map_loader.hpp"

class CostmapGeneratorNode : public rclcpp::Node
{
public:
    CostmapGeneratorNode();
    void generateCostmap();
};