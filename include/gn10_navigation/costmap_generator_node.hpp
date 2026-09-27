#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <string>

#include "gn10_navigation/field_object.hpp"
#include "gn10_navigation/map_loader.hpp"

class CostmapGeneratorNode : public rclcpp::Node
{
public:
    CostmapGeneratorNode();

private:
    void declareAndGetParameters();
    void generateAndPublishCostmap();
    int getObjectCost(const std::string& comment, ObjectType type) const;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    std::string map_source_type_;
    std::string map_file_path_;
    std::vector<std::string> map_objects_params_;

    // グリッドマップパラメータ
    double resolution_;
    double map_min_x_;
    double map_max_x_;
    double map_min_y_;
    double map_max_y_;
    double robot_z_min_;
    double robot_z_max_;

    // コストパラメータを保持するメンバ変数
    int cost_wall_;
    int cost_partition_;
    int cost_obstacle_;
    int cost_default_;
};