#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    void getCostmapMsg(nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg);

    bool getRobotPose();

    // 変換や確認用関数
    std::pair<int, int> worldToGrid(double world_x, double world_y);

    std::size_t gridtoIndex(int x, int y) const;

    bool isInsideGrid(int x, int y);
    bool isPassable(int x, int y);

    // A*が使用するCostmap　1次元配列で保持
    std::vector<uint8_t> planning_grid_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 受信コストマップパラメータ
    std::string map_frame_;
    std::string base_frame_;

    // Map information
    double resolution_   = 0.0;
    unsigned int width_  = 0;
    unsigned int height_ = 0;
    double origin_x_     = 0.0;
    double origin_y_     = 0.0;

    // map座標系から見たrobot座標系の格納場所
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
};
