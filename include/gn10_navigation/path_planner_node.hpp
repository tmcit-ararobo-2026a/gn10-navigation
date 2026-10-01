#pragma once

#include <tf2/utils.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct AStarNode {
    int x;
    int y;

    double g;
    double h;
    double f;
};

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void getCostmapMsg(nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg);

    bool getRobotPose();

    void getGoalPose(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);

    // 変換や確認用関数
    std::pair<int, int> worldToGrid(double world_x, double world_y);

    std::size_t gridtoIndex(int x, int y) const;
    std::pair<double, double> gridToWorld(int grid_x, int grid_y);

    bool isInsideGrid(int x, int y);
    bool isPassable(int x, int y);

    // 指定セルから最も近い通行可能セルを探す関数 (max_radius_cells 以内)
    std::optional<std::pair<int, int>> findNearestPassable(int x, int y, int max_radius_cells);

    // A*が使用するCostmap 1次元配列で保持
    std::vector<uint8_t> planning_grid_;

    // A*
    std::vector<std::pair<int, int>> aStar(int start_x, int start_y, int goal_x, int goal_y);

    /* 未実装のため、一旦コメントアウト
    std::vector<std::pair<double, double>> bsplineSmoothPath(
        const std::vector<std::pair<int, int>>& path
    );
    std::vector<std::pair<double, double>> extractImportantPoints(
        const std::vector<std::pair<int, int>>& path
    );
    */

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
    double robot_x_   = 0.0;
    double robot_y_   = 0.0;
    double robot_yaw_ = 0.0;

    // snap
    double start_snap_radius_;

    // map座標系
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    double cost_factor_;
};