#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
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
    // コストマップのSubscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    // goalの座標系を受信するsubscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    // 元のCostmap
    // Costmap Generatorから受け取った値をそのまま保持する
    std::vector<std::vector<int>> raw_grid_;

    // A*が実際に使用するCostmap
    // Safety Zoneなどを反映した後の値
    std::vector<std::vector<int>> planning_grid_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // コストマップを受信するコールバック
    void getCostmapMsg(const nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg);

    // map上のゴール座標系を受信(/goal_poseより)
    void getGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);

    // map上のロボット座標系を受け取る関数(tfより)
    void getRobotPose();

    // World座標系からGrid座標に変換する関数
    std::pair<int, int> worldToGrid(double world_x, double world_y);

    // grid座標系からWorld座標に変換する関数
    std::pair<double, double> gridToWorld(int grid_x, int grid_y);

    // 座標が範囲内に収まっているかどうかcheckする関数
    bool isInsideGrid(int x, int y);

    // 通行可能化checkする関数
    bool isPassable(int x, int y);

    // 障害物周りのセーフティーゾーン作成
    void generateSafetyZone();

    // A*
    std::vector<std::pair<int, int>> aStar(int start_x, int start_y, int goal_x, int goal_y);

    // マップの情報
    unsigned int width_  = 0;
    unsigned int height_ = 0;
    double resolution_   = 0.0;
    double origin_x_     = 0.0;
    double origin_y_     = 0.0;

    // robotの情報
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    // ゴールの情報
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    // セーフティーゾーンの設定
    double safety_zone_radius_ = 0.50;
};