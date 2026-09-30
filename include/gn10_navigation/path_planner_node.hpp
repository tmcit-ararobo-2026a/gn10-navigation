#pragma once

#include <cstddef>
#include <cstdint>
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

    // A*が使用するCostmap（costmap_generator_node から受信した最新マップ）
    // 1次元配列 (index = y * width_ + x) で保持する
    std::vector<uint8_t> planning_grid_;

    // 計算経路のpublisher
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
    // 取得に成功し、かつコストマップ内にいる場合に true を返す
    bool getRobotPose();

    // World座標系からGrid座標に変換する関数
    std::pair<int, int> worldToGrid(double world_x, double world_y);

    // grid座標系からWorld座標に変換する関数
    std::pair<double, double> gridToWorld(int grid_x, int grid_y);

    // 座標が範囲内に収まっているかどうかcheckする関数
    bool isInsideGrid(int x, int y);

    // 通行可能かcheckする関数 (254未満を通行可能と判定)
    bool isPassable(int x, int y);

    // grid座標を1次元indexに変換する関数
    std::size_t gridIndex(int x, int y) const;

    // 指定セルから最も近い通行可能セルを探す関数 (max_radius_cells 以内)
    std::optional<std::pair<int, int>> findNearestPassable(int x, int y, int max_radius_cells);

    // A*
    std::vector<std::pair<int, int>> aStar(int start_x, int start_y, int goal_x, int goal_y);

    // 実行中のパラメータ変更を受け付けるコールバック
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // B-spline
    std::vector<std::pair<double, double>> bsplineSmoothPath(
        const std::vector<std::pair<int, int>>& path
    );

    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& params
    );

    std::vector<std::pair<double, double>> extractImportantPoints(
        const std::vector<std::pair<int, int>>& path
    );

    void simplifyPathRecursive(
        const std::vector<std::pair<double, double>>& points,
        int start,
        int end,
        double tolerance,
        std::vector<bool>& keep
    );

    double pointToLineDistance(
        const std::pair<double, double>& point,
        const std::pair<double, double>& line_start,
        const std::pair<double, double>& line_end
    );

    bool isSmoothPathValid(const std::vector<std::pair<double, double>>& path);

    double calculatePathLength(const std::vector<std::pair<double, double>>& path);

    double calculateCurvatureCost(const std::vector<std::pair<double, double>>& path);

    double calculateSmoothnessCost(const std::vector<std::pair<double, double>>& path);

    // パス上のサンプル点が通るコストマップ値の平均 (0.0〜1.0 に正規化)
    double calculateObstacleCost(const std::vector<std::pair<double, double>>& path);

    double calculateControlPointSmoothnessCost(
        const std::vector<std::pair<double, double>>& control_points
    );

    double calculatePathCost(
        const std::vector<std::pair<double, double>>& path,
        const std::vector<std::pair<double, double>>& control_points,
        double base_length,
        double base_curvature,
        double base_smoothness,
        double base_control_smoothness
    );

    std::vector<std::pair<double, double>> evaluateBSplinePath(
        const std::vector<std::pair<double, double>>& control_points
    );

    // Frame / Topic names
    std::string map_frame_     = "map";
    std::string base_frame_    = "base_link";
    std::string costmap_topic_ = "costmap";
    std::string goal_topic_    = "/goal_pose";
    std::string path_topic_    = "/planned_path";

    // スタート/ゴールが通行不可セルにある場合に、最寄りの通行可能セルを探す半径 [m]
    // 0.0 にするとスナップしない
    double start_snap_radius_ = 0.5;
    double goal_snap_radius_  = 0.3;

    // B-spline parameters
    double bspline_length_weight_     = 1.0;
    double bspline_curvature_weight_  = 1.0;
    double bspline_smoothness_weight_ = 3.0;  // 滑らかさの程度を設定する関数

    // 障害物(膨張領域)への近さに対するペナルティの重み
    double bspline_obstacle_cost_weight_ = 150.0;

    int bspline_smoothing_iterations_ = 5;  // 何回平滑化を行うか

    double bspline_control_point_step_   = 0.05;
    int bspline_optimization_iterations_ = 20;
    int bspline_samples_per_segment_     = 10;

    // A*経路から重要点を抽出する際の閾値
    double bspline_turning_angle_threshold_ = 10.0;

    double bspline_simplification_tolerance_ = 0.10;

    // A*コスト
    double cost_factor_ = 0.05;

    // Map information
    unsigned int width_  = 0;
    unsigned int height_ = 0;

    double resolution_ = 0.0;

    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    // Robot
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    // Goal
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    // 通行不可と判定するコスト値 (この値以上は通行不可)
    int impassable_cost_ = 254;
};