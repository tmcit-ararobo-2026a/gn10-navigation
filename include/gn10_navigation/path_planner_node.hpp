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

    /**
     * @brief グリッドのインデックス (gx, gy) を算出
     *
     * @param wx ワールド座標wx
     * @param wy ワールド座標wy
     * @return std::pair<int, int> グリッドのインデックス
     */
    std::pair<int, int> worldToGrid(double wx, double wy) const;

    /**
     * @brief ワールド座標 (wx, wy) を算出
     *
     * @param gx グリッドのインデックスgx
     * @param gy グリッドのインデックスgy
     * @return std::pair<double, double> ワールド座標 (wx, wy)
     */
    std::pair<double, double> gridToWorld(int gx, int gy) const;

    /**
     * @brief 座標 (gx, gy) がコストマップの範囲内にあるか判定
     *
     * @param gx グリッドのインデックスgx
     * @param gy グリッドのインデックスgy
     * @return true コストマップの範囲内にある
     * @return false コストマップの範囲外
     */
    bool isInsideGrid(int gx, int gy) const;

    /**
     * @brief 通行可能領域かどうか 0 <= cost < 100
     *
     * @param gx グリッドのインデックスgx
     * @param gy グリッドのインデックスgy
     * @return true 通行可能領域である
     * @return false 通行不可能である
     */
    bool isPassable(int gx, int gy) const;

    /**
     * @brief 指定されたセル障害物内にある場合に、その周辺から最も近い通行可能なセルを探す
     *
     * @param gx グリッドのインデックスgx
     * @param gy グリッドのインデックスgy
     * @param max_radius_cells ロボットの半径
     * @return std::optional<std::pair<int, int>> 同心円状最も近い通行可能セル
     */
    std::optional<std::pair<int, int>> findNearestPassable(
        int gx, int gy, int max_radius_cells
    ) const;

    /**
     * @brief 8方向移動に対応したA-Star
     *
     * @param start_x スタート座標のグリッドのインデックスx
     * @param start_y スタート座標のグリッドのインデックスy
     * @param goal_x 目標座標のグリッドのインデックスx
     * @param goal_y 目標座標のグリッドのインデックスy
     * @return std::vector<std::pair<int, int>> (x, y)の組の配列で経路
     */
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
