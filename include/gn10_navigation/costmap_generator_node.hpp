#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "gn10_navigation/field_object.hpp"
#include "gn10_navigation/map_loader.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gn10_navigation {

class CostmapGeneratorNode : public rclcpp::Node
{
public:
    explicit CostmapGeneratorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void declareAndGetParameters();

    /**
     * @brief コストマップの生成
     *
     * @return nav_msgs::msg::OccupancyGrid コストマップ
     */
    nav_msgs::msg::OccupancyGrid generateCostmap();
    nav_msgs::msg::OccupancyGrid generateCostmapFromJSON();
    nav_msgs::msg::OccupancyGrid generateCostmapFromESDF();

    /**
     * @brief オブジェクトのコストを返す関数
     *
     * @param obj フィールドオブジェクト（cost が指定されていれば優先、未指定なら comment から判定）
     * @return int コスト
     */
    int getObjectCost(const FieldObject& obj) const;

    /**
     * @brief BFSを用いたコストの波及処理
     *
     * @param obstacle_costs 入力障害物コストマップ
     * @param penalty_costs 通過可能ペナルティ領域（避けたい場所）マップ
     * @param width コストマップの幅（セル数）
     * @param height コストマップの高さ（セル数）
     * @param output_grid コスト波及処理後の出力コストマップ
     */
    void applyMultiSourceBfsInflation(
        const std::vector<uint8_t>& obstacle_costs,
        const std::vector<uint8_t>& penalty_costs,
        int width,
        int height,
        std::vector<int8_t>& output_grid
    );

    void publishCachedCostmap();

    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters
    );

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    nav_msgs::msg::OccupancyGrid cached_costmap_msg_;

    std::string map_source_type_;
    std::string map_file_path_;

    // グリッドマップパラメータ
    double resolution_;
    double map_min_x_;
    double map_max_x_;
    double map_min_y_;
    double map_max_y_;
    double robot_z_min_;
    double robot_z_max_;

    // 膨張コスト（インフレーション）パラメータ
    double robot_radius_;      // ロボット物理半径 [m] (絶対不可侵: 254)
    double inflation_margin_;  // コスト波及マージン幅 [m]

    // オブジェクト別基準コスト
    int cost_wall_;       // 外壁: 120
    int cost_partition_;  // 仕切り板: 200
    int cost_obstacle_;   // 絶対不可侵障害物: 254
    int cost_default_;    // デフォルト: 254
};

}  // namespace gn10_navigation
