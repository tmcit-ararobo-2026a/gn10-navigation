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

    /**
     * @brief オブジェクトのコストを返す関数
     *
     * @param comment オブジェクトのコメント（文字列一致でコストを決定）
     * @param type オブジェクト形状など
     * @return int コスト
     */
    int getObjectCost(const std::string& comment, ObjectType type) const;

    /**
     * @brief BFSを用いたコストの波及処理
     *
     * @param base_costs 入力コストマップ
     * @param width コストマップの幅（セル数）
     * @param height コストマップの高さ（セル数）
     * @param output_grid コスト波及処理後の出力コストマップ
     */
    void applyMultiSourceBfsInflation(
        const std::vector<uint8_t>& base_costs,
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
