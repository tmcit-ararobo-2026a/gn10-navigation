#pragma once

#include <chrono>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "gn10_navigation/field_object.hpp"
#include "gn10_navigation/map_loader.hpp"

class CostmapGeneratorNode : public rclcpp::Node
{
public:
    CostmapGeneratorNode();

private:
    void declareAndGetParameters();
    nav_msgs::msg::OccupancyGrid generateCostmap();
    int getObjectCost(const std::string& comment, ObjectType type) const;
    void applyInflation(nav_msgs::msg::OccupancyGrid& costmap_msg);
    void publishCachedCostmap();

    // 動的パラメータ変更時のコールバック関数
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
    double robot_radius_;       // ロボットの物理半径 (254: 侵入禁止) [m]
    double inflation_margin_;   // 膨張マージン (グラデーション領域) [m]
    double cost_scale_factor_;  // コストによるマージン変化の度合い (0.0〜2.0程度)

    // コストパラメータを保持するメンバ変数
    int cost_wall_;
    int cost_partition_;
    int cost_obstacle_;
    int cost_default_;
};