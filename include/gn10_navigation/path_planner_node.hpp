#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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
     * @brief 指定されたワールド座標のセルコストを取得 (範囲外・未知領域は100)
     */
    int getCellCostWorld(double wx, double wy) const;

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

    /**
     * @brief A* の格子経路を動的適応型 B-spline (障害物に干渉しない最大平滑化度合い) で平滑化する。
     *        始点と終点にはグリッド量子化前の正確なワールド座標を保持する。
     *
     * @param grid_path A* が生成した格子経路
     * @param exact_start 正確な始点ワールド座標
     * @param exact_goal 正確な終点ワールド座標
     * @return 平滑化後のワールド座標列
     */
    std::vector<std::pair<double, double>> smoothPath(
        const std::vector<std::pair<int, int>>& grid_path,
        const std::pair<double, double>& exact_start,
        const std::pair<double, double>& exact_goal
    ) const;

    /**
     * @brief 与えられた制御点列と平滑化設定から 3 次 clamped B-spline 経路を生成する
     *
     * @param control_points 制御点列 (ワールド座標)
     * @param smoothing_strength 1反復あたりの内部制御点の平滑化強度 (0.0〜1.0)
     * @param smoothing_iterations 反復ラプラシアン平滑化の回数
     * @param allowed_max_cost 制御点移動時に許容する最大セルコスト
     * @return std::vector<std::pair<double, double>> 生成された B-spline 補間点列
     */
    std::vector<std::pair<double, double>> evaluateBspline(
        const std::vector<std::pair<double, double>>& control_points,
        double smoothing_strength,
        int smoothing_iterations,
        int allowed_max_cost
    ) const;

    /**
     * @brief ワールド座標列がすべて許容コスト以下の通行可能セル上にあるか確認する。
     *
     * @param path ワールド座標列
     * @param max_allowed_cost 許容する最大コスト (1〜99)
     * @return true すべて許容コスト以下で通行可能
     * @return false 障害物またはコスト超過セルと干渉している
     */
    bool isSmoothPathValid(
        const std::vector<std::pair<double, double>>& path,
        int max_allowed_cost = 99
    ) const;

    /**
     * @brief 指定した位置 (wx, wy) および姿勢 (yaw) におけるロボットフットプリントの最大コストを算出する。
     *
     * @param wx ワールドX座標
     * @param wy ワールドY座標
     * @param yaw ロボットのヨー角
     * @param cutoff_cost このコストを超えた時点で早期打ち切りする閾値
     * @return int フットプリント内の最大コスト値 (0〜100)
     */
    int getFootprintMaxCost(double wx, double wy, double yaw, int cutoff_cost = 100) const;

    /**
     * @brief 指定した位置 (wx, wy) および姿勢 (yaw) において、ロボットのフットプリントが安全か判定する。
     *
     * @param wx ワールドX座標
     * @param wy ワールドY座標
     * @param yaw ロボットのヨー角
     * @param max_cost 許容する最大コスト値 (0〜100)
     * @return true すべてのフットプリント領域が max_cost 以下で安全
     * @return false 障害物または max_cost 超過のセルと接触
     */
    bool isFootprintSafe(double wx, double wy, double yaw, int max_cost) const;

    /**
     * @brief パス上の旋回区間全域にわたりフットプリントが最も安全な領域を探索し、各ポーズのYaw角を計画する。
     *
     * @param smoothed_path 平滑化された座標列 (x, y)
     * @param start_yaw スタート時のロボットのYaw角
     * @param goal_yaw ゴール目標のYaw角
     * @return std::vector<geometry_msgs::msg::PoseStamped> 姿勢(Orientation)付きポーズ列
     */
    std::vector<geometry_msgs::msg::PoseStamped> planOrientations(
        const std::vector<std::pair<double, double>>& smoothed_path,
        double start_yaw,
        double goal_yaw
    ) const;

    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters
    );

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // パラメータ
    std::string map_frame_;
    std::string base_frame_;
    double planning_cost_factor_{1.5};  // a_star.planning_cost
    double snap_distance_{0.4};         // a_star.snap_distance [m]
    int bspline_samples_per_segment_{10};
    double bspline_smoothing_strength_{1.0};
    int bspline_smoothing_iterations_{5};
    int bspline_max_stride_{4};
    int bspline_smooth_max_cost_{80};

    // フットプリントおよび旋回計画パラメータ
    std::vector<std::pair<double, double>> footprint_polygon_;
    int turn_max_cost_{20};
    double angle_step_{0.1};
    double turn_distance_{0.6};

    // コストマップ情報
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool has_costmap_{false};
};

}  // namespace gn10_navigation
