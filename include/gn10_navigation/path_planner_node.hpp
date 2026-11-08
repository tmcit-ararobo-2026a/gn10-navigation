#pragma once

/**
 * @file path_planner_node.hpp
 * @brief NHKロボコン2026向け大域経路計画ノード (8方向A* + 動的適応B-spline + 安全領域旋回計画)
 *
 * 【大域経路計画アルゴリズムの概要】
 * 1. 始点・終点の同定と安全スナップ:
 *    - TF (map -> base_link) からロボット現在位置を取得し、グリッド座標へ変換。
 *    - 始点またはゴールがコストマップの障害物領域 (cost >= 100) にある場合、
 *      BFS探索により snap_distance (0.4m) 以内の最も近い通行可能セルへスナップ。
 *    - 格子量子化誤差を防ぐため、始点・終点は元の正確な連続ワールド座標 (exact_start, exact_goal)
 * を保持。
 *    - ゴールが障害物内にめり込んでいる場合 (教壇突き当て等)
 * は、安全アプローチ点までの経路を生成し、 末尾に直接直線進入区間を付加。
 *
 * 2. 8方向 A* 格子探索:
 *    - コスト関数: cost = step_distance + costmap_val * a_star.planning_cost
 *    - ユークリッド距離ヒューリスティックによる最短・低コスト格子経路探索。
 *
 * 3. 動的適応型 B-spline 平滑化 & 直線近似:
 *    - 制御点の間引きステップ (stride を max_stride: 10 から 1 まで順次試行) と、
 *      反復ラプラシアン平滑化 (隣接点の中点ブレンドを smoothing_iterations: 8 回実行)
 * を組み合わせる。
 *    - 経路上の全サンプルの最大セルコストが smooth_max_cost (40) 以下で干渉のない最大の stride
 * を動的に採択。
 *    - 低コストかつ十分な長さの直線区間を直線近似 (line_approximation.max_cost: -1 で無効化可能)。
 *
 * 4. 安全領域での旋回・姿勢計画 (planOrientations):
 *    - 機体外形ポリゴン (footprint.polygon) を経路上の各点で 360度 (angle_step: 0.15 rad 刻み)
 *      回転させた際の最大フットプリントコストを評価。
 *    - 移動長 turn_distance (0.6m) のウィンドウを走査し、区間全域で全方位のフットプリントコストが
 *      turn_max_cost (20) 以下かつ最小となる最も広い安全領域を選定。
 *    - Smoothstep (3次S字関数: 3t^2 - 2t^3)
 * により、安全区間内で開始Yawから目標Yawへ滑らかに姿勢遷移。
 */

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
    std::optional<std::pair<int, int>> findNearestPassable(int gx, int gy, int max_radius_cells)
        const;

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
        const std::vector<std::pair<double, double>>& path, int max_allowed_cost = 99
    ) const;

    /**
     * @brief 許容コスト以下の領域を通過し、かつ最小距離以上の区間を直線近似する。
     *
     * @param path 平滑化後のワールド座標列
     * @return std::vector<std::pair<double, double>> 直線近似適用後のワールド座標列
     */
    std::vector<std::pair<double, double>> approximateStraightLines(
        const std::vector<std::pair<double, double>>& path
    ) const;

    /**
     * @brief 指定した位置 (wx, wy) および姿勢 (yaw)
     * におけるロボットフットプリントの最大コストを算出する。
     *
     * @param wx ワールドX座標
     * @param wy ワールドY座標
     * @param yaw ロボットのヨー角
     * @param cutoff_cost このコストを超えた時点で早期打ち切りする閾値
     * @return int フットプリント内の最大コスト値 (0〜100)
     */
    int getFootprintMaxCost(double wx, double wy, double yaw, int cutoff_cost = 100) const;

    /**
     * @brief 指定した位置 (wx, wy) および姿勢 (yaw)
     * において、ロボットのフットプリントが安全か判定する。
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
     * @brief
     * パス上の旋回区間全域にわたりフットプリントが最も安全な領域を探索し、各ポーズのYaw角を計画する。
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

    // パラメータ (config/path_planner_params.yaml と整合)
    std::string map_frame_{"map"};
    std::string base_frame_{"base_link"};
    double planning_cost_factor_{0.5};  // a_star.planning_cost (コストマップ潜在コストの重み)
    double snap_distance_{0.4};  // a_star.snap_distance [m] (障害物からのスナップ探索半径)
    int bspline_samples_per_segment_{10};
    double bspline_smoothing_strength_{1.0};
    int bspline_smoothing_iterations_{8};  // 反復ラプラシアン平滑化の回数
    int bspline_max_stride_{20};           // 制御点間引きステップの最大試行値
    int bspline_smooth_max_cost_{40};      // 平滑化経路が許容する最大セルコスト

    // パス直線近似パラメータ
    int line_approx_max_cost_{-1};          // -1 で無効化
    double line_approx_min_distance_{1.2};  // 直線化を適用する最小セグメント距離 [m]

    // フットプリントおよび旋回計画パラメータ
    std::vector<std::pair<double, double>> footprint_polygon_;
    int turn_max_cost_{20};  // 旋回区間全域で許容する最大フットプリントコスト
    double angle_step_{0.15};    // 旋回干渉判定の角度ステップ [rad] (約8.6度)
    double turn_distance_{0.6};  // 旋回を行う移動区間長 [m]

    // コストマップ情報
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool has_costmap_{false};
};

}  // namespace gn10_navigation
