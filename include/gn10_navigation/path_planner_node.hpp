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
 * 3. 2段ハイブリッド平滑化 (Elastic Band + 決定論的幾何フィレット):
 *    - 第1段 (smoothPathElasticBand):
 *      A* 格子経路に対し、内部張力 (曲率最小化) とコスト勾配反発 (-∇Cost) による連続緩和を実施。
 *      5cm 格子の離散階段ノイズを解消し、障害物ポテンシャルに沿った安全で有機的な大局経路を形成。
 *    - 第2段 (smoothPathFillet):
 *      第1段で滑らかになった実座標経路から RDP 法で主要コーナー頂点を抽出。
 *      各コーナーに目標旋回半径 (target_radius: 2.0m) の大円弧フィレットを幾何学的に接続し、曲率を大幅に低減。
 *      障害物近傍では max_cost (60) を満たす安全半径 (R >= min_radius: 0.2m) へ二分探索で自動調停。
 *      直線部と円弧の接点にはイージングブレンド (curvature_blend_iterations: 3) を施し、連続曲率化を実現。
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

    /**
     * @brief コストマップコールバック (テストからも利用可能)
     */
    void onCostmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

    /**
     * @brief A* の格子経路をコスト勾配による連続緩和 (Elastic Band) で平滑化する。
     *        内部張力 (曲率最小化・直線化) とコストマップ反発力 (-∇Cost) の釣り合いにより、
     *        障害物を安全に回避しつつ 5cm 格子階段ノイズを連続的かつ滑らかに緩和する。
     *
     * @param grid_path A* が生成した格子経路
     * @param exact_start 正確な始点ワールド座標
     * @param exact_goal 正確な終点ワールド座標
     * @return 平滑化後のワールド座標列
     */
    std::vector<std::pair<double, double>> smoothPathElasticBand(
        const std::vector<std::pair<int, int>>& grid_path,
        const std::pair<double, double>& exact_start,
        const std::pair<double, double>& exact_goal
    ) const;

    /**
     * @brief 実座標経路に対して決定論的幾何コーナーフィレットを適用し、大円弧化による曲率低減を行う。
     *        RDP法で大域コーナーを抽出し、目標旋回半径 target_radius の円弧フィレットと
     *        局所イージングブレンドにより、曲率を劇的に低減しつつ滑らかに接続する。
     *
     * @param raw_points 第1段(Elastic Band)を通過した実座標経路
     * @param exact_start 正確な始点ワールド座標
     * @param exact_goal 正確な終点ワールド座標
     * @return フィレット平滑化後のワールド座標列
     */
    std::vector<std::pair<double, double>> smoothPathFillet(
        const std::vector<std::pair<double, double>>& raw_points,
        const std::pair<double, double>& exact_start,
        const std::pair<double, double>& exact_goal
    ) const;

    /**
     * @brief 2段ハイブリッド平滑化エントリーポイント (Elastic Band -> 幾何コーナーフィレット)
     */
    std::vector<std::pair<double, double>> smoothPath(
        const std::vector<std::pair<int, int>>& grid_path,
        const std::pair<double, double>& exact_start,
        const std::pair<double, double>& exact_goal
    ) const;

private:
    void declareAndGetParameters();
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
    // コスト勾配による連続緩和 (Elastic Band) パラメータ
    int eband_iterations_{30};                  // elastic_band.iterations (反復緩和回数)
    double eband_elastic_strength_{1.0};        // elastic_band.elastic_strength (内部張力ゲイン 0.0〜1.0)
    double eband_repulsive_strength_{0.05};     // elastic_band.repulsive_strength (コスト反発ゲイン)
    int eband_repulsive_cost_threshold_{40};    // elastic_band.repulsive_cost_threshold (反発力発生閾値 0〜99)
    int eband_max_cost_{60};                    // elastic_band.max_cost (許容最大セルコスト 0〜99)
    double eband_path_resolution_{0.05};        // elastic_band.path_resolution [m] (サンプリング解像度)

    // 幾何コーナーフィレット (Fillet) パラメータ
    double fillet_rdp_epsilon_{0.15};           // fillet.rdp_epsilon [m] (RDP許容誤差)
    double fillet_target_radius_{2.0};          // fillet.target_radius [m] (目標大旋回半径)
    double fillet_min_radius_{0.2};             // fillet.min_radius [m] (最小許容旋回半径)
    int fillet_max_cost_{60};                   // fillet.max_cost (許容最大セルコスト 0〜99)
    double fillet_path_resolution_{0.05};       // fillet.path_resolution [m] (サンプリング解像度)
    int fillet_curvature_blend_iterations_{3};  // fillet.curvature_blend_iterations (イージング反復回数)

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
