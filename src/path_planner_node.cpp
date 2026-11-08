/**
 * @file path_planner_node.cpp
 * @brief NHKロボコン2026向け大域経路計画実装 (8方向A* + コスト勾配連続緩和Elastic Band + 安全旋回計画)
 *
 * 【大域経路計画アルゴリズムの詳細】
 * 1. 始点・終点の安全スナップ:
 *    - 始点・終点がインフレーション障害物セル (cost >= 100) にある場合、
 *      a_star.snap_distance (0.4m) の同心円BFSで最近傍の通行可能セル (cost < 100) を探索。
 *    - 教壇や壁への突き当て接触位置など障害物内にゴールがある場合は、
 *      通行可能セルを安全アプローチ点としてA*探索を行い、末尾に本来の目標座標への直線進入アプローチ区間を結合。
 *
 * 2. 8方向 A* 格子探索 (aStarSearch):
 *    - 評価関数: f(n) = g(n) + h(n)
 *    - 移動コスト: step_distance + costmap_val * a_star.planning_cost
 *    - a_star.planning_cost を大きくすると障害物回避マージンを大きく取る遠回り経路に、
 *      小さくすると幾何学的最短経路を優先する。
 *
 * 3. コスト勾配による連続緩和 Elastic Band (smoothPathElasticBand):
 *    - 内部張力 F_elastic (曲率最小化・平滑化) とコストマップ反発力 F_rep (-∇Cost) の釣り合いにより、
 *      障害物を安全に回避しつつ 5cm 格子階段ノイズを連続的かつ滑らかに緩和する。
 *    - 二値 (0/1) のハードしきい値判定によるジグザグや試行錯誤フォールバックを排除し、
 *      開けた場所では直線化、障害物近傍では滑らかに膨らむ決定論的軌道を生成。
 *
 * 4. 安全領域での旋回・姿勢計画 (planOrientations):
 *    - 機体の実外形ポリゴン (footprint.polygon) を経路上の各候補点で 360 度回転 (angle_step 刻み)
 * させ、 フットプリントが重なるセルの最大コストを全探索。
 *    - 長さ turn_distance (0.6m) のスライド窓を走査し、窓全域でフットプリント最大コストが
 *      turn_max_cost (20) 以下となる「最も安全な領域」で Smoothstep 補間により旋回を実行。
 */

#include "gn10_navigation/path_planner_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "gn10_navigation/robot_geometry.hpp"

namespace gn10_navigation {

namespace {

double normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q_msg)
{
    const double q_norm =
        q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w;
    if (q_norm <= 1e-4) {
        return 0.0;
    }
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

}  // namespace

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions& options)
    : Node("path_planner_node", options)
{
    declareAndGetParameters();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", map_qos, std::bind(&PathPlannerNode::onCostmap, this, std::placeholders::_1)
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PathPlannerNode::onGoalPose, this, std::placeholders::_1)
    );

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    // Dynamic parameters callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PathPlannerNode::onSetParameters, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PathPlannerNode initialized.");
}

void PathPlannerNode::declareAndGetParameters()
{
    // ==========================================
    // 1. 座標フレーム設定
    // ==========================================
    this->declare_parameter("map_configs.map_frame", "map");
    this->declare_parameter("map_configs.base_frame", "base_link");

    // ==========================================
    // 2. A* 探索パラメータ
    // ==========================================
    // a_star.planning_cost: コストマップ潜在コストの評価重み (g値 = step_dist + cost *
    // planning_cost)。
    // 大きいほど障害物の周囲のインフレーションマージンを遠回りし、小さいほど直線最短ルートを優先。
    this->declare_parameter("a_star.planning_cost", 0.5);
    // a_star.snap_distance [m]: スタート/ゴールが障害物内 (cost >= 100)
    // にある場合の通行可能セル探索半径。
    this->declare_parameter("a_star.snap_distance", 0.4);

    // ==========================================
    // 3. コスト勾配による連続緩和 (Elastic Band) 平滑化パラメータ
    // ==========================================
    // elastic_band.iterations: 反復緩和回数 (10〜100, 既定値: 30)
    this->declare_parameter("elastic_band.iterations", 30);
    // elastic_band.elastic_strength: 内部張力ゲイン (直線化・曲率最小化の強さ, 0.0〜1.0, 既定値: 0.5)
    this->declare_parameter("elastic_band.elastic_strength", 0.5);
    // elastic_band.repulsive_strength: コスト反発ゲイン (障害物・ポテンシャル勾配から押し離す強さ, 既定値: 0.05)
    this->declare_parameter("elastic_band.repulsive_strength", 0.05);
    // elastic_band.repulsive_cost_threshold: 反発力が働き始めるコスト閾値 (0〜99, 既定値: 15)
    this->declare_parameter("elastic_band.repulsive_cost_threshold", 15);
    // elastic_band.max_cost: 平滑化経路が通過を許容する最大セルコスト (0〜99, 既定値: 40)
    this->declare_parameter("elastic_band.max_cost", 40);
    // elastic_band.path_resolution: 出力経路サンプリング解像度 [m] (既定値: 0.05)
    this->declare_parameter("elastic_band.path_resolution", 0.05);

    // ==========================================
    // 4. 決定論的幾何コーナーフィレット平滑化パラメータ
    // ==========================================
    // fillet.rdp_epsilon [m]: RDP法で大域的コーナー頂点を抽出する許容誤差 (既定値: 0.15)
    this->declare_parameter("fillet.rdp_epsilon", 0.15);
    // fillet.target_radius [m]: 目標旋回半径 (既定値: 2.0, 2.4m/sで横G抑制)
    this->declare_parameter("fillet.target_radius", 2.0);
    // fillet.min_radius [m]: 狭所・障害物近傍での最小許容旋回半径 (既定値: 0.2)
    this->declare_parameter("fillet.min_radius", 0.2);
    // fillet.max_cost: コーナー円弧が許容する最大セルコスト (0〜99, 既定値: 60)
    this->declare_parameter("fillet.max_cost", 60);
    // fillet.path_resolution [m]: 出力経路サンプリング解像度 (既定値: 0.05)
    this->declare_parameter("fillet.path_resolution", 0.05);
    // fillet.curvature_blend_iterations: 直線と円弧の接続部を滑らかにブレンドするイージング回数 (既定値: 3)
    this->declare_parameter("fillet.curvature_blend_iterations", 3);

    // ==========================================
    // 5. フットプリントおよび旋回計画パラメータ
    // ==========================================
    // footprint.polygon: 機体の外形ポリゴン頂点列 (8頂点 [m])。原点は機体幾何中心。
    this->declare_parameter("footprint.polygon", kDefaultFootprint);
    // rotation_planning.turn_max_cost: 旋回区間全域で許容する最大フットプリントコスト。
    // 狭路や壁際での旋回を禁止し、広い空間でのみ旋回させる。
    this->declare_parameter("rotation_planning.turn_max_cost", 20);
    // rotation_planning.angle_step [rad]: 旋回干渉判定の角度ステップ (約8.6度)。
    this->declare_parameter("rotation_planning.angle_step", 0.15);
    // rotation_planning.turn_distance [m]: 旋回を行う移動区間長。
    this->declare_parameter("rotation_planning.turn_distance", 0.6);

    map_frame_            = this->get_parameter("map_configs.map_frame").as_string();
    base_frame_           = this->get_parameter("map_configs.base_frame").as_string();
    planning_cost_factor_ = this->get_parameter("a_star.planning_cost").as_double();
    snap_distance_        = this->get_parameter("a_star.snap_distance").as_double();
    eband_iterations_     = std::clamp(
        static_cast<int>(this->get_parameter("elastic_band.iterations").as_int()), 1, 200
    );
    eband_elastic_strength_   = this->get_parameter("elastic_band.elastic_strength").as_double();
    eband_repulsive_strength_ = this->get_parameter("elastic_band.repulsive_strength").as_double();
    eband_repulsive_cost_threshold_ = std::clamp(
        static_cast<int>(this->get_parameter("elastic_band.repulsive_cost_threshold").as_int()), 0, 99
    );
    eband_max_cost_ = std::clamp(
        static_cast<int>(this->get_parameter("elastic_band.max_cost").as_int()), 0, 99
    );
    eband_path_resolution_ = this->get_parameter("elastic_band.path_resolution").as_double();

    fillet_rdp_epsilon_   = this->get_parameter("fillet.rdp_epsilon").as_double();
    fillet_target_radius_ = this->get_parameter("fillet.target_radius").as_double();
    fillet_min_radius_    = this->get_parameter("fillet.min_radius").as_double();
    fillet_max_cost_      = std::clamp(
        static_cast<int>(this->get_parameter("fillet.max_cost").as_int()), 0, 99
    );
    fillet_path_resolution_ = this->get_parameter("fillet.path_resolution").as_double();
    fillet_curvature_blend_iterations_ = std::clamp(
        static_cast<int>(this->get_parameter("fillet.curvature_blend_iterations").as_int()), 0, 20
    );

    if (!std::isfinite(eband_elastic_strength_) || eband_elastic_strength_ < 0.0 ||
        eband_elastic_strength_ > 1.0) {
        throw std::invalid_argument("elastic_band.elastic_strength must be in [0.0, 1.0]");
    }
    if (!std::isfinite(eband_repulsive_strength_) || eband_repulsive_strength_ < 0.0) {
        throw std::invalid_argument("elastic_band.repulsive_strength must be non-negative");
    }
    if (!std::isfinite(eband_path_resolution_) || eband_path_resolution_ <= 0.0) {
        throw std::invalid_argument("elastic_band.path_resolution must be finite and positive");
    }
    if (!std::isfinite(fillet_rdp_epsilon_) || fillet_rdp_epsilon_ <= 0.0) {
        throw std::invalid_argument("fillet.rdp_epsilon must be finite and positive");
    }
    if (!std::isfinite(fillet_target_radius_) || fillet_target_radius_ <= 0.0) {
        throw std::invalid_argument("fillet.target_radius must be finite and positive");
    }
    if (!std::isfinite(fillet_min_radius_) || fillet_min_radius_ <= 0.0) {
        throw std::invalid_argument("fillet.min_radius must be finite and positive");
    }
    if (fillet_min_radius_ > fillet_target_radius_) {
        throw std::invalid_argument("fillet.min_radius must not exceed fillet.target_radius");
    }
    if (!std::isfinite(fillet_path_resolution_) || fillet_path_resolution_ <= 0.0) {
        throw std::invalid_argument("fillet.path_resolution must be finite and positive");
    }
    if (!std::isfinite(snap_distance_) || snap_distance_ <= 0.0) {
        throw std::invalid_argument("a_star.snap_distance must be finite and positive");
    }

    const auto polygon_flat = this->get_parameter("footprint.polygon").as_double_array();
    footprint_polygon_.clear();
    for (std::size_t i = 0; i + 1 < polygon_flat.size(); i += 2) {
        footprint_polygon_.emplace_back(polygon_flat[i], polygon_flat[i + 1]);
    }
    if (polygon_flat.size() % 2 != 0 || footprint_polygon_.size() < 3 ||
        !std::all_of(polygon_flat.begin(), polygon_flat.end(), [](double value) {
            return std::isfinite(value);
        })) {
        throw std::invalid_argument("footprint.polygon must contain at least 3 finite x/y pairs");
    }

    turn_max_cost_ =
        static_cast<int>(this->get_parameter("rotation_planning.turn_max_cost").as_int());
    angle_step_    = this->get_parameter("rotation_planning.angle_step").as_double();
    turn_distance_ = this->get_parameter("rotation_planning.turn_distance").as_double();

    RCLCPP_INFO(
        this->get_logger(),
        "Parameters loaded: planning_cost=%.2f, snap_distance=%.3f, "
        "eband=[iters=%d, elastic=%.2f, rep=%.3f, rep_th=%d, max_cost=%d, res=%.3f], "
        "fillet=[rdp_eps=%.3f, target_r=%.2f, min_r=%.2f, max_cost=%d, res=%.3f, blend_iters=%d], "
        "rotation=[turn_max_cost=%d, angle_step=%.2f, turn_dist=%.2f]",
        planning_cost_factor_,
        snap_distance_,
        eband_iterations_,
        eband_elastic_strength_,
        eband_repulsive_strength_,
        eband_repulsive_cost_threshold_,
        eband_max_cost_,
        eband_path_resolution_,
        fillet_rdp_epsilon_,
        fillet_target_radius_,
        fillet_min_radius_,
        fillet_max_cost_,
        fillet_path_resolution_,
        fillet_curvature_blend_iterations_,
        turn_max_cost_,
        angle_step_,
        turn_distance_
    );
}

rcl_interfaces::msg::SetParametersResult PathPlannerNode::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        const auto& name = param.get_name();

        if (name == "a_star.planning_cost") {
            double val = param.as_double();
            if (std::isfinite(val) && val >= 0.0) {
                planning_cost_factor_ = val;
            } else {
                result.successful = false;
                result.reason     = "planning_cost must be non-negative";
                return result;
            }
        } else if (name == "a_star.snap_distance") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0) {
                snap_distance_ = val;
            } else {
                result.successful = false;
                result.reason     = "snap_distance must be positive";
                return result;
            }
        } else if (name == "elastic_band.iterations") {
            int val = static_cast<int>(param.as_int());
            if (val >= 1 && val <= 200) {
                eband_iterations_ = val;
            } else {
                result.successful = false;
                result.reason     = "elastic_band.iterations must be in [1, 200]";
                return result;
            }
        } else if (name == "elastic_band.elastic_strength") {
            double val = param.as_double();
            if (std::isfinite(val) && val >= 0.0 && val <= 1.0) {
                eband_elastic_strength_ = val;
            } else {
                result.successful = false;
                result.reason     = "elastic_band.elastic_strength must be in [0.0, 1.0]";
                return result;
            }
        } else if (name == "elastic_band.repulsive_strength") {
            double val = param.as_double();
            if (std::isfinite(val) && val >= 0.0) {
                eband_repulsive_strength_ = val;
            } else {
                result.successful = false;
                result.reason     = "elastic_band.repulsive_strength must be non-negative";
                return result;
            }
        } else if (name == "elastic_band.repulsive_cost_threshold") {
            int val = static_cast<int>(param.as_int());
            if (val >= 0 && val <= 99) {
                eband_repulsive_cost_threshold_ = val;
            } else {
                result.successful = false;
                result.reason     = "elastic_band.repulsive_cost_threshold must be in [0, 99]";
                return result;
            }
        } else if (name == "elastic_band.max_cost") {
            int val = static_cast<int>(param.as_int());
            if (val >= 0 && val <= 99) {
                eband_max_cost_ = val;
            } else {
                result.successful = false;
                result.reason     = "elastic_band.max_cost must be in [0, 99]";
                return result;
            }
        } else if (name == "elastic_band.path_resolution") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0) {
                eband_path_resolution_ = val;
            } else {
                result.successful = false;
                result.reason     = "elastic_band.path_resolution must be positive";
                return result;
            }
        } else if (name == "fillet.rdp_epsilon") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0) {
                fillet_rdp_epsilon_ = val;
            } else {
                result.successful = false;
                result.reason     = "fillet.rdp_epsilon must be positive";
                return result;
            }
        } else if (name == "fillet.target_radius") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0 && val >= fillet_min_radius_) {
                fillet_target_radius_ = val;
            } else {
                result.successful = false;
                result.reason     = "fillet.target_radius must be positive and >= min_radius";
                return result;
            }
        } else if (name == "fillet.min_radius") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0 && val <= fillet_target_radius_) {
                fillet_min_radius_ = val;
            } else {
                result.successful = false;
                result.reason     = "fillet.min_radius must be positive and <= target_radius";
                return result;
            }
        } else if (name == "fillet.max_cost") {
            int val = static_cast<int>(param.as_int());
            if (val >= 0 && val <= 99) {
                fillet_max_cost_ = val;
            } else {
                result.successful = false;
                result.reason     = "fillet.max_cost must be in [0, 99]";
                return result;
            }
        } else if (name == "fillet.path_resolution") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0) {
                fillet_path_resolution_ = val;
            } else {
                result.successful = false;
                result.reason     = "fillet.path_resolution must be positive";
                return result;
            }
        } else if (name == "fillet.curvature_blend_iterations") {
            int val = static_cast<int>(param.as_int());
            if (val >= 0 && val <= 20) {
                fillet_curvature_blend_iterations_ = val;
            } else {
                result.successful = false;
                result.reason     = "fillet.curvature_blend_iterations must be in [0, 20]";
                return result;
            }
        } else if (name == "rotation_planning.turn_max_cost") {
            int val = static_cast<int>(param.as_int());
            if (val >= 0 && val <= 100) {
                turn_max_cost_ = val;
            } else {
                result.successful = false;
                result.reason     = "turn_max_cost must be in [0, 100]";
                return result;
            }
        } else if (name == "rotation_planning.angle_step") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0) {
                angle_step_ = val;
            } else {
                result.successful = false;
                result.reason     = "angle_step must be positive";
                return result;
            }
        } else if (name == "rotation_planning.turn_distance") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0) {
                turn_distance_ = val;
            } else {
                result.successful = false;
                result.reason     = "turn_distance must be positive";
                return result;
            }
        } else if (name == "footprint.polygon") {
            const auto polygon_flat = param.as_double_array();
            std::vector<std::pair<double, double>> new_poly;
            for (std::size_t i = 0; i + 1 < polygon_flat.size(); i += 2) {
                new_poly.emplace_back(polygon_flat[i], polygon_flat[i + 1]);
            }
            if (polygon_flat.size() % 2 == 0 && new_poly.size() >= 3 &&
                std::all_of(polygon_flat.begin(), polygon_flat.end(), [](double value) {
                    return std::isfinite(value);
                })) {
                footprint_polygon_ = new_poly;
            } else {
                result.successful = false;
                result.reason     = "footprint.polygon must contain at least 3 finite x/y pairs";
                return result;
            }
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Parameter '%s' updated to: %s",
            name.c_str(),
            param.value_to_string().c_str()
        );
    }
    return result;
}

void PathPlannerNode::onCostmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    latest_costmap_ = *msg;
    has_costmap_    = true;
}

std::pair<int, int> PathPlannerNode::worldToGrid(double wx, double wy) const
{
    double res = latest_costmap_.info.resolution;
    int gx     = static_cast<int>(std::floor((wx - latest_costmap_.info.origin.position.x) / res));
    int gy     = static_cast<int>(std::floor((wy - latest_costmap_.info.origin.position.y) / res));
    return {gx, gy};
}

std::pair<double, double> PathPlannerNode::gridToWorld(int gx, int gy) const
{
    double res = latest_costmap_.info.resolution;
    double wx  = latest_costmap_.info.origin.position.x + (gx + 0.5) * res;
    double wy  = latest_costmap_.info.origin.position.y + (gy + 0.5) * res;
    return {wx, wy};
}

bool PathPlannerNode::isInsideGrid(int gx, int gy) const
{
    return gx >= 0 && gx < static_cast<int>(latest_costmap_.info.width) && gy >= 0 &&
           gy < static_cast<int>(latest_costmap_.info.height);
}

bool PathPlannerNode::isPassable(int gx, int gy) const
{
    if (!isInsideGrid(gx, gy)) {
        return false;
    }
    int idx     = gy * static_cast<int>(latest_costmap_.info.width) + gx;
    int8_t cost = latest_costmap_.data[idx];
    // 0〜99 が通行可能領域、100は不可侵、負数は未知領域
    return cost >= 0 && cost < 100;
}

int PathPlannerNode::getCellCostWorld(double wx, double wy) const
{
    const auto [gx, gy] = worldToGrid(wx, wy);
    if (!isInsideGrid(gx, gy)) {
        return 100;
    }
    const int idx     = gy * static_cast<int>(latest_costmap_.info.width) + gx;
    const int8_t cost = latest_costmap_.data[idx];
    return (cost >= 0) ? static_cast<int>(cost) : 100;
}

std::optional<std::pair<int, int>> PathPlannerNode::findNearestPassable(
    int gx, int gy, int max_radius_cells
) const
{
    if (isPassable(gx, gy)) {
        return std::make_pair(gx, gy);
    }

    double best_dist_sq                          = std::numeric_limits<double>::infinity();
    std::optional<std::pair<int, int>> best_cell = std::nullopt;

    for (int r = 1; r <= max_radius_cells; ++r) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::abs(dx) != r && std::abs(dy) != r) {
                    continue;
                }
                int nx = gx + dx;
                int ny = gy + dy;
                if (isPassable(nx, ny)) {
                    double d_sq = dx * dx + dy * dy;
                    if (d_sq < best_dist_sq) {
                        best_dist_sq = d_sq;
                        best_cell    = std::make_pair(nx, ny);
                    }
                }
            }
        }
        if (best_cell.has_value()) {
            return best_cell;
        }
    }
    return std::nullopt;
}

std::vector<std::pair<int, int>> PathPlannerNode::aStarSearch(
    int start_x, int start_y, int goal_x, int goal_y
)
{
    const int width     = static_cast<int>(latest_costmap_.info.width);
    const int height    = static_cast<int>(latest_costmap_.info.height);
    const int num_cells = width * height;

    struct AStarElement {
        int x;
        int y;
        double f;

        bool operator>(const AStarElement& other) const
        {
            return f > other.f;
        }
    };

    std::priority_queue<AStarElement, std::vector<AStarElement>, std::greater<AStarElement>>
        open_set;
    std::vector<double> g_score(num_cells, std::numeric_limits<double>::infinity());
    std::vector<int> parent_index(num_cells, -1);
    std::vector<bool> closed_set(num_cells, false);

    auto getIndex = [width](int x, int y) { return y * width + x; };

    // スタートとゴールを結ぶベクトル (Tie-breaking 用)
    const double line_dx = static_cast<double>(goal_x - start_x);
    const double line_dy = static_cast<double>(goal_y - start_y);

    auto heuristic =
        [start_x, start_y, line_dx, line_dy](int x1, int y1, int x2, int y2) -> double {
        int dx = std::abs(x1 - x2);
        int dy = std::abs(y1 - y2);
        // Octile distance
        int min_d     = std::min(dx, dy);
        int max_d     = std::max(dx, dy);
        double base_h = (max_d - min_d) + 1.41421356237 * min_d;

        // Tie-breaking: スタート-ゴール直線からの外積距離による微小バイアス
        double cur_dx = static_cast<double>(x1 - start_x);
        double cur_dy = static_cast<double>(y1 - start_y);
        double cross  = std::abs(cur_dx * line_dy - cur_dy * line_dx);
        return base_h + cross * 0.001;
    };

    int start_idx      = getIndex(start_x, start_y);
    g_score[start_idx] = 0.0;
    open_set.push({start_x, start_y, heuristic(start_x, start_y, goal_x, goal_y)});

    const int dx[8]            = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8]            = {0, 0, -1, 1, -1, 1, -1, 1};
    const double step_costs[8] = {
        1.0, 1.0, 1.0, 1.0, 1.41421356237, 1.41421356237, 1.41421356237, 1.41421356237};

    bool found = false;

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();

        int curr_idx = getIndex(current.x, current.y);
        if (closed_set[curr_idx]) {
            continue;
        }
        closed_set[curr_idx] = true;

        if (current.x == goal_x && current.y == goal_y) {
            found = true;
            break;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!isPassable(nx, ny)) {
                continue;
            }

            // 斜め移動時の角抜け防止
            if (dx[i] != 0 && dy[i] != 0) {
                if (!isPassable(current.x + dx[i], current.y) &&
                    !isPassable(current.x, current.y + dy[i])) {
                    continue;
                }
            }

            int next_idx = getIndex(nx, ny);
            if (closed_set[next_idx]) {
                continue;
            }

            int8_t costmap_val = latest_costmap_.data[next_idx];
            double edge_cost =
                step_costs[i] + static_cast<double>(costmap_val) * planning_cost_factor_;
            double tentative_g = g_score[curr_idx] + edge_cost;

            if (tentative_g < g_score[next_idx]) {
                g_score[next_idx]      = tentative_g;
                parent_index[next_idx] = curr_idx;
                double h               = heuristic(nx, ny, goal_x, goal_y);
                open_set.push({nx, ny, tentative_g + h});
            }
        }
    }

    std::vector<std::pair<int, int>> path;
    if (!found) {
        return path;
    }

    int curr = getIndex(goal_x, goal_y);
    while (curr != -1) {
        int cx = curr % width;
        int cy = curr / width;
        path.push_back({cx, cy});
        curr = parent_index[curr];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<double, double>> PathPlannerNode::smoothPathElasticBand(
    const std::vector<std::pair<int, int>>& grid_path,
    const std::pair<double, double>& exact_start,
    const std::pair<double, double>& exact_goal
) const
{
    using Point = std::pair<double, double>;
    if (grid_path.empty()) {
        return {};
    }

    const double point_resolution = std::max(0.01, eband_path_resolution_);
    const double res =
        (latest_costmap_.info.resolution > 0.0) ? latest_costmap_.info.resolution : 0.05;
    const double delta = std::max(0.01, res);

    if (grid_path.size() <= 2) {
        const double dist =
            std::hypot(exact_goal.first - exact_start.first, exact_goal.second - exact_start.second);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / point_resolution)));
        std::vector<Point> pts;
        pts.reserve(steps + 1);
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            pts.emplace_back(
                exact_start.first + t * (exact_goal.first - exact_start.first),
                exact_start.second + t * (exact_goal.second - exact_start.second)
            );
        }
        return pts;
    }

    // 1. グリッドパスからワールド座標列を生成
    std::vector<Point> raw_points;
    raw_points.reserve(grid_path.size());
    for (const auto& cell : grid_path) {
        raw_points.push_back(gridToWorld(cell.first, cell.second));
    }
    raw_points.front() = exact_start;
    raw_points.back()  = exact_goal;

    // 重複・微小セグメント（1mm未満）の除去
    std::vector<Point> unique_pts;
    unique_pts.reserve(raw_points.size());
    for (const auto& p : raw_points) {
        if (unique_pts.empty() ||
            std::hypot(p.first - unique_pts.back().first, p.second - unique_pts.back().second) >
                1e-3) {
            unique_pts.push_back(p);
        }
    }
    if (unique_pts.size() <= 2) {
        unique_pts = {exact_start, exact_goal};
    } else {
        unique_pts.front() = exact_start;
        unique_pts.back()  = exact_goal;
    }

    // 2. 初期バンドの等間隔リサンプリング (折れ線に沿って point_resolution 刻み)
    std::vector<Point> band;
    band.push_back(unique_pts.front());
    for (std::size_t i = 0; i + 1 < unique_pts.size(); ++i) {
        const auto& p0       = unique_pts[i];
        const auto& p1       = unique_pts[i + 1];
        const double seg_len = std::hypot(p1.first - p0.first, p1.second - p0.second);
        if (seg_len < 1e-4) {
            continue;
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / point_resolution)));
        for (int s = 1; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            band.emplace_back(
                p0.first + t * (p1.first - p0.first), p0.second + t * (p1.second - p0.second)
            );
        }
    }
    band.front() = exact_start;
    band.back()  = exact_goal;

    if (band.size() <= 2) {
        return band;
    }

    // 3. コスト勾配算出ヘルパー (中心差分法)
    auto getCostGradient = [&](double wx, double wy) -> std::pair<double, double> {
        const double c_right = getCellCostWorld(wx + delta, wy);
        const double c_left  = getCellCostWorld(wx - delta, wy);
        const double c_top   = getCellCostWorld(wx, wy + delta);
        const double c_bot   = getCellCostWorld(wx, wy - delta);
        const double gx      = (c_right - c_left) / (2.0 * delta);
        const double gy      = (c_top - c_bot) / (2.0 * delta);
        return {gx, gy};
    };

    // 4. Elastic Band 連続緩和反復ループ
    const int total_iters = std::max(1, eband_iterations_);
    for (int iter = 0; iter < total_iters; ++iter) {
        const auto prev = band;
        for (std::size_t i = 1; i + 1 < band.size(); ++i) {
            // (a) 内部張力 (曲率最小化・平滑化力): 中点へ引き戻すスプリング力 (マルチスケール)
            double f_elastic_x =
                0.5 * (prev[i - 1].first + prev[i + 1].first) - prev[i].first;
            double f_elastic_y =
                0.5 * (prev[i - 1].second + prev[i + 1].second) - prev[i].second;

            if (i >= 2 && i + 2 < band.size()) {
                const double f2_x = 0.5 * (prev[i - 2].first + prev[i + 2].first) - prev[i].first;
                const double f2_y = 0.5 * (prev[i - 2].second + prev[i + 2].second) - prev[i].second;
                f_elastic_x = 0.6 * f_elastic_x + 0.4 * f2_x;
                f_elastic_y = 0.6 * f_elastic_y + 0.4 * f2_y;
            }
            f_elastic_x *= eband_elastic_strength_;
            f_elastic_y *= eband_elastic_strength_;

            // (b) コストマップ反発力 (-∇Cost): 障害物ポテンシャルから離れる方向の力
            const int current_cost = getCellCostWorld(prev[i].first, prev[i].second);
            double f_rep_x         = 0.0;
            double f_rep_y         = 0.0;

            if (current_cost > eband_repulsive_cost_threshold_) {
                const auto [gx, gy] = getCostGradient(prev[i].first, prev[i].second);
                const double g_norm = std::hypot(gx, gy);
                if (g_norm > 1e-4) {
                    const double cost_excess =
                        static_cast<double>(current_cost - eband_repulsive_cost_threshold_) /
                        std::max(1.0, 100.0 - eband_repulsive_cost_threshold_);
                    const double rep_mag =
                        eband_repulsive_strength_ * std::clamp(cost_excess, 0.0, 1.0) * delta;
                    // 負の勾配方向 (下り坂) へ反発
                    f_rep_x = -rep_mag * (gx / g_norm);
                    f_rep_y = -rep_mag * (gy / g_norm);
                }
            }

            // (c) 合力とステップ長制限
            double dx             = f_elastic_x + f_rep_x;
            double dy             = f_elastic_y + f_rep_y;
            const double step_len = std::hypot(dx, dy);
            const double max_step = delta;
            if (step_len > max_step) {
                dx = dx * (max_step / step_len);
                dy = dy * (max_step / step_len);
            }

            // (d) 安全性チェック (許容最大コストを超えない範囲で段階的に試行)
            Point candidate{prev[i].first + dx, prev[i].second + dy};
            int cand_cost = getCellCostWorld(candidate.first, candidate.second);

            if (cand_cost >= 0 && cand_cost < 100 && cand_cost <= eband_max_cost_) {
                band[i] = candidate;
            } else {
                // 半ステップでの試行 (減衰)
                candidate = {prev[i].first + 0.5 * dx, prev[i].second + 0.5 * dy};
                cand_cost = getCellCostWorld(candidate.first, candidate.second);
                if (cand_cost >= 0 && cand_cost < 100 && cand_cost <= eband_max_cost_) {
                    band[i] = candidate;
                }
                // それでも不可なら移動せず現状維持
            }
        }
    }

    // 5. 緩和後パスの等間隔リサンプリング (累積弧長による均一補間)
    std::vector<double> cum_dist(band.size(), 0.0);
    for (std::size_t i = 1; i < band.size(); ++i) {
        cum_dist[i] =
            cum_dist[i - 1] +
            std::hypot(band[i].first - band[i - 1].first, band[i].second - band[i - 1].second);
    }
    const double total_path_len = cum_dist.back();
    if (total_path_len < 1e-4) {
        return {exact_start, exact_goal};
    }

    const int final_steps =
        std::max(1, static_cast<int>(std::round(total_path_len / point_resolution)));
    std::vector<Point> smoothed_result;
    smoothed_result.reserve(final_steps + 1);
    smoothed_result.push_back(exact_start);

    std::size_t seg_idx = 0;
    for (int s = 1; s < final_steps; ++s) {
        const double target_d = (static_cast<double>(s) / final_steps) * total_path_len;
        while (seg_idx + 1 < cum_dist.size() && cum_dist[seg_idx + 1] < target_d) {
            ++seg_idx;
        }
        if (seg_idx + 1 >= band.size()) {
            break;
        }
        const double seg_d0   = cum_dist[seg_idx];
        const double seg_d1   = cum_dist[seg_idx + 1];
        const double seg_span = seg_d1 - seg_d0;
        const double t        = (seg_span > 1e-6) ? (target_d - seg_d0) / seg_span : 0.0;
        Point interp{
            band[seg_idx].first + t * (band[seg_idx + 1].first - band[seg_idx].first),
            band[seg_idx].second + t * (band[seg_idx + 1].second - band[seg_idx].second)};
        // 補間点が万一 max_cost を超える場合は安全な最も近いバンド点へクランプ
        int interp_cost = getCellCostWorld(interp.first, interp.second);
        if (interp_cost < 0 || interp_cost >= 100 || interp_cost > eband_max_cost_) {
            interp = (t < 0.5) ? band[seg_idx] : band[seg_idx + 1];
        }
        smoothed_result.push_back(interp);
    }
    smoothed_result.push_back(exact_goal);

    return smoothed_result;
}

std::vector<std::pair<double, double>> PathPlannerNode::smoothPathFillet(
    const std::vector<std::pair<double, double>>& raw_points,
    const std::pair<double, double>& exact_start,
    const std::pair<double, double>& exact_goal
) const
{
    using Point = std::pair<double, double>;
    if (raw_points.empty()) {
        return {};
    }

    const double point_resolution = std::max(0.01, fillet_path_resolution_);
    const double check_step = (latest_costmap_.info.resolution > 0.0)
                                  ? latest_costmap_.info.resolution * 0.5
                                  : 0.025;

    if (raw_points.size() <= 2) {
        const double dist =
            std::hypot(exact_goal.first - exact_start.first, exact_goal.second - exact_start.second);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / point_resolution)));
        std::vector<Point> pts;
        pts.reserve(steps + 1);
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            pts.emplace_back(
                exact_start.first + t * (exact_goal.first - exact_start.first),
                exact_start.second + t * (exact_goal.second - exact_start.second)
            );
        }
        return pts;
    }

    // 1. 重複・微小セグメント（1mm未満）の除去
    std::vector<Point> unique_pts;
    unique_pts.reserve(raw_points.size());
    for (const auto& p : raw_points) {
        if (unique_pts.empty() ||
            std::hypot(p.first - unique_pts.back().first, p.second - unique_pts.back().second) >
                1e-3) {
            unique_pts.push_back(p);
        }
    }
    if (unique_pts.size() <= 2) {
        unique_pts = {exact_start, exact_goal};
    } else {
        unique_pts.front() = exact_start;
        unique_pts.back()  = exact_goal;
    }

    // 始点・終点の折り返し除去
    if (unique_pts.size() >= 3) {
        const double v0x = unique_pts[1].first - unique_pts[0].first;
        const double v0y = unique_pts[1].second - unique_pts[0].second;
        const double v1x = unique_pts[2].first - unique_pts[1].first;
        const double v1y = unique_pts[2].second - unique_pts[1].second;
        if (v0x * v1x + v0y * v1y <= 0.0) {
            unique_pts.erase(unique_pts.begin() + 1);
        }
    }
    if (unique_pts.size() >= 3) {
        const std::size_t m = unique_pts.size();
        const double v0x    = unique_pts[m - 2].first - unique_pts[m - 3].first;
        const double v0y    = unique_pts[m - 2].second - unique_pts[m - 3].second;
        const double v1x    = unique_pts[m - 1].first - unique_pts[m - 2].first;
        const double v1y    = unique_pts[m - 1].second - unique_pts[m - 2].second;
        if (v0x * v1x + v0y * v1y <= 0.0) {
            unique_pts.erase(unique_pts.begin() + static_cast<std::ptrdiff_t>(m - 2));
        }
    }

    // 2. コストマップ制約付き Ramer-Douglas-Peucker (RDP) 法による大域的コーナー抽出
    // Elastic Band により微細な階段ノイズは除去されているため、真の大域コーナーのみを抽出
    const double rdp_epsilon = fillet_rdp_epsilon_;
    std::vector<bool> keep_mask(unique_pts.size(), false);
    keep_mask.front() = true;
    keep_mask.back()  = true;

    auto isSegmentSafe = [&](const Point& p1, const Point& p2) -> bool {
        const double dist = std::hypot(p2.first - p1.first, p2.second - p1.second);
        if (dist < 1e-4) {
            return true;
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / check_step)));
        for (int s = 0; s <= steps; ++s) {
            const double t  = static_cast<double>(s) / steps;
            const double wx = p1.first + t * (p2.first - p1.first);
            const double wy = p1.second + t * (p2.second - p1.second);
            const int cost  = getCellCostWorld(wx, wy);
            if (cost < 0 || cost > fillet_max_cost_ || cost >= 100) {
                return false;
            }
        }
        return true;
    };

    auto perpDist = [](const Point& pt, const Point& line_start, const Point& line_end) -> double {
        const double dx  = line_end.first - line_start.first;
        const double dy  = line_end.second - line_start.second;
        const double len = std::hypot(dx, dy);
        if (len < 1e-9) {
            return std::hypot(pt.first - line_start.first, pt.second - line_start.second);
        }
        return std::abs(
                   dy * pt.first - dx * pt.second + line_end.first * line_start.second -
                   line_end.second * line_start.first
               ) /
               len;
    };

    std::function<void(std::size_t, std::size_t)> rdpFunc = [&](std::size_t s, std::size_t e) {
        if (e <= s + 1) {
            return;
        }
        double max_d    = 0.0;
        std::size_t idx = s;
        for (std::size_t i = s + 1; i < e; ++i) {
            const double d = perpDist(unique_pts[i], unique_pts[s], unique_pts[e]);
            if (d > max_d) {
                max_d = d;
                idx   = i;
            }
        }
        if (max_d > rdp_epsilon || !isSegmentSafe(unique_pts[s], unique_pts[e])) {
            if (idx == s) {
                idx = s + (e - s) / 2;
            }
            keep_mask[idx] = true;
            rdpFunc(s, idx);
            rdpFunc(idx, e);
        }
    };
    rdpFunc(0, unique_pts.size() - 1);

    std::vector<Point> waypoints;
    waypoints.reserve(unique_pts.size());
    for (std::size_t i = 0; i < unique_pts.size(); ++i) {
        if (keep_mask[i]) {
            waypoints.push_back(unique_pts[i]);
        }
    }

    // ほぼ一直線上に並ぶ連続微小屈曲 (偏角 約5度未満) の整理
    if (waypoints.size() >= 3) {
        std::vector<Point> filtered;
        filtered.reserve(waypoints.size());
        filtered.push_back(waypoints.front());
        for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
            const double d1x = waypoints[i].first - filtered.back().first;
            const double d1y = waypoints[i].second - filtered.back().second;
            const double d2x = waypoints[i + 1].first - waypoints[i].first;
            const double d2y = waypoints[i + 1].second - waypoints[i].second;
            const double l1  = std::hypot(d1x, d1y);
            const double l2  = std::hypot(d2x, d2y);
            if (l1 > 1e-4 && l2 > 1e-4) {
                const double dot = (d1x * d2x + d1y * d2y) / (l1 * l2);
                if (dot > 0.996 && isSegmentSafe(filtered.back(), waypoints[i + 1])) {
                    continue;
                }
            }
            filtered.push_back(waypoints[i]);
        }
        filtered.push_back(waypoints.back());
        waypoints = std::move(filtered);
    }

    if (waypoints.size() <= 2) {
        const double dist =
            std::hypot(exact_goal.first - exact_start.first, exact_goal.second - exact_start.second);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / point_resolution)));
        std::vector<Point> pts;
        pts.reserve(steps + 1);
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            pts.emplace_back(
                exact_start.first + t * (exact_goal.first - exact_start.first),
                exact_start.second + t * (exact_goal.second - exact_start.second)
            );
        }
        return pts;
    }

    // 3. 各コーナーへの幾何学的円弧フィレット（大円弧化による曲率低減）
    struct CornerFillet {
        Point t1;                 // 進入接点
        Point t2;                 // 退出接点
        Point center;             // 円弧中心
        double radius{0.0};       // 旋回半径
        double angle_start{0.0};  // 始点角度
        double angle_sweep{0.0};  // 符号付き開角 (正: 左回り, 負: 右回り)
        bool has_arc{false};
    };

    std::vector<CornerFillet> fillets(waypoints.size());

    for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
        const auto& p_prev = waypoints[i - 1];
        const auto& p_curr = waypoints[i];
        const auto& p_next = waypoints[i + 1];

        const double d1x  = p_curr.first - p_prev.first;
        const double d1y  = p_curr.second - p_prev.second;
        const double d2x  = p_next.first - p_curr.first;
        const double d2y  = p_next.second - p_curr.second;
        const double len1 = std::hypot(d1x, d1y);
        const double len2 = std::hypot(d2x, d2y);

        if (len1 < 1e-4 || len2 < 1e-4) {
            continue;
        }

        const double u1x = d1x / len1;
        const double u1y = d1y / len1;
        const double u2x = d2x / len2;
        const double u2y = d2y / len2;

        const double dot   = std::clamp(u1x * u2x + u1y * u2y, -1.0, 1.0);
        const double cross = u1x * u2y - u1y * u2x;  // 正: 左旋回, 負: 右旋回
        const double theta = std::atan2(cross, dot);

        // ほぼ直線 (約 2.5 度未満) または 178度以上の急反転折り返しはフィレット対象外
        if (std::abs(theta) < 0.045 || std::abs(theta) > 3.10) {
            continue;
        }

        const double half_theta = std::abs(theta) * 0.5;
        const double tan_half   = std::tan(half_theta);
        if (tan_half < 1e-6) {
            continue;
        }

        // セグメント長を侵食しない最大接線長
        const double max_tangent = 0.45 * std::min(len1, len2);
        const double geom_max_r  = max_tangent / tan_half;
        const double target_r    = std::min(fillet_target_radius_, geom_max_r);

        // 円弧上の安全性を検証するヘルパー
        auto checkArcSafety = [&](double r, CornerFillet& out_fillet) -> bool {
            if (r < 1e-3) {
                return false;
            }
            const double t = r * tan_half;
            const Point pt_t1{p_curr.first - t * u1x, p_curr.second - t * u1y};
            const Point pt_t2{p_curr.first + t * u2x, p_curr.second + t * u2y};

            const double nx = (theta > 0.0) ? -u1y : u1y;
            const double ny = (theta > 0.0) ? u1x : -u1x;

            const Point center{pt_t1.first + r * nx, pt_t1.second + r * ny};

            const double phi1  = std::atan2(pt_t1.second - center.second, pt_t1.first - center.first);
            const double sweep = theta;

            const double arc_len = r * std::abs(theta);
            const int samples    = std::max(4, static_cast<int>(std::ceil(arc_len / check_step)));

            for (int s = 0; s <= samples; ++s) {
                const double ratio = static_cast<double>(s) / samples;
                const double ang   = phi1 + ratio * sweep;
                const double wx    = center.first + r * std::cos(ang);
                const double wy    = center.second + r * std::sin(ang);
                const int cost     = getCellCostWorld(wx, wy);
                if (cost < 0 || cost > fillet_max_cost_ || cost >= 100) {
                    return false;
                }
            }

            out_fillet.t1          = pt_t1;
            out_fillet.t2          = pt_t2;
            out_fillet.center      = center;
            out_fillet.radius      = r;
            out_fillet.angle_start = phi1;
            out_fillet.angle_sweep = sweep;
            out_fillet.has_arc     = true;
            return true;
        };

        CornerFillet cand_fillet;
        if (checkArcSafety(target_r, cand_fillet)) {
            fillets[i] = cand_fillet;
        } else {
            // 障害物近傍での安全半径を決定論的二分探索
            double r_low  = std::min(target_r, fillet_min_radius_);
            double r_high = target_r;
            CornerFillet best_fillet;
            bool found_safe = false;

            if (checkArcSafety(r_low, best_fillet)) {
                found_safe = true;
                for (int iter = 0; iter < 7; ++iter) {
                    const double r_mid = 0.5 * (r_low + r_high);
                    CornerFillet mid_fillet;
                    if (checkArcSafety(r_mid, mid_fillet)) {
                        best_fillet = mid_fillet;
                        r_low       = r_mid;
                    } else {
                        r_high = r_mid;
                    }
                }
            }
            if (found_safe) {
                fillets[i] = best_fillet;
            }
        }
    }

    // 4. 直線セグメントと円弧セグメントを結合
    std::vector<Point> smoothed_result;
    std::vector<std::size_t> junction_indices;

    auto addLine = [&](const Point& p1, const Point& p2) {
        const double dist = std::hypot(p2.first - p1.first, p2.second - p1.second);
        if (dist < 1e-4) {
            return;
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / point_resolution)));
        for (int s = 1; s <= steps; ++s) {
            const double frac = static_cast<double>(s) / steps;
            smoothed_result.emplace_back(
                p1.first + frac * (p2.first - p1.first), p1.second + frac * (p2.second - p1.second)
            );
        }
    };

    auto addArc = [&](const CornerFillet& f) {
        const double arc_len = f.radius * std::abs(f.angle_sweep);
        const int steps      = std::max(1, static_cast<int>(std::ceil(arc_len / point_resolution)));
        for (int s = 1; s <= steps; ++s) {
            const double frac = static_cast<double>(s) / steps;
            const double ang  = f.angle_start + frac * f.angle_sweep;
            smoothed_result.emplace_back(
                f.center.first + f.radius * std::cos(ang),
                f.center.second + f.radius * std::sin(ang)
            );
        }
    };

    smoothed_result.push_back(waypoints.front());

    for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
        const auto& f = fillets[i];
        if (f.has_arc) {
            addLine(smoothed_result.back(), f.t1);
            junction_indices.push_back(smoothed_result.size() - 1);

            addArc(f);
            junction_indices.push_back(smoothed_result.size() - 1);
        } else {
            addLine(smoothed_result.back(), waypoints[i]);
            junction_indices.push_back(smoothed_result.size() - 1);
        }
    }
    addLine(smoothed_result.back(), waypoints.back());

    // 5. 直線と円弧の接続部 (Junction) の局所連続曲率化ブレンド
    if (fillet_curvature_blend_iterations_ > 0 && !junction_indices.empty() &&
        smoothed_result.size() >= 5) {
        std::vector<bool> blend_mask(smoothed_result.size(), false);
        const int blend_window =
            std::max(2, static_cast<int>(std::round(0.15 / point_resolution)));
        for (std::size_t j_idx : junction_indices) {
            const std::size_t start_k = (j_idx > static_cast<std::size_t>(blend_window))
                                            ? (j_idx - blend_window)
                                            : 1;
            const std::size_t end_k =
                std::min(smoothed_result.size() - 2, j_idx + blend_window);
            for (std::size_t k = start_k; k <= end_k; ++k) {
                blend_mask[k] = true;
            }
        }

        for (int iter = 0; iter < fillet_curvature_blend_iterations_; ++iter) {
            const auto prev = smoothed_result;
            for (std::size_t k = 1; k + 1 < smoothed_result.size(); ++k) {
                if (!blend_mask[k]) {
                    continue;
                }
                const Point candidate{
                    0.25 * prev[k - 1].first + 0.50 * prev[k].first + 0.25 * prev[k + 1].first,
                    0.25 * prev[k - 1].second + 0.50 * prev[k].second + 0.25 * prev[k + 1].second};
                const int cost = getCellCostWorld(candidate.first, candidate.second);
                if (cost >= 0 && cost <= fillet_max_cost_ && cost < 100) {
                    smoothed_result[k] = candidate;
                }
            }
        }
    }

    smoothed_result.front() = exact_start;
    smoothed_result.back()  = exact_goal;
    return smoothed_result;
}

std::vector<std::pair<double, double>> PathPlannerNode::smoothPath(
    const std::vector<std::pair<int, int>>& grid_path,
    const std::pair<double, double>& exact_start,
    const std::pair<double, double>& exact_goal
) const
{
    // 第1段: コスト勾配による連続緩和 (Elastic Band)
    // 5cm格子の階段ノイズを除去し、障害物ポテンシャルに沿った滑らかな大局経路を生成
    const auto eb_path = smoothPathElasticBand(grid_path, exact_start, exact_goal);
    if (eb_path.size() <= 2) {
        return eb_path;
    }

    // 第2段: 決定論的幾何コーナーフィレット
    // 滑らかになった実座標経路から大域コーナーを抽出し、目標半径 target_radius の円弧で曲率を低減
    return smoothPathFillet(eb_path, exact_start, exact_goal);
}

int PathPlannerNode::getFootprintMaxCost(double wx, double wy, double yaw, int cutoff_cost) const
{
    if (footprint_polygon_.empty() || !has_costmap_) {
        return 100;
    }

    const int center_cost = getCellCostWorld(wx, wy);
    if (center_cost >= 100 || center_cost > cutoff_cost) {
        return center_cost;
    }
    int max_seen_cost = center_cost;

    const double cos_yaw    = std::cos(yaw);
    const double sin_yaw    = std::sin(yaw);
    const int num_vertices  = static_cast<int>(footprint_polygon_.size());
    const double resolution = latest_costmap_.info.resolution;

    std::vector<std::pair<double, double>> world_poly(num_vertices);
    for (int i = 0; i < num_vertices; ++i) {
        const double lx = footprint_polygon_[i].first;
        const double ly = footprint_polygon_[i].second;
        world_poly[i]   = {wx + lx * cos_yaw - ly * sin_yaw, wy + lx * sin_yaw + ly * cos_yaw};
    }

    for (int i = 0; i < num_vertices; ++i) {
        const auto& p1    = world_poly[i];
        const auto& p2    = world_poly[(i + 1) % num_vertices];
        const double dist = std::hypot(p2.first - p1.first, p2.second - p1.second);
        const int steps   = std::max(1, static_cast<int>(std::ceil(dist / (0.5 * resolution))));

        for (int s = 0; s <= steps; ++s) {
            const double frac = static_cast<double>(s) / steps;
            const double sx   = p1.first + frac * (p2.first - p1.first);
            const double sy   = p1.second + frac * (p2.second - p1.second);
            const int cost    = getCellCostWorld(sx, sy);
            if (cost > max_seen_cost) {
                max_seen_cost = cost;
                if (max_seen_cost >= 100 || max_seen_cost > cutoff_cost) {
                    return max_seen_cost;
                }
            }
        }
    }

    return max_seen_cost;
}

bool PathPlannerNode::isFootprintSafe(double wx, double wy, double yaw, int max_cost) const
{
    return getFootprintMaxCost(wx, wy, yaw, max_cost) <= max_cost;
}

std::vector<geometry_msgs::msg::PoseStamped> PathPlannerNode::planOrientations(
    const std::vector<std::pair<double, double>>& smoothed_path, double start_yaw, double goal_yaw
) const
{
    const std::size_t n = smoothed_path.size();
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    if (n == 0) {
        return poses;
    }

    if (n == 1) {
        poses.resize(1);
        poses[0].header.stamp    = this->now();
        poses[0].header.frame_id = map_frame_;
        poses[0].pose.position.x = smoothed_path[0].first;
        poses[0].pose.position.y = smoothed_path[0].second;
        poses[0].pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, goal_yaw);
        poses[0].pose.orientation = tf2::toMsg(q);
        return poses;
    }

    // 累積移動距離を計算
    std::vector<double> accum_dist(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
        accum_dist[i] =
            accum_dist[i - 1] + std::hypot(
                                    smoothed_path[i].first - smoothed_path[i - 1].first,
                                    smoothed_path[i].second - smoothed_path[i - 1].second
                                );
    }
    const double total_dist = accum_dist.back();
    const double delta_yaw  = normalizeAngle(goal_yaw - start_yaw);

    // 旋回量がほぼゼロの場合は全域 goal_yaw とする
    if (std::abs(delta_yaw) <= 1e-3 || total_dist <= 1e-4) {
        poses.resize(n);
        const auto stamp = this->now();
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, goal_yaw);
        const auto q_msg = tf2::toMsg(q);
        for (std::size_t i = 0; i < n; ++i) {
            poses[i].header.stamp     = stamp;
            poses[i].header.frame_id  = map_frame_;
            poses[i].pose.position.x  = smoothed_path[i].first;
            poses[i].pose.position.y  = smoothed_path[i].second;
            poses[i].pose.position.z  = 0.0;
            poses[i].pose.orientation = q_msg;
        }
        return poses;
    }

    const double step_angle = std::max(0.01, angle_step_);
    const int yaw_check_steps =
        std::max(2, static_cast<int>(std::ceil(std::abs(delta_yaw) / step_angle)));
    const double desired_turn_dist =
        std::clamp(turn_distance_, 0.2, std::max(0.2, total_dist * 0.8));

    // 各パス点において、start_yaw〜goal_yaw の全回転角度で生じる最大フットプリントコストを算出
    // これにより、旋回中に機体がパスから多少ずれたり回転タイミングが前後したりしても
    // 旋回区間全域にわたって障害物と干渉しない最も広い安全領域を探索する。
    std::vector<int> sweep_cost(n, 100);
    for (std::size_t i = 0; i < n; ++i) {
        const auto [wx, wy]  = smoothed_path[i];
        int worst_angle_cost = 0;
        for (int k = 0; k <= yaw_check_steps; ++k) {
            const double frac     = static_cast<double>(k) / yaw_check_steps;
            const double test_yaw = normalizeAngle(start_yaw + frac * delta_yaw);
            const int c           = getFootprintMaxCost(wx, wy, test_yaw, 99);
            worst_angle_cost      = std::max(worst_angle_cost, c);
            if (worst_angle_cost >= 100) {
                break;
            }
        }
        sweep_cost[i] = worst_angle_cost;
    }

    // 長さ desired_turn_dist の旋回区間ウィンドウ [s_idx, e_idx] を走査し、
    // 1. 区間内の最大フットプリントコスト (window_max_cost) が turn_max_cost_ 以下であるか
    // 2. 区間内の最大コストおよび平均コストが最小（最も障害物から離れた広い安全領域）であるか
    // を基準に最適な旋回区間を決定する。
    double best_start_dist = 0.0;
    double best_end_dist   = std::min(total_dist, desired_turn_dist);
    int best_window_max    = std::numeric_limits<int>::max();
    double best_window_avg = std::numeric_limits<double>::infinity();
    bool found_safe_window = false;

    std::size_t e_idx = 0;
    for (std::size_t s_idx = 0; s_idx < n; ++s_idx) {
        while (e_idx + 1 < n && (accum_dist[e_idx] - accum_dist[s_idx]) < desired_turn_dist) {
            ++e_idx;
        }
        if ((accum_dist[e_idx] - accum_dist[s_idx]) + 1e-3 < desired_turn_dist && s_idx > 0) {
            break;
        }

        int win_max             = 0;
        double win_sum          = 0.0;
        const std::size_t count = e_idx - s_idx + 1;
        for (std::size_t k = s_idx; k <= e_idx; ++k) {
            win_max = std::max(win_max, sweep_cost[k]);
            win_sum += static_cast<double>(sweep_cost[k]);
        }
        const double win_avg = win_sum / static_cast<double>(count);
        const bool is_safe   = (win_max <= turn_max_cost_);

        if (is_safe) {
            // turn_max_cost_
            // 以下を満たす安全領域の中では、平均コストが最も低い（最も広い）区間を選び、
            // 同等の広さであればゴールに近い側（>=）を優先する
            if (!found_safe_window || win_avg <= best_window_avg + 1.0) {
                found_safe_window = true;
                best_window_max   = win_max;
                best_window_avg   = win_avg;
                best_start_dist   = accum_dist[s_idx];
                best_end_dist     = accum_dist[e_idx];
            }
        } else if (!found_safe_window) {
            if (win_max < best_window_max ||
                (win_max == best_window_max && win_avg <= best_window_avg)) {
                best_window_max = win_max;
                best_window_avg = win_avg;
                best_start_dist = accum_dist[s_idx];
                best_end_dist   = accum_dist[e_idx];
            }
        }
    }

    if (found_safe_window) {
        RCLCPP_INFO(
            this->get_logger(),
            "Selected safe rotation zone [%.2f m, %.2f m] (max_cost=%d <= %d, avg_cost=%.1f).",
            best_start_dist,
            best_end_dist,
            best_window_max,
            turn_max_cost_,
            best_window_avg
        );
    } else {
        RCLCPP_WARN(
            this->get_logger(),
            "No rotation zone with footprint cost <= %d found. Using safest available zone [%.2f m, %.2f m] (max_cost=%d, avg_cost=%.1f).",
            turn_max_cost_,
            best_start_dist,
            best_end_dist,
            best_window_max,
            best_window_avg
        );
    }

    double turn_start_dist = best_start_dist;
    double turn_end_dist   = best_end_dist;
    if (turn_end_dist <= turn_start_dist + 1e-4) {
        turn_start_dist = 0.0;
        turn_end_dist   = total_dist;
    }

    poses.resize(n);
    const auto stamp = this->now();
    for (std::size_t i = 0; i < n; ++i) {
        poses[i].header.stamp    = stamp;
        poses[i].header.frame_id = map_frame_;
        poses[i].pose.position.x = smoothed_path[i].first;
        poses[i].pose.position.y = smoothed_path[i].second;
        poses[i].pose.position.z = 0.0;

        const double d = accum_dist[i];
        double yaw     = start_yaw;

        if (d <= turn_start_dist) {
            yaw = start_yaw;
        } else if (d >= turn_end_dist) {
            yaw = goal_yaw;
        } else {
            // S字（Smoothstep）補間: 3t^2 - 2t^3
            const double t        = (d - turn_start_dist) / (turn_end_dist - turn_start_dist);
            const double smooth_t = t * t * (3.0 - 2.0 * t);
            yaw                   = normalizeAngle(start_yaw + smooth_t * delta_yaw);
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        poses[i].pose.orientation = tf2::toMsg(q);
    }

    // 最終ポーズは確実に goal_yaw と厳密一致させる
    tf2::Quaternion goal_q;
    goal_q.setRPY(0.0, 0.0, goal_yaw);
    poses.back().pose.orientation = tf2::toMsg(goal_q);

    return poses;
}

void PathPlannerNode::onGoalPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    if (!has_costmap_) {
        RCLCPP_WARN(this->get_logger(), "Costmap not received yet. Cannot plan path.");
        return;
    }

    // 1. ロボット現在位置をTFから取得
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not transform %s to %s: %s",
            map_frame_.c_str(),
            base_frame_.c_str(),
            ex.what()
        );
        return;
    }

    double start_wx = tf_stamped.transform.translation.x;
    double start_wy = tf_stamped.transform.translation.y;
    double goal_wx  = msg->pose.position.x;
    double goal_wy  = msg->pose.position.y;

    auto [start_gx, start_gy] = worldToGrid(start_wx, start_wy);
    auto [goal_gx, goal_gy]   = worldToGrid(goal_wx, goal_wy);

    // 2. スタートとゴールの通行可能性チェック＆スナップ
    const int snap_radius_cells =
        static_cast<int>(std::ceil(snap_distance_ / latest_costmap_.info.resolution));

    auto snapped_start = findNearestPassable(start_gx, start_gy, snap_radius_cells);
    if (!snapped_start.has_value()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Start pose (%d, %d) is inside obstacle and cannot be snapped.",
            start_gx,
            start_gy
        );
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp    = this->now();
        empty_path.header.frame_id = map_frame_;
        path_pub_->publish(empty_path);
        return;
    }

    auto snapped_goal = findNearestPassable(goal_gx, goal_gy, snap_radius_cells);
    if (!snapped_goal.has_value()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Goal pose (%d, %d) is inside obstacle and cannot be snapped.",
            goal_gx,
            goal_gy
        );
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp    = this->now();
        empty_path.header.frame_id = map_frame_;
        path_pub_->publish(empty_path);
        return;
    }

    int actual_start_x = snapped_start->first;
    int actual_start_y = snapped_start->second;
    int actual_goal_x  = snapped_goal->first;
    int actual_goal_y  = snapped_goal->second;

    const bool goal_was_snapped = (actual_goal_x != goal_gx || actual_goal_y != goal_gy);

    // 障害物スナップが発生していない場合は元の連続ワールド座標を厳密に保持し、
    // ゴールが障害物・インフレーション内にめり込んでいる（スナップされた）場合は、
    // 安全通行可能セル中心を「アプローチ中継点」としてA*探索・平滑化を行い、
    // 平滑化経路の終端に本来の目標地点への直線進入アプローチ区間を付加する。
    const std::pair<double, double> exact_start =
        (actual_start_x == start_gx && actual_start_y == start_gy)
            ? std::make_pair(start_wx, start_wy)
            : gridToWorld(actual_start_x, actual_start_y);
    const std::pair<double, double> approach_goal = goal_was_snapped
                                                        ? gridToWorld(actual_goal_x, actual_goal_y)
                                                        : std::make_pair(goal_wx, goal_wy);
    const std::pair<double, double> target_goal   = std::make_pair(goal_wx, goal_wy);

    RCLCPP_INFO(
        this->get_logger(),
        "Planning path from (%.3f, %.3f) [grid (%d, %d)] to approach (%.3f, %.3f) [grid (%d, %d)] (target=(%.3f, %.3f), snapped=%s)...",
        exact_start.first,
        exact_start.second,
        actual_start_x,
        actual_start_y,
        approach_goal.first,
        approach_goal.second,
        actual_goal_x,
        actual_goal_y,
        target_goal.first,
        target_goal.second,
        goal_was_snapped ? "true" : "false"
    );

    // 3. 8方向A*探索（安全領域内）
    auto grid_path = aStarSearch(actual_start_x, actual_start_y, actual_goal_x, actual_goal_y);

    if (grid_path.empty()) {
        RCLCPP_WARN(this->get_logger(), "A* search failed to find a path.");
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp    = this->now();
        empty_path.header.frame_id = map_frame_;
        path_pub_->publish(empty_path);
        return;
    }

    // 4. コスト勾配による連続緩和 (Elastic Band) による平滑化（連続始点〜アプローチ点間）
    auto smoothed_path = smoothPath(grid_path, exact_start, approach_goal);
    if (smoothed_path.empty()) {
        RCLCPP_WARN(this->get_logger(), "Smooth path generated empty. Using grid points.");
        if (grid_path.size() <= 1) {
            smoothed_path = {exact_start, approach_goal};
        } else {
            for (const auto& cell : grid_path) {
                smoothed_path.push_back(gridToWorld(cell.first, cell.second));
            }
            smoothed_path.front() = exact_start;
            smoothed_path.back()  = approach_goal;
        }
    }

    // 5. ゴールが障害物内にめり込んでいる場合、アプローチ点から本来の目標地点への直線進入区間を付加
    if (goal_was_snapped) {
        const double dx   = target_goal.first - smoothed_path.back().first;
        const double dy   = target_goal.second - smoothed_path.back().second;
        const double dist = std::hypot(dx, dy);
        if (dist > 1e-4) {
            const double step     = (latest_costmap_.info.resolution > 0)
                                        ? latest_costmap_.info.resolution * 0.5
                                        : 0.025;
            const int steps       = std::max(1, static_cast<int>(std::ceil(dist / step)));
            const auto p_approach = smoothed_path.back();
            for (int s = 1; s <= steps; ++s) {
                const double t = static_cast<double>(s) / steps;
                smoothed_path.emplace_back(p_approach.first + t * dx, p_approach.second + t * dy);
            }
            smoothed_path.back() = target_goal;
        }
    }

    const double start_yaw = getYawFromQuaternion(tf_stamped.transform.rotation);
    const double goal_yaw  = getYawFromQuaternion(msg->pose.orientation);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = map_frame_;
    path_msg.poses           = planOrientations(smoothed_path, start_yaw, goal_yaw);

    path_pub_->publish(path_msg);
    RCLCPP_INFO(
        this->get_logger(),
        "Published planned_path with %zu points. Exact goal=(%.4f, %.4f, yaw=%.4f)",
        path_msg.poses.size(),
        path_msg.poses.back().pose.position.x,
        path_msg.poses.back().pose.position.y,
        goal_yaw
    );
}

}  // namespace gn10_navigation

#ifndef GN10_NAVIGATION_PATH_PLANNER_NO_MAIN
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif
