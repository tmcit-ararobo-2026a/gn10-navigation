/**
 * @file path_planner_node.cpp
 * @brief NHKロボコン2026向け大域経路計画実装 (8方向A* + 動的適応B-spline + 安全旋回計画)
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
 * 3. 動的適応型 B-spline 平滑化 (smoothPathAdaptiveBspline):
 *    - A*格子経路の階段状カドを取り除くため、制御点間引きステップ stride を max_stride (10) から 1
 * へ 段階的に低減しながら、反復ラプラシアン平滑化 (smoothing_iterations: 8 回) を適用。
 *    - 経路上の全サンプルの最大セルコストが smooth_max_cost (40) 以下となる最大の stride を採用。
 *    - これにより、広い空間では大きく間引いて滑らかなロングカーブを描き、
 *      狭路では自動的に細かい間引きへ適応して壁際を安全にすり抜ける。
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
    // 3. 動的適応型 B-spline 平滑化パラメータ
    // ==========================================
    // bspline.samples_per_segment: B-spline 制御点間の補間サンプル数。
    this->declare_parameter("bspline.samples_per_segment", 10);
    // bspline.smoothing_strength: 1反復あたりの内部制御点を隣接点の中点へ寄せる割合 (0.0〜1.0)。
    this->declare_parameter("bspline.smoothing_strength", 1.0);
    // bspline.smoothing_iterations: 内部制御点の反復ラプラシアン平滑化回数 (0〜50)。
    // 多いほど滑らかになり階段ノイズが消える。
    this->declare_parameter("bspline.smoothing_iterations", 8);
    // bspline.max_stride: 動的適応平滑化で試行する制御点間引きステップの最大値 (1〜50)。
    // 広い空間で大きな stride を使うことで、A* の折れ線を一気に直線・大半径円弧へ平滑化。
    this->declare_parameter("bspline.max_stride", 20);
    // bspline.smooth_max_cost: 平滑化経路が通過を許容する最大セルコスト (0〜99)。
    // このコストを超えるショートカットは棄却され、低コスト安全領域内での平滑化が保証される。
    this->declare_parameter("bspline.smooth_max_cost", 40);

    // ==========================================
    // 4. 直線近似パラメータ
    // ==========================================
    // line_approximation.max_cost: 直線近似を許容する最大セルコスト (-1: 無効, 0〜99)。
    this->declare_parameter("line_approximation.max_cost", -1);
    // line_approximation.min_distance [m]: 直線化する最小セグメント長。
    this->declare_parameter("line_approximation.min_distance", 1.2);

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
    bspline_samples_per_segment_ =
        std::max(1, static_cast<int>(this->get_parameter("bspline.samples_per_segment").as_int()));
    bspline_smoothing_strength_   = this->get_parameter("bspline.smoothing_strength").as_double();
    bspline_smoothing_iterations_ = std::clamp(
        static_cast<int>(this->get_parameter("bspline.smoothing_iterations").as_int()), 0, 50
    );
    bspline_max_stride_ =
        std::clamp(static_cast<int>(this->get_parameter("bspline.max_stride").as_int()), 1, 50);
    bspline_smooth_max_cost_ = std::clamp(
        static_cast<int>(this->get_parameter("bspline.smooth_max_cost").as_int()), 0, 99
    );
    line_approx_max_cost_ = std::clamp(
        static_cast<int>(this->get_parameter("line_approximation.max_cost").as_int()), -1, 99
    );
    line_approx_min_distance_ = this->get_parameter("line_approximation.min_distance").as_double();

    if (!std::isfinite(bspline_smoothing_strength_) || bspline_smoothing_strength_ < 0.0 ||
        bspline_smoothing_strength_ > 1.0) {
        throw std::invalid_argument("bspline.smoothing_strength must be in [0.0, 1.0]");
    }
    if (!std::isfinite(snap_distance_) || snap_distance_ <= 0.0) {
        throw std::invalid_argument("a_star.snap_distance must be finite and positive");
    }
    if (!std::isfinite(line_approx_min_distance_) || line_approx_min_distance_ <= 0.0) {
        throw std::invalid_argument("line_approximation.min_distance must be finite and positive");
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
        "bspline=[samples=%d, strength=%.2f, iters=%d, max_stride=%d, max_cost=%d], "
        "line_approx=[max_cost=%d, min_dist=%.2f], "
        "rotation=[turn_max_cost=%d, angle_step=%.2f, turn_dist=%.2f]",
        planning_cost_factor_,
        snap_distance_,
        bspline_samples_per_segment_,
        bspline_smoothing_strength_,
        bspline_smoothing_iterations_,
        bspline_max_stride_,
        bspline_smooth_max_cost_,
        line_approx_max_cost_,
        line_approx_min_distance_,
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
        } else if (name == "bspline.samples_per_segment") {
            int val = static_cast<int>(param.as_int());
            if (val >= 1) {
                bspline_samples_per_segment_ = val;
            } else {
                result.successful = false;
                result.reason     = "bspline.samples_per_segment must be >= 1";
                return result;
            }
        } else if (name == "bspline.smoothing_strength") {
            double val = param.as_double();
            if (std::isfinite(val) && val >= 0.0 && val <= 1.0) {
                bspline_smoothing_strength_ = val;
            } else {
                result.successful = false;
                result.reason     = "bspline.smoothing_strength must be in [0.0, 1.0]";
                return result;
            }
        } else if (name == "bspline.smoothing_iterations") {
            int val = static_cast<int>(param.as_int());
            if (val >= 0 && val <= 50) {
                bspline_smoothing_iterations_ = val;
            } else {
                result.successful = false;
                result.reason     = "bspline.smoothing_iterations must be in [0, 50]";
                return result;
            }
        } else if (name == "bspline.max_stride") {
            int val = static_cast<int>(param.as_int());
            if (val >= 1 && val <= 50) {
                bspline_max_stride_ = val;
            } else {
                result.successful = false;
                result.reason     = "bspline.max_stride must be in [1, 50]";
                return result;
            }
        } else if (name == "bspline.smooth_max_cost") {
            int val = static_cast<int>(param.as_int());
            if (val >= 0 && val <= 99) {
                bspline_smooth_max_cost_ = val;
            } else {
                result.successful = false;
                result.reason     = "bspline.smooth_max_cost must be in [0, 99]";
                return result;
            }
        } else if (name == "line_approximation.max_cost") {
            int val = static_cast<int>(param.as_int());
            if (val >= -1 && val <= 99) {
                line_approx_max_cost_ = val;
            } else {
                result.successful = false;
                result.reason     = "line_approximation.max_cost must be in [-1, 99]";
                return result;
            }
        } else if (name == "line_approximation.min_distance") {
            double val = param.as_double();
            if (std::isfinite(val) && val > 0.0) {
                line_approx_min_distance_ = val;
            } else {
                result.successful = false;
                result.reason     = "line_approximation.min_distance must be positive";
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

std::vector<std::pair<double, double>> PathPlannerNode::evaluateBspline(
    const std::vector<std::pair<double, double>>& input_control_points,
    double smoothing_strength,
    int smoothing_iterations,
    int allowed_max_cost
) const
{
    using Point = std::pair<double, double>;
    if (input_control_points.empty()) {
        return {};
    }
    if (input_control_points.size() <= 2) {
        // 2点以下の場合は始点・終点を厳密に結ぶ密な線形補間
        const auto& p0    = input_control_points.front();
        const auto& p1    = input_control_points.back();
        const double dist = std::hypot(p1.first - p0.first, p1.second - p0.second);
        const double step =
            (latest_costmap_.info.resolution > 0) ? latest_costmap_.info.resolution * 0.5 : 0.025;
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / step)));
        std::vector<Point> line_pts;
        line_pts.reserve(steps + 1);
        for (int s = 0; s <= steps; ++s) {
            double t = static_cast<double>(s) / steps;
            line_pts.emplace_back(
                p0.first + t * (p1.first - p0.first), p0.second + t * (p1.second - p0.second)
            );
        }
        line_pts.front() = p0;
        line_pts.back()  = p1;
        return line_pts;
    }

    // 制御点数が 4 未満なら中点を補間して 4 点以上にする
    std::vector<Point> control_points = input_control_points;
    while (control_points.size() < 4) {
        std::vector<Point> expanded;
        expanded.reserve(control_points.size() * 2);
        for (std::size_t i = 0; i + 1 < control_points.size(); ++i) {
            expanded.push_back(control_points[i]);
            expanded.push_back(
                {0.5 * (control_points[i].first + control_points[i + 1].first),
                 0.5 * (control_points[i].second + control_points[i + 1].second)}
            );
        }
        expanded.push_back(control_points.back());
        control_points = std::move(expanded);
    }

    // 内部制御点の反復ラプラシアン平滑化 (各反復でコスト制約を満たす移動のみ適用)
    const double clamped_strength = std::clamp(smoothing_strength, 0.0, 1.0);
    const int iterations          = std::max(0, smoothing_iterations);
    if (clamped_strength > 0.0 && iterations > 0) {
        for (int iter = 0; iter < iterations; ++iter) {
            const auto prev_pts = control_points;
            for (std::size_t index = 1; index + 1 < control_points.size(); ++index) {
                const auto& previous = prev_pts[index - 1];
                const auto& current  = prev_pts[index];
                const auto& next     = prev_pts[index + 1];
                const Point midpoint{
                    0.5 * (previous.first + next.first), 0.5 * (previous.second + next.second)};
                const Point candidate{
                    current.first + clamped_strength * (midpoint.first - current.first),
                    current.second + clamped_strength * (midpoint.second - current.second)};
                // 移動先および前後との中点が許容コスト以内であれば更新を適用
                if (getCellCostWorld(candidate.first, candidate.second) <= allowed_max_cost &&
                    getCellCostWorld(
                        0.5 * (previous.first + candidate.first),
                        0.5 * (previous.second + candidate.second)
                    ) <= allowed_max_cost &&
                    getCellCostWorld(
                        0.5 * (next.first + candidate.first), 0.5 * (next.second + candidate.second)
                    ) <= allowed_max_cost) {
                    control_points[index] = candidate;
                }
            }
        }
    }

    constexpr int degree    = 3;
    const int control_count = static_cast<int>(control_points.size());
    const int last_span     = control_count - degree;
    std::vector<double> knots(control_count + degree + 1);
    for (int index = 0; index < static_cast<int>(knots.size()); ++index) {
        if (index <= degree) {
            knots[index] = 0.0;
        } else if (index >= control_count) {
            knots[index] = static_cast<double>(last_span);
        } else {
            knots[index] = static_cast<double>(index - degree);
        }
    }

    const auto evaluate = [&](double parameter) {
        if (parameter <= 0.0) {
            return control_points.front();
        }
        if (parameter >= static_cast<double>(last_span)) {
            return control_points.back();
        }

        int span = degree;
        for (int index = degree; index < control_count; ++index) {
            if (parameter >= knots[index] && parameter < knots[index + 1]) {
                span = index;
                break;
            }
        }

        std::array<Point, degree + 1> values{};
        for (int offset = 0; offset <= degree; ++offset) {
            values[offset] = control_points[span - degree + offset];
        }
        for (int order = 1; order <= degree; ++order) {
            for (int offset = degree; offset >= order; --offset) {
                const int knot_index = span - degree + offset;
                const double denominator =
                    knots[knot_index + degree - order + 1] - knots[knot_index];
                double alpha = 0.0;
                if (denominator > 0.0) {
                    alpha = (parameter - knots[knot_index]) / denominator;
                }
                values[offset].first =
                    (1.0 - alpha) * values[offset - 1].first + alpha * values[offset].first;
                values[offset].second =
                    (1.0 - alpha) * values[offset - 1].second + alpha * values[offset].second;
            }
        }
        return values[degree];
    };

    const int sample_count = std::max(last_span * bspline_samples_per_segment_, 10);
    std::vector<Point> smoothed_path;
    smoothed_path.reserve(sample_count + 1);
    for (int sample = 0; sample <= sample_count; ++sample) {
        const double parameter =
            static_cast<double>(last_span) * sample / static_cast<double>(sample_count);
        smoothed_path.push_back(evaluate(parameter));
    }
    smoothed_path.front() = input_control_points.front();
    smoothed_path.back()  = input_control_points.back();
    return smoothed_path;
}

std::vector<std::pair<double, double>> PathPlannerNode::smoothPath(
    const std::vector<std::pair<int, int>>& grid_path,
    const std::pair<double, double>& exact_start,
    const std::pair<double, double>& exact_goal
) const
{
    using Point = std::pair<double, double>;
    if (grid_path.empty()) {
        return {};
    }

    // 全グリッド点のワールド座標を生成し、始点・終点はグリッド量子化前の正確なワールド座標に置換
    std::vector<Point> raw_world_points;
    raw_world_points.reserve(std::max(std::size_t{2}, grid_path.size()));
    for (const auto& cell : grid_path) {
        raw_world_points.push_back(gridToWorld(cell.first, cell.second));
    }
    // 同一セル内の微小移動時 (grid_path.size() == 1) に front と back が同じ要素となり
    // exact_goal で上書きされて始点が潰れるのを防止する
    if (raw_world_points.size() <= 1) {
        raw_world_points = {exact_start, exact_goal};
    } else {
        raw_world_points.front() = exact_start;
        raw_world_points.back()  = exact_goal;
    }

    // 始点・終点を連続座標に置換した際、隣接セル中心との間で逆向き微小折り返しが生じる場合は除去
    if (raw_world_points.size() >= 3) {
        const double v0x = raw_world_points[1].first - raw_world_points[0].first;
        const double v0y = raw_world_points[1].second - raw_world_points[0].second;
        const double v1x = raw_world_points[2].first - raw_world_points[1].first;
        const double v1y = raw_world_points[2].second - raw_world_points[1].second;
        if (v0x * v1x + v0y * v1y <= 0.0) {
            raw_world_points.erase(raw_world_points.begin() + 1);
        }
    }
    if (raw_world_points.size() >= 3) {
        const std::size_t m = raw_world_points.size();
        const double v0x    = raw_world_points[m - 2].first - raw_world_points[m - 3].first;
        const double v0y    = raw_world_points[m - 2].second - raw_world_points[m - 3].second;
        const double v1x    = raw_world_points[m - 1].first - raw_world_points[m - 2].first;
        const double v1y    = raw_world_points[m - 1].second - raw_world_points[m - 2].second;
        if (v0x * v1x + v0y * v1y <= 0.0) {
            raw_world_points.erase(raw_world_points.begin() + static_cast<std::ptrdiff_t>(m - 2));
        }
    }

    const int allowed_max_cost = std::clamp(bspline_smooth_max_cost_, 0, 99);

    if (raw_world_points.size() <= 2) {
        return evaluateBspline(raw_world_points, 0.0, 0, allowed_max_cost);
    }

    const double max_strength = std::clamp(bspline_smoothing_strength_, 0.0, 1.0);

    // 平滑化度合いを動的に変更するための候補リスト
    // (stride: 制御点の間引きステップ、strength: 内部制御点の平滑化強度)
    struct SmoothingLevel {
        int stride;
        double strength;
        int iterations;
    };

    std::vector<SmoothingLevel> levels;
    const int max_allowed_stride = std::min(
        bspline_max_stride_, std::max(1, static_cast<int>(raw_world_points.size() - 1) / 3)
    );
    const std::vector<double> ratios = {1.0, 0.7, 0.4, 0.2, 0.0};

    for (int s = max_allowed_stride; s >= 1; --s) {
        for (double r : ratios) {
            const int iters =
                (r > 0.0)
                    ? std::max(1, static_cast<int>(std::ceil(bspline_smoothing_iterations_ * r)))
                    : 0;
            levels.push_back({s, max_strength * r, iters});
        }
    }

    for (const auto& level : levels) {
        std::vector<Point> control_points;
        control_points.reserve(raw_world_points.size() / level.stride + 2);
        for (std::size_t i = 0; i < raw_world_points.size(); i += level.stride) {
            control_points.push_back(raw_world_points[i]);
        }
        if (control_points.back() != raw_world_points.back()) {
            control_points.push_back(raw_world_points.back());
        }

        auto candidate_path =
            evaluateBspline(control_points, level.strength, level.iterations, allowed_max_cost);
        if (!candidate_path.empty() && isSmoothPathValid(candidate_path, allowed_max_cost)) {
            return candidate_path;
        }
    }


    // フォールバック: 始点・終点の連続座標を維持したまま安全なA*点へクランプ
    auto base_curve = evaluateBspline(raw_world_points, 0.0, 0, 99);
    for (std::size_t i = 1; i + 1 < base_curve.size(); ++i) {
        auto& pt            = base_curve[i];
        const auto [gx, gy] = worldToGrid(pt.first, pt.second);
        if (!isPassable(gx, gy)) {
            double min_d  = std::numeric_limits<double>::max();
            Point best_pt = pt;
            for (const auto& orig : raw_world_points) {
                double d = std::hypot(pt.first - orig.first, pt.second - orig.second);
                if (d < min_d) {
                    min_d   = d;
                    best_pt = orig;
                }
            }
            pt = best_pt;
        }
    }

    if (base_curve.size() >= 3) {
        const auto temp = base_curve;
        for (std::size_t i = 1; i + 1 < base_curve.size(); ++i) {
            Point avg{
                0.25 * temp[i - 1].first + 0.5 * temp[i].first + 0.25 * temp[i + 1].first,
                0.25 * temp[i - 1].second + 0.5 * temp[i].second + 0.25 * temp[i + 1].second};
            auto [agx, agy] = worldToGrid(avg.first, avg.second);
            if (isPassable(agx, agy)) {
                base_curve[i] = avg;
            }
        }
    }

    base_curve.front() = exact_start;
    base_curve.back()  = exact_goal;
    return base_curve;
}

bool PathPlannerNode::isSmoothPathValid(
    const std::vector<std::pair<double, double>>& path, int max_allowed_cost
) const
{
    if (path.empty()) {
        return false;
    }

    const auto& p_start  = path.front();
    const auto& p_goal   = path.back();
    const int start_cost = getCellCostWorld(p_start.first, p_start.second);
    const int goal_cost  = getCellCostWorld(p_goal.first, p_goal.second);

    // 始点・終点が壁際やインフレーションマージン内に配置されている場合、
    // 端点から脱出/進入する区間 (0.35m以内) は端点自体のコストまで許容し、中間経路で厳格に判定する
    constexpr double endpoint_margin_dist = 0.35;

    for (const auto& point : path) {
        const int cost = getCellCostWorld(point.first, point.second);
        if (cost >= 100 || cost < 0) {
            return false;
        }

        int allowed = max_allowed_cost;
        const double d_start =
            std::hypot(point.first - p_start.first, point.second - p_start.second);
        const double d_goal =
            std::hypot(point.first - p_goal.first, point.second - p_goal.second);

        if (d_start <= endpoint_margin_dist) {
            allowed = std::max(allowed, start_cost);
        }
        if (d_goal <= endpoint_margin_dist) {
            allowed = std::max(allowed, goal_cost);
        }

        if (cost > allowed) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<double, double>> PathPlannerNode::approximateStraightLines(
    const std::vector<std::pair<double, double>>& path
) const
{
    using Point = std::pair<double, double>;
    if (path.size() < 3 || line_approx_max_cost_ < 0 || line_approx_min_distance_ <= 0.0) {
        return path;
    }

    const double res =
        (latest_costmap_.info.resolution > 0.0) ? latest_costmap_.info.resolution : 0.05;
    const double check_step = std::max(0.01, res * 0.5);
    const double point_step = std::clamp(res * 0.5, 0.01, 0.05);

    const auto isSegmentLowCost = [&](const Point& p0, const Point& p1) -> bool {
        if (getCellCostWorld(p0.first, p0.second) > line_approx_max_cost_ ||
            getCellCostWorld(p1.first, p1.second) > line_approx_max_cost_) {
            return false;
        }
        const double dist     = std::hypot(p1.first - p0.first, p1.second - p0.second);
        const int num_samples = std::max(1, static_cast<int>(std::ceil(dist / check_step)));
        for (int s = 0; s <= num_samples; ++s) {
            const double t  = static_cast<double>(s) / num_samples;
            const double wx = p0.first + t * (p1.first - p0.first);
            const double wy = p0.second + t * (p1.second - p0.second);
            const int cost  = getCellCostWorld(wx, wy);
            if (cost < 0 || cost > line_approx_max_cost_ || cost >= 100) {
                return false;
            }
        }
        return true;
    };

    std::vector<Point> result;
    result.reserve(path.size());
    result.push_back(path.front());
    std::vector<std::size_t> junction_indices;

    std::size_t i = 0;
    while (i + 1 < path.size()) {
        std::size_t best_j = i;
        if (getCellCostWorld(path[i].first, path[i].second) <= line_approx_max_cost_) {
            for (std::size_t j = path.size() - 1; j > i + 1; --j) {
                const double chord_dist =
                    std::hypot(path[j].first - path[i].first, path[j].second - path[i].second);
                if (chord_dist < line_approx_min_distance_) {
                    continue;
                }
                if (isSegmentLowCost(path[i], path[j])) {
                    best_j = j;
                    break;
                }
            }
        }

        if (best_j > i + 1) {
            if (i > 0) {
                junction_indices.push_back(result.size() - 1);
            }
            const Point& p0        = path[i];
            const Point& p1        = path[best_j];
            const double chord_len = std::hypot(p1.first - p0.first, p1.second - p0.second);
            const int steps = std::max(1, static_cast<int>(std::ceil(chord_len / point_step)));
            for (int s = 1; s <= steps; ++s) {
                const double t = static_cast<double>(s) / steps;
                result.emplace_back(
                    p0.first + t * (p1.first - p0.first), p0.second + t * (p1.second - p0.second)
                );
            }
            if (best_j + 1 < path.size()) {
                junction_indices.push_back(result.size() - 1);
            }
            i = best_j;
        } else {
            result.push_back(path[i + 1]);
            ++i;
        }
    }

    // 直線区間と曲線区間の接続点 (junction) のみ局所的な微小コーナーラウンディングを行い、
    // 直線区間の直線性を維持しつつ接続点の局所曲率スパイクを防止する
    if (!junction_indices.empty() && result.size() >= 5) {
        std::vector<bool> smooth_mask(result.size(), false);
        for (std::size_t j_idx : junction_indices) {
            const std::size_t start_k = (j_idx > 3) ? (j_idx - 3) : 1;
            const std::size_t end_k   = std::min(result.size() - 2, j_idx + 3);
            for (std::size_t k = start_k; k <= end_k; ++k) {
                smooth_mask[k] = true;
            }
        }
        for (int iter = 0; iter < 3; ++iter) {
            const auto prev = result;
            for (std::size_t k = 1; k + 1 < result.size(); ++k) {
                if (!smooth_mask[k]) {
                    continue;
                }
                const Point candidate{
                    0.25 * prev[k - 1].first + 0.5 * prev[k].first + 0.25 * prev[k + 1].first,
                    0.25 * prev[k - 1].second + 0.5 * prev[k].second + 0.25 * prev[k + 1].second};
                if (getCellCostWorld(candidate.first, candidate.second) <=
                    std::max(line_approx_max_cost_, bspline_smooth_max_cost_)) {
                    result[k] = candidate;
                }
            }
        }
    }

    result.front() = path.front();
    result.back()  = path.back();
    return result;
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

    // 4. 動的適応型 B-spline による平滑化（連続始点〜アプローチ点間）
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

    // 4.5. 低コスト領域かつ最小距離以上の区間を直線近似
    smoothed_path = approximateStraightLines(smoothed_path);

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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
